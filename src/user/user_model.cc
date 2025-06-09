// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "user/user_model.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <array>
#include <csetjmp>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjplugin.h>
#include "cc/array_safety.h"
#include "engine/engine_forward.h"
#include "engine/engine_io.h"
#include "engine/engine_name.h"
#include "engine/engine_plugin.h"
#include "engine/engine_setconst.h"
#include "engine/engine_support.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_api.h"
#include "user/user_objects.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::string;
using std::vector;
constexpr int kMaxCompilerThreads = 16;



//---------------------------------- LOCAL UTILITY FUNCTIONS ---------------------------------------

constexpr double kFrameEps = 1e-6;  // difference below which frames are considered equal

// return true if two 3-vectors are element-wise less than kFrameEps apart
template <typename T>
bool IsSameVec(const T pos1[3], const T pos2[3]) {
  static_assert(std::is_floating_point_v<T>);
  return std::abs(pos1[0] - pos2[0]) < kFrameEps &&
         std::abs(pos1[1] - pos2[1]) < kFrameEps &&
         std::abs(pos1[2] - pos2[2]) < kFrameEps;
}

// return true if two quaternions are element-wise less than kFrameEps apart, including double-cover
template <typename T>
bool IsSameQuat(const T quat1[4], const T quat2[4]) {
  static_assert(std::is_floating_point_v<T>);
  bool same_quat_minus = std::abs(quat1[0] - quat2[0]) < kFrameEps &&
                         std::abs(quat1[1] - quat2[1]) < kFrameEps &&
                         std::abs(quat1[2] - quat2[2]) < kFrameEps &&
                         std::abs(quat1[3] - quat2[3]) < kFrameEps;

  bool same_quat_plus = std::abs(quat1[0] + quat2[0]) < kFrameEps &&
                        std::abs(quat1[1] + quat2[1]) < kFrameEps &&
                        std::abs(quat1[2] + quat2[2]) < kFrameEps &&
                        std::abs(quat1[3] + quat2[3]) < kFrameEps;

  return same_quat_minus || same_quat_plus;
}


// compare two poses
template <typename T>
bool IsSamePose(const T pos1[3], const T pos2[3], const T quat1[4], const T quat2[4]) {
  // check position if given
  if (pos1 && pos2 && !IsSameVec(pos1, pos2)) {
    return false;
  }

  // check orientation if given
  if (quat1 && quat2 && !IsSameQuat(quat1, quat2)) {
    return false;
  }

  return true;
}

// detect null pose
template <typename T>
bool IsNullPose(const T pos[3], const T quat[4]) {
  T zero[3] = {0, 0, 0};
  T qunit[4] = {1, 0, 0, 0};
  return IsSamePose(pos, zero, quat, qunit);
}


// set ids, check for repeated names
template <class T>
static void processlist(mjListKeyMap& ids, vector<T*>& list,
                        mjtObj type, bool checkrepeat = true) {
  // assign ids for regular elements
  if (type < mjNOBJECT) {
    for (size_t i=0; i < list.size(); i++) {
      // check for incompatible id setting; SHOULD NOT OCCUR
      if (list[i]->id != -1 && list[i]->id != i) {
        throw mjCError(list[i], "incompatible id in %s array, position %d", mju_type2Str(type), i);
      }

      // id equals position in array
      list[i]->id = i;

      // add to ids map
      ids[type][list[i]->name] = i;
    }
  }

  // check for repeated names
  if (checkrepeat) {
    // created vectors with all names
    vector<string> allnames;
    for (size_t i=0; i < list.size(); i++) {
      if (!list[i]->name.empty()) {
        allnames.push_back(list[i]->name);
      }
    }

    // sort and check for duplicates
    if (allnames.size() > 1) {
      std::sort(allnames.begin(), allnames.end());
      auto adjacent = std::adjacent_find(allnames.begin(), allnames.end());
      if (adjacent != allnames.end()) {
        string msg = "repeated name '" + *adjacent + "' in " + mju_type2Str(type);
        throw mjCError(nullptr, "%s", msg.c_str());
      }
    }
  }
}


}  // namespace

//---------------------------------- CONSTRUCTOR AND DESTRUCTOR ------------------------------------

// constructor
mjCModel::mjCModel() {
  mjs_defaultSpec(&spec);
  elemtype = mjOBJ_MODEL;
  spec_comment_.clear();
  spec_modelfiledir_.clear();
  spec_meshdir_.clear();
  spec_texturedir_.clear();
  spec_modelname_ = "MuJoCo Model";

  //------------------------ auto-computed statistics
#ifndef MEMORY_SANITIZER
  // initializing as best practice, but want MSAN to catch uninitialized use
  meaninertia_auto = 0;
  meanmass_auto = 0;
  meansize_auto = 0;
  extent_auto = 0;
  center_auto[0] = center_auto[1] = center_auto[2] = 0;
#endif

  deepcopy_ = false;
  nplugin = 0;
  Clear();

  //------------------------ master default set
  defaults_.push_back(new mjCDef(this));
  defaults_.back()->name = "main";

  // point to model from spec
  PointToLocal();

  // world body
  mjCBody* world = new mjCBody(this);
  mjuu_zerovec(world->pos, 3);
  mjuu_setvec(world->quat, 1, 0, 0, 0);
  world->mass = 0;
  mjuu_zerovec(world->inertia, 3);
  world->id = 0;
  world->parent = nullptr;
  world->weldid = 0;
  world->name = "world";
  world->classname = "main";
  def_map["main"] = Default();
  bodies_.push_back(world);

  // create mjCBase lists from children lists
  CreateObjectLists();

  // set the signature
  spec.element->signature = 0;
}



mjCModel::mjCModel(const mjCModel& other) {
  CreateObjectLists();
  *this = other;
}



mjCModel& mjCModel::operator=(const mjCModel& other) {
  deepcopy_ = true;
  if (this != &other) {
    this->spec = other.spec;
    *static_cast<mjCModel_*>(this) = static_cast<const mjCModel_&>(other);
    *static_cast<mjSpec*>(this) = static_cast<const mjSpec&>(other);

    // copy attached specs first so that we can resolve references to them
    for (const auto* s : other.specs_) {
      specs_.push_back(mj_copySpec(s));
      compiler2spec_[&s->compiler] = specs_.back();
    }

    // the world copy constructor takes care of copying the tree
    mjCBody* world = new mjCBody(*other.bodies_[0], this);
    bodies_.push_back(world);

    // update tree lists
    ResetTreeLists();
    MakeTreeLists();

    // add everything else
    *this += other;

    // add keyframes
    CopyList(keys_, other.keys_);

    // create new default tree
    mjCDef* subtree = new mjCDef(*other.defaults_[0]);
    *this += *subtree;

    // copy name maps
    for (int i=0; i < mjNOBJECT; i++) {
      ids[i] = other.ids[i];
    }

    // update signature after we updated everything
    spec.element->signature = Signature();
  }
  deepcopy_ = other.deepcopy_;
  return *this;
}



// copy vector of elements from another model to this model
template <class T>
void mjCModel::CopyList(std::vector<T*>& dest,
                        const std::vector<T*>& source) {
  // loop over the elements from the other model
  int nsource = (int)source.size();
  for (int i = 0; i < nsource; i++) {
    T* candidate = deepcopy_ ? new T(*source[i]) : source[i];
    try {
      // try to find the referenced object in this model
      mjCModel* source_model = source[i]->model;
      candidate->model = this;
      candidate->NameSpace(source_model);
      candidate->CopyFromSpec();
      candidate->ResolveReferences(this);
    } catch (mjCError err) {
      // if not present, skip the element
      // TODO: do not skip elements that contain user errors
      if (deepcopy_) {
        candidate->model = nullptr;
        delete candidate;
      }
      continue;
    }
    // copy the element from the other model to this model
    if (deepcopy_) {
      source[i]->ForgetKeyframes();
    } else {
      candidate->AddRef();
    }
    mjSpec* origin = FindSpec(source[i]->compiler);
    dest.push_back(candidate);
    dest.back()->model = this;
    dest.back()->compiler = origin ? &origin->compiler : &spec.compiler;
    dest.back()->id = -1;
    dest.back()->CopyPlugin();
  }
  if (!dest.empty()) {
    processlist(ids, dest, dest[0]->elemtype);
  }
}



template <class T>
static void resetlist(std::vector<T*>& list) {
  for (auto element : list) {
    element->id = -1;
  }
  list.clear();
}



void mjCModel::ResetTreeLists() {
  mjCBody *world = bodies_[0];
  resetlist(bodies_);
  resetlist(joints_);
  resetlist(geoms_);
  resetlist(sites_);
  resetlist(cameras_);
  resetlist(lights_);
  resetlist(frames_);
  world->id = 0;
  bodies_.push_back(world);
}



// save associated state addresses in related elements
void mjCModel::SaveDofOffsets(bool computesize) {
  int qposadr = 0;
  int dofadr = 0;
  int actadr = 0;
  int mocapadr = 0;

  for (auto joint : joints_) {
    joint->qposadr_ = qposadr;
    joint->dofadr_ = dofadr;
    qposadr += joint->nq();
    dofadr += joint->nv();
  }

  for (auto actuator : actuators_) {
    if (actuator->spec.actdim > 0) {
      actuator->actdim_ = actuator->spec.actdim;
    } else {
      actuator->actdim_ = (actuator->spec.dyntype != mjDYN_NONE);
    }
    actuator->actadr_ = actuator->actdim_ ? actadr : -1;
    actadr += actuator->actdim_;
  }

  for (mjCBody* body : bodies_) {
    if (body->spec.mocap) {
      body->mocapid = mocapadr++;
    } else {
      body->mocapid = -1;
    }
  }

  if (computesize) {
    nq = qposadr;
    nv = dofadr;
    na = actadr;
    nu = (int)actuators_.size();
    nmocap = mocapadr;
  }
}



template <class T>
void mjCModel::CopyExplicitPlugin(T* obj) {
  if (!obj->plugin.active || !obj->plugin_instance_name.empty() || !obj->spec.plugin.element) {
    return;
  }
  mjCPlugin* origin = static_cast<mjCPlugin*>(obj->spec.plugin.element);
  mjCPlugin* candidate = deepcopy_ ? new mjCPlugin(*origin) : origin;
  candidate->id = plugins_.size();
  candidate->model = this;
  if (!deepcopy_) {
    candidate->AddRef();
  }
  plugins_.push_back(candidate);
  obj->spec.plugin.element = candidate;
}

template void mjCModel::CopyExplicitPlugin<mjCBody>(mjCBody* obj);
template void mjCModel::CopyExplicitPlugin<mjCGeom>(mjCGeom* obj);
template void mjCModel::CopyExplicitPlugin<mjCMesh>(mjCMesh* obj);
template void mjCModel::CopyExplicitPlugin<mjCActuator>(mjCActuator* obj);
template void mjCModel::CopyExplicitPlugin<mjCSensor>(mjCSensor* obj);



template <class T>
void mjCModel::CopyPlugin(const std::vector<mjCPlugin*>& source,
                          const std::vector<T*>& list) {
  // store elements that reference a plugin instance
  std::unordered_map<std::string, T*> instances;
  for (const auto& element : list) {
    if (!element->plugin_instance_name.empty()) {
      instances[element->plugin_instance_name] = element;
    }
  }

  // only copy plugins that are referenced
  for (const auto& plugin : source) {
    if (plugin->name.empty() && plugin->model == this) {
      continue;
    }
    mjCPlugin* candidate = new mjCPlugin(*plugin);
    candidate->model = this;
    candidate->NameSpace(plugin->model);
    bool referenced = instances.find(candidate->name) != instances.end();
    auto same_name = [candidate](const mjCPlugin* dest) {
                       return dest->name == candidate->name;
                     };
    bool instance_exists = std::find_if(plugins_.begin(), plugins_.end(),
                                        same_name) != plugins_.end();
    if (referenced && !instance_exists) {
      plugins_.push_back(candidate);
      instances.at(candidate->name)->spec.plugin.element = candidate;
    } else {
      delete candidate;
    }
  }
}



// return true if the plugin is already in the list of active plugins
static bool IsPluginActive(
  const mjpPlugin* plugin,
  const std::vector<std::pair<const mjpPlugin*, int> >& active_plugins) {
  return std::find_if(
    active_plugins.begin(), active_plugins.end(),
    [&plugin](const std::pair<const mjpPlugin*, int>& element) {
    return element.first == plugin;
  }) != active_plugins.end();
}



mjCModel& mjCModel::operator+=(const mjCModel& other) {
  // create global lists
  ResetTreeLists();
  MakeTreeLists();
  ProcessLists(/*checkrepeat=*/false);

  // copy all elements not in the tree
  if (this != &other) {
    // do not copy assets for self-attach
    // TODO: asset should be copied only when referenced
    CopyList(meshes_, other.meshes_);
    CopyList(skins_, other.skins_);
    CopyList(hfields_, other.hfields_);
    CopyList(textures_, other.textures_);
    CopyList(materials_, other.materials_);
    for (const auto& key : other.key_pending_) {
      key_pending_.push_back(key);
    }
    CopyList(numerics_, other.numerics_);
    CopyList(texts_, other.texts_);
  }
  CopyList(flexes_, other.flexes_);
  CopyList(pairs_, other.pairs_);
  CopyList(excludes_, other.excludes_);
  CopyList(tendons_, other.tendons_);
  CopyList(equalities_, other.equalities_);
  CopyList(actuators_, other.actuators_);
  CopyList(sensors_, other.sensors_);
  CopyList(tuples_, other.tuples_);

  // create new plugins and map them
  CopyPlugin(other.plugins_, bodies_);
  CopyPlugin(other.plugins_, geoms_);
  CopyPlugin(other.plugins_, meshes_);
  CopyPlugin(other.plugins_, actuators_);
  CopyPlugin(other.plugins_, sensors_);
  for (const auto& [plugin, slot] : other.active_plugins_) {
    if (!IsPluginActive(plugin, active_plugins_)) {
      active_plugins_.emplace_back(std::make_pair(plugin, slot));
    }
  }

  // resize keyframes in the parent model
  if (!keys_.empty()) {
    SaveDofOffsets(/*computesize=*/true);
    ComputeReference();
    for (auto* key : keys_) {
      ResizeKeyframe(key, qpos0.data(), body_pos0.data(), body_quat0.data());
    }
    nq = nv = na = nu = nmocap = 0;
  }

  // update pointers to local elements
  PointToLocal();

  // update signature after we updated the tree lists and we updated the pointers
  spec.element->signature = Signature();
  return *this;
}



template <class T>
void mjCModel::RemoveFromList(std::vector<T*>& list, const mjCModel& other) {
  int nlist = (int)list.size();
  int removed = 0;
  for (int i = 0; i < nlist; i++) {
    T* element = list[i];
    element->id -= removed;
    try {
      // check if the element contains an error
      element->NameSpace(&other);
      element->CopyFromSpec();
      element->ResolveReferences(&other);
    } catch (mjCError err) {
      continue;
    }
    try {
      // check if the element references something that was removed
      element->NameSpace(this);
      element->CopyFromSpec();
      element->ResolveReferences(this);
    } catch (mjCError err) {
      ids[element->elemtype].erase(element->name);
      element->Release();
      list.erase(list.begin() + i);
      nlist--;
      i--;
      removed++;
    }
  }
  if (removed > 0 && !list.empty()) {
    // if any elements were removed, update ids using processlist
    processlist(ids, list, list[0]->elemtype, /*checkrepeat=*/false);
  }
}



template <>
void mjCModel::DeleteAll<mjCKey>(std::vector<mjCKey*>& elements) {
  for (mjCKey* element : elements) {
    element->Release();
  }
  elements.clear();
}



template <class T>
void mjCModel::MarkPluginInstance(std::unordered_map<std::string, bool>& instances,
                                  const std::vector<T*>& list) {
  for (const auto& element : list) {
    if (!element->plugin_instance_name.empty()) {
      instances[element->plugin_instance_name] = true;
    }
  }
}



void mjCModel::RemovePlugins() {
  // store elements that reference a plugin instance
  std::unordered_map<std::string, bool> instances;
  MarkPluginInstance(instances, bodies_);
  MarkPluginInstance(instances, geoms_);
  MarkPluginInstance(instances, meshes_);
  MarkPluginInstance(instances, actuators_);
  MarkPluginInstance(instances, sensors_);

  // remove plugins that are not referenced
  int nlist = (int)plugins_.size();
  int removed = 0;
  for (int i = 0; i < nlist; i++) {
    if (plugins_[i]->name.empty()) {
      continue;
    }
    if (instances.find(plugins_[i]->name) == instances.end()) {
      ids[plugins_[i]->elemtype].erase(plugins_[i]->name);
      plugins_[i]->Release();
      plugins_.erase(plugins_.begin() + i);
      nlist--;
      i--;
      removed++;
    }
  }

  // if any elements were removed, update ids using processlist
  if (removed > 0 && !plugins_.empty()) {
    processlist(ids, plugins_, plugins_[0]->elemtype, /*checkrepeat=*/false);
  }
}



mjCModel& mjCModel::operator-=(const mjCBody& subtree) {
  mjCModel oldmodel(*this);

  // create global lists in the old model if not compiled
  if (!oldmodel.IsCompiled()) {
    oldmodel.ProcessLists(/*checkrepeat=*/false);
  }

  // create global lists in this model if not compiled
  if (!IsCompiled()) {
    ProcessLists(/*checkrepeat=*/false);
  }

  // all keyframes are now pending and they will be resized
  StoreKeyframes(this);
  DeleteAll(keys_);

  // remove body from tree
  mjCBody* world = bodies_[0];
  *world -= subtree;

  // update global lists
  ResetTreeLists();
  MakeTreeLists();
  ProcessLists(/*checkrepeat=*/false);

  // check if we have to remove anything else
  RemoveFromList(pairs_, oldmodel);
  RemoveFromList(excludes_, oldmodel);
  RemoveFromList(tendons_, oldmodel);
  RemoveFromList(equalities_, oldmodel);
  RemoveFromList(actuators_, oldmodel);
  RemoveFromList(sensors_, oldmodel);
  RemovePlugins();

  // update signature before we reset the tree lists
  spec.element->signature = Signature();

  return *this;
}



// add default tree to this model
mjCModel_& mjCModel::operator+=(mjCDef& subtree) {
  defaults_.push_back(&subtree);
  def_map[subtree.name] = &subtree;
  subtree.model = this;

  // set parent to the main default if this is not the only default in the model
  if (!subtree.parent && &subtree != defaults_[0]) {
    subtree.parent = defaults_[0];
    defaults_[0]->child.push_back(&subtree);
  }

  for (auto def : subtree.child) {
    *this += *def;  // triggers recursive call
  }
  return *this;
}



// remove default class from array
mjCModel& mjCModel::operator-=(const mjCDef& subtree) {

  // check we aren't trying to remove the 'main' default
  if (subtree.id == 0) {
    throw mjCError(0, "cannot remove the global default ('main')");
  }

  // remove this default from parent's child list
  mjCDef* parent = subtree.parent;
  if (parent) {
    for (int i = 0; i < parent->child.size(); ++i) {
      if (parent->child[i] == &subtree) {
        parent->child.erase(parent->child.begin() + i);
        break;
      }
    }
  }

  // traverse tree to find all descendants starting from subtree.id
  std::vector<int> default_ids_to_remove;
  std::vector<int> stack;
  stack.push_back(subtree.id);
  while (!stack.empty()) {
    int id = stack.back();
    stack.pop_back();
    default_ids_to_remove.push_back(id);
    for (int i=0; i<defaults_[id]->child.size(); i++) {
      stack.push_back(defaults_[id]->child[i]->id);
    }
  }

  // remove from the tree
  std::sort(default_ids_to_remove.begin(),
            default_ids_to_remove.end(),
            std::greater<int>());

  for (int id : default_ids_to_remove) {
    delete defaults_[id];
    defaults_.erase(defaults_.begin() + id);
  }

  // reset default ids
  for (int i = 0; i < defaults_.size(); ++i) {
    defaults_[i]->id = i;
  }

  return *this;
}



template <class T>
void deletefromlist(std::vector<T*>* list, mjsElement* element) {
  if (!list) {
    return;
  }
  for (int j = 0; j < list->size(); ++j) {
    list->at(j)->id = -1;
    if (list->at(j) == element) {
      list->at(j)->Release();
      list->erase(list->begin() + j);
      j--;
    }
  }
}



// discard all invalid elements from all lists
void mjCModel::DeleteElement(mjsElement* el) {
  ResetTreeLists();

  switch (el->elemtype) {
    case mjOBJ_BODY:
      MakeTreeLists();  // rebuild lists that were reset at the beginning of the function
      throw mjCError(nullptr, "bodies cannot be deleted, use detach instead");
      break;

    case mjOBJ_DEFAULT:
      MakeTreeLists();  // rebuild lists that were reset at the beginning of the function
      throw mjCError(nullptr, "defaults cannot be deleted, use detach instead");
      break;

    case mjOBJ_GEOM:
    {
      mjCGeom* geom = static_cast<mjCGeom*>(el);
      if (geom->plugin.active && geom->plugin.name->empty() && geom->GetRef() == 1) {
        DeleteElement(geom->plugin.element);
      }
      deletefromlist(&(geom->body->geoms), el);
      break;
    }

    case mjOBJ_SITE:
      deletefromlist(&(static_cast<mjCSite*>(el)->body->sites), el);
      break;

    case mjOBJ_JOINT:
      deletefromlist(&(static_cast<mjCJoint*>(el)->body->joints), el);
      break;

    case mjOBJ_LIGHT:
      deletefromlist(&(static_cast<mjCLight*>(el)->body->lights), el);
      break;

    case mjOBJ_CAMERA:
      deletefromlist(&(static_cast<mjCCamera*>(el)->body->cameras), el);
      break;

    case mjOBJ_MESH:
    {
      mjCMesh* mesh = static_cast<mjCMesh*>(el);
      if (mesh->plugin.active && mesh->plugin.name->empty() && mesh->GetRef() == 1) {
        DeleteElement(mesh->plugin.element);
      }
      deletefromlist(object_lists_[mjOBJ_MESH], el);
      break;
    }

    case mjOBJ_ACTUATOR:
    {
      mjCActuator* actuator = static_cast<mjCActuator*>(el);
      if (actuator->plugin.active && actuator->plugin.name->empty() && actuator->GetRef() == 1) {
        DeleteElement(actuator->plugin.element);
      }
      deletefromlist(object_lists_[mjOBJ_ACTUATOR], el);
      break;
    }

    case mjOBJ_SENSOR:
    {
      mjCSensor* sensor = static_cast<mjCSensor*>(el);
      if (sensor->plugin.active && sensor->plugin.name->empty() && sensor->GetRef() == 1) {
        DeleteElement(sensor->plugin.element);
      }
      deletefromlist(object_lists_[mjOBJ_SENSOR], el);
      break;
    }

    default:
      deletefromlist(object_lists_[el->elemtype], el);
      break;
  }

  ResetTreeLists();  // in case of a nested delete
  MakeTreeLists();
  ProcessLists(/*checkrepeat=*/false);

  // update signature after we updated everything
  spec.element->signature = Signature();
}



// recursively delete all plugins in the subtree
void deletesubtreeplugin(mjCBody* subtree, mjCModel* model) {
  mjsPlugin* plugin = &(subtree->spec.plugin);
  if (plugin->active && plugin->name->empty()) {
    model->DeleteElement(plugin->element);
  }
  for (auto* body : subtree->Bodies()) {
    deletesubtreeplugin(body, model);
  }
}



// deletes all plugins in the subtree and then the subtree itself
void mjCModel::Detach(mjCBody* subtree) {
  if (subtree->GetRef() == 1)  {
    deletesubtreeplugin(subtree, this);
  }
  subtree->Release();
}



// TODO: we should not use C-type casting with multiple C++ inheritance
void mjCModel::CreateObjectLists() {
  for (int i = 0; i < mjNOBJECT; ++i) {
    object_lists_[i] = nullptr;
  }

  object_lists_[mjOBJ_BODY]     = (std::vector<mjCBase*>*) &bodies_;
  object_lists_[mjOBJ_XBODY]    = (std::vector<mjCBase*>*) &bodies_;
  object_lists_[mjOBJ_JOINT]    = (std::vector<mjCBase*>*) &joints_;
  object_lists_[mjOBJ_GEOM]     = (std::vector<mjCBase*>*) &geoms_;
  object_lists_[mjOBJ_SITE]     = (std::vector<mjCBase*>*) &sites_;
  object_lists_[mjOBJ_CAMERA]   = (std::vector<mjCBase*>*) &cameras_;
  object_lists_[mjOBJ_LIGHT]    = (std::vector<mjCBase*>*) &lights_;
  object_lists_[mjOBJ_FLEX]     = (std::vector<mjCBase*>*) &flexes_;
  object_lists_[mjOBJ_MESH]     = (std::vector<mjCBase*>*) &meshes_;
  object_lists_[mjOBJ_SKIN]     = (std::vector<mjCBase*>*) &skins_;
  object_lists_[mjOBJ_HFIELD]   = (std::vector<mjCBase*>*) &hfields_;
  object_lists_[mjOBJ_TEXTURE]  = (std::vector<mjCBase*>*) &textures_;
  object_lists_[mjOBJ_MATERIAL] = (std::vector<mjCBase*>*) &materials_;
  object_lists_[mjOBJ_PAIR]     = (std::vector<mjCBase*>*) &pairs_;
  object_lists_[mjOBJ_EXCLUDE]  = (std::vector<mjCBase*>*) &excludes_;
  object_lists_[mjOBJ_EQUALITY] = (std::vector<mjCBase*>*) &equalities_;
  object_lists_[mjOBJ_TENDON]   = (std::vector<mjCBase*>*) &tendons_;
  object_lists_[mjOBJ_ACTUATOR] = (std::vector<mjCBase*>*) &actuators_;
  object_lists_[mjOBJ_SENSOR]   = (std::vector<mjCBase*>*) &sensors_;
  object_lists_[mjOBJ_NUMERIC]  = (std::vector<mjCBase*>*) &numerics_;
  object_lists_[mjOBJ_TEXT]     = (std::vector<mjCBase*>*) &texts_;
  object_lists_[mjOBJ_TUPLE]    = (std::vector<mjCBase*>*) &tuples_;
  object_lists_[mjOBJ_KEY]      = (std::vector<mjCBase*>*) &keys_;
  object_lists_[mjOBJ_PLUGIN]   = (std::vector<mjCBase*>*) &plugins_;
}



void mjCModel::PointToLocal() {
  spec.element = static_cast<mjsElement*>(this);
  spec.comment = &spec_comment_;
  spec.modelfiledir = &spec_modelfiledir_;
  spec.modelname = &spec_modelname_;
  spec.meshdir = &spec_meshdir_;
  spec.texturedir = &spec_texturedir_;
  comment = nullptr;
  modelfiledir = nullptr;
  modelname = nullptr;
  meshdir = nullptr;
  texturedir = nullptr;
}



void mjCModel::CopyFromSpec() {
  *static_cast<mjSpec*>(this) = spec;
  comment_ = spec_comment_;
  modelfiledir_ = spec_modelfiledir_;
  modelname_ = spec_modelname_;
  meshdir_ = spec_meshdir_;
  texturedir_ = spec_texturedir_;
}



// destructor
mjCModel::~mjCModel() {
  // do not rebuild lists if we are in the process of deleting the model
  compiled = false;

  // delete kinematic tree and all objects allocated in it
  bodies_[0]->Release();

  // delete objects allocated in mjCModel
  for (int i=0; i < flexes_.size(); i++) flexes_[i]->Release();
  for (int i=0; i < meshes_.size(); i++) meshes_[i]->Release();
  for (int i=0; i < skins_.size(); i++) skins_[i]->Release();
  for (int i=0; i < hfields_.size(); i++) hfields_[i]->Release();
  for (int i=0; i < textures_.size(); i++) textures_[i]->Release();
  for (int i=0; i < materials_.size(); i++) materials_[i]->Release();
  for (int i=0; i < pairs_.size(); i++) pairs_[i]->Release();
  for (int i=0; i < excludes_.size(); i++) excludes_[i]->Release();
  for (int i=0; i < equalities_.size(); i++) equalities_[i]->Release();
  for (int i=0; i < tendons_.size(); i++) tendons_[i]->Release();  // also deletes wraps
  for (int i=0; i < actuators_.size(); i++) actuators_[i]->Release();
  for (int i=0; i < sensors_.size(); i++) sensors_[i]->Release();
  for (int i=0; i < numerics_.size(); i++) numerics_[i]->Release();
  for (int i=0; i < texts_.size(); i++) texts_[i]->Release();
  for (int i=0; i < tuples_.size(); i++) tuples_[i]->Release();
  for (int i=0; i < keys_.size(); i++) keys_[i]->Release();
  for (int i=0; i < defaults_.size(); i++) delete defaults_[i];
  for (int i=0; i < specs_.size(); i++) mj_deleteSpec(specs_[i]);
  for (int i=0; i < plugins_.size(); i++) plugins_[i]->Release();

  // clear sizes and pointer lists created in Compile
  Clear();
}



// clear objects allocated by Compile
void mjCModel::Clear() {
  // sizes set from list lengths
  nbody = 0;
  nbvh = 0;
  nbvhstatic = 0;
  nbvhdynamic = 0;
  njnt = 0;
  ngeom = 0;
  nsite = 0;
  ncam = 0;
  nlight = 0;
  nflex = 0;
  nmesh = 0;
  nskin = 0;
  nhfield = 0;
  ntex = 0;
  nmat = 0;
  npair = 0;
  nexclude = 0;
  neq = 0;
  ntendon = 0;
  nsensor = 0;
  nnumeric = 0;
  ntext = 0;

  // sizes set by Compile
  nq = 0;
  nv = 0;
  nu = 0;
  na = 0;
  nflexnode = 0;
  nflexvert = 0;
  nflexedge = 0;
  nflexelem = 0;
  nflexelemdata = 0;
  nflexelemedge = 0;
  nflexshelldata = 0;
  nflexevpair = 0;
  nflextexcoord = 0;
  nmeshvert = 0;
  nmeshnormal = 0;
  nmeshtexcoord = 0;
  nmeshface = 0;
  nmeshgraph = 0;
  nmeshpoly = 0;
  nmeshpolyvert = 0;
  nmeshpolymap = 0;
  nskinvert = 0;
  nskintexvert = 0;
  nskinface = 0;
  nskinbone = 0;
  nskinbonevert = 0;
  nhfielddata = 0;
  ntexdata = 0;
  nwrap = 0;
  nsensordata = 0;
  nnumericdata = 0;
  ntextdata = 0;
  ntupledata = 0;
  npluginattr = 0;
  nnames = 0;
  npaths = 0;
  memory = -1;
  nstack = -1;
  nemax = 0;
  nM = 0;
  nD = 0;
  nB = 0;
  nJmom = 0;
  njmax = -1;
  nconmax = -1;
  nmocap = 0;

  // internal variables
  hasImplicitPluginElem = false;
  compiled = false;
  errInfo = mjCError();
  qpos0.clear();
}



//------------------------ API FOR ADDING MODEL ELEMENTS -------------------------------------------

// add object of any type
template <class T>
T* mjCModel::AddObject(vector<T*>& list, string type) {
  T* obj = new T(this);
  obj->id = (int)list.size();
  list.push_back(obj);
  spec.element->signature = Signature();
  return obj;
}


// add object of any type, with default parameter
template <class T>
T* mjCModel::AddObjectDefault(vector<T*>& list, string type, mjCDef* def) {
  T* obj = new T(this, def ? def : defaults_[0]);
  obj->id = (int)list.size();
  obj->classname = def ? def->name : "main";
  list.push_back(obj);
  spec.element->signature = Signature();
  return obj;
}


// add flex
mjCFlex* mjCModel::AddFlex() {
  return AddObject(flexes_, "flex");
}


// add mesh
mjCMesh* mjCModel::AddMesh(mjCDef* def) {
  return AddObjectDefault(meshes_, "mesh", def);
}


// add skin
mjCSkin* mjCModel::AddSkin() {
  return AddObject(skins_, "skin");
}


// add hfield
mjCHField* mjCModel::AddHField() {
  return AddObject(hfields_, "hfield");
}


// add texture
mjCTexture* mjCModel::AddTexture() {
  return AddObject(textures_, "texture");
}


// add material
mjCMaterial* mjCModel::AddMaterial(mjCDef* def) {
  return AddObjectDefault(materials_, "material", def);
}


// add geom pair to include in collisions
mjCPair* mjCModel::AddPair(mjCDef* def) {
  return AddObjectDefault(pairs_, "pair", def);
}


// add body pair to exclude from collisions
mjCBodyPair* mjCModel::AddExclude() {
  return AddObject(excludes_, "exclude");
}


// add constraint
mjCEquality* mjCModel::AddEquality(mjCDef* def) {
  return AddObjectDefault(equalities_, "equality", def);
}


// add tendon
mjCTendon* mjCModel::AddTendon(mjCDef* def) {
  return AddObjectDefault(tendons_, "tendon", def);
}


// add actuator
mjCActuator* mjCModel::AddActuator(mjCDef* def) {
  return AddObjectDefault(actuators_, "actuator", def);
}


// add sensor
mjCSensor* mjCModel::AddSensor() {
  return AddObject(sensors_, "sensor");
}



// add custom
mjCNumeric* mjCModel::AddNumeric() {
  return AddObject(numerics_, "numeric");
}


// add text
mjCText* mjCModel::AddText() {
  return AddObject(texts_, "text");
}


// add tuple
mjCTuple* mjCModel::AddTuple() {
  return AddObject(tuples_, "tuple");
}


// add keyframe
mjCKey* mjCModel::AddKey() {
  return AddObject(keys_, "key");
}


// add plugin instance
mjCPlugin* mjCModel::AddPlugin() {
  return AddObject(plugins_, "plugin");
}


// append spec to spec
void mjCModel::AppendSpec(mjSpec* spec, const mjsCompiler* compiler_) {
  // TODO: check if the spec is already in the list
  specs_.push_back(spec);

  if (compiler_) {
    compiler2spec_[compiler_] = spec;
  }
}



//------------------------ API FOR ACCESS TO MODEL ELEMENTS  ---------------------------------------

// get number of objects of specified type
int mjCModel::NumObjects(mjtObj type) {
  if (!object_lists_[type]) {
    return 0;
  }
  return (int) object_lists_[type]->size();
}



// get pointer to specified object
mjCBase* mjCModel::GetObject(mjtObj type, int id) {
  if (id < 0 || id >= NumObjects(type)) {
    return nullptr;
  }
  return (*object_lists_[type])[id];
}



template <class T>
static mjsElement* GetNext(std::vector<T*>& list, mjsElement* child) {
  if (!child) {
    if (list.empty()) {
      return nullptr;
    }
    return list[0]->spec.element;
  }

  // TODO: use id for direct indexing instead of a loop
  for (unsigned int i = 0; i < list.size()-1; i++) {
    if (list[i]->spec.element == child) {
      return list[i+1]->spec.element;
    }
  }
  return nullptr;
}



// next object of specified type
mjsElement* mjCModel::NextObject(mjsElement* object, mjtObj type) {
  if (type == mjOBJ_UNKNOWN) {
    if (!object) {
      throw mjCError(nullptr, "type must be specified if no element is given");
    } else {
      type = object->elemtype;
    }
  } else if (object && object->elemtype != type) {
    throw mjCError(nullptr, "element is not of requested type");
  }

  switch (type) {
    case mjOBJ_BODY:
      if (!object) {
        return bodies_[0]->spec.element;
      } else if (object == bodies_[0]->spec.element) {
        return bodies_[0]->NextChild(NULL, type, /*recursive=*/true);
      } else {
        return bodies_[0]->NextChild(object, type, /*recursive=*/true);
      }
    case mjOBJ_SITE:
    case mjOBJ_GEOM:
    case mjOBJ_JOINT:
    case mjOBJ_CAMERA:
    case mjOBJ_LIGHT:
    case mjOBJ_FRAME:
      return bodies_[0]->NextChild(object, type, /*recursive=*/true);
    case mjOBJ_ACTUATOR:
      return GetNext(actuators_, object);
    case mjOBJ_SENSOR:
      return GetNext(sensors_, object);
    case mjOBJ_FLEX:
      return GetNext(flexes_, object);
    case mjOBJ_PAIR:
      return GetNext(pairs_, object);
    case mjOBJ_EXCLUDE:
      return GetNext(excludes_, object);
    case mjOBJ_EQUALITY:
      return GetNext(equalities_, object);
    case mjOBJ_TENDON:
      return GetNext(tendons_, object);
    case mjOBJ_NUMERIC:
      return GetNext(numerics_, object);
    case mjOBJ_TEXT:
      return GetNext(texts_, object);
    case mjOBJ_TUPLE:
      return GetNext(tuples_, object);
    case mjOBJ_KEY:
      return GetNext(keys_, object);
    case mjOBJ_MESH:
      return GetNext(meshes_, object);
    case mjOBJ_HFIELD:
      return GetNext(hfields_, object);
    case mjOBJ_SKIN:
      return GetNext(skins_, object);
    case mjOBJ_TEXTURE:
      return GetNext(textures_, object);
    case mjOBJ_MATERIAL:
      return GetNext(materials_, object);
    case mjOBJ_PLUGIN:
      return GetNext(plugins_, object);
    default:
      return nullptr;
  }
}


//------------------------ API FOR ACCESS TO PRIVATE VARIABLES -------------------------------------

// compiled flag
bool mjCModel::IsCompiled() const {
  return compiled;
}



// get reference of error object
const mjCError& mjCModel::GetError() const {
  return errInfo;
}



// pointer to world body
mjCBody* mjCModel::GetWorld() {
  return bodies_[0];
}



// find default class name in array
mjCDef* mjCModel::FindDefault(string name) {
  for (int i=0; i < (int)defaults_.size(); i++) {
    if (defaults_[i]->name == name) {
      return defaults_[i];
    }
  }
  return nullptr;
}



// add default class to array
mjCDef* mjCModel::AddDefault(string name, mjCDef* parent) {
  // check for repeated name
  int thisid = (int)defaults_.size();
  for (int i=0; i < thisid; i++) {
    if (defaults_[i]->name == name) {
      return 0;
    }
  }

  // create new object
  mjCDef* def = new mjCDef(parent->model);
  defaults_.push_back(def);
  def->id = thisid;

  // initialize contents
  if (parent && parent->id < thisid) {
    parent->CopyFromSpec();
    def->CopyWithoutChildren(*parent);
    parent->child.push_back(def);
  }
  def->parent = parent;
  def->name = name;
  def->child.clear();
  def_map[name] = def;

  return def;
}



// find object by name in given list
template <class T>
static T* findobject(std::string_view name, const vector<T*>& list, const mjKeyMap& ids) {
  // this can occur in the URDF parser
  if (ids.empty()) {
    for (unsigned int i=0; i < list.size(); i++) {
      if (list[i]->name == name) {
        return list[i];
      }
    }
    return nullptr;
  }

  // during model compilation
  auto id = ids.find(name);
  if (id == ids.end()) {
    return nullptr;
  }
  if (id->second > (int)list.size() - 1) {
    throw mjCError(0, "object not found");
  }
  return list[id->second];
}

// find object in global lists given string type and name
mjCBase* mjCModel::FindObject(mjtObj type, string name) const {
  if (!object_lists_[type]) {
    return nullptr;
  }
  return findobject(name, *object_lists_[type], ids[type]);
}



// find body by name
mjCBase* mjCModel::FindTree(mjCBody* body, mjtObj type, std::string name) {
  switch (type) {
    case mjOBJ_BODY:
      if (body->name == name) {
        return body;
      }
      break;
    case mjOBJ_SITE:
      for (auto site : body->sites) {
        if (site->name == name) {
          return site;
        }
      }
      break;
    case mjOBJ_GEOM:
      for (auto geom : body->geoms) {
        if (geom->name == name) {
          return geom;
        }
      }
      break;
    case mjOBJ_JOINT:
      for (auto joint : body->joints) {
        if (joint->name == name) {
          return joint;
        }
      }
      break;
    case mjOBJ_CAMERA:
      for (auto camera : body->cameras) {
        if (camera->name == name) {
          return camera;
        }
      }
      break;
    case mjOBJ_LIGHT:
      for (auto light : body->lights) {
        if (light->name == name) {
          return light;
        }
      }
      break;
    case mjOBJ_FRAME:
      for (auto frame : body->frames) {
        if (frame->name == name) {
          return frame;
        }
      }
      break;
    default:
      return nullptr;
  }

  for (auto child : body->bodies) {
    auto candidate = FindTree(child, type, name);
    if (candidate) {
      return candidate;
    }
  }

  return nullptr;
}



// find spec by name
mjSpec* mjCModel::FindSpec(std::string name) const {
  for (auto spec : specs_) {
    if (mjs_getString(spec->modelname) == name) {
      return spec;
    }
  }
  return nullptr;
}



// find spec by mjsCompiler pointer
mjSpec* mjCModel::FindSpec(const mjsCompiler* compiler_) {
  if (compiler_ == &spec.compiler) {
    return &spec;
  }

  if (compiler2spec_.find(compiler_) != compiler2spec_.end()) {
    return compiler2spec_[compiler_];
  }

  for (auto s : specs_) {
    mjSpec* source = static_cast<mjCModel*>(s->element)->FindSpec(compiler_);
    if (source) {
      return source;
    }
  }
  return nullptr;
}



//------------------------------- COMPILER PHASES --------------------------------------------------

// make lists of objects in tree: bodies, geoms, joints, sites, cameras, lights
void mjCModel::MakeTreeLists(mjCBody* body) {
  if (body == nullptr) {
    body = bodies_[0];
  }

  // add this body if not world
  if (body != bodies_[0]) {
    bodies_.push_back(body);
  }

  // add body's geoms, joints, sites, cameras, lights
  for (mjCGeom *geom : body->geoms) geoms_.push_back(geom);
  for (mjCJoint *joint : body->joints) joints_.push_back(joint);
  for (mjCSite *site : body->sites) sites_.push_back(site);
  for (mjCCamera *camera : body->cameras) cameras_.push_back(camera);
  for (mjCLight *light : body->lights) lights_.push_back(light);
  for (mjCFrame *frame : body->frames) frames_.push_back(frame);

  // recursive call to all child bodies
  for (mjCBody* body : body->bodies) MakeTreeLists(body);
}


// delete material with given name or all materials if the name is omitted
template <class T>
void mjCModel::DeleteMaterial(std::vector<T*>& list, std::string_view name) {
  for (T* plist : list) {
    if (name.empty() || plist->get_material() == name) {
      plist->del_material();
    }
  }
}



// delete all textures
template <class T>
static void DeleteAllTextures(std::vector<T*>& list) {
  for (T* plist : list) {
    plist->del_textures();
  }
}


// delete all texture coordinates
template <class T>
static void DeleteTexcoord(std::vector<T*>& list) {
  for (T* plist : list) {
    if (plist->HasTexcoord()) {
      plist->DelTexcoord();
    }
  }
}


// returns a vector that stores the reference correction for each entry
template <class T>
static void DeleteElements(std::vector<T*>& elements,
                           const std::vector<bool>& discard) {
  if (elements.empty()) {
    return;
  }

  std::vector<int> ndiscard(elements.size(), 0);

  int i = 0;
  for (int j=0; j < elements.size(); j++) {
    if (discard[j]) {
      elements[j]->Release();
    } else {
      elements[i] = elements[j];
      i++;
    }
  }

  // count cumulative discarded elements
  for (int i=0; i < elements.size()-1; i++) {
    ndiscard[i+1] = ndiscard[i] + discard[i];
  }

  // erase elements from vector
  if (i < elements.size()) {
    elements.erase(elements.begin() + i, elements.end());
  }

  // update elements
  for (T* element : elements) {
    if (element->id > 0) {
      element->id -= ndiscard[element->id];
    }
  }
}


template <>
void mjCModel::Delete<mjCGeom>(std::vector<mjCGeom*>& elements,
                               const std::vector<bool>& discard) {
  // update bodies
  for (mjCBody* body : bodies_) {
    body->geoms.erase(
      std::remove_if(body->geoms.begin(), body->geoms.end(),
                     [&discard](mjCGeom* geom) {
      return discard[geom->id];
    }),
      body->geoms.end());
  }

  // remove geoms from the main vector
  DeleteElements(elements, discard);
}


template <>
void mjCModel::Delete<mjCMesh>(std::vector<mjCMesh*>& elements,
                               const std::vector<bool>& discard) {
  DeleteElements(elements, discard);
}


template <>
void mjCModel::DeleteAll<mjCMaterial>(std::vector<mjCMaterial*>& elements) {
  DeleteMaterial(geoms_);
  DeleteMaterial(skins_);
  DeleteMaterial(sites_);
  DeleteMaterial(tendons_);
  for (mjCMaterial* element : elements) {
    element->Release();
  }
  elements.clear();
}


template <>
void mjCModel::DeleteAll<mjCTexture>(std::vector<mjCTexture*>& elements) {
  DeleteAllTextures(materials_);
  for (mjCTexture* element : elements) {
    element->Release();
  }
  elements.clear();
}


// set nuser fields
void mjCModel::SetNuser() {
  if (nuser_body == -1) {
    nuser_body = 0;
    for (int i = 0; i < bodies_.size(); i++) {
      nuser_body = mjMAX(nuser_body, bodies_[i]->spec_userdata_.size());
    }
  }
  if (nuser_jnt == -1) {
    nuser_jnt = 0;
    for (int i = 0; i < joints_.size(); i++) {
      nuser_jnt = mjMAX(nuser_jnt, joints_[i]->spec_userdata_.size());
    }
  }
  if (nuser_geom == -1) {
    nuser_geom = 0;
    for (int i = 0; i < geoms_.size(); i++) {
      nuser_geom = mjMAX(nuser_geom, geoms_[i]->spec_userdata_.size());
    }
  }
  if (nuser_site == -1) {
    nuser_site = 0;
    for (int i = 0; i < sites_.size(); i++) {
      nuser_site = mjMAX(nuser_site, sites_[i]->spec_userdata_.size());
    }
  }
  if (nuser_cam == -1) {
    nuser_cam = 0;
    for (int i = 0; i < cameras_.size(); i++) {
      nuser_cam = mjMAX(nuser_cam, cameras_[i]->spec_userdata_.size());
    }
  }
  if (nuser_tendon == -1) {
    nuser_tendon = 0;
    for (int i = 0; i < tendons_.size(); i++) {
      nuser_tendon = mjMAX(nuser_tendon, tendons_[i]->spec_userdata_.size());
    }
  }
  if (nuser_actuator == -1) {
    nuser_actuator = 0;
    for (int i = 0; i < actuators_.size(); i++) {
      nuser_actuator = mjMAX(nuser_actuator, actuators_[i]->spec_userdata_.size());
    }
  }
  if (nuser_sensor == -1) {
    nuser_sensor = 0;
    for (int i = 0; i < sensors_.size(); i++) {
      nuser_sensor = mjMAX(nuser_sensor, sensors_[i]->spec_userdata_.size());
    }
  }
}

// index assets
void mjCModel::IndexAssets(bool discard) {
  // assets referenced in geoms
  for (int i=0; i < geoms_.size(); i++) {
    mjCGeom* geom = geoms_[i];

    // find material by name
    if (!geom->get_material().empty()) {
      mjCBase* material = FindObject(mjOBJ_MATERIAL, geom->get_material());
      if (material) {
        geom->matid = material->id;
      } else {
        throw mjCError(geom, "material '%s' not found in geom %d", geom->get_material().c_str(), i);
      }
    }

    // find mesh by name
    if (!geom->get_meshname().empty()) {
      mjCBase* mesh = FindObject(mjOBJ_MESH, geom->get_meshname());
      if (mesh) {
        if (!geom->visual_) {
          ((mjCMesh*)mesh)->SetNotVisual();  // reset to true by mesh->Compile()
        }
        geom->mesh = (discard && geom->visual_) ? nullptr : (mjCMesh*)mesh;
      } else {
        throw mjCError(geom, "mesh '%s' not found in geom %d", geom->get_meshname().c_str(), i);
      }
    }

    // find hfield by name
    if (!geom->get_hfieldname().empty()) {
      mjCBase* hfield = FindObject(mjOBJ_HFIELD, geom->get_hfieldname());
      if (hfield) {
        geom->hfield = (mjCHField*)hfield;
      } else {
        throw mjCError(geom, "hfield '%s' not found in geom %d", geom->get_hfieldname().c_str(), i);
      }
    }
  }

  // assets referenced in skins
  for (int i=0; i < skins_.size(); i++) {
    mjCSkin* skin = skins_[i];

    // find material by name
    if (!skin->material_.empty()) {
      mjCBase* material = FindObject(mjOBJ_MATERIAL, skin->material_);
      if (material) {
        skin->matid = material->id;
      } else {
        throw mjCError(skin, "material '%s' not found in skin %d", skin->material_.c_str(), i);
      }
    }
  }

  // materials referenced in sites
  for (int i=0; i < sites_.size(); i++) {
    mjCSite* site = sites_[i];

    // find material by name
    if (!site->material_.empty()) {
      mjCBase* material = FindObject(mjOBJ_MATERIAL, site->get_material());
      if (material) {
        site->matid = material->id;
      } else {
        throw mjCError(site, "material '%s' not found in site %d", site->material_.c_str(), i);
      }
    }
  }

  // materials referenced in tendons
  for (int i=0; i < tendons_.size(); i++) {
    mjCTendon* tendon = tendons_[i];

    // find material by name
    if (!tendon->material_.empty()) {
      mjCBase* material = FindObject(mjOBJ_MATERIAL, tendon->material_);
      if (material) {
        tendon->matid = material->id;
      } else {
        throw mjCError(tendon, "material '%s' not found in tendon %d", tendon->material_.c_str(), i);
      }
    }
  }

  // textures referenced in materials
  for (int i=0; i < materials_.size(); i++) {
    mjCMaterial* material = materials_[i];

    // find textures by name
    for (int j=0; j < mjNTEXROLE; j++) {
      if (!material->textures_[j].empty()) {
        mjCBase* texture = FindObject(mjOBJ_TEXTURE, material->textures_[j]);
        if (texture) {
          material->texid[j] = texture->id;
        } else {
          throw mjCError(material, "texture '%s' not found in material %d", material->textures_[j].c_str(), i);
        }
      }
    }
  }

  if (discard) {
    std::vector<bool> discard_mesh(meshes_.size(), false);
    std::vector<bool> discard_geom(geoms_.size(), false);

    std::transform(meshes_.begin(), meshes_.end(), discard_mesh.begin(),
                   [](const mjCMesh* mesh) {
      return mesh->IsVisual();
    });
    std::transform(geoms_.begin(), geoms_.end(), discard_geom.begin(),
                   [](const mjCGeom* geom) {
      return geom->IsVisual();
    });

    // update inertia in bodies
    for (auto body : bodies_) {
      if (body->spec.explicitinertial) {
        continue;
      }
      for (auto geom : body->geoms) {
        if (geom->IsVisual()) {
          if (compiler.inertiafromgeom == mjINERTIAFROMGEOM_TRUE) {
            compiler.inertiafromgeom = mjINERTIAFROMGEOM_AUTO;
          }
          body->explicitinertial = true;  // for XML writer
          body->spec.explicitinertial = true;
          body->spec.mass = body->mass;
          mjuu_copyvec(body->spec.ipos, body->ipos, 3);
          mjuu_copyvec(body->spec.iquat, body->iquat, 4);
          mjuu_copyvec(body->spec.inertia, body->inertia, 3);
          break;
        }
      }
    }

    // discard visual meshes and geoms
    Delete(meshes_, discard_mesh);
    Delete(geoms_, discard_geom);
  }
}



// throw error if a name is missing
void mjCModel::CheckEmptyNames(void) {
  // meshes
  for (int i=0; i < meshes_.size(); i++) {
    if (meshes_[i]->name.empty()) {
      throw mjCError(meshes_[i], "empty name in mesh");
    }
  }

  // hfields
  for (int i=0; i < hfields_.size(); i++) {
    if (hfields_[i]->name.empty()) {
      throw mjCError(hfields_[i], "empty name in height field");
    }
  }

  // textures
  for (int i=0; i < textures_.size(); i++) {
    if (textures_[i]->name.empty() && textures_[i]->type != mjTEXTURE_SKYBOX) {
      throw mjCError(textures_[i], "empty name in texture");
    }
  }

  // materials
  for (int i=0; i < materials_.size(); i++) {
    if (materials_[i]->name.empty()) {
      throw mjCError(materials_[i], "empty name in material");
    }
  }
}



template <typename T>
static size_t getpathslength(std::vector<T> list) {
  size_t result = 0;
  for (const auto& element : list) {
    if (!element->File().empty()) {
      result += element->File().length() + 1;
    }
  }

  return result;
}

// set array sizes
void mjCModel::SetSizes() {
  // set from object list sizes
  nbody = (int)bodies_.size();
  njnt = (int)joints_.size();
  ngeom = (int)geoms_.size();
  nsite = (int)sites_.size();
  ncam = (int)cameras_.size();
  nlight = (int)lights_.size();
  nflex = (int)flexes_.size();
  nmesh = (int)meshes_.size();
  nskin = (int)skins_.size();
  nhfield = (int)hfields_.size();
  ntex = (int)textures_.size();
  nmat = (int)materials_.size();
  npair = (int)pairs_.size();
  nexclude = (int)excludes_.size();
  neq = (int)equalities_.size();
  ntendon = (int)tendons_.size();
  nsensor = (int)sensors_.size();
  nnumeric = (int)numerics_.size();
  ntext = (int)texts_.size();
  ntuple = (int)tuples_.size();
  nkey = (int)keys_.size();
  nplugin = (int)plugins_.size();
  nq = nv = nu = na = nmocap = 0;

  // nq, nv
  for (int i=0; i < njnt; i++) {
    nq += joints_[i]->nq();
    nv += joints_[i]->nv();
  }

  // nu, na
  for (int i=0; i < actuators_.size(); i++) {
    nu++;
    na += actuators_[i]->actdim;
  }

  // nbvh, nbvhstatic, nbvhdynamic
  for (int i=0; i < nbody; i++) {
    nbvhstatic += bodies_[i]->tree.Nbvh();
  }
  for (int i=0; i < nmesh; i++) {
    nbvhstatic += meshes_[i]->tree().Nbvh();
  }
  for (int i=0; i < nflex; i++) {
    nbvhdynamic += flexes_[i]->tree.Nbvh();
  }
  nbvh = nbvhstatic + nbvhdynamic;

  // flex counts
  for (int i=0; i < nflex; i++) {
    nflexnode += flexes_[i]->nnode;
    nflexvert += flexes_[i]->nvert;
    nflexedge += flexes_[i]->nedge;
    nflexelem += flexes_[i]->nelem;
    nflexelemdata += flexes_[i]->nelem * (flexes_[i]->dim + 1);
    nflexelemedge += flexes_[i]->nelem * mjCFlex::kNumEdges[flexes_[i]->dim - 1];
    nflexshelldata += (int)flexes_[i]->shell.size();
    nflexevpair += (int)flexes_[i]->evpair.size()/2;
    nflextexcoord += (flexes_[i]->HasTexcoord() ? flexes_[i]->get_texcoord().size()/2 : 0);
  }

  // mesh counts
  for (int i=0; i < nmesh; i++) {
    nmeshvert += meshes_[i]->nvert();
    nmeshnormal += meshes_[i]->nnormal();
    nmeshface += meshes_[i]->nface();
    nmeshtexcoord += (meshes_[i]->HasTexcoord() ? meshes_[i]->ntexcoord() : 0);
    nmeshgraph += meshes_[i]->szgraph();
    nmeshpoly += meshes_[i]->npolygon();
    nmeshpolyvert += meshes_[i]->npolygonvert();
    nmeshpolymap += meshes_[i]->npolygonmap();
  }

  // skin counts
  for (int i=0; i < nskin; i++) {
    nskinvert += skins_[i]->get_vert().size()/3;
    nskintexvert += skins_[i]->get_texcoord().size()/2;
    nskinface += skins_[i]->get_face().size()/3;
    nskinbone += skins_[i]->bodyid.size();
    for (int j=0; j < skins_[i]->bodyid.size(); j++) {
      nskinbonevert += skins_[i]->get_vertid()[j].size();
    }
  }

  // nhfielddata
  for (int i=0; i < nhfield; i++)nhfielddata += hfields_[i]->nrow * hfields_[i]->ncol;

  // ntexdata
  for (int i=0; i < ntex; i++)ntexdata += textures_[i]->nchannel * textures_[i]->width * textures_[i]->height;

  // nwrap
  for (int i=0; i < ntendon; i++)nwrap += (int)tendons_[i]->path.size();

  // nsensordata
  for (int i=0; i < nsensor; i++)nsensordata += sensors_[i]->dim;

  // nnumericdata
  for (int i=0; i < nnumeric; i++)nnumericdata += numerics_[i]->size;

  // ntextdata
  for (int i=0; i < ntext; i++)ntextdata += (int)texts_[i]->data_.size() + 1;

  // ntupledata
  for (int i=0; i < ntuple; i++)ntupledata += (int)tuples_[i]->objtype_.size();

  // npluginattr
  for (int i=0; i < nplugin; i++)npluginattr += (int)plugins_[i]->flattened_attributes.size();

  // nnames
  nnames = (int)modelname_.size() + 1;
  for (int i=0; i < nbody; i++)    nnames += (int)bodies_[i]->name.length() + 1;
  for (int i=0; i < njnt; i++)     nnames += (int)joints_[i]->name.length() + 1;
  for (int i=0; i < ngeom; i++)    nnames += (int)geoms_[i]->name.length() + 1;
  for (int i=0; i < nsite; i++)    nnames += (int)sites_[i]->name.length() + 1;
  for (int i=0; i < ncam; i++)     nnames += (int)cameras_[i]->name.length() + 1;
  for (int i=0; i < nlight; i++)   nnames += (int)lights_[i]->name.length() + 1;
  for (int i=0; i < nflex; i++)    nnames += (int)flexes_[i]->name.length() + 1;
  for (int i=0; i < nmesh; i++)    nnames += (int)meshes_[i]->name.length() + 1;
  for (int i=0; i < nskin; i++)    nnames += (int)skins_[i]->name.length() + 1;
  for (int i=0; i < nhfield; i++)  nnames += (int)hfields_[i]->name.length() + 1;
  for (int i=0; i < ntex; i++)     nnames += (int)textures_[i]->name.length() + 1;
  for (int i=0; i < nmat; i++)     nnames += (int)materials_[i]->name.length() + 1;
  for (int i=0; i < npair; i++)    nnames += (int)pairs_[i]->name.length() + 1;
  for (int i=0; i < nexclude; i++) nnames += (int)excludes_[i]->name.length() + 1;
  for (int i=0; i < neq; i++)      nnames += (int)equalities_[i]->name.length() + 1;
  for (int i=0; i < ntendon; i++)  nnames += (int)tendons_[i]->name.length() + 1;
  for (int i=0; i < nu; i++)       nnames += (int)actuators_[i]->name.length() + 1;
  for (int i=0; i < nsensor; i++)  nnames += (int)sensors_[i]->name.length() + 1;
  for (int i=0; i < nnumeric; i++) nnames += (int)numerics_[i]->name.length() + 1;
  for (int i=0; i < ntext; i++)    nnames += (int)texts_[i]->name.length() + 1;
  for (int i=0; i < ntuple; i++)   nnames += (int)tuples_[i]->name.length() + 1;
  for (int i=0; i < nkey; i++)     nnames += (int)keys_[i]->name.length() + 1;
  for (int i=0; i < nplugin; i++)  nnames += (int)plugins_[i]->name.length() + 1;

  // npaths
  npaths = 0;
  npaths += getpathslength(hfields_);
  npaths += getpathslength(meshes_);
  npaths += getpathslength(skins_);
  npaths += getpathslength(textures_);
  if (npaths == 0) {
    npaths = 1;
  }

  // nemax
  for (int i=0; i < neq; i++) {
    if (equalities_[i]->type == mjEQ_CONNECT) {
      nemax += 3;
    } else if (equalities_[i]->type == mjEQ_WELD) {
      nemax += 7;
    } else {
      nemax += 1;
    }
  }
}



// automatic stiffness and damping computation
void mjCModel::AutoSpringDamper(mjModel* m) {
  // process all joints
  for (int n=0; n < m->njnt; n++) {
    // get joint dof address and number of dimensions
    int adr = m->jnt_dofadr[n];
    int ndim = mjCJoint::nv((mjtJoint)m->jnt_type[n]);

    // get timeconst and dampratio from joint specification
    mjtNum timeconst = (mjtNum)joints_[n]->springdamper[0];
    mjtNum dampratio = (mjtNum)joints_[n]->springdamper[1];

    // skip joint if either parameter is non-positive
    if (timeconst <= 0 || dampratio <= 0) {
      continue;
    }

    // get average inertia (dof_invweight0 in free joint is different for tran and rot)
    mjtNum inertia = 0;
    for (int i=0; i < ndim; i++) {
      inertia += m->dof_invweight0[adr+i];
    }
    inertia = ((mjtNum)ndim) / std::max(mjMINVAL, inertia);

    // compute stiffness and damping (same as solref computation)
    mjtNum stiffness = inertia / std::max(mjMINVAL, timeconst*timeconst*dampratio*dampratio);
    mjtNum damping = 2 * inertia / std::max(mjMINVAL, timeconst);

    // save stiffness and damping in the private mjsJoints
    joints_[n]->stiffness = stiffness;
    joints_[n]->damping = damping;

    // assign
    m->jnt_stiffness[n] = stiffness;
    for (int i=0; i < ndim; i++) {
      m->dof_damping[adr+i] = damping;
    }
  }
}



// arguments for lengthrange thread function
struct _LRThreadArg {
  mjModel* m;
  mjData* data;
  int start;
  int num;
  const mjLROpt* LRopt;
  char* error;
  int error_sz;
};
typedef struct _LRThreadArg LRThreadArg;


// thread function for lengthrange computation
void* LRfunc(void* arg) {
  LRThreadArg* larg = (LRThreadArg*)arg;

  for (int i=larg->start; i < larg->start+larg->num; i++) {
    if (i < larg->m->nu) {
      if (!mj_setLengthRange(larg->m, larg->data, i, larg->LRopt, larg->error, larg->error_sz)) {
        return nullptr;
      }
    }
  }

  return nullptr;
}


// compute actuator lengthrange
void mjCModel::LengthRange(mjModel* m, mjData* data) {
  // save options and modify
  mjOption saveopt = m->opt;
  m->opt.disableflags = mjDSBL_FRICTIONLOSS | mjDSBL_CONTACT | mjDSBL_PASSIVE |
                        mjDSBL_GRAVITY | mjDSBL_ACTUATION;
  if (compiler.LRopt.timestep > 0) {
    m->opt.timestep = compiler.LRopt.timestep;
  }

  // number of threads available
  int hardware_threads = std::thread::hardware_concurrency();
  const int nthread = mjMAX(1, mjMIN(kMaxCompilerThreads, hardware_threads/2));

  // count actuators that need computation
  int cnt = 0;
  for (int i=0; i < m->nu; i++) {
    // skip depending on mode and type
    int ismuscle = (m->actuator_gaintype[i] == mjGAIN_MUSCLE ||
                    m->actuator_biastype[i] == mjBIAS_MUSCLE);
    int isuser = (m->actuator_gaintype[i] == mjGAIN_USER ||
                  m->actuator_biastype[i] == mjBIAS_USER);
    if ((compiler.LRopt.mode == mjLRMODE_NONE) ||
        (compiler.LRopt.mode == mjLRMODE_MUSCLE && !ismuscle) ||
        (compiler.LRopt.mode == mjLRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
      continue;
    }

    // use existing length range if available
    if (compiler.LRopt.useexisting &&
        (m->actuator_lengthrange[2*i] < m->actuator_lengthrange[2*i+1])) {
      continue;
    }

    // count
    cnt++;
  }

  // single thread
  if (!compiler.usethread || cnt < 2 || nthread < 2) {
    char err[200];
    for (int i=0; i < m->nu; i++) {
      if (!mj_setLengthRange(m, data, i, &compiler.LRopt, err, 200)) {
        throw mjCError(0, "%s", err);
      }
    }
  }

  // multiple threads
  else {
    // allocate mjData for each thread
    char err[kMaxCompilerThreads][200];
    mjData* pdata[kMaxCompilerThreads] = {data};
    for (int i=1; i < nthread; i++) {
      pdata[i] = mj_makeData(m);
    }

    // number of actuators per thread
    int num = m->nu / nthread;
    while (num*nthread < m->nu) {
      num++;
    }

    // prepare thread function arguments, clear errors
    LRThreadArg arg[kMaxCompilerThreads];
    for (int i=0; i < nthread; i++) {
      LRThreadArg temp = {m, pdata[i], i*num, num, &compiler.LRopt, err[i], 200};
      arg[i] = temp;
      err[i][0] = 0;
    }

    // launch threads
    std::thread th[kMaxCompilerThreads];
    for (int i=0; i < nthread; i++) {
      th[i] = std::thread(LRfunc, arg+i);
    }

    // wait for threads to finish
    for (int i=0; i < nthread; i++) {
      th[i].join();
    }

    // free mjData allocated here
    for (int i=1; i < nthread; i++) {
      mj_deleteData(pdata[i]);
    }

    // report first error
    for (int i=0; i < nthread; i++) {
      if (err[i][0]) {
        throw mjCError(0, "%s", err[i]);
      }
    }
  }

  // restore options
  m->opt = saveopt;
}


// Add items to a generic list. This enables the names and paths to be stored.
// input - string to add
// adr - current address in the list
// output_adr_field - the field where the address should be stored (name_meshadr)
// output_buffer - the field where the data should be stored (i.e. names or paths)
static int addtolist(const std::string& input, int adr, int* output_adr_field, char* output_buffer) {
  *output_adr_field = adr;

  // copy input
  memcpy(output_buffer+adr, input.c_str(), input.size());
  adr += (int)input.size();

  // append 0
  output_buffer[adr] = 0;
  adr++;

  return adr;
}

// process names from one list: concatenate, compute addresses
template <class T>
static int namelist(vector<T*>& list, int adr, int* name_adr, char* names, int* map) {
  // compute hash map addresses
  int map_size = mjLOAD_MULTIPLE*list.size();
  for (unsigned int i=0; i < list.size(); i++) {
    // ignore empty strings
    if (list[i]->name.empty()) {
      continue;
    }

    uint64_t j = mj_hashString(list[i]->name.c_str(), map_size);

    // find first empty slot using linear probing
    for (; map[j] != -1; j=(j+1) % map_size) {}
    map[j] = i;
  }

  for (unsigned int i=0; i < list.size(); i++) {
    adr = addtolist(list[i]->name, adr, &name_adr[i], names);
  }

  return adr;
}


// copy names, compute name addresses
void mjCModel::CopyNames(mjModel* m) {
  // start with model name
  int adr = (int)modelname_.size()+1;
  int* map_adr = m->names_map;
  mju_strncpy(m->names, modelname_.c_str(), m->nnames);
  memset(m->names_map, -1, sizeof(int) * m->nnames_map);

  // process all lists
  adr = namelist(bodies_, adr, m->name_bodyadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*bodies_.size();

  adr = namelist(joints_, adr, m->name_jntadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*joints_.size();

  adr = namelist(geoms_, adr, m->name_geomadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*geoms_.size();

  adr = namelist(sites_, adr, m->name_siteadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*sites_.size();

  adr = namelist(cameras_, adr, m->name_camadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*cameras_.size();

  adr = namelist(lights_, adr, m->name_lightadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*lights_.size();

  adr = namelist(flexes_, adr, m->name_flexadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*flexes_.size();

  adr = namelist(meshes_, adr, m->name_meshadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*meshes_.size();

  adr = namelist(skins_, adr, m->name_skinadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*skins_.size();

  adr = namelist(hfields_, adr, m->name_hfieldadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*hfields_.size();

  adr = namelist(textures_, adr, m->name_texadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*textures_.size();

  adr = namelist(materials_, adr, m->name_matadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*materials_.size();

  adr = namelist(pairs_, adr, m->name_pairadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*pairs_.size();

  adr = namelist(excludes_, adr, m->name_excludeadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*excludes_.size();

  adr = namelist(equalities_, adr, m->name_eqadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*equalities_.size();

  adr = namelist(tendons_, adr, m->name_tendonadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*tendons_.size();

  adr = namelist(actuators_, adr, m->name_actuatoradr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*actuators_.size();

  adr = namelist(sensors_, adr, m->name_sensoradr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*sensors_.size();

  adr = namelist(numerics_, adr, m->name_numericadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*numerics_.size();

  adr = namelist(texts_, adr, m->name_textadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*texts_.size();

  adr = namelist(tuples_, adr, m->name_tupleadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*tuples_.size();

  adr = namelist(keys_, adr, m->name_keyadr, m->names, map_adr);
  map_adr += mjLOAD_MULTIPLE*keys_.size();

  adr = namelist(plugins_, adr, m->name_pluginadr, m->names, map_adr);

  // check size, SHOULD NOT OCCUR
  if (adr != nnames) {
    throw mjCError(0, "size mismatch in %s: expected %d, got %d", "names", nnames, adr);
  }
}

// process paths from one list: concatenate, compute addresses
template <class T>
static int pathlist(vector<T*>& list, int adr, int* path_adr, char* paths) {
  for (unsigned int i = 0; i < list.size(); ++i) {
    path_adr[i] = -1;
    if (!list[i] || list[i]->File().empty()) {
      continue;
    }
    adr = addtolist(list[i]->File(), adr, &path_adr[i], paths);
  }

  return adr;
}

void mjCModel::CopyPaths(mjModel* m) {
  // start with 0 address, unlike m->names m->paths might be empty
  size_t adr = 0;
  m->paths[0] = 0;
  adr = pathlist(hfields_, adr, m->hfield_pathadr, m->paths);
  adr = pathlist(meshes_, adr, m->mesh_pathadr, m->paths);
  adr = pathlist(skins_, adr, m->skin_pathadr, m->paths);
  adr = pathlist(textures_, adr, m->tex_pathadr, m->paths);
}



// copy objects inside kinematic tree
void mjCModel::CopyTree(mjModel* m) {
  int jntadr = 0;         // addresses in global arrays
  int dofadr = 0;
  int qposadr = 0;
  int bvh_adr = 0;

  // main loop over bodies
  for (int i=0; i < nbody; i++) {
    // get body and parent pointers
    mjCBody* pb = bodies_[i];
    mjCBody* par = pb->parent;

    // set body fields
    m->body_parentid[i] = pb->parent ? pb->parent->id : 0;
    m->body_weldid[i] = pb->weldid;
    m->body_mocapid[i] = pb->mocapid;
    m->body_jntnum[i] = (int)pb->joints.size();
    m->body_jntadr[i] = (!pb->joints.empty() ? jntadr : -1);
    m->body_dofnum[i] = pb->dofnum;
    m->body_dofadr[i] = (pb->dofnum ? dofadr : -1);
    m->body_geomnum[i] = (int)pb->geoms.size();
    m->body_geomadr[i] = (!pb->geoms.empty() ? pb->geoms[0]->id : -1);
    mjuu_copyvec(m->body_pos+3*i, pb->pos, 3);
    mjuu_copyvec(m->body_quat+4*i, pb->quat, 4);
    mjuu_copyvec(m->body_ipos+3*i, pb->ipos, 3);
    mjuu_copyvec(m->body_iquat+4*i, pb->iquat, 4);
    m->body_mass[i] = (mjtNum)pb->mass;
    mjuu_copyvec(m->body_inertia+3*i, pb->inertia, 3);
    m->body_gravcomp[i] = pb->gravcomp;
    mjuu_copyvec(m->body_user+nuser_body*i, pb->get_userdata().data(), nuser_body);

    m->body_contype[i] = pb->contype;
    m->body_conaffinity[i] = pb->conaffinity;
    m->body_margin[i] = (mjtNum)pb->margin;

    // bounding volume hierarchy
    m->body_bvhadr[i] = pb->tree.Nbvh() ? bvh_adr : -1;
    m->body_bvhnum[i] = pb->tree.Nbvh();
    if (pb->tree.Nbvh()) {
      memcpy(m->bvh_aabb + 6*bvh_adr, pb->tree.Bvh().data(), 6*pb->tree.Nbvh()*sizeof(mjtNum));
      memcpy(m->bvh_child + 2*bvh_adr, pb->tree.Child().data(), 2*pb->tree.Nbvh()*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pb->tree.Level().data(), pb->tree.Nbvh()*sizeof(int));
      for (int i=0; i < pb->tree.Nbvh(); i++) {
        m->bvh_nodeid[i + bvh_adr] = pb->tree.Nodeidptr(i) ? *(pb->tree.Nodeidptr(i)) : -1;
      }
    }
    bvh_adr += pb->tree.Nbvh();

    // count free joints
    int cntfree = 0;
    for (int j=0; j < (int)pb->joints.size(); j++) {
      cntfree += (pb->joints[j]->type == mjJNT_FREE);
    }

    // check validity of free joint
    if (cntfree > 1 || (cntfree == 1 && pb->joints.size() > 1)) {
      throw mjCError(pb, "free joint can only appear by itself");
    }
    if (cntfree && par && par->name != "world") {
      throw mjCError(pb, "free joint can only be used on top level");
    }

    // rootid: self if world or child of world, otherwise parent's rootid
    if (i == 0 || (par && par->name == "world")) {
      m->body_rootid[i] = i;
    } else {
      m->body_rootid[i] = m->body_rootid[par->id];
    }

    // init lastdof from parent
    pb->lastdof = par ? par->lastdof : -1;

    // set sameframe
    mjtSameFrame sameframe;
    mjtNum* nullnum = static_cast<mjtNum*>(nullptr);
    if (IsNullPose(m->body_ipos+3*i, m->body_iquat+4*i)) {
      sameframe = mjSAMEFRAME_BODY;
    } else if (IsNullPose(nullnum, m->body_iquat+4*i)) {
      sameframe = mjSAMEFRAME_BODYROT;
    } else {
      sameframe = mjSAMEFRAME_NONE;
    }
    m->body_sameframe[i] = sameframe;

    // init simple: sameframe, and (self-root, or parent is fixed child of world)
    int parentid = m->body_parentid[i];
    m->body_simple[i] = (sameframe == mjSAMEFRAME_BODY &&
                         (m->body_rootid[i] == i ||
                          (m->body_parentid[parentid] == 0 &&
                           m->body_dofnum[parentid] == 0)));

    // a parent body is never simple (unless world)
    if (m->body_parentid[i] > 0) {
      m->body_simple[m->body_parentid[i]] = 0;
    }

    // loop over joints for this body
    int rotfound = 0;
    for (int j=0; j < (int)pb->joints.size(); j++) {
      // get pointer and id
      mjCJoint* pj = pb->joints[j];
      int jid = pj->id;

      // set joint fields
      pj->qposadr_ = qposadr;
      pj->dofadr_ = dofadr;
      m->jnt_type[jid] = pj->type;
      m->jnt_group[jid] = pj->group;
      m->jnt_limited[jid] = (mjtByte)pj->is_limited();
      m->jnt_actfrclimited[jid] = (mjtByte)pj->is_actfrclimited();
      m->jnt_actgravcomp[jid] = pj->actgravcomp;
      m->jnt_qposadr[jid] = qposadr;
      m->jnt_dofadr[jid] = dofadr;
      m->jnt_bodyid[jid] = pj->body->id;
      mjuu_copyvec(m->jnt_pos+3*jid, pj->pos, 3);
      mjuu_copyvec(m->jnt_axis+3*jid, pj->axis, 3);
      m->jnt_stiffness[jid] = (mjtNum)pj->stiffness;
      mjuu_copyvec(m->jnt_range+2*jid, pj->range, 2);
      mjuu_copyvec(m->jnt_actfrcrange+2*jid, pj->actfrcrange, 2);
      mjuu_copyvec(m->jnt_solref+mjNREF*jid, pj->solref_limit, mjNREF);
      mjuu_copyvec(m->jnt_solimp+mjNIMP*jid, pj->solimp_limit, mjNIMP);
      m->jnt_margin[jid] = (mjtNum)pj->margin;
      mjuu_copyvec(m->jnt_user+nuser_jnt*jid, pj->get_userdata().data(), nuser_jnt);

      // not simple if: rotation already found, or pos not zero, or mis-aligned axis
      bool axis_aligned = ((std::abs(pj->axis[0]) > mjEPS) +
                           (std::abs(pj->axis[1]) > mjEPS) +
                           (std::abs(pj->axis[2]) > mjEPS)) == 1;
      if (rotfound || !IsNullPose(m->jnt_pos+3*jid, nullnum) ||
          ((pj->type == mjJNT_HINGE || pj->type == mjJNT_SLIDE) && !axis_aligned)) {
        m->body_simple[i] = 0;
      }

      // mark rotation
      if (pj->type == mjJNT_BALL || pj->type == mjJNT_HINGE) {
        rotfound = 1;
      }

      // set qpos0 and qpos_spring, check type
      switch (pj->type) {
        case mjJNT_FREE:
          mjuu_copyvec(m->qpos0+qposadr, pb->pos, 3);
          mjuu_copyvec(m->qpos0+qposadr+3, pb->quat, 4);
          mjuu_copyvec(m->qpos_spring+qposadr, m->qpos0+qposadr, 7);
          break;

        case mjJNT_BALL:
          m->qpos0[qposadr] = 1;
          m->qpos0[qposadr+1] = 0;
          m->qpos0[qposadr+2] = 0;
          m->qpos0[qposadr+3] = 0;
          mjuu_copyvec(m->qpos_spring+qposadr, m->qpos0+qposadr, 4);
          break;

        case mjJNT_SLIDE:
        case mjJNT_HINGE:
          m->qpos0[qposadr] = (mjtNum)pj->ref;
          m->qpos_spring[qposadr] = (mjtNum)pj->springref;
          break;

        default:
          throw mjCError(pj, "unknown joint type");
      }

      // set dof fields for this joint
      for (int j1=0; j1 < pj->nv(); j1++) {
        // set attributes
        m->dof_bodyid[dofadr] = pb->id;
        m->dof_jntid[dofadr] = jid;
        mjuu_copyvec(m->dof_solref+mjNREF*dofadr, pj->solref_friction, mjNREF);
        mjuu_copyvec(m->dof_solimp+mjNIMP*dofadr, pj->solimp_friction, mjNIMP);
        m->dof_frictionloss[dofadr] = (mjtNum)pj->frictionloss;
        m->dof_armature[dofadr] = (mjtNum)pj->armature;
        m->dof_damping[dofadr] = (mjtNum)pj->damping;

        // set dof_parentid, update body.lastdof
        m->dof_parentid[dofadr] = pb->lastdof;
        pb->lastdof = dofadr;

        // advance dof counter
        dofadr++;
      }

      // advance joint and qpos counters
      jntadr++;
      qposadr += pj->nq();
    }

    // simple body with sliders and no rotational dofs: promote to simple level 2
    if (m->body_simple[i] && m->body_dofnum[i]) {
      m->body_simple[i] = 2;
      for (int j=0; j < (int)pb->joints.size(); j++) {
        if (pb->joints[j]->type != mjJNT_SLIDE) {
          m->body_simple[i] = 1;
          break;
        }
      }
    }

    // loop over geoms for this body
    for (int j=0; j < (int)pb->geoms.size(); j++) {
      // get pointer and id
      mjCGeom* pg = pb->geoms[j];
      int gid = pg->id;

      // set geom fields
      m->geom_type[gid] = pg->type;
      m->geom_contype[gid] = pg->contype;
      m->geom_conaffinity[gid] = pg->conaffinity;
      m->geom_condim[gid] = pg->condim;
      m->geom_bodyid[gid] = pg->body->id;
      if (pg->mesh) {
        m->geom_dataid[gid] = pg->mesh->id;
      } else if (pg->hfield) {
        m->geom_dataid[gid] = pg->hfield->id;
      } else {
        m->geom_dataid[gid] = -1;
      }
      m->geom_matid[gid] = pg->matid;
      m->geom_group[gid] = pg->group;
      m->geom_priority[gid] = pg->priority;
      mjuu_copyvec(m->geom_size+3*gid, pg->size, 3);
      mjuu_copyvec(m->geom_aabb+6*gid, pg->aabb, 6);
      mjuu_copyvec(m->geom_pos+3*gid, pg->pos, 3);
      mjuu_copyvec(m->geom_quat+4*gid, pg->quat, 4);
      mjuu_copyvec(m->geom_friction+3*gid, pg->friction, 3);
      m->geom_solmix[gid] = (mjtNum)pg->solmix;
      mjuu_copyvec(m->geom_solref+mjNREF*gid, pg->solref, mjNREF);
      mjuu_copyvec(m->geom_solimp+mjNIMP*gid, pg->solimp, mjNIMP);
      m->geom_margin[gid] = (mjtNum)pg->margin;
      m->geom_gap[gid] = (mjtNum)pg->gap;
      mjuu_copyvec(m->geom_fluid+mjNFLUID*gid, pg->fluid, mjNFLUID);
      mjuu_copyvec(m->geom_user+nuser_geom*gid, pg->get_userdata().data(), nuser_geom);
      mjuu_copyvec(m->geom_rgba+4*gid, pg->rgba, 4);

      // determine sameframe
      double* nulldouble = static_cast<double*>(nullptr);
      if (IsNullPose(m->geom_pos+3*gid, m->geom_quat+4*gid)) {
        sameframe = mjSAMEFRAME_BODY;
      } else if (IsNullPose(nullnum, m->geom_quat+4*gid)) {
        sameframe = mjSAMEFRAME_BODYROT;
      } else if (IsSamePose(pg->pos, pb->ipos, pg->quat, pb->iquat)) {
        sameframe = mjSAMEFRAME_INERTIA;
      } else if (IsSamePose(nulldouble, nulldouble, pg->quat, pb->iquat)) {
        sameframe = mjSAMEFRAME_INERTIAROT;
      } else {
        sameframe = mjSAMEFRAME_NONE;
      }
      m->geom_sameframe[gid] = sameframe;

      // compute rbound
      m->geom_rbound[gid] = (mjtNum)pg->GetRBound();
    }

    // loop over sites for this body
    for (int j=0; j < (int)pb->sites.size(); j++) {
      // get pointer and id
      mjCSite* ps = pb->sites[j];
      int sid = ps->id;

      // set site fields
      m->site_type[sid] = ps->type;
      m->site_bodyid[sid] = ps->body->id;
      m->site_matid[sid] = ps->matid;
      m->site_group[sid] = ps->group;
      mjuu_copyvec(m->site_size+3*sid, ps->size, 3);
      mjuu_copyvec(m->site_pos+3*sid, ps->pos, 3);
      mjuu_copyvec(m->site_quat+4*sid, ps->quat, 4);
      mjuu_copyvec(m->site_user+nuser_site*sid, ps->userdata_.data(), nuser_site);
      mjuu_copyvec(m->site_rgba+4*sid, ps->rgba, 4);

      // determine sameframe
      double* nulldouble = static_cast<double*>(nullptr);
      if (IsNullPose(m->site_pos+3*sid, m->site_quat+4*sid)) {
        sameframe = mjSAMEFRAME_BODY;
      } else if (IsNullPose(nullnum, m->site_quat+4*sid)) {
        sameframe = mjSAMEFRAME_BODYROT;
      } else if (IsSamePose(ps->pos, pb->ipos, ps->quat, pb->iquat)) {
        sameframe = mjSAMEFRAME_INERTIA;
      } else if (IsSamePose(nulldouble, nulldouble, ps->quat, pb->iquat)) {
        sameframe = mjSAMEFRAME_INERTIAROT;
      } else {
        sameframe = mjSAMEFRAME_NONE;
      }
      m->site_sameframe[sid] = sameframe;
    }

    // loop over cameras for this body
    for (int j=0; j < (int)pb->cameras.size(); j++) {
      // get pointer and id
      mjCCamera* pc = pb->cameras[j];
      int cid = pc->id;

      // set camera fields
      m->cam_bodyid[cid] = pc->body->id;
      m->cam_mode[cid] = pc->mode;
      m->cam_targetbodyid[cid] = pc->targetbodyid;
      mjuu_copyvec(m->cam_pos+3*cid, pc->pos, 3);
      mjuu_copyvec(m->cam_quat+4*cid, pc->quat, 4);
      m->cam_orthographic[cid] = pc->orthographic;
      m->cam_fovy[cid] = (mjtNum)pc->fovy;
      m->cam_ipd[cid] = (mjtNum)pc->ipd;
      mjuu_copyvec(m->cam_resolution+2*cid, pc->resolution, 2);
      mjuu_copyvec(m->cam_sensorsize+2*cid, pc->sensor_size, 2);
      mjuu_copyvec(m->cam_intrinsic+4*cid, pc->intrinsic, 4);
      mjuu_copyvec(m->cam_user+nuser_cam*cid, pc->get_userdata().data(), nuser_cam);
    }

    // loop over lights for this body
    for (int j=0; j < (int)pb->lights.size(); j++) {
      // get pointer and id
      mjCLight* pl = pb->lights[j];
      int lid = pl->id;

      // set light fields
      m->light_bodyid[lid] = pl->body->id;
      m->light_mode[lid] = (int)pl->mode;
      m->light_targetbodyid[lid] = pl->targetbodyid;
      m->light_type[lid] = pl->type;
      m->light_texid[lid] = pl->texid;
      m->light_castshadow[lid] = (mjtByte)pl->castshadow;
      m->light_active[lid] = (mjtByte)pl->active;
      mjuu_copyvec(m->light_pos+3*lid, pl->pos, 3);
      mjuu_copyvec(m->light_dir+3*lid, pl->dir, 3);
      m->light_bulbradius[lid] = pl->bulbradius;
      m->light_intensity[lid] = pl->intensity;
      m->light_range[lid] = pl->range;
      mjuu_copyvec(m->light_attenuation+3*lid, pl->attenuation, 3);
      m->light_cutoff[lid] = pl->cutoff;
      m->light_exponent[lid] = pl->exponent;
      mjuu_copyvec(m->light_ambient+3*lid, pl->ambient, 3);
      mjuu_copyvec(m->light_diffuse+3*lid, pl->diffuse, 3);
      mjuu_copyvec(m->light_specular+3*lid, pl->specular, 3);
    }
  }

  // check number of dof's constructed, SHOULD NOT OCCUR
  if (nv != dofadr) {
    throw mjCError(0, "unexpected number of DOFs");
  }

  // count kinematic trees under world body, compute dof_treeid
  int ntree = 0;
  for (int i=0; i < nv; i++) {
    if (m->dof_parentid[i] == -1) {
      ntree++;
    }
    m->dof_treeid[i] = ntree - 1;
  }
  m->ntree = ntree;

  // compute body_treeid
  for (int i=0; i < nbody; i++) {
    int weldid = m->body_weldid[i];
    if (m->body_dofnum[weldid]) {
      m->body_treeid[i] = m->dof_treeid[m->body_dofadr[weldid]];
    } else {
      m->body_treeid[i] = -1;
    }
  }

  // count bodies with gravity compensation, compute ngravcomp
  int ngravcomp = 0;
  for (int i=0; i < nbody; i++) {
    ngravcomp += (m->body_gravcomp[i] > 0);
  }
  m->ngravcomp = ngravcomp;

  // compute nM and dof_Madr
  nM = 0;
  for (int i=0; i < nv; i++) {
    // set address of this dof
    m->dof_Madr[i] = nM;

    // count ancestor dofs including self
    int j = i;
    while (j >= 0) {
      nM++;
      j = m->dof_parentid[j];
    }
  }
  m->nM = nM;

  // compute nD
  nD = 2 * nM - nv;
  m->nD = nD;

  // compute subtreedofs in backward pass over bodies
  for (int i = nbody - 1; i > 0; i--) {
    // add body dofs to self count
    bodies_[i]->subtreedofs += bodies_[i]->dofnum;

    // add to parent count
    if (bodies_[i]->parent) {
      bodies_[i]->parent->subtreedofs += bodies_[i]->subtreedofs;
    }
  }

  // make sure all dofs are in world "subtree", SHOULD NOT OCCUR
  if (bodies_[0]->subtreedofs != nv) {
    throw mjCError(0, "all DOFs should be in world subtree");
  }

  // compute nB
  nB = 0;
  for (int i = 0; i < nbody; i++) {
    // add subtree dofs (including self)
    nB += bodies_[i]->subtreedofs;

    // add dofs in ancestor bodies
    int j = bodies_[i]->parent ? bodies_[i]->parent->id : 0;
    while (j > 0) {
      nB += bodies_[j]->dofnum;
      j = bodies_[j]->parent ? bodies_[j]->parent->id : 0;
    }
  }
  m->nB = nB;
}

// copy plugin data
void mjCModel::CopyPlugins(mjModel* m) {
  // assign plugin slots and copy plugin config attributes
  {
    int adr = 0;
    for (int i = 0; i < nplugin; ++i) {
      m->plugin[i] = plugins_[i]->plugin_slot;
      const int size = plugins_[i]->flattened_attributes.size();
      std::memcpy(m->plugin_attr + adr,
                  plugins_[i]->flattened_attributes.data(), size);
      m->plugin_attradr[i] = adr;
      adr += size;
    }
  }

  // query and set plugin-related information
  {
    // set actuator_plugin to the plugin instance ID
    std::vector<std::vector<int> > plugin_to_actuators(nplugin);
    for (int i = 0; i < nu; ++i) {
      if (actuators_[i]->plugin.active) {
        int actuator_plugin = static_cast<mjCPlugin*>(actuators_[i]->plugin.element)->id;
        m->actuator_plugin[i] = actuator_plugin;
        plugin_to_actuators[actuator_plugin].push_back(i);
      } else {
        m->actuator_plugin[i] = -1;
      }
    }

    for (int i = 0; i < nbody; ++i) {
      if (bodies_[i]->plugin.active) {
        m->body_plugin[i] = static_cast<mjCPlugin*>(bodies_[i]->plugin.element)->id;
      } else {
        m->body_plugin[i] = -1;
      }
    }

    for (int i = 0; i < ngeom; ++i) {
      if (geoms_[i]->plugin.active) {
        m->geom_plugin[i] = static_cast<mjCPlugin*>(geoms_[i]->plugin.element)->id;
      } else {
        m->geom_plugin[i] = -1;
      }
    }

    std::vector<std::vector<int> > plugin_to_sensors(nplugin);
    for (int i = 0; i < nsensor; ++i) {
      if (sensors_[i]->type == mjSENS_PLUGIN) {
        int sensor_plugin = static_cast<mjCPlugin*>(sensors_[i]->plugin.element)->id;
        m->sensor_plugin[i] = sensor_plugin;
        plugin_to_sensors[sensor_plugin].push_back(i);
      } else {
        m->sensor_plugin[i] = -1;
      }
    }

    // query plugin->nstate, compute and set plugin_state and plugin_stateadr
    // for sensor plugins, also query plugin->nsensordata and set nsensordata
    int stateadr = 0;
    for (int i = 0; i < nplugin; ++i) {
      const mjpPlugin* plugin = mjp_getPluginAtSlot(m->plugin[i]);
      if (!plugin->nstate) {
        mju_error("`nstate` is null for plugin at slot %d", m->plugin[i]);
      }
      int nstate = plugin->nstate(m, i);
      m->plugin_stateadr[i] = stateadr;
      m->plugin_statenum[i] = nstate;
      stateadr += nstate;
      if (plugin->capabilityflags & mjPLUGIN_SENSOR) {
        for (int sensor_id : plugin_to_sensors[i]) {
          if (!plugin->nsensordata) {
            mju_error("`nsensordata` is null for plugin at slot %d", m->plugin[i]);
          }
          int nsensordata = plugin->nsensordata(m, i, sensor_id);
          sensors_[sensor_id]->dim = nsensordata;
          sensors_[sensor_id]->needstage =
            static_cast<mjtStage>(plugin->needstage);
          this->nsensordata += nsensordata;
        }
      }
    }
    m->npluginstate = stateadr;
  }
}



// compute non-zeros in actuator_moment matrix
int mjCModel::CountNJmom(const mjModel* m) {
  int nu = m->nu;
  int nv = m->nv;

  int count = 0;
  for (int i = 0; i < nu; i++) {
    // extract info
    int id = m->actuator_trnid[2 * i];

    // process according to transmission type
    switch ((mjtTrn)m->actuator_trntype[i]) {
      case mjTRN_JOINT:
      case mjTRN_JOINTINPARENT:
        switch ((mjtJoint)m->jnt_type[id]) {
          case mjJNT_SLIDE:
          case mjJNT_HINGE:
            count += 1;
            break;

          case mjJNT_BALL:
            count += 3;
            break;

          case mjJNT_FREE:
            count += 6;
            break;
        }
        break;
      // TODO(taylorhowell): improve upper bounds
      case mjTRN_SLIDERCRANK:
        count += nv;
        break;

      case mjTRN_TENDON:
        count += nv;
        break;

      case mjTRN_SITE:
        count += nv;
        break;

      case mjTRN_BODY:
        count += nv;
        break;

      default:
        // SHOULD NOT OCCUR
        throw mjCError(0, "unknown transmission type");
        break;
    }
  }
  return count;
}



// copy objects outside kinematic tree
void mjCModel::CopyObjects(mjModel* m) {
  int adr, bone_adr, vert_adr, node_adr, normal_adr, face_adr, texcoord_adr;
  int edge_adr, elem_adr, elemdata_adr, elemedge_adr, shelldata_adr, evpair_adr;
  int bonevert_adr, graph_adr, data_adr, bvh_adr;
  int poly_adr, polymap_adr, polyvert_adr;

  // sizes outside call to mj_makeModel
  m->nemax = nemax;
  m->njmax = njmax;
  m->nconmax = nconmax;
  m->nsensordata = nsensordata;
  m->nuserdata = nuserdata;
  m->na = na;

  // find bvh_adr after bodies
  bvh_adr = 0;
  for (int i=0; i < nbody; i++) {
    bvh_adr = mjMAX(bvh_adr, m->body_bvhadr[i] + m->body_bvhnum[i]);
  }

  // meshes
  vert_adr = 0;
  normal_adr = 0;
  texcoord_adr = 0;
  face_adr = 0;
  graph_adr = 0;
  poly_adr = 0;
  polyvert_adr = 0;
  polymap_adr = 0;
  for (int i=0; i < nmesh; i++) {
    // get pointer
    mjCMesh* pme = meshes_[i];

    // set fields
    m->mesh_polyadr[i] = poly_adr;
    m->mesh_polynum[i] = pme->npolygon();
    m->mesh_vertadr[i] = vert_adr;
    m->mesh_vertnum[i] = pme->nvert();
    m->mesh_normaladr[i] = normal_adr;
    m->mesh_normalnum[i] = pme->nnormal();
    m->mesh_texcoordadr[i] = (pme->HasTexcoord() ? texcoord_adr : -1);
    m->mesh_texcoordnum[i] = pme->ntexcoord();
    m->mesh_faceadr[i] = face_adr;
    m->mesh_facenum[i] = pme->nface();
    m->mesh_graphadr[i] = (pme->szgraph() ? graph_adr : -1);
    m->mesh_bvhnum[i] = pme->tree().Nbvh();
    m->mesh_bvhadr[i] = pme->tree().Nbvh() ? bvh_adr : -1;
    mjuu_copyvec(&m->mesh_scale[3 * i], pme->Scale(), 3);
    mjuu_copyvec(&m->mesh_pos[3 * i], pme->GetPosPtr(), 3);
    mjuu_copyvec(&m->mesh_quat[4 * i], pme->GetQuatPtr(), 4);

    // copy vertices, normals, faces, texcoords, aux data
    pme->CopyVert(m->mesh_vert + 3*vert_adr);
    pme->CopyNormal(m->mesh_normal + 3*normal_adr);
    pme->CopyFace(m->mesh_face + 3*face_adr);
    pme->CopyFaceNormal(m->mesh_facenormal + 3*face_adr);
    if (pme->HasTexcoord()) {
      pme->CopyTexcoord(m->mesh_texcoord + 2*texcoord_adr);
      pme->CopyFaceTexcoord(m->mesh_facetexcoord + 3*face_adr);
    } else {
      memset(m->mesh_facetexcoord + 3*face_adr, 0, 3*pme->nface()*sizeof(int));
    }
    if (pme->szgraph()) {
      pme->CopyGraph(m->mesh_graph + graph_adr);
    }
    pme->CopyPolygonNormals(m->mesh_polynormal + 3*poly_adr);
    pme->CopyPolygons(m->mesh_polyvert + polyvert_adr, m->mesh_polyvertadr + poly_adr,
                      m->mesh_polyvertnum + poly_adr, polyvert_adr);
    pme->CopyPolygonMap(m->mesh_polymap + polymap_adr, m->mesh_polymapadr + vert_adr,
                        m->mesh_polymapnum + vert_adr, polymap_adr);

    // copy bvh data
    if (pme->tree().Nbvh()) {
      memcpy(m->bvh_aabb + 6*bvh_adr, pme->tree().Bvh().data(), 6*pme->tree().Nbvh()*sizeof(mjtNum));
      memcpy(m->bvh_child + 2*bvh_adr, pme->tree().Child().data(), 2*pme->tree().Nbvh()*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pme->tree().Level().data(), pme->tree().Nbvh()*sizeof(int));
      for (int j=0; j < pme->tree().Nbvh(); j++) {
        m->bvh_nodeid[j + bvh_adr] = pme->tree().Nodeid(j) > -1 ? pme->tree().Nodeid(j) : -1;
      }
    }

    // advance counters
    poly_adr += pme->npolygon();
    polyvert_adr += pme->npolygonvert();
    polymap_adr += pme->npolygonmap();
    vert_adr += pme->nvert();
    normal_adr += pme->nnormal();
    texcoord_adr += (pme->HasTexcoord() ? pme->ntexcoord() : 0);
    face_adr += pme->nface();
    graph_adr += pme->szgraph();
    bvh_adr += pme->tree().Nbvh();
  }

  // flexes
  vert_adr = 0;
  node_adr = 0;
  edge_adr = 0;
  elem_adr = 0;
  elemdata_adr = 0;
  elemedge_adr = 0;
  shelldata_adr = 0;
  evpair_adr = 0;
  texcoord_adr = 0;
  for (int i=0; i < nflex; i++) {
    // get pointer
    mjCFlex* pfl = flexes_[i];

    // set fields: geom-like
    m->flex_contype[i] = pfl->contype;
    m->flex_conaffinity[i] = pfl->conaffinity;
    m->flex_condim[i] = pfl->condim;
    m->flex_matid[i] = pfl->matid;
    m->flex_group[i] = pfl->group;
    m->flex_priority[i] = pfl->priority;
    m->flex_solmix[i] = (mjtNum)pfl->solmix;
    mjuu_copyvec(m->flex_solref + mjNREF * i, pfl->solref, mjNREF);
    mjuu_copyvec(m->flex_solimp + mjNIMP * i, pfl->solimp, mjNIMP);
    m->flex_radius[i] = (mjtNum)pfl->radius;
    mjuu_copyvec(m->flex_friction + 3 * i, pfl->friction, 3);
    m->flex_margin[i] = (mjtNum)pfl->margin;
    m->flex_gap[i] = (mjtNum)pfl->gap;
    mjuu_copyvec(m->flex_rgba + 4 * i, pfl->rgba, 4);

    // elasticity
    if (!pfl->stiffness.empty()) {
      mjuu_copyvec(m->flex_stiffness + 21 * elem_adr, pfl->stiffness.data(), pfl->stiffness.size());
    } else {
      mjuu_zerovec(m->flex_stiffness + 21 * elem_adr, 21 * pfl->nelem);
    }
    if (!pfl->bending.empty()) {
      mjuu_copyvec(m->flex_bending + 16 * edge_adr, pfl->bending.data(), pfl->bending.size());
    } else {
      mjuu_zerovec(m->flex_bending + 16 * edge_adr, 16 * pfl->nedge);
    }
    m->flex_damping[i] = (mjtNum)pfl->damping;

    // set fields: mesh-like
    m->flex_dim[i] = pfl->dim;
    m->flex_vertadr[i] = vert_adr;
    m->flex_vertnum[i] = pfl->nvert;
    m->flex_nodeadr[i] = node_adr;
    m->flex_nodenum[i] = pfl->nnode;
    m->flex_edgeadr[i] = edge_adr;
    m->flex_edgenum[i] = pfl->nedge;
    m->flex_elemadr[i] = elem_adr;
    m->flex_elemdataadr[i] = elemdata_adr;
    m->flex_elemedgeadr[i] = elemedge_adr;
    m->flex_shellnum[i] = (int)pfl->shell.size()/pfl->dim;
    m->flex_shelldataadr[i] = m->flex_shellnum[i] ? shelldata_adr : -1;
    if (pfl->evpair.empty()) {
      m->flex_evpairadr[i] = -1;
      m->flex_evpairnum[i] = 0;
    } else {
      m->flex_evpairadr[i] = evpair_adr;
      m->flex_evpairnum[i] = (int)pfl->evpair.size()/2;
      memcpy(m->flex_evpair + 2*evpair_adr, pfl->evpair.data(), pfl->evpair.size()*sizeof(int));
    }
    if (pfl->texcoord_.empty()) {
      m->flex_texcoordadr[i] = -1;
      memcpy(m->flex_elemtexcoord + elemdata_adr, pfl->elem_.data(), pfl->elem_.size()*sizeof(int));
    } else {
      m->flex_texcoordadr[i] = texcoord_adr;
      memcpy(m->flex_texcoord + 2*texcoord_adr,
             pfl->texcoord_.data(), pfl->texcoord_.size()*sizeof(float));
      memcpy(m->flex_elemtexcoord + elemdata_adr, pfl->elemtexcoord_.data(),
             pfl->elemtexcoord_.size()*sizeof(int));
    }
    m->flex_elemnum[i] = pfl->nelem;
    memcpy(m->flex_elem + elemdata_adr, pfl->elem_.data(), pfl->elem_.size()*sizeof(int));
    memcpy(m->flex_elemedge + elemedge_adr, pfl->edgeidx_.data(), pfl->edgeidx_.size()*sizeof(int));
    memcpy(m->flex_elemlayer + elem_adr, pfl->elemlayer.data(), pfl->nelem*sizeof(int));
    if (m->flex_shellnum[i]) {
      memcpy(m->flex_shell + shelldata_adr, pfl->shell.data(), pfl->shell.size()*sizeof(int));
    }
    m->flex_edgestiffness[i] = (mjtNum)pfl->edgestiffness;
    m->flex_edgedamping[i] = (mjtNum)pfl->edgedamping;
    m->flex_rigid[i] = pfl->rigid;
    m->flex_centered[i] = pfl->centered;
    m->flex_internal[i] = pfl->internal;
    m->flex_flatskin[i] = pfl->flatskin;
    m->flex_selfcollide[i] = pfl->selfcollide;
    m->flex_activelayers[i] = pfl->activelayers;
    m->flex_bvhnum[i] = pfl->tree.Nbvh();
    m->flex_bvhadr[i] = pfl->tree.Nbvh() ? bvh_adr : -1;

    // find equality constraint referencing this flex
    m->flex_edgeequality[i] = 0;
    for (int k=0; k < (int)equalities_.size(); k++) {
      if (equalities_[k]->type == mjEQ_FLEX && equalities_[k]->name1_ == pfl->name) {
        m->flex_edgeequality[i] = 1;
        break;
      }
    }

    // copy bvh data (flex aabb computed dynamically in mjData)
    if (pfl->tree.Nbvh()) {
      memcpy(m->bvh_child + 2*bvh_adr, pfl->tree.Child().data(), 2*pfl->tree.Nbvh()*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pfl->tree.Level().data(), pfl->tree.Nbvh()*sizeof(int));
      for (int i=0; i < pfl->tree.Nbvh(); i++) {
        m->bvh_nodeid[i+ bvh_adr] = pfl->tree.Nodeidptr(i) ? *(pfl->tree.Nodeidptr(i)) : -1;
      }
    }

    // copy or set vert
    if (pfl->centered && !pfl->interpolated) {
      mjuu_zerovec(m->flex_vert + 3*vert_adr, 3*pfl->nvert);
    }
    else {
      mjuu_copyvec(m->flex_vert + 3*vert_adr, pfl->vert_.data(), 3*pfl->nvert);
    }

    // copy or set node
    if (pfl->centered && pfl->interpolated) {
      mjuu_zerovec(m->flex_node + 3*node_adr, 3*pfl->nnode);
    }
    else if (pfl->interpolated) {
      mjuu_copyvec(m->flex_node + 3*node_adr, pfl->node_.data(), 3*pfl->nnode);
    }

    // copy vert0
    mjuu_copyvec(m->flex_vert0 + 3*vert_adr, pfl->vert0_.data(), 3*pfl->nvert);

    // copy node0
    mjuu_copyvec(m->flex_node0 + 3*node_adr, pfl->node0_.data(), 3*pfl->nnode);

    // copy or set vertbodyid
    if (pfl->rigid) {
      for (int k=0; k < pfl->nvert; k++) {
        m->flex_vertbodyid[vert_adr + k] = pfl->vertbodyid[0];
      }
    }
    else {
      memcpy(m->flex_vertbodyid + vert_adr, pfl->vertbodyid.data(), pfl->nvert*sizeof(int));
    }

    // copy or set nodebodyid
    if (pfl->rigid) {
      for (int k=0; k < pfl->nnode; k++) {
        m->flex_nodebodyid[node_adr + k] = pfl->nodebodyid[0];
      }
    } else {
      memcpy(m->flex_nodebodyid + node_adr, pfl->nodebodyid.data(), pfl->nnode*sizeof(int));
    }

    // set interpolation type, only two types for now
    m->flex_interp[i] = pfl->interpolated;

    // convert edge pairs to int array, set edge rigid
    for (int k=0; k < pfl->nedge; k++) {
      m->flex_edge[2*(edge_adr+k)] = pfl->edge[k].first;
      m->flex_edge[2*(edge_adr+k)+1] = pfl->edge[k].second;
      if (pfl->dim == 2 && (pfl->elastic2d == 1 || pfl->elastic2d == 3)) {
        m->flex_edgeflap[2*(edge_adr+k)+0] = pfl->flaps[k].vertices[2];
        m->flex_edgeflap[2*(edge_adr+k)+1] = pfl->flaps[k].vertices[3];
      } else {
        m->flex_edgeflap[2*(edge_adr+k)+0] = -1;
        m->flex_edgeflap[2*(edge_adr+k)+1] = -1;
      }

      if (pfl->rigid) {
        m->flexedge_rigid[edge_adr+k] = 1;
      } else if (!pfl->interpolated) {
        // check if vertex body weldids are the same
        // unsupported by trilinear interpolation
        int b1 = pfl->vertbodyid[pfl->edge[k].first];
        int b2 = pfl->vertbodyid[pfl->edge[k].second];
        m->flexedge_rigid[edge_adr+k] = (bodies_[b1]->weldid == bodies_[b2]->weldid);
      }
    }

    // advance counters
    vert_adr += pfl->nvert;
    node_adr += pfl->nnode;
    edge_adr += pfl->nedge;
    elem_adr += pfl->nelem;
    elemdata_adr += (pfl->dim+1) * pfl->nelem;
    elemedge_adr += (pfl->kNumEdges[pfl->dim-1]) * pfl->nelem;
    shelldata_adr += (int)pfl->shell.size();
    evpair_adr += (int)pfl->evpair.size()/2;
    texcoord_adr += (int)pfl->texcoord_.size()/2;
    bvh_adr += pfl->tree.Nbvh();
  }

  // skins
  vert_adr = 0;
  face_adr = 0;
  texcoord_adr = 0;
  bone_adr = 0;
  bonevert_adr = 0;
  for (int i=0; i < nskin; i++) {
    // get pointer
    mjCSkin* psk = skins_[i];

    // set fields
    m->skin_matid[i] = psk->matid;
    m->skin_group[i] = psk->group;
    mjuu_copyvec(m->skin_rgba+4*i, psk->rgba, 4);
    m->skin_inflate[i] = psk->inflate;
    m->skin_vertadr[i] = vert_adr;
    m->skin_vertnum[i] = psk->get_vert().size()/3;
    m->skin_texcoordadr[i] = (!psk->get_texcoord().empty() ? texcoord_adr : -1);
    m->skin_faceadr[i] = face_adr;
    m->skin_facenum[i] = psk->get_face().size()/3;
    m->skin_boneadr[i] = bone_adr;
    m->skin_bonenum[i] = psk->bodyid.size();

    // copy mesh data
    memcpy(m->skin_vert + 3*vert_adr, psk->get_vert().data(), psk->get_vert().size()*sizeof(float));
    if (!psk->get_texcoord().empty())
      memcpy(m->skin_texcoord + 2*texcoord_adr, psk->get_texcoord().data(),
             psk->get_texcoord().size()*sizeof(float));
    memcpy(m->skin_face + 3*face_adr, psk->get_face().data(), psk->get_face().size()*sizeof(int));

    // copy bind poses and body ids
    memcpy(m->skin_bonebindpos+3*bone_adr, psk->get_bindpos().data(),
           psk->get_bindpos().size()*sizeof(float));
    memcpy(m->skin_bonebindquat+4*bone_adr, psk->get_bindquat().data(),
           psk->get_bindquat().size()*sizeof(float));
    memcpy(m->skin_bonebodyid+bone_adr, psk->bodyid.data(),
           psk->bodyid.size()*sizeof(int));

    // copy per-bone vertex data, advance vertex counter
    for (int j=0; j < m->skin_bonenum[i]; j++) {
      // set fields
      m->skin_bonevertadr[bone_adr+j] = bonevert_adr;
      m->skin_bonevertnum[bone_adr+j] = (int)psk->get_vertid()[j].size();

      // copy data
      memcpy(m->skin_bonevertid+bonevert_adr, psk->get_vertid()[j].data(),
             psk->get_vertid()[j].size()*sizeof(int));
      memcpy(m->skin_bonevertweight+bonevert_adr, psk->get_vertweight()[j].data(),
             psk->get_vertid()[j].size()*sizeof(float));

      // advance counter
      bonevert_adr += m->skin_bonevertnum[bone_adr+j];
    }

    // advance mesh and bone counters
    vert_adr += m->skin_vertnum[i];
    texcoord_adr += psk->get_texcoord().size()/2;
    face_adr += m->skin_facenum[i];
    bone_adr += m->skin_bonenum[i];
  }

  // hfields
  data_adr = 0;
  for (int i=0; i < nhfield; i++) {
    // get pointer
    mjCHField* phf = hfields_[i];

    // set fields
    mjuu_copyvec(m->hfield_size+4*i, phf->size, 4);
    m->hfield_nrow[i] = phf->nrow;
    m->hfield_ncol[i] = phf->ncol;
    m->hfield_adr[i] = data_adr;

    // copy elevation data
    memcpy(m->hfield_data + data_adr, phf->data.data(), phf->nrow*phf->ncol*sizeof(float));

    // advance counter
    data_adr += phf->nrow*phf->ncol;
  }

  // textures
  data_adr = 0;
  for (int i=0; i < ntex; i++) {
    // get pointer
    mjCTexture* ptex = textures_[i];

    // set fields
    m->tex_type[i] = ptex->type;
    m->tex_colorspace[i] = ptex->colorspace;
    m->tex_height[i] = ptex->height;
    m->tex_width[i] = ptex->width;
    m->tex_nchannel[i] = ptex->nchannel;
    m->tex_adr[i] = data_adr;

    // copy rgb data
    memcpy(m->tex_data + data_adr, ptex->data_.data(),
           ptex->nchannel * ptex->width * ptex->height);

    // advance counter
    data_adr += ptex->nchannel * ptex->width * ptex->height;
  }

  // materials
  for (int i=0; i < nmat; i++) {
    // get pointer
    mjCMaterial* pmat = materials_[i];

    // set fields
    for (int j=0; j < mjNTEXROLE; j++) {
      m->mat_texid[mjNTEXROLE*i+j] = pmat->texid[j];
    }
    m->mat_texuniform[i] = pmat->texuniform;
    mjuu_copyvec(m->mat_texrepeat+2*i, pmat->texrepeat, 2);
    m->mat_emission[i] = pmat->emission;
    m->mat_specular[i] = pmat->specular;
    m->mat_shininess[i] = pmat->shininess;
    m->mat_reflectance[i] = pmat->reflectance;
    m->mat_metallic[i] = pmat->metallic;
    m->mat_roughness[i] = pmat->roughness;
    mjuu_copyvec(m->mat_rgba+4*i, pmat->rgba, 4);
  }

  // geom pairs to include
  for (int i=0; i < npair; i++) {
    m->pair_dim[i] = pairs_[i]->condim;
    m->pair_geom1[i] = pairs_[i]->geom1->id;
    m->pair_geom2[i] = pairs_[i]->geom2->id;
    m->pair_signature[i] = pairs_[i]->signature;
    mjuu_copyvec(m->pair_solref+mjNREF*i, pairs_[i]->solref, mjNREF);
    mjuu_copyvec(m->pair_solreffriction+mjNREF*i, pairs_[i]->solreffriction, mjNREF);
    mjuu_copyvec(m->pair_solimp+mjNIMP*i, pairs_[i]->solimp, mjNIMP);
    m->pair_margin[i] = (mjtNum)pairs_[i]->margin;
    m->pair_gap[i] = (mjtNum)pairs_[i]->gap;
    mjuu_copyvec(m->pair_friction+5*i, pairs_[i]->friction, 5);
  }

  // body pairs to exclude
  for (int i=0; i < nexclude; i++) {
    m->exclude_signature[i] = excludes_[i]->signature;
  }

  // equality constraints
  for (int i=0; i < neq; i++) {
    // get pointer
    mjCEquality* peq = equalities_[i];

    // set fields
    m->eq_type[i] = peq->type;
    m->eq_obj1id[i] = peq->obj1id;
    m->eq_obj2id[i] = peq->obj2id;
    m->eq_objtype[i] = peq->objtype;
    m->eq_active0[i] = peq->active;
    mjuu_copyvec(m->eq_solref+mjNREF*i, peq->solref, mjNREF);
    mjuu_copyvec(m->eq_solimp+mjNIMP*i, peq->solimp, mjNIMP);
    mjuu_copyvec(m->eq_data+mjNEQDATA*i, peq->data, mjNEQDATA);
  }

  // tendons and wraps
  adr = 0;
  for (int i=0; i < ntendon; i++) {
    // get pointer
    mjCTendon* pte = tendons_[i];

    // set fields
    m->tendon_adr[i] = adr;
    m->tendon_num[i] = (int)pte->path.size();
    m->tendon_matid[i] = pte->matid;
    m->tendon_group[i] = pte->group;
    m->tendon_limited[i] = (mjtByte)pte->is_limited();
    m->tendon_actfrclimited[i] = (mjtByte)pte->is_actfrclimited();
    m->tendon_width[i] = (mjtNum)pte->width;
    mjuu_copyvec(m->tendon_solref_lim+mjNREF*i, pte->solref_limit, mjNREF);
    mjuu_copyvec(m->tendon_solimp_lim+mjNIMP*i, pte->solimp_limit, mjNIMP);
    mjuu_copyvec(m->tendon_solref_fri+mjNREF*i, pte->solref_friction, mjNREF);
    mjuu_copyvec(m->tendon_solimp_fri+mjNIMP*i, pte->solimp_friction, mjNIMP);
    m->tendon_range[2*i] = (mjtNum)pte->range[0];
    m->tendon_range[2*i+1] = (mjtNum)pte->range[1];
    m->tendon_actfrcrange[2*i] = (mjtNum)pte->actfrcrange[0];
    m->tendon_actfrcrange[2*i+1] = (mjtNum)pte->actfrcrange[1];
    m->tendon_margin[i] = (mjtNum)pte->margin;
    m->tendon_stiffness[i] = (mjtNum)pte->stiffness;
    m->tendon_damping[i] = (mjtNum)pte->damping;
    m->tendon_armature[i] = (mjtNum)pte->armature;
    m->tendon_frictionloss[i] = (mjtNum)pte->frictionloss;
    m->tendon_lengthspring[2*i] = (mjtNum)pte->springlength[0];
    m->tendon_lengthspring[2*i+1] = (mjtNum)pte->springlength[1];
    mjuu_copyvec(m->tendon_user+nuser_tendon*i, pte->get_userdata().data(), nuser_tendon);
    mjuu_copyvec(m->tendon_rgba+4*i, pte->rgba, 4);

    // set wraps
    for (int j=0; j < (int)pte->path.size(); j++) {
      m->wrap_type[adr+j] = pte->path[j]->type;
      m->wrap_objid[adr+j] = pte->path[j]->obj ? pte->path[j]->obj->id : -1;
      m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->prm;
      if (pte->path[j]->type == mjWRAP_SPHERE || pte->path[j]->type == mjWRAP_CYLINDER) {
        m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->sideid;
      }
    }

    // advance address counter
    adr += (int)pte->path.size();
  }

  // actuators
  adr = 0;
  for (int i=0; i < nu; i++) {
    // get pointer
    mjCActuator* pac = actuators_[i];

    // set fields
    m->actuator_trntype[i] = pac->trntype;
    m->actuator_dyntype[i] = pac->dyntype;
    m->actuator_gaintype[i] = pac->gaintype;
    m->actuator_biastype[i] = pac->biastype;
    m->actuator_trnid[2*i] = pac->trnid[0];
    m->actuator_trnid[2*i+1] = pac->trnid[1];
    m->actuator_actnum[i] = pac->actdim;
    m->actuator_actadr[i] = m->actuator_actnum[i] ? adr : -1;
    pac->actadr_ = m->actuator_actadr[i];
    pac->actdim_ = m->actuator_actnum[i];
    adr += m->actuator_actnum[i];
    m->actuator_group[i] = pac->group;
    m->actuator_ctrllimited[i] = (mjtByte)pac->is_ctrllimited();
    m->actuator_forcelimited[i] = (mjtByte)pac->is_forcelimited();
    m->actuator_actlimited[i] = (mjtByte)pac->is_actlimited();
    m->actuator_actearly[i] = pac->actearly;
    m->actuator_cranklength[i] = (mjtNum)pac->cranklength;
    mjuu_copyvec(m->actuator_gear + 6*i, pac->gear, 6);
    mjuu_copyvec(m->actuator_dynprm + mjNDYN*i, pac->dynprm, mjNDYN);
    mjuu_copyvec(m->actuator_gainprm + mjNGAIN*i, pac->gainprm, mjNGAIN);
    mjuu_copyvec(m->actuator_biasprm + mjNBIAS*i, pac->biasprm, mjNBIAS);
    mjuu_copyvec(m->actuator_ctrlrange + 2*i, pac->ctrlrange, 2);
    mjuu_copyvec(m->actuator_forcerange + 2*i, pac->forcerange, 2);
    mjuu_copyvec(m->actuator_actrange + 2*i, pac->actrange, 2);
    mjuu_copyvec(m->actuator_lengthrange + 2*i, pac->lengthrange, 2);
    mjuu_copyvec(m->actuator_user+nuser_actuator*i, pac->get_userdata().data(), nuser_actuator);
  }

  // sensors
  adr = 0;
  for (int i=0; i < nsensor; i++) {
    // get pointer
    mjCSensor* psen = sensors_[i];

    // set fields
    m->sensor_type[i] = psen->type;
    m->sensor_datatype[i] = psen->datatype;
    m->sensor_needstage[i] = psen->needstage;
    m->sensor_objtype[i] = psen->objtype;
    m->sensor_objid[i] = psen->obj ? psen->obj->id : -1;
    m->sensor_reftype[i] = psen->reftype;
    m->sensor_refid[i] = psen->refid;
    m->sensor_dim[i] = psen->dim;
    m->sensor_cutoff[i] = (mjtNum)psen->cutoff;
    m->sensor_noise[i] = (mjtNum)psen->noise;
    mjuu_copyvec(m->sensor_user+nuser_sensor*i, psen->get_userdata().data(), nuser_sensor);

    // calculate address and advance
    m->sensor_adr[i] = adr;
    adr += psen->dim;
  }

  // numeric fields
  adr = 0;
  for (int i=0; i < nnumeric; i++) {
    // get pointer
    mjCNumeric* pcu = numerics_[i];

    // set fields
    m->numeric_adr[i] = adr;
    m->numeric_size[i] = pcu->size;
    for (int j=0; j < (int)pcu->data_.size(); j++) {
      m->numeric_data[adr+j] = (mjtNum)pcu->data_[j];
    }
    for (int j=(int)pcu->data_.size(); j < (int)pcu->size; j++) {
      m->numeric_data[adr+j] = 0;
    }

    // advance address counter
    adr += m->numeric_size[i];
  }

  // text fields
  adr = 0;
  for (int i=0; i < ntext; i++) {
    // get pointer
    mjCText* pte = texts_[i];

    // set fields
    m->text_adr[i] = adr;
    m->text_size[i] = (int)pte->data_.size()+1;
    mju_strncpy(m->text_data + adr, pte->data_.c_str(), m->ntextdata - adr);

    // advance address counter
    adr += m->text_size[i];
  }

  // tuple fields
  adr = 0;
  for (int i=0; i < ntuple; i++) {
    // get pointer
    mjCTuple* ptu = tuples_[i];

    // set fields
    m->tuple_adr[i] = adr;
    m->tuple_size[i] = (int)ptu->objtype_.size();
    for (int j=0; j < m->tuple_size[i]; j++) {
      m->tuple_objtype[adr+j] = (int)ptu->objtype_[j];
      m->tuple_objid[adr+j] = ptu->obj[j]->id;
      m->tuple_objprm[adr+j] = (mjtNum)ptu->objprm_[j];
    }

    // advance address counter
    adr += m->tuple_size[i];
  }

  // copy keyframe data
  for (int i=0; i < nkey; i++) {
    // copy data
    m->key_time[i] = (mjtNum)keys_[i]->time;
    mjuu_copyvec(m->key_qpos+i*nq, keys_[i]->qpos_.data(), nq);
    mjuu_copyvec(m->key_qvel+i*nv, keys_[i]->qvel_.data(), nv);
    if (na) {
      mjuu_copyvec(m->key_act+i*na, keys_[i]->act_.data(), na);
    }
    if (nmocap) {
      mjuu_copyvec(m->key_mpos + i*3*nmocap, keys_[i]->mpos_.data(), 3*nmocap);
      mjuu_copyvec(m->key_mquat + i*4*nmocap, keys_[i]->mquat_.data(), 4*nmocap);
    }

    // normalize quaternions in m->key_qpos
    for (int j=0; j < m->njnt; j++) {
      if (m->jnt_type[j] == mjJNT_BALL || m->jnt_type[j] == mjJNT_FREE) {
        mjuu_normvec(m->key_qpos+i*nq+m->jnt_qposadr[j]+3*(m->jnt_type[j] == mjJNT_FREE), 4);
      }
    }

    // normalize quaternions in m->key_mquat
    for (int j=0; j < nmocap; j++) {
      mjuu_normvec(m->key_mquat+i*4*nmocap+4*j, 4);
    }

    mjuu_copyvec(m->key_ctrl+i*nu, keys_[i]->ctrl_.data(), nu);
  }

  // save qpos0 in user model (to recognize changed key_qpos in write)
  qpos0.resize(nq);
  body_pos0.resize(3*nbody);
  body_quat0.resize(4*nbody);
  mjuu_copyvec(qpos0.data(), m->qpos0, nq);
  mjuu_copyvec(body_pos0.data(), m->body_pos, 3*nbody);
  mjuu_copyvec(body_quat0.data(), m->body_quat, 4*nbody);
}



// finalize simple bodies/dofs including tendon information
void mjCModel::FinalizeSimple(mjModel* m) {
  // demote bodies affected by inertia-bearing tendon to non-simple
  for (int i=0; i < ntendon; i++) {
    if (m->tendon_armature[i] == 0) {
      continue;
    }
    int adr = m->tendon_adr[i];
    int num = m->tendon_num[i];
    for (int j=adr; j < adr+num; j++) {
      int objid = m->wrap_objid[j];
      if (m->wrap_type[j] == mjWRAP_SITE) {
        m->body_simple[m->site_bodyid[objid]] = 0;
      }
      if (m->wrap_type[j] == mjWRAP_CYLINDER || m->wrap_type[j] == mjWRAP_SPHERE) {
        m->body_simple[m->geom_bodyid[objid]] = 0;
      }
    }
  }

  // set dof_simplenum
  int count = 0;
  for (int i=nv-1; i >= 0; i--) {
    if (m->body_simple[m->dof_bodyid[i]]) {
      count++;    // increment counter
    } else {
      count = 0;  // reset
    }
    m->dof_simplenum[i] = count;
  }

  // compute nC
  int nOD = 0;  // number of off-diagonal (non-simple) parent dofs
  for (int i=0; i < nv; i++) {
    // count ancestor (off-diagonal) dofs
    if (!m->dof_simplenum[i]) {
      int j = i;
      while (j >= 0) {
        if (j != i) nOD++;
        j = m->dof_parentid[j];
      }
    }
  }
  m->nC = nC = nOD + nv;
}



// save the current state
template <class T>
void mjCModel::SaveState(const std::string& state_name, const T* qpos, const T* qvel, const T* act,
                         const T* ctrl, const T* mpos, const T* mquat) {
  for (auto joint : joints_) {
    if (joint->qposadr_ < -1 || joint->dofadr_ < -1) {
      throw mjCError(nullptr, "SaveState: joint %s has invalid address", joint->name.c_str());
    }
    if (qpos && joint->qposadr_ != -1) {
      mjuu_copyvec(joint->qpos(state_name), qpos + joint->qposadr_, joint->nq());
    }
    if (qvel && joint->dofadr_ != -1) {
      mjuu_copyvec(joint->qvel(state_name), qvel + joint->dofadr_, joint->nv());
    }
  }

  for (unsigned int i=0; i < actuators_.size(); i++) {
    auto actuator = actuators_[i];
    if (actuator->actadr_ != -1 && actuator->actdim_ != -1 && act) {
      actuator->act(state_name).assign(actuator->actdim_, 0);
      mjuu_copyvec(actuator->act(state_name).data(), act + actuator->actadr_, actuator->actdim_);
    }
    if (ctrl) {
      actuator->ctrl(state_name) = ctrl[i];
    }
  }

  for (auto body : bodies_) {
    if (!body->spec.mocap || body->mocapid == -1) {
      continue;
    }
    if (mpos) {
      mjuu_copyvec(body->mpos(state_name), mpos + 3*body->mocapid, 3);
    }
    if (mquat) {
      mjuu_copyvec(body->mquat(state_name), mquat + 4*body->mocapid, 4);
    }
  }
}



// clear existing data
void mjCModel::MakeData(const mjModel* m, mjData** dest) {
  mj_makeRawData(dest, m);
  mjData* d = *dest;
  if (d) {
    mj_initPlugin(m, d);
    mj_resetData(m, d);
  }
}



// restore the previous state
template <class T>
void mjCModel::RestoreState(const std::string& state_name, const mjtNum* pos0,
                            const mjtNum* mpos0, const mjtNum* mquat0, T* qpos,
                            T* qvel, T* act, T* ctrl, T* mpos, T* mquat) {
  for (auto joint : joints_) {
    if (qpos) {
      if (mjuu_defined(joint->qpos(state_name)[0])) {
        mjuu_copyvec(qpos + joint->qposadr_, joint->qpos(state_name), joint->nq());
      } else {
        mjuu_copyvec(qpos + joint->qposadr_, pos0 + joint->qposadr_, joint->nq());
      }
    }
    if (mjuu_defined(joint->qvel(state_name)[0]) && qvel) {
      mjuu_copyvec(qvel + joint->dofadr_, joint->qvel(state_name), joint->nv());
    }
  }

  // restore act
  for (unsigned int i=0; i < actuators_.size(); i++) {
    auto actuator = actuators_[i];
    if (!actuator->act(state_name).empty() && mjuu_defined(actuator->act(state_name)[0]) && act) {
      mjuu_copyvec(act + actuator->actadr_, actuator->act(state_name).data(), actuator->actdim_);
    }
    if (ctrl) {
      ctrl[i] = mjuu_defined(actuator->ctrl(state_name)) ? actuator->ctrl(state_name) : 0;
    }
  }

  for (unsigned int i=0; i < bodies_.size(); i++) {
    auto body = bodies_[i];
    if (!body->spec.mocap) {
      continue;
    }
    if (mpos) {
      if (mjuu_defined(body->mpos(state_name)[0])) {
        mjuu_copyvec(mpos + 3*body->mocapid, body->mpos(state_name), 3);
      } else {
        mjuu_copyvec(mpos + 3*body->mocapid, mpos0 + 3*i, 3);
      }
    }
    if (mquat) {
      if (mjuu_defined(body->mquat(state_name)[0])) {
        mjuu_copyvec(mquat + 4*body->mocapid, body->mquat(state_name), 4);
      } else {
        mjuu_copyvec(mquat + 4*body->mocapid, mquat0 + 4*i, 4);
      }
    }
  }
}

// force explicit instantiations
template void mjCModel::SaveState<mjtNum>(
  const std::string& name, const mjtNum* qpos, const mjtNum* qvel, const mjtNum* act,
  const mjtNum* ctrl, const mjtNum* mpos, const mjtNum* mquat);

template void mjCModel::RestoreState<mjtNum>(
  const std::string& name, const mjtNum* qpos0, const mjtNum* mpos0, const mjtNum* mquat0,
  mjtNum* qpos, mjtNum* qvel, mjtNum* act, mjtNum* ctrl, mjtNum* mpos, mjtNum* mquat);



// resolve keyframe references
void mjCModel::StoreKeyframes(mjCModel* dest) {
  if (this != dest && !key_pending_.empty()) {
    mju_warning(
      "Child model has pending keyframes. They will not be namespaced correctly. "
      "To prevent this, compile the child model before attaching it again.");
  }

  // do not change compilation quantities in case the user wants to recompile preserving the state
  if (!compiled) {
    SaveDofOffsets(/*computesize=*/true);
    ComputeReference();
  }

  // save keyframe info and resize keyframes
  for (auto& key : keys_) {
    mjKeyInfo info;
    info.name = prefix + key->name + suffix;
    info.time = key->spec.time;
    info.qpos = !key->spec_qpos_.empty();
    info.qvel = !key->spec_qvel_.empty();
    info.act = !key->spec_act_.empty();
    info.ctrl = !key->spec_ctrl_.empty();
    info.mpos = !key->spec_mpos_.empty();
    info.mquat = !key->spec_mquat_.empty();
    dest->key_pending_.push_back(info);
    if (!key->spec_qpos_.empty() && key->spec_qpos_.size() != nq) {
      throw mjCError(nullptr, "Keyframe '%s' has invalid qpos size, got %d, should be %d",
                     key->name.c_str(), key->spec_qpos_.size(), nq);
    }
    if (!key->spec_qvel_.empty() && key->spec_qvel_.size() != nv) {
      throw mjCError(nullptr, "Keyframe %s has invalid qvel size, got %d, should be %d",
                     key->name.c_str(), key->spec_qvel_.size(), nv);
    }
    if (!key->spec_act_.empty() && key->spec_act_.size() != na) {
      throw mjCError(nullptr, "Keyframe %s has invalid act size, got %d, should be %d",
                     key->name.c_str(), key->spec_act_.size(), na);
    }
    if (!key->spec_ctrl_.empty() && key->spec_ctrl_.size() != nu) {
      throw mjCError(nullptr, "Keyframe %s has invalid ctrl size, got %d, should be %d",
                     key->name.c_str(), key->spec_ctrl_.size(), nu);
    }
    if (!key->spec_mpos_.empty() && key->spec_mpos_.size() != 3*nmocap) {
      throw mjCError(nullptr, "Keyframe %s has invalid mpos size, got %d, should be %d",
                     key->name.c_str(), key->spec_mpos_.size(), 3*nmocap);
    }
    if (!key->spec_mquat_.empty() && key->spec_mquat_.size() != 4*nmocap) {
      throw mjCError(nullptr, "Keyframe %s has invalid mquat size, got %d, should be %d",
                     key->name.c_str(), key->spec_mquat_.size(), 4*nmocap);
    }
    SaveState(info.name, key->spec_qpos_.data(), key->spec_qvel_.data(),
              key->spec_act_.data(), key->spec_ctrl_.data(),
              key->spec_mpos_.data(), key->spec_mquat_.data());
  }

  if (!compiled) {
    nq = nv = na = nu = nmocap = 0;
  }
}



//------------------------------- FUSE STATIC ------------------------------------------------------

template <class T>
static void makelistid(std::vector<T*>& dest, std::vector<T*>& source) {
  for (int i=0; i < source.size(); i++) {
    source[i]->id = (int)dest.size();
    dest.push_back(source[i]);
  }
}

// change frame to parent body
static void changeframe(double childpos[3], double childquat[4],
                        const double bodypos[3], const double bodyquat[4]) {
  double pos[3], quat[4];
  mjuu_copyvec(pos, bodypos, 3);
  mjuu_copyvec(quat, bodyquat, 4);
  mjuu_frameaccum(pos, quat, childpos, childquat);
  mjuu_copyvec(childpos, pos, 3);
  mjuu_copyvec(childquat, quat, 4);
}



// reindex elements during fuse
void mjCModel::FuseReindex(mjCBody* body) {
  // set parentid and weldid of children
  for (int i=0; i < body->bodies.size(); i++) {
    body->bodies[i]->parent = body;
    body->bodies[i]->weldid = (!body->bodies[i]->joints.empty() ?
                               body->bodies[i]->id : body->weldid);
  }

  makelistid(joints_, body->joints);
  makelistid(geoms_, body->geoms);
  makelistid(sites_, body->sites);

  // process children recursively
  for (int i=0; i < body->bodies.size(); i++) {
    FuseReindex(body->bodies[i]);
  }
}



template <class T>
void mjCModel::ReassignChild(std::vector<T*>& dest, std::vector<T*>& list,
                             mjCBody* parent, mjCBody* body) {
  for (int j=0; j < list.size(); j++) {
    // assign
    list[j]->body = parent;
    dest.push_back(list[j]);

    // change frame
    changeframe(list[j]->pos, list[j]->quat, body->pos, body->quat);
  }
  list.clear();
}



template <class T>
void mjCModel::ResolveReferences(std::vector<T*>& list, mjCBody* body) {
  for (auto& item : list) {
    item->CopyFromSpec();
    item->ResolveReferences(this);
  }
}



template <>
void mjCModel::ResolveReferences(std::vector<mjCSensor*>& list, mjCBody* body) {
  for (auto& item : list) {
    item->CopyFromSpec();
    item->ResolveReferences(this);
  }
  for (mjCSensor* sensor : list) {
    if (sensor->objtype == mjOBJ_SITE &&
        (sensor->type == mjSENS_FORCE || sensor->type == mjSENS_TORQUE) &&
        static_cast<mjCSite*>(sensor->obj)->body == body) {
      throw mjCError(sensor, "cannot fuse a body used by a force/torque sensor");
    }
  }
}



// fuse static bodies with their parent
void mjCModel::FuseStatic(void) {
  for (int i=1; i < bodies_.size(); i++) {
    // check if the body can be fused
    if (!bodies_[i]->name.empty()) {
      ids[mjOBJ_BODY].erase(bodies_[i]->name);

      // try to resolve references without the name of this body, if it fails, skip
      try {
        ResolveReferences(cameras_);
        ResolveReferences(lights_);
        ResolveReferences(skins_);
        ResolveReferences(pairs_);
        ResolveReferences(excludes_);
        ResolveReferences(equalities_);
        ResolveReferences(tendons_);
        ResolveReferences(actuators_);
        ResolveReferences(sensors_, bodies_[i]);
        ResolveReferences(tuples_);
      } catch (mjCError err) {
        ids[mjOBJ_BODY].insert({bodies_[i]->name, i});
        continue;
      }

      // put body back the body name in the map
      ids[mjOBJ_BODY].insert({bodies_[i]->name, i});
    }

    // get body and parent
    mjCBody* body = bodies_[i];
    mjCBody* par = body->parent;

    // skip if body has joints or mocap
    if (!body->joints.empty() || body->mocap) {
      continue;
    }

    //------------- add mass and inertia (if parent not world)
    if (body->parent && body->parent->name != "world" && body->mass >= mjMINVAL) {
      par->AccumulateInertia(body);
    }

    //------------- replace body with its children in parent body list

    // change frames of child bodies
    for (int j=0; j < body->bodies.size(); j++)
      changeframe(body->bodies[j]->pos, body->bodies[j]->quat,
                  body->pos, body->quat);

    // find body in parent list, insert children before it
    bool found = false;
    for (auto iter=par->bodies.begin(); iter != par->bodies.end(); iter++) {
      if (*iter == body) {
        par->bodies.insert(iter, body->bodies.begin(), body->bodies.end());
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    // find body in parent list, erase
    found = false;
    for (auto iter=par->bodies.begin(); iter != par->bodies.end(); iter++) {
      if (*iter == body) {
        par->bodies.erase(iter);
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    //------------- assign geoms and sites to parent, change frames

    ReassignChild(par->geoms, body->geoms, par, body);
    ReassignChild(par->sites, body->sites, par, body);

    //------------- remove from global body list, reduce global counts

    // find in global and erase
    found = false;
    for (auto iter=bodies_.begin(); iter != bodies_.end(); iter++) {
      if (*iter == body) {
        bodies_.erase(iter);
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    // reduce counts
    nbody--;
    nnames -= ((int)body->name.length() + 1);

    //------------- re-index bodies, joints, geoms, sites

    // body ids
    for (int j=0; j < bodies_.size(); j++) {
      bodies_[j]->id = j;
    }

    // everything else
    joints_.clear();
    geoms_.clear();
    sites_.clear();
    FuseReindex(bodies_[0]);

    // recompute parent contype, conaffinity, and margin
    par->contype = par->conaffinity = 0;
    par->margin = 0;
    for (const auto& geom : par->geoms) {
      par->contype |= geom->contype;
      par->conaffinity |= geom->conaffinity;
      par->margin = std::max(par->margin, geom->margin);
    }

    // recompute BVH
    int nbvhfuse = body->tree.Nbvh() + par->tree.Nbvh();
    par->ComputeBVH();
    nbvhstatic += par->tree.Nbvh() - nbvhfuse;
    nbvh += par->tree.Nbvh() - nbvhfuse;

    //------------- delete body (without deleting children)

    // remove body name from map
    if (!body->name.empty()) {
      ids[mjOBJ_BODY].erase(body->name);
    }

    // delete allocation
    body->bodies.clear();
    delete body;

    // check index i again (we have a new body at this index)
    i--;
  }

  // remove empty names
  processlist(ids, bodies_, mjOBJ_BODY, true);
}



//------------------------------- COMPILER ---------------------------------------------------------

// signature comparisons
static int comparePair(mjCPair* el1, mjCPair* el2) {
  return el1->GetSignature() < el2->GetSignature();
}
static int compareBodyPair(mjCBodyPair* el1, mjCBodyPair* el2) {
  return el1->GetSignature() < el2->GetSignature();
}


// reassign ids
template <class T>
static void reassignid(vector<T*>& list) {
  for (int i=0; i < (int)list.size(); i++) {
    list[i]->id = i;
  }
}



// set object ids, check for repeated names
void mjCModel::ProcessLists(bool checkrepeat) {
  for (int i = 0; i < mjNOBJECT; i++) {
    if (i != mjOBJ_XBODY && object_lists_[i]) {
      ids[i].clear();
      processlist(ids, *object_lists_[i], (mjtObj) i, checkrepeat);
    }
  }

  // check repeated names in meta elements
  processlist(ids, frames_, mjOBJ_FRAME, checkrepeat);
}



// error handler for low-level engine
constexpr int kErrorBufferSize = 500;
static thread_local std::jmp_buf error_jmp_buf;
static thread_local char errortext[kErrorBufferSize] = "";
static void errorhandler(const char* msg) {
  mju::strcpy_arr(errortext, msg);
  std::longjmp(error_jmp_buf, 1);
}


// warning handler for low-level engine
static thread_local char warningtext[kErrorBufferSize] = "";       // top-level warning buffer
static thread_local std::string* local_warningtext_ptr = nullptr;  // sub-thread warning buffer
static void warninghandler(const char* msg) {
  if (local_warningtext_ptr) {
    *local_warningtext_ptr = msg;
  } else {
    mju::strcpy_arr(warningtext, msg);
  }
}


// compiler
mjModel* mjCModel::Compile(const mjVFS* vfs, mjModel** m) {
  if (compiled) {
    Clear();
  }

  CopyFromSpec();

  // The volatile keyword is necessary to prevent a possible memory leak due to
  // an interaction between longjmp and compiler optimization. Specifically, at
  // the point where the setjmp takes places, these pointers have never been
  // reassigned from their nullptr initialization. Without the volatile keyword,
  // the compiler is free to assume that these pointers remain nullptr when the
  // setjmp returns, and therefore to pass nullptr directly to the
  // mj_deleteModel and mj_deleteData calls in the subsequent catch block,
  // without ever reading the actual pointer values.
  mjModel* volatile model = (m && *m) ? *m : nullptr;
  mjData* volatile data = nullptr;

  // save error and warning handlers
  void (*save_error)(const char*) = _mjPRIVATE__get_tls_error_fn();
  void (*save_warning)(const char*) = _mjPRIVATE__get_tls_warning_fn();

  // install error and warning handlers, clear error and warning
  _mjPRIVATE__set_tls_error_fn(errorhandler);
  _mjPRIVATE__set_tls_warning_fn(warninghandler);

  errInfo = mjCError();
  warningtext[0] = 0;

  try {
    if (attached_) {
      throw mjCError(0, "cannot compile child spec if attached by reference to a parent spec");
    }
    if (setjmp(error_jmp_buf) != 0) {
      // TryCompile resulted in an mju_error which was converted to a longjmp.
      std::string error_msg = errortext;
      // also include the last warning that was issued. this is useful for
      // warnings that came out of plugin implementations.
      if (warningtext[0]) {
        error_msg += "\n";
        error_msg += warningtext;
      }
      throw mjCError(0, "engine error: %s", error_msg.c_str());
    }
    TryCompile(*const_cast<mjModel**>(&model), *const_cast<mjData**>(&data), vfs);
  } catch (mjCError err) {
    // deallocate everything allocated in Compile
    mj_deleteModel(model);
    mj_deleteData(data);
    Clear();

    // save error info
    errInfo = err;

    // restore handler, return 0
    _mjPRIVATE__set_tls_error_fn(save_error);
    _mjPRIVATE__set_tls_warning_fn(save_warning);
    return nullptr;
  }

  // restore error handler, mark as compiled, return mjModel
  _mjPRIVATE__set_tls_error_fn(save_error);
  _mjPRIVATE__set_tls_warning_fn(save_warning);
  compiled = true;
  return model;
}



// mesh compilation function to be used in threads
void CompileMesh(mjCMesh* mesh, const mjVFS* vfs, std::exception_ptr& exception,
                 std::mutex& exception_mutex, std::string* warningtext) {
  // set warning text buffer to this local thread
  local_warningtext_ptr = warningtext;
  auto previous_handler = _mjPRIVATE__get_tls_warning_fn();
  _mjPRIVATE__set_tls_warning_fn(warninghandler);

  // compile the mesh, catch exception to be rethrown later
  try {
    mesh->Compile(vfs);
  } catch (...) {
    std::lock_guard<std::mutex> lock(exception_mutex);
    if (!exception) {
      exception = std::current_exception();
    }
  }

  // restore warning handler to top-level
  _mjPRIVATE__set_tls_warning_fn(previous_handler);
  local_warningtext_ptr = nullptr;
}



// multi-threaded mesh compilation
void mjCModel::CompileMeshes(const mjVFS* vfs) {
  std::vector<std::thread> threads;
  int nmesh = meshes_.size();
  int hardware_threads = std::thread::hardware_concurrency() / 2;
  int maxthread = std::min(nmesh, std::min(kMaxCompilerThreads, hardware_threads));
  int nthread = std::max(1, maxthread);

  threads.reserve(nthread);

  // holds an exception thrown by a worker thread
  std::exception_ptr exception;
  std::mutex except_mutex;

  std::atomic_int next_mesh = 0;
  std::vector<std::string> mesh_warningtext(nmesh);
  for (int i = 0; i < nthread; ++i) {
    threads.emplace_back([&] {
      for (int meshid = next_mesh++; meshid < nmesh; meshid = next_mesh++) {
        auto& mesh = meshes_[meshid];
        CompileMesh(mesh, vfs, exception, except_mutex,
                    &mesh_warningtext[meshid]);
      }
    });
  }

  // join threads
  for (auto& thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }

  // concatenate all warnings from threads, copy into warningtext
  std::string concatenated_warnings;
  bool has_warning = false;
  for (int i = 0; i < nmesh; i++) {
    if (!mesh_warningtext[i].empty()) {
      if (has_warning) {
        concatenated_warnings += "\n";
      }
      concatenated_warnings += mesh_warningtext[i];
      has_warning = true;
    }
  }
  mju::strcpy_arr(warningtext, concatenated_warnings.c_str());

  // if exception was caught, rethrow it
  if (exception) {
    std::rethrow_exception(exception);
  }
}



// compute qpos0
void mjCModel::ComputeReference() {
  int b = 0;
  qpos0.resize(nq);
  body_pos0.resize(3*bodies_.size());
  body_quat0.resize(4*bodies_.size());
  for (auto body : bodies_) {
    mjuu_copyvec(body_pos0.data()+3*b, body->spec.pos, 3);
    mjuu_copyvec(body_quat0.data()+4*b, body->spec.quat, 4);
    for (auto joint : body->joints) {
      switch (joint->type) {
        case mjJNT_FREE:
          mjuu_copyvec(qpos0.data()+joint->qposadr_, body->spec.pos, 3);
          mjuu_copyvec(qpos0.data()+joint->qposadr_+3, body->spec.quat, 4);
          break;

        case mjJNT_BALL:
          mjuu_setvec(qpos0.data()+joint->qposadr_, 1, 0, 0, 0);
          break;

        case mjJNT_SLIDE:
        case mjJNT_HINGE:
          qpos0[joint->qposadr_] = (mjtNum)joint->spec.ref;
          break;

        default:
          throw mjCError(joint, "unknown joint type");
      }
    }
    b++;
  }
}



// resizes a keyframe, filling in missing values
void mjCModel::ResizeKeyframe(mjCKey* key, const mjtNum* qpos0_,
                              const mjtNum* bpos, const mjtNum* bquat) {
  if (!key->spec_qpos_.empty()) {
    int nq0 = key->spec_qpos_.size();
    key->spec_qpos_.resize(nq);
    for (int i=nq0; i < nq; i++) {
      key->spec_qpos_[i] = (double)qpos0_[i];
    }
  }
  if (!key->spec_qvel_.empty()) {
    key->spec_qvel_.resize(nv);
  }
  if (!key->spec_act_.empty()) {
    key->spec_act_.resize(na);
  }
  if (!key->spec_ctrl_.empty()) {
    key->spec_ctrl_.resize(nu);
  }
  if (!key->spec_mpos_.empty()) {
    int nmocap0 = key->spec_mpos_.size() / 3;
    key->spec_mpos_.resize(3*nmocap);
    for (unsigned int j = 0; j < bodies_.size(); j++) {
      if (bodies_[j]->mocapid < nmocap0) {
        continue;
      }
      int i = bodies_[j]->mocapid;
      key->spec_mpos_[3*i+0] = (double)bpos[3*j+0];
      key->spec_mpos_[3*i+1] = (double)bpos[3*j+1];
      key->spec_mpos_[3*i+2] = (double)bpos[3*j+2];
    }
  }
  if (!key->spec_mquat_.empty()) {
    int nmocap0 = key->spec_mquat_.size() / 4;
    key->spec_mquat_.resize(4*nmocap);
    for (unsigned int j = 0; j < bodies_.size(); j++) {
      if (bodies_[j]->mocapid < nmocap0) {
        continue;
      }
      int i = bodies_[j]->mocapid;
      key->spec_mquat_[4*i+0] = (double)bquat[4*j+0];
      key->spec_mquat_[4*i+1] = (double)bquat[4*j+1];
      key->spec_mquat_[4*i+2] = (double)bquat[4*j+2];
      key->spec_mquat_[4*i+3] = (double)bquat[4*j+3];
    }
  }
}

// convert pending keyframes info to actual keyframes
void mjCModel::ResolveKeyframes(const mjModel* m) {
  // store dof offsets in joints and actuators
  SaveDofOffsets();

  // create new keyframes, fill in missing default values
  for (const auto& info : key_pending_) {
    mjCKey* key = (mjCKey*)FindObject(mjOBJ_KEY, info.name);
    key->name = info.name;
    key->spec.time = info.time;
    if (info.qpos) key->spec_qpos_.assign(nq, 0);
    if (info.qvel) key->spec_qvel_.assign(nv, 0);
    if (info.act) key->spec_act_.assign(na, 0);
    if (info.ctrl) key->spec_ctrl_.assign(nu, 0);
    if (info.mpos) key->spec_mpos_.assign(3*nmocap, 0);
    if (info.mquat) key->spec_mquat_.assign(4*nmocap, 0);
    RestoreState(info.name, m->qpos0, m->body_pos, m->body_quat,
                 key->spec_qpos_.data(), key->spec_qvel_.data(),
                 key->spec_act_.data(), key->spec_ctrl_.data(),
                 key->spec_mpos_.data(), key->spec_mquat_.data());
  }

  // the attached keyframes have been copied into the model
  key_pending_.clear();
}



void mjCModel::TryCompile(mjModel*& m, mjData*& d, const mjVFS* vfs) {
  // check if nan test works
  double test = mjNAN;
  if (mjuu_defined(test)) {
    throw mjCError(0, "NaN test does not work for present compiler/options");
  }

  // check for joints in world body
  if (!bodies_[0]->joints.empty()) {
    throw mjCError(0, "joint found in world body");
  }

  // check for too many body+flex
  if (bodies_.size()+flexes_.size() >= 65534) {
    throw mjCError(0, "number of bodies plus flexes must be less than 65534");
  }

  // append directory separator
  if (!meshdir_.empty()) {
    int n = meshdir_.length();
    if (meshdir_[n-1] != '/' && meshdir_[n-1] != '\\') {
      meshdir_ += '/';
    }
  }
  if (!texturedir_.empty()) {
    int n = texturedir_.length();
    if (texturedir_[n-1] != '/' && texturedir_[n-1] != '\\') {
      texturedir_ += '/';
    }
  }

  // add missing keyframes
  for (int i=keys_.size(); i < nkey; i++) {
    AddKey();
  }

  // clear subtreedofs
  for (int i=0; i < bodies_.size(); i++) {
    bodies_[i]->subtreedofs = 0;
  }

  // initialize spec signature (needed if the user changed sensor or joint types)
  spec.element->signature = Signature();

  // fill missing names and check that they are all filled
  for (const auto& asset : meshes_) asset->CopyFromSpec();
  for (const auto& asset : skins_) asset->CopyFromSpec();
  for (const auto& asset : hfields_) asset->CopyFromSpec();
  for (const auto& asset : textures_) asset->CopyFromSpec();
  CheckEmptyNames();

  // create pending keyframes
  for (const auto& info : key_pending_) {
    mjCKey* key = AddKey();
    key->name = info.name;
  }

  // set object ids, check for repeated names
  ProcessLists();

  // delete visual assets
  if (compiler.discardvisual) {
    DeleteAll(materials_);
    DeleteTexcoord(flexes_);
    DeleteTexcoord(meshes_);
    DeleteAll(textures_);
  }

  // map names to asset references
  IndexAssets(/*discard=*/false);

  // mark meshes that need convex hull
  for (int i=0; i < geoms_.size(); i++) {
    if (geoms_[i]->mesh &&
        (geoms_[i]->spec.type == mjGEOM_MESH || geoms_[i]->spec.type == mjGEOM_SDF) &&
        (geoms_[i]->spec.contype || geoms_[i]->spec.conaffinity ||
         geoms_[i]->mesh->spec.inertia == mjMESH_INERTIA_CONVEX)) {
      geoms_[i]->mesh->SetNeedHull(true);
    }
  }

  // automatically set nuser fields
  SetNuser();

  // compile meshes (needed for geom compilation)
  if (!compiler.usethread && meshes_.size() > 1) {
    // multi-threaded mesh compile
    CompileMeshes(vfs);
  } else {
    // single-threaded mesh compile
    for (int i=0; i < meshes_.size(); i++) {
      meshes_[i]->Compile(vfs);
    }
  }

  // compile objects in kinematic tree
  for (int i=0; i < bodies_.size(); i++) {
    bodies_[i]->Compile();  // also compiles joints, geoms, sites, cameras, lights, frames
  }

  // fuse static if enabled
  if (compiler.fusestatic) {
    FuseStatic();
    for (int i=0; i < lights_.size(); i++) {
      lights_[i]->Compile();
    }
    for (int i=0; i < cameras_.size(); i++) {
      cameras_[i]->Compile();
    }
  }

  // compile all other objects except for keyframes
  for (auto flex : flexes_) flex->Compile(vfs);
  for (auto skin : skins_) skin->Compile(vfs);
  for (auto hfield : hfields_) hfield->Compile(vfs);
  for (auto texture : textures_) texture->Compile(vfs);
  for (auto material : materials_) material->Compile();
  for (auto pair : pairs_) pair->Compile();
  for (auto exclude : excludes_) exclude->Compile();
  for (auto equality : equalities_) equality->Compile();
  for (auto tendon : tendons_) tendon->Compile();
  for (auto actuator : actuators_) actuator->Compile();
  for (auto sensor : sensors_) sensor->Compile();
  for (auto numeric : numerics_) numeric->Compile();
  for (auto text : texts_) text->Compile();
  for (auto tuple : tuples_) tuple->Compile();
  for (auto plugin : plugins_) plugin->Compile();

  // compile def: to enforce userdata length for writer
  for (mjCDef* def : defaults_) {
    def->Compile(this);
  }

  // sort pair, exclude in increasing signature order; reassign ids
  std::stable_sort(pairs_.begin(), pairs_.end(), comparePair);
  std::stable_sort(excludes_.begin(), excludes_.end(), compareBodyPair);
  reassignid(pairs_);
  reassignid(excludes_);

  // resolve asset references, compute sizes
  IndexAssets(compiler.discardvisual);
  SetSizes();

  // set nmocap and body.mocapid
  for (mjCBody* body : bodies_) {
    if (body->mocap) {
      body->mocapid = nmocap;
      nmocap++;
    } else {
      body->mocapid = -1;
    }
  }

  // check mass and inertia of moving bodies
  if (bodies_.size() > 1) {
    // we ignore the first body as it is the world body
    if (!CheckBodiesMassInertia(std::vector<mjCBody*>(bodies_.begin()+1, bodies_.end()))) {
      throw mjCError(0, "mass and inertia of moving bodies must be larger than mjMINVAL");
    }
  }

  // create low-level model
  mj_makeModel(&m,
               nq, nv, nu, na, nbody, nbvh, nbvhstatic, nbvhdynamic, njnt, ngeom, nsite,
               ncam, nlight, nflex, nflexnode, nflexvert, nflexedge, nflexelem,
               nflexelemdata, nflexelemedge, nflexshelldata, nflexevpair, nflextexcoord,
               nmesh, nmeshvert, nmeshnormal, nmeshtexcoord, nmeshface, nmeshgraph, nmeshpoly,
               nmeshpolyvert, nmeshpolymap, nskin, nskinvert, nskintexvert, nskinface, nskinbone,
               nskinbonevert, nhfield, nhfielddata, ntex, ntexdata, nmat, npair, nexclude,
               neq, ntendon, nwrap, nsensor, nnumeric, nnumericdata, ntext, ntextdata,
               ntuple, ntupledata, nkey, nmocap, nplugin, npluginattr,
               nuser_body, nuser_jnt, nuser_geom, nuser_site, nuser_cam,
               nuser_tendon, nuser_actuator, nuser_sensor, nnames, npaths);
  if (!m) {
    throw mjCError(0, "could not create mjModel");
  }

  // copy everything into low-level model
  m->opt = option;
  m->vis = visual;
  CopyNames(m);
  CopyPaths(m);
  CopyTree(m);
  CopyPlugins(m);

  // keyframe compilation needs access to nq, nv, na, nmocap, qpos0
  ResolveKeyframes(m);

  for (int i=0; i < keys_.size(); i++) {
    keys_[i]->Compile(m);
  }

  // copy objects outsite kinematic tree (including keyframes)
  CopyObjects(m);

  // finalize simple bodies/dofs including tendon information
  FinalizeSimple(m);

  // compute non-zeros in actuator_moment
  m->nJmom = nJmom = CountNJmom(m);

  // scale mass
  if (compiler.settotalmass > 0) {
    mj_setTotalmass(m, compiler.settotalmass);
  }

  // set arena size into m->narena
  if (memory != -1) {
    // memory size is user-specified in bytes
    m->narena = memory;
  } else {
    const int nconmax = m->nconmax == -1 ? 100 : m->nconmax;
    const int njmax = m->njmax == -1 ? 500 : m->njmax;
    if (nstack != -1) {
      // (legacy) stack size is user-specified as multiple of sizeof(mjtNum)
      m->narena = sizeof(mjtNum) * nstack;
    } else {
      // use a conservative heuristic if neither memory nor nstack is specified in XML
      m->narena = sizeof(mjtNum) * static_cast<size_t>(mjMAX(
          1000,
          5*(njmax + m->neq + m->nv)*(njmax + m->neq + m->nv) +
          20*(m->nq + m->nv + m->nu + m->na + m->nbody + m->njnt +
              m->ngeom + m->nsite + m->neq + m->ntendon +  m->nwrap)));
    }

    // add an arena space equal to memory footprint prior to the introduction of the arena
    const std::size_t arena_bytes = (
      nconmax * sizeof(mjContact) +
      njmax * (8 * sizeof(int) + 14 * sizeof(mjtNum)) +
      m->nv * (3 * sizeof(int)) +
      njmax * m->nv * (2 * sizeof(int) + 2 * sizeof(mjtNum)) +
      njmax * njmax * (sizeof(int) + sizeof(mjtNum)));
    m->narena += arena_bytes;

    // round up to the nearest megabyte
    constexpr std::size_t kMegabyte = 1 << 20;
    std::size_t nstack_mb = m->narena / kMegabyte;
    std::size_t residual_mb = m->narena % kMegabyte ? 1 : 0;
    m->narena = kMegabyte * (nstack_mb + residual_mb);
  }

  // create data
  int disableflags = m->opt.disableflags;
  m->opt.disableflags |= mjDSBL_CONTACT;
  mj_makeRawData(&d, m);
  if (!d) {
    // m will be deleted by the catch statement in mjCModel::Compile()
    throw mjCError(0, "could not create mjData");
  }
  mj_resetData(m, d);

  // normalize keyframe quaternions
  for (int i=0; i < m->nkey; i++) {
    mj_normalizeQuat(m, m->key_qpos+i*m->nq);
  }

  // set constant fields
  mj_setConst(m, d);

  // automatic spring-damper adjustment
  AutoSpringDamper(m);

  // actuator lengthrange computation
  LengthRange(m, d);

  // save automatically-computed statistics, to disambiguate when saving
  extent_auto = m->stat.extent;
  meaninertia_auto = m->stat.meaninertia;
  meanmass_auto = m->stat.meanmass;
  meansize_auto = m->stat.meansize;
  mjuu_copyvec(center_auto, m->stat.center, 3);

  // override model statistics if defined by user
  if (mjuu_defined(stat.extent)) m->stat.extent = (mjtNum)stat.extent;
  if (mjuu_defined(stat.meaninertia)) m->stat.meaninertia = (mjtNum)stat.meaninertia;
  if (mjuu_defined(stat.meanmass)) m->stat.meanmass = (mjtNum)stat.meanmass;
  if (mjuu_defined(stat.meansize)) m->stat.meansize = (mjtNum)stat.meansize;
  if (mjuu_defined(stat.center[0])) mjuu_copyvec(m->stat.center, stat.center, 3);

  // assert that model has valid references
  const char* validationerr = mj_validateReferences(m);
  if (validationerr) {  // SHOULD NOT OCCUR
    // m and d will be deleted by the catch statement in mjCModel::Compile()
    throw mjCError(0, "%s", validationerr);
  }

  // delete partial mjData (no plugins), make a complete one
  mj_deleteData(d);
  d = nullptr;
  d = mj_makeData(m);
  if (!d) {
    // m will be deleted by the catch statement in mjCModel::Compile()
    throw mjCError(0, "could not create mjData");
  }

  // test forward simulation
  mj_step(m, d);

  // delete data
  mj_deleteData(d);
  m->opt.disableflags = disableflags;
  d = nullptr;

  // pass warning back
  if (warningtext[0]) {
    mju::strcpy_arr(errInfo.message, warningtext);
    errInfo.warning = true;
  }

  // save signature
  m->signature = Signature();

  // special cases that are not caused by user edits
  if (compiler.fusestatic || compiler.discardvisual ||
      !pairs_.empty() || !excludes_.empty()) {
    spec.element->signature = m->signature;
  }

  // check that the signature matches the spec
  if (m->signature != spec.element->signature) {
    throw mjCError(0, "signature mismatch");  // SHOULD NOT OCCUR
  }
}



std::string mjCModel::PrintTree(const mjCBody* body, std::string indent) {
  std::string tree;
  tree += indent + "<body>\n";
  indent += "  ";
  for (const auto& joint : body->joints) {
    tree += indent + "<joint>" + std::to_string(joint->nq()) + "</joint>\n";
  }
  for (uint64_t i = 0; i < body->geoms.size(); ++i) {
    tree += indent + "<geom/>\n";
  }
  for (uint64_t i = 0; i < body->sites.size(); ++i) {
    tree += indent + "<site/>\n";
  }
  for (uint64_t i = 0; i < body->cameras.size(); ++i) {
    tree += indent + "<camera/>\n";
  }
  for (uint64_t i = 0; i < body->lights.size(); ++i) {
    tree += indent + "<light/>\n";
  }
  for (uint64_t i = 0; i < body->bodies.size(); ++i) {
    tree += PrintTree(body->bodies[i], indent);
  }
  indent.pop_back();
  indent.pop_back();
  tree += indent + "</body>\n";
  return tree;
}



uint64_t mjCModel::Signature() {
  std::string tree = "\n" + PrintTree(bodies_[0]);
  for (unsigned int i = 0; i < flexes_.size(); ++i) {
    tree += "<flex/>\n";
  }
  for (unsigned int i = 0; i < meshes_.size(); ++i) {
    tree += "<mesh/>\n";
  }
  for (unsigned int i = 0; i < skins_.size(); ++i) {
    tree += "<skin/>\n";
  }
  for (unsigned int i = 0; i < hfields_.size(); ++i) {
    tree += "<heightfield/>\n";
  }
  for (unsigned int i = 0; i < textures_.size(); ++i) {
    tree += "<texture/>\n";
  }
  for (unsigned int i = 0; i < materials_.size(); ++i) {
    tree += "<material/>\n";
  }
  for (unsigned int i = 0; i < pairs_.size(); ++i) {
    tree += "<pair/>\n";
  }
  for (unsigned int i = 0; i < excludes_.size(); ++i) {
    tree += "<exclude/>\n";
  }
  for (unsigned int i = 1; i < equalities_.size(); ++i) {
    tree += "<equality/>\n";
  }
  for (unsigned int i = 0; i < tendons_.size(); ++i) {
    tree += "<tendon/>\n";
  }
  for (unsigned int i = 0; i < actuators_.size(); ++i) {
    tree += "<actuator/>\n";
  }
  for (unsigned int i = 0; i < sensors_.size(); ++i) {
    tree += "<sensor>" + std::to_string(sensors_[i]->spec.type) + "<sensor/>\n";
  }
  for (unsigned int i = 0; i < keys_.size(); ++i) {
    tree += "<key/>\n";
  }
  return mj_hashString(tree.c_str(), UINT64_MAX);
}



bool mjCModel::CheckBodiesMassInertia(std::vector<mjCBody*> bodies) {
  // check mass and inertia of moving bodies
  for (int i=0; i < bodies.size(); i++) {
    if (!bodies[i]->joints.empty()) {
      if (!CheckBodyMassInertia(bodies[i])) {
        return false;
      }
    }
  }
  return true;
}



bool mjCModel::CheckBodyMassInertia(mjCBody* body) {
  // check if body has valid mass and inertia
  if (body->mass >= mjMINVAL &&
      body->inertia[0] >= mjMINVAL &&
      body->inertia[1] >= mjMINVAL &&
      body->inertia[2] >= mjMINVAL) {
    return true;
  }

  // body is valid if we find a single static child with valid mass and inertia
  for (int i=0; i < body->Bodies().size(); i++) {
    // if we find a child with a joint, time to move on to the next moving body
    if (!body->Bodies()[i]->joints.empty()) {
      continue;
    }
    if (CheckBodyMassInertia(body->Bodies()[i])) {
      return true;
    }
  }

  // we did not find a child with valid mass and inertia
  return false;
}



//------------------------------- DECOMPILER -------------------------------------------------------

// get numeric data back from mjModel
bool mjCModel::CopyBack(const mjModel* m) {
  // check for null pointer
  if (!m) {
    errInfo = mjCError(0, "mjModel pointer is null in CopyBack");
    return false;
  }

  // make sure model has been compiled
  if (!compiled) {
    errInfo = mjCError(0, "mjCModel has not been compiled in CopyBack");
    return false;
  }

  // make sure sizes match
  if (nq != m->nq || nv != m->nv || nu != m->nu || na != m->na ||
      nbody != m->nbody ||njnt != m->njnt || ngeom != m->ngeom || nsite != m->nsite ||
      ncam != m->ncam || nlight != m->nlight || nmesh != m->nmesh ||
      nskin != m->nskin || nhfield != m->nhfield ||
      nmat != m->nmat || ntex != m->ntex || npair != m->npair || nexclude != m->nexclude ||
      neq != m->neq || ntendon != m->ntendon || nwrap != m->nwrap || nsensor != m->nsensor ||
      nnumeric != m->nnumeric || nnumericdata != m->nnumericdata || ntext != m->ntext ||
      ntextdata != m->ntextdata || nnames != m->nnames ||
      nM != m->nM || nD != m->nD || nC != m->nC || nB != m->nB || nJmom != m->nJmom ||
      nemax != m->nemax || nconmax != m->nconmax || njmax != m->njmax ||
      npaths != m->npaths) {
    errInfo = mjCError(0, "incompatible models in CopyBack");
    return false;
  }

  if (spec.element->signature != m->signature) {
    errInfo = mjCError(0, "incompatible signatures in CopyBack");
    return false;
  }

  // option and visual
  option = m->opt;
  visual = m->vis;

  // runtime-modifiable members of mjStatistic, if different from computed values
  if (m->stat.meaninertia != meaninertia_auto) stat.meaninertia = m->stat.meaninertia;
  if (m->stat.meanmass != meanmass_auto) stat.meanmass = m->stat.meanmass;
  if (m->stat.meansize != meansize_auto) stat.meansize = m->stat.meansize;
  if (m->stat.extent != extent_auto) stat.extent = m->stat.extent;
  if (m->stat.center[0] != center_auto[0] ||
      m->stat.center[1] != center_auto[1] ||
      m->stat.center[2] != center_auto[2]) {
    mjuu_copyvec(stat.center, m->stat.center, 3);
  }

  // qpos0, qpos_spring
  for (int i=0; i < njnt; i++) {
    switch (joints_[i]->type) {
      case mjJNT_FREE:
        mjuu_copyvec(bodies_[m->jnt_bodyid[i]]->pos, m->qpos0+m->jnt_qposadr[i], 3);
        mjuu_copyvec(bodies_[m->jnt_bodyid[i]]->quat, m->qpos0+m->jnt_qposadr[i]+3, 4);
        break;

      case mjJNT_SLIDE:
      case mjJNT_HINGE:
        joints_[i]->ref = (double)m->qpos0[m->jnt_qposadr[i]];
        joints_[i]->springref = (double)m->qpos_spring[m->jnt_qposadr[i]];
        break;

      case mjJNT_BALL:
        // nothing to do, qpos = unit quaternion always
        break;
    }
  }
  mjuu_copyvec(qpos0.data(), m->qpos0, m->nq);

  // body
  mjCBody* pb;
  for (int i=0; i < nbody; i++) {
    pb = bodies_[i];

    mjuu_copyvec(pb->pos, m->body_pos+3*i, 3);
    mjuu_copyvec(pb->quat, m->body_quat+4*i, 4);
    mjuu_copyvec(pb->ipos, m->body_ipos+3*i, 3);
    mjuu_copyvec(pb->iquat, m->body_iquat+4*i, 4);
    pb->mass = (double)m->body_mass[i];
    mjuu_copyvec(pb->inertia, m->body_inertia+3*i, 3);

    if (nuser_body) {
      mjuu_copyvec(pb->userdata_.data(), m->body_user + nuser_body*i, nuser_body);
    }
  }

  // joint and dof
  mjCJoint* pj;
  for (int i=0; i < njnt; i++) {
    pj = joints_[i];

    // joint data
    mjuu_copyvec(pj->pos, m->jnt_pos+3*i, 3);
    mjuu_copyvec(pj->axis, m->jnt_axis+3*i, 3);
    pj->stiffness = (double)m->jnt_stiffness[i];
    mjuu_copyvec(pj->range, m->jnt_range+2*i, 2);
    mjuu_copyvec(pj->solref_limit, m->jnt_solref+mjNREF*i, mjNREF);
    mjuu_copyvec(pj->solimp_limit, m->jnt_solimp+mjNIMP*i, mjNIMP);
    pj->margin = (double)m->jnt_margin[i];

    if (nuser_jnt) {
      mjuu_copyvec(pj->userdata_.data(), m->jnt_user + nuser_jnt*i, nuser_jnt);
    }

    // dof data
    int j = m->jnt_dofadr[i];
    mjuu_copyvec(pj->solref_friction, m->dof_solref+mjNREF*j, mjNREF);
    mjuu_copyvec(pj->solimp_friction, m->dof_solimp+mjNIMP*j, mjNIMP);
    pj->armature = (double)m->dof_armature[j];
    pj->damping = (double)m->dof_damping[j];
    pj->frictionloss = (double)m->dof_frictionloss[j];
  }

  // geom
  mjCGeom* pg;
  for (int i=0; i < ngeom; i++) {
    pg = geoms_[i];

    mjuu_copyvec(pg->size, m->geom_size+3*i, 3);
    mjuu_copyvec(pg->pos, m->geom_pos+3*i, 3);
    mjuu_copyvec(pg->quat, m->geom_quat+4*i, 4);
    mjuu_copyvec(pg->friction, m->geom_friction+3*i, 3);
    mjuu_copyvec(pg->solref, m->geom_solref+mjNREF*i, mjNREF);
    mjuu_copyvec(pg->solimp, m->geom_solimp+mjNIMP*i, mjNIMP);
    mjuu_copyvec(pg->rgba, m->geom_rgba+4*i, 4);
    pg->solmix = (double)m->geom_solmix[i];
    pg->margin = (double)m->geom_margin[i];
    pg->gap = (double)m->geom_gap[i];

    if (nuser_geom) {
      mjuu_copyvec(pg->userdata_.data(), m->geom_user + nuser_geom*i, nuser_geom);
    }
  }

  // mesh
  mjCMesh* pm;
  for (int i=0; i < nmesh; i++) {
    pm = meshes_[i];
    mjuu_copyvec(pm->GetPosPtr(), m->mesh_pos+3*i, 3);
    mjuu_copyvec(pm->GetQuatPtr(), m->mesh_quat+4*i, 4);
  }

  // heightfield
  mjCHField* phf;
  for (int i=0; i < nhfield; i++) {
    phf = hfields_[i];
    int size = phf->get_userdata().size();
    if (size) {
      int nrow = m->hfield_nrow[i];
      int ncol = m->hfield_ncol[i];
      float* userdata = phf->get_userdata().data();
      float* modeldata = m->hfield_data + m->hfield_adr[i];
      // copy back in reverse row order
      for (int j=0; j < nrow; j++) {
        int flip = nrow-1-j;
        mjuu_copyvec(userdata + flip*ncol, modeldata+j*ncol, ncol);
      }
    }
  }

  // sites
  for (int i=0; i < nsite; i++) {
    mjuu_copyvec(sites_[i]->size, m->site_size + 3 * i, 3);
    mjuu_copyvec(sites_[i]->pos, m->site_pos+3*i, 3);
    mjuu_copyvec(sites_[i]->quat, m->site_quat+4*i, 4);
    mjuu_copyvec(sites_[i]->rgba, m->site_rgba+4*i, 4);

    if (nuser_site) {
      mjuu_copyvec(sites_[i]->userdata_.data(), m->site_user + nuser_site*i, nuser_site);
    }
  }

  // cameras
  for (int i=0; i < ncam; i++) {
    mjuu_copyvec(cameras_[i]->pos, m->cam_pos+3*i, 3);
    mjuu_copyvec(cameras_[i]->quat, m->cam_quat+4*i, 4);
    cameras_[i]->fovy = (double)m->cam_fovy[i];
    cameras_[i]->ipd = (double)m->cam_ipd[i];
    mjuu_copyvec(cameras_[i]->resolution, m->cam_resolution+2*i, 2);
    mjuu_copyvec(cameras_[i]->intrinsic, m->cam_intrinsic+4*i, 4);

    if (nuser_cam) {
      mjuu_copyvec(cameras_[i]->userdata_.data(), m->cam_user + nuser_cam*i, nuser_cam);
    }
  }

  // lights
  for (int i=0; i < nlight; i++) {
    mjuu_copyvec(lights_[i]->pos, m->light_pos+3*i, 3);
    mjuu_copyvec(lights_[i]->dir, m->light_dir+3*i, 3);
    mjuu_copyvec(lights_[i]->attenuation, m->light_attenuation+3*i, 3);
    lights_[i]->cutoff = m->light_cutoff[i];
    lights_[i]->exponent = m->light_exponent[i];
    mjuu_copyvec(lights_[i]->ambient, m->light_ambient+3*i, 3);
    mjuu_copyvec(lights_[i]->diffuse, m->light_diffuse+3*i, 3);
    mjuu_copyvec(lights_[i]->specular, m->light_specular+3*i, 3);
  }

  // materials
  for (int i=0; i < nmat; i++) {
    mjuu_copyvec(materials_[i]->texrepeat, m->mat_texrepeat+2*i, 2);
    materials_[i]->emission = m->mat_emission[i];
    materials_[i]->specular = m->mat_specular[i];
    materials_[i]->shininess = m->mat_shininess[i];
    materials_[i]->reflectance = m->mat_reflectance[i];
    mjuu_copyvec(materials_[i]->rgba, m->mat_rgba+4*i, 4);
  }

  // pairs
  for (int i=0; i < npair; i++) {
    mjuu_copyvec(pairs_[i]->solref, m->pair_solref+mjNREF*i, mjNREF);
    mjuu_copyvec(pairs_[i]->solreffriction, m->pair_solreffriction+mjNREF*i, mjNREF);
    mjuu_copyvec(pairs_[i]->solimp, m->pair_solimp+mjNIMP*i, mjNIMP);
    pairs_[i]->margin = (double)m->pair_margin[i];
    pairs_[i]->gap = (double)m->pair_gap[i];
    mjuu_copyvec(pairs_[i]->friction, m->pair_friction+5*i, 5);
  }

  // equality constraints
  for (int i=0; i < neq; i++) {
    mjuu_copyvec(equalities_[i]->data, m->eq_data+mjNEQDATA*i, mjNEQDATA);
    mjuu_copyvec(equalities_[i]->solref, m->eq_solref+mjNREF*i, mjNREF);
    mjuu_copyvec(equalities_[i]->solimp, m->eq_solimp+mjNIMP*i, mjNIMP);
  }

  // tendons
  for (int i=0; i < ntendon; i++) {
    mjuu_copyvec(tendons_[i]->range, m->tendon_range+2*i, 2);
    mjuu_copyvec(tendons_[i]->actfrcrange, m->tendon_actfrcrange+2*i, 2);
    mjuu_copyvec(tendons_[i]->solref_limit, m->tendon_solref_lim+mjNREF*i, mjNREF);
    mjuu_copyvec(tendons_[i]->solimp_limit, m->tendon_solimp_lim+mjNIMP*i, mjNIMP);
    mjuu_copyvec(tendons_[i]->solref_friction, m->tendon_solref_fri+mjNREF*i, mjNREF);
    mjuu_copyvec(tendons_[i]->solimp_friction, m->tendon_solimp_fri+mjNIMP*i, mjNIMP);
    mjuu_copyvec(tendons_[i]->rgba, m->tendon_rgba+4*i, 4);
    tendons_[i]->width = (double)m->tendon_width[i];
    tendons_[i]->margin = (double)m->tendon_margin[i];
    tendons_[i]->stiffness = (double)m->tendon_stiffness[i];
    tendons_[i]->damping = (double)m->tendon_damping[i];
    tendons_[i]->armature = (double)m->tendon_armature[i];
    tendons_[i]->frictionloss = (double)m->tendon_frictionloss[i];

    if (nuser_tendon) {
      mjuu_copyvec(tendons_[i]->userdata_.data(), m->tendon_user + nuser_tendon*i, nuser_tendon);
    }
  }

  // actuators
  mjCActuator* pa;
  for (int i=0; i < nu; i++) {
    pa = actuators_[i];

    mjuu_copyvec(pa->dynprm, m->actuator_dynprm+i*mjNDYN, mjNDYN);
    mjuu_copyvec(pa->gainprm, m->actuator_gainprm+i*mjNGAIN, mjNGAIN);
    mjuu_copyvec(pa->biasprm, m->actuator_biasprm+i*mjNBIAS, mjNBIAS);
    mjuu_copyvec(pa->ctrlrange, m->actuator_ctrlrange+2*i, 2);
    mjuu_copyvec(pa->forcerange, m->actuator_forcerange+2*i, 2);
    mjuu_copyvec(pa->actrange, m->actuator_actrange+2*i, 2);
    mjuu_copyvec(pa->lengthrange, m->actuator_lengthrange+2*i, 2);
    mjuu_copyvec(pa->gear, m->actuator_gear+6*i, 6);
    pa->cranklength = (double)m->actuator_cranklength[i];

    if (nuser_actuator) {
      mjuu_copyvec(pa->userdata_.data(), m->actuator_user + nuser_actuator*i, nuser_actuator);
    }
  }

  // sensors
  for (int i=0; i < nsensor; i++) {
    sensors_[i]->cutoff = (double)m->sensor_cutoff[i];
    sensors_[i]->noise = (double)m->sensor_noise[i];

    if (nuser_sensor) {
      mjuu_copyvec(sensors_[i]->userdata_.data(), m->sensor_user + nuser_sensor*i, nuser_sensor);
    }
  }

  // numeric data
  for (int i=0; i < nnumeric; i++) {
    for (int j=0; j < m->numeric_size[i]; j++) {
      numerics_[i]->data_[j] = (double)m->numeric_data[m->numeric_adr[i]+j];
    }
  }

  // tuple data
  for (int i=0; i < ntuple; i++) {
    for (int j=0; j < m->tuple_size[i]; j++) {
      tuples_[i]->objprm_[j] = (double)m->tuple_objprm[m->tuple_adr[i]+j];
    }
  }

  // keyframes
  for (int i=0; i < m->nkey; i++) {
    mjCKey* pk = keys_[i];

    pk->time = (double)m->key_time[i];
    mjuu_copyvec(pk->qpos_.data(), m->key_qpos + i*nq, nq);
    mjuu_copyvec(pk->qvel_.data(), m->key_qvel + i*nv, nv);
    if (na) {
      mjuu_copyvec(pk->act_.data(), m->key_act + i*na, na);
    }
    if (nmocap) {
      mjuu_copyvec(pk->mpos_.data(), m->key_mpos + i*3*nmocap, 3*nmocap);
      mjuu_copyvec(pk->mquat_.data(), m->key_mquat + i*4*nmocap, 4*nmocap);
    }
    if (nu) {
      mjuu_copyvec(pk->ctrl_.data(), m->key_ctrl + i*nu, nu);
    }
  }

  return true;
}



void mjCModel::ActivatePlugin(const mjpPlugin* plugin, int slot) {
  bool already_declared = false;
  for (const auto& [existing_plugin, existing_slot] : active_plugins_) {
    if (plugin == existing_plugin) {
      already_declared = true;
      break;
    }
  }
  if (!already_declared) {
    active_plugins_.emplace_back(std::make_pair(plugin, slot));
  }
}



void mjCModel::ResolvePlugin(mjCBase* obj, const std::string& plugin_name,
                             const std::string& plugin_instance_name, mjCPlugin** plugin_instance) {
  std::string pname = plugin_name;

  // if the plugin name is not specified by the user, infer it from the plugin instance
  if (plugin_name.empty() && !plugin_instance_name.empty()) {
    mjCBase* plugin_obj = FindObject(mjOBJ_PLUGIN, plugin_instance_name);
    if (plugin_obj) {
      pname = static_cast<mjCPlugin*>(plugin_obj)->plugin_name;
    } else {
      throw mjCError(obj, "unrecognized name '%s' for plugin instance",
                     plugin_instance_name.c_str());
    }
  }

  // if plugin_name is specified, check if it is in the list of active plugins
  // (in XML, active plugins are those declared as <required>)
  int plugin_slot = -1;
  if (!pname.empty()) {
    for (int i = 0; i < active_plugins_.size(); ++i) {
      if (active_plugins_[i].first->name == pname) {
        plugin_slot = active_plugins_[i].second;
        break;
      }
    }
    if (plugin_slot == -1) {
      throw mjCError(obj, "unrecognized plugin '%s'", pname.c_str());
    }
  }

  // implicit plugin instance
  if (*plugin_instance && (*plugin_instance)->plugin_slot == -1) {
    (*plugin_instance)->plugin_slot = plugin_slot;
    (*plugin_instance)->parent = obj;
  }

  // explicit plugin instance, look up existing mjCPlugin by instance name
  else if (!*plugin_instance) {
    *plugin_instance =
      static_cast<mjCPlugin*>(FindObject(mjOBJ_PLUGIN, plugin_instance_name));
    (*plugin_instance)->plugin_slot = plugin_slot;
    if (!*plugin_instance) {
      throw mjCError(
              obj, "unrecognized name '%s' for plugin instance", plugin_instance_name.c_str());
    }
    if (plugin_slot != -1 && plugin_slot != (*plugin_instance)->plugin_slot) {
      throw mjCError(
              obj, "'plugin' attribute does not match that of the instance");
    }
    plugin_slot = (*plugin_instance)->plugin_slot;
  }
}
