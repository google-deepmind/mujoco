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
#include <csetjmp>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjvisualize.h>
#include "cc/array_safety.h"
#include "engine/engine_forward.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_setconst.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "user/user_api.h"
#include "user/user_objects.h"
#include "user/user_util.h"

namespace {
namespace mju = ::mujoco::util;
using std::string;
using std::vector;
}  // namespace

// pthread on Linux, std::thread on Mac and Windows
#if defined(__APPLE__) || defined(_WIN32)
  #include <thread>
  using std::thread;

  int getnumproc(void) {
    return thread::hardware_concurrency();
  }
#else
  #include <pthread.h>
  #include <sys/sysinfo.h>

  int getnumproc(void) {
    return get_nprocs();
  }
#endif


// copy real-valued vector
template <class T1, class T2>
static void copyvec(T1* dest, T2* src, int n) {
  for (int i=0; i<n; i++) {
    dest[i] = (T1)src[i];
  }
}



//---------------------------------- CONSTRUCTOR AND DESTRUCTOR ------------------------------------

// constructor
mjCModel::mjCModel() {
  mjs_defaultSpec(&spec);
  elemtype = mjOBJ_UNKNOWN;
  spec_comment_.clear();
  spec_modelfiledir_.clear();
  spec_meshdir_.clear();
  spec_texturedir_.clear();
  spec_modelname_ = "MuJoCo Model";

  //------------------------ auto-computed statistics
#ifndef MEMORY_SANITIZER
  // initializing as best practice, but want MSAN to catch unintialized use
  meaninertia_auto = 0;
  meanmass_auto = 0;
  meansize_auto = 0;
  extent_auto = 0;
  center_auto[0] = center_auto[1] = center_auto[2] = 0;
#endif

  nplugin = 0;
  Clear();

  //------------------------ master default set
  defaults_.push_back(new mjCDef);

  // world body
  mjCBody* world = new mjCBody(this);
  mjuu_zerovec(world->pos, 3);
  mjuu_setvec(world->quat, 1, 0, 0, 0);
  world->mass = 0;
  mjuu_zerovec(world->inertia, 3);
  world->id = 0;
  world->parentid = 0;
  world->weldid = 0;
  world->name = "world";
  world->def = defaults_[0];
  bodies_.push_back(world);

  // create mjCBase lists from children lists
  CreateObjectLists();

  // point to model from spec
  PointToLocal();

  // this class allocated the plugins
  plugin_owner = true;
}



mjCModel::mjCModel(const mjCModel& other) {
  *this = other;
}



mjCModel& mjCModel::operator=(const mjCModel& other) {
  if (this != &other) {
    plugin_owner = false;
    this->spec = other.spec;
    *static_cast<mjCModel_*>(this) = static_cast<const mjCModel_&>(other);
    *static_cast<mjSpec*>(this) = static_cast<const mjSpec&>(other);

    // the world copy constructor takes care of copying the tree
    mjCBody* world = new mjCBody(*other.bodies_[0], this);
    bodies_.push_back(world);

    // add everything else
    *this += other;

    // copy name maps
    for (int i=0; i<mjNOBJECT; i++) {
      ids[i] = other.ids[i];
    }
  }
  return *this;
}



// copy vector of elements from another model to this model
template <class T>
void mjCModel::CopyList(std::vector<T*>& dest,
                        const std::vector<T*>& source,
                        std::map<mjCDef*, int>& def_map,
                        const std::vector<mjCDef*>& defaults) {
  // loop over the elements from the other model
  int nsource = (int)source.size();
  for (int i = 0; i < nsource; i++) {
    T* candidate = new T(*source[i]);
    try {
      // try to find the referenced object in this model
      candidate->NameSpace(source[i]->model);
      candidate->CopyFromSpec();
      candidate->ResolveReferences(this);
    } catch (mjCError err) {
      // if not present, skip the element
      delete candidate;
      continue;
    }
    // copy the element from the other model to this model
    dest.push_back(candidate);
    dest.back()->model = this;
    dest.back()->def = defaults[def_map[candidate->def]];
    dest.back()->id = -1;
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



mjCModel& mjCModel::operator+=(const mjCModel& other) {
  // create global lists
  MakeLists(bodies_[0]);
  CreateObjectLists();
  ProcessLists(/*checkrepeat=*/false);

  // copy all elements not in the tree
  std::map<mjCDef*, int> def_map;
  int ndefaults = (int)other.defaults_.size();
  for (int i = 0; i < ndefaults; i++) {
    if (this != &other) {
      defaults_.push_back(new mjCDef(*other.defaults_[i]));
    }
    def_map[other.defaults_[i]] = i;
  }

  if (this != &other) {
    // do not copy assets for self-attach
    CopyList(meshes_, other.meshes_, def_map, defaults_);
    CopyList(skins_, other.skins_, def_map, defaults_);
    CopyList(hfields_, other.hfields_, def_map, defaults_);
    CopyList(textures_, other.textures_, def_map, defaults_);
    CopyList(materials_, other.materials_, def_map, defaults_);
    CopyList(keys_, other.keys_, def_map, defaults_);
  }
  CopyList(flexes_, other.flexes_, def_map, defaults_);
  CopyList(pairs_, other.pairs_, def_map, defaults_);
  CopyList(excludes_, other.excludes_, def_map, defaults_);
  CopyList(tendons_, other.tendons_, def_map, defaults_);
  CopyList(equalities_, other.equalities_, def_map, defaults_);
  CopyList(actuators_, other.actuators_, def_map, defaults_);
  CopyList(sensors_, other.sensors_, def_map, defaults_);
  CopyList(numerics_, other.numerics_, def_map, defaults_);
  CopyList(texts_, other.texts_, def_map, defaults_);
  CopyList(tuples_, other.tuples_, def_map, defaults_);

  // plugins are global
  plugins_ = other.plugins_;
  active_plugins_ = other.active_plugins_;

  // update defaults for the copied objects
  for (int i = 1; i < other.bodies_.size(); i++) {
    bodies_[i]->def = defaults_[def_map[other.bodies_[i]->def]];
  }
  for (int i = 0; i < other.joints_.size(); i++) {
    joints_[i]->def = defaults_[def_map[other.joints_[i]->def]];
  }
  for (int i = 0; i < other.geoms_.size(); i++) {
    geoms_[i]->def = defaults_[def_map[other.geoms_[i]->def]];
  }
  for (int i = 0; i < other.sites_.size(); i++) {
    sites_[i]->def = defaults_[def_map[other.sites_[i]->def]];
  }
  for (int i = 0; i < other.cameras_.size(); i++) {
    cameras_[i]->def = defaults_[def_map[other.cameras_[i]->def]];
  }
  for (int i = 0; i < other.lights_.size(); i++) {
    lights_[i]->def= defaults_[def_map[other.lights_[i]->def]];
  }

  // restore to the same state as other
  if (!compiled) {
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

  PointToLocal();
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
      element->ResolveReferences(this);
    } catch (mjCError err) {
      delete element;
      list.erase(list.begin() + i);
      nlist--;
      i--;
      removed++;
    }
  }
}



mjCModel& mjCModel::operator-=(const mjCBody& subtree) {
  mjCModel oldmodel(*this);
  oldmodel.MakeLists(oldmodel.bodies_[0]);
  oldmodel.CreateObjectLists();
  oldmodel.ProcessLists(/*checkrepeat=*/false);

  // remove body from tree
  *bodies_[0] -= subtree;

  // create global lists
  MakeLists(bodies_[0]);
  CreateObjectLists();
  ProcessLists(/*checkrepeat=*/false);

  // check if we have to remove anything else
  RemoveFromList(pairs_, oldmodel);
  RemoveFromList(excludes_, oldmodel);
  RemoveFromList(tendons_, oldmodel);
  RemoveFromList(equalities_, oldmodel);
  RemoveFromList(actuators_, oldmodel);
  RemoveFromList(sensors_, oldmodel);

  // restore to the same state as before call
  if (!compiled) {
    mjCBody* world = bodies_[0];
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

  PointToLocal();
  return *this;
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
  spec.element = static_cast<mjElement*>(this);
  spec.comment = (mjString)&spec_comment_;
  spec.modelfiledir = (mjString)&spec_modelfiledir_;
  spec.modelname = (mjString)&spec_modelname_;
  spec.meshdir = (mjString)&spec_meshdir_;
  spec.texturedir = (mjString)&spec_texturedir_;
}



void mjCModel::CopyFromSpec() {
  *static_cast<mjSpec*>(this) = spec;
  comment_ = spec_comment_;
  modelfiledir_ = spec_modelfiledir_;
  modelname_ = spec_modelname_;
  meshdir_ = spec_meshdir_;
  texturedir_ = spec_texturedir_;
  comment = (mjString)&comment_;
  modelfiledir = (mjString)&modelfiledir_;
  modelname = (mjString)&modelname_;
  meshdir = (mjString)&meshdir_;
  texturedir = (mjString)&texturedir_;
}



// destructor
mjCModel::~mjCModel() {
  // delete kinematic tree and all objects allocated in it
  delete bodies_[0];

  // delete objects allocated in mjCModel
  for (int i=0; i<flexes_.size(); i++) delete flexes_[i];
  for (int i=0; i<meshes_.size(); i++) delete meshes_[i];
  for (int i=0; i<skins_.size(); i++) delete skins_[i];
  for (int i=0; i<hfields_.size(); i++) delete hfields_[i];
  for (int i=0; i<textures_.size(); i++) delete textures_[i];
  for (int i=0; i<materials_.size(); i++) delete materials_[i];
  for (int i=0; i<pairs_.size(); i++) delete pairs_[i];
  for (int i=0; i<excludes_.size(); i++) delete excludes_[i];
  for (int i=0; i<equalities_.size(); i++) delete equalities_[i];
  for (int i=0; i<tendons_.size(); i++) delete tendons_[i];  // also deletes wraps
  for (int i=0; i<actuators_.size(); i++) delete actuators_[i];
  for (int i=0; i<sensors_.size(); i++) delete sensors_[i];
  for (int i=0; i<numerics_.size(); i++) delete numerics_[i];
  for (int i=0; i<texts_.size(); i++) delete texts_[i];
  for (int i=0; i<tuples_.size(); i++) delete tuples_[i];
  for (int i=0; i<keys_.size(); i++) delete keys_[i];
  for (int i=0; i<defaults_.size(); i++) delete defaults_[i];

  if (plugin_owner) {
    for (int i=0; i<plugins_.size(); i++) delete plugins_[i];
  }

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
  nflexvert = 0;
  nflexedge = 0;
  nflexelem = 0;
  nflexelemdata = 0;
  nflexshelldata = 0;
  nflexevpair = 0;
  nflextexcoord = 0;
  nmeshvert = 0;
  nmeshnormal = 0;
  nmeshtexcoord = 0;
  nmeshface = 0;
  nmeshgraph = 0;
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
  njmax = -1;
  nconmax = -1;
  nmocap = 0;

  // pointer lists created by Compile
  bodies_.clear();
  joints_.clear();
  geoms_.clear();
  sites_.clear();
  cameras_.clear();
  lights_.clear();

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
  return obj;
}


// add object of any type, with default parameter
template <class T>
T* mjCModel::AddObjectDefault(vector<T*>& list, string type, mjCDef* def) {
  T* obj = new T(this, def ? def : defaults_[0]);
  obj->id = (int)list.size();
  obj->def = def ? def : defaults_[0];
  list.push_back(obj);
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
  for (int i=0; i<(int)defaults_.size(); i++) {
    if (defaults_[i]->name == name) {
      return defaults_[i];
    }
  }
  return nullptr;
}



// add default class to array
mjCDef* mjCModel::AddDefault(string name, int parentid) {
  // check for repeated name
  int thisid = (int)defaults_.size();
  for (int i=0; i<thisid; i++) {
    if (defaults_[i]->name==name) {
      return 0;
    }
  }

  // create new object
  mjCDef* def = new mjCDef;
  defaults_.push_back(def);

  // initialize contents
  if (parentid>=0 && parentid<thisid) {
    defaults_[parentid]->CopyFromSpec();
    *def = *defaults_[parentid];
    defaults_[parentid]->childid.push_back(thisid);
  }
  def->parentid = parentid;
  def->name = name;
  def->childid.clear();

  return def;
}



// find object by name in given list
template <class T>
static T* findobject(std::string_view name, const vector<T*>& list, const mjKeyMap& ids) {
  // this can occur in the URDF parser
  if (ids.empty()) {
    for (unsigned int i=0; i<list.size(); i++) {
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
mjCBody* mjCModel::FindBody(mjCBody* body, std::string name) {
  if (body->name == name) {
    return body;
  }

  for (auto child : body->bodies) {
    auto candidate = FindBody(child, name);
    if (candidate) {
      return candidate;
    }
  }

  return nullptr;
}



// find frame by name
mjCFrame* mjCModel::FindFrame(mjCBody* body, std::string name) const{
  for (auto frame : body->frames) {
    if (frame->name == name) {
      return frame;
    }
  }

  for (auto body : body->bodies) {
    auto candidate = FindFrame(body, name);
    if (candidate) {
      return candidate;
    }
  }

  return nullptr;
}



// detect null pose
bool mjCModel::IsNullPose(const mjtNum* pos, const mjtNum* quat) const {
  bool result = true;

  // check position if given
  if (pos) {
    if (pos[0] || pos[1] || pos[2]) {
      result = false;
    }
  }

  // check orientation if given
  if (quat) {
    if (quat[0]!=1 || quat[1] || quat[2] || quat[3]) {
      result = false;
    }
  }

  return result;
}



//------------------------------- COMPILER PHASES --------------------------------------------------

// make lists of objects in tree: bodies, geoms, joints, sites, cameras, lights
void mjCModel::MakeLists(mjCBody* body) {
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
  for (mjCBody* body : body->bodies) MakeLists(body);
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



// delete texture with given name or all textures if the name is omitted
template <class T>
static void DeleteTexture(std::vector<T*>& list, std::string_view name = "") {
  for (T* plist : list) {
    if (name.empty() || plist->get_texture() == name) {
      plist->del_texture();
    }
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
  for (int j=0; j<elements.size(); j++) {
    if (discard[j]) {
      delete elements[j];
    } else {
      elements[i] = elements[j];
      i++;
    }
  }

  // count cumulative discarded elements
  for (int i=0; i<elements.size()-1; i++) {
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
                       [&discard](mjCGeom* geom) { return discard[geom->id]; }),
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
    delete element;
  }
  elements.clear();
}


template <>
void mjCModel::DeleteAll<mjCTexture>(std::vector<mjCTexture*>& elements) {
  DeleteTexture(materials_);
  for (mjCTexture* element : elements) {
    delete element;
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
  for (int i=0; i<geoms_.size(); i++) {
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
  for (int i=0; i<skins_.size(); i++) {
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
  for (int i=0; i<sites_.size(); i++) {
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
  for (int i=0; i<tendons_.size(); i++) {
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
  for (int i=0; i<materials_.size(); i++) {
    mjCMaterial* material = materials_[i];

    // find texture by name
    if (!material->texture_.empty()) {
      mjCBase* texture = FindObject(mjOBJ_TEXTURE, material->texture_);
      if (texture) {
        material->texid = texture->id;
      } else {
        throw mjCError(material, "texture '%s' not found in material %d", material->texture_.c_str(), i);
      }
    }
  }

  // discard visual meshes and geoms
  if (discard) {
    std::vector<bool> discard_mesh(meshes_.size(), false);
    std::vector<bool> discard_geom(geoms_.size(), false);

    std::transform(meshes_.begin(), meshes_.end(), discard_mesh.begin(),
                  [](const mjCMesh* mesh) { return mesh->IsVisual(); });
    std::transform(geoms_.begin(), geoms_.end(), discard_geom.begin(),
                  [](const mjCGeom* geom) { return geom->IsVisual(); });

    Delete(meshes_, discard_mesh);
    Delete(geoms_, discard_geom);
  }
}



// if asset name is missing, set to filename
template <typename T>
void mjCModel::SetDefaultNames(std::vector<T*>& assets) {
  string stripped;
  std::map<string, std::vector<int>> names;

  // use filename if name is missing
  for (int i=0; i<assets.size(); i++) {
    assets[i]->CopyFromSpec();
    if (assets[i]->name.empty()) {
      stripped = mjuu_strippath(assets[i]->get_file());
      assets[i]->name = mjuu_stripext(stripped);
      names[assets[i]->name].push_back(i);
    }
  }

  // add suffix if duplicates
  for (auto const& [name, indices] : names) {
    if (indices.size() > 1) {
      for (int i=0; i<indices.size(); i++) {
        assets[indices[i]]->name += "_" + std::to_string(i);
      }
    }
  }
}



// throw error if a name is missing
void mjCModel::CheckEmptyNames(void) {
  // meshes
  for (int i=0; i<meshes_.size(); i++) {
    if (meshes_[i]->name.empty()) {
      throw mjCError(meshes_[i], "empty name in mesh");
    }
  }

  // hfields
  for (int i=0; i<hfields_.size(); i++) {
    if (hfields_[i]->name.empty()) {
      throw mjCError(hfields_[i], "empty name in height field");
    }
  }

  // textures
  for (int i=0; i < textures_.size(); i++) {
    if (textures_[i]->name.empty() && textures_[i]->type!=mjTEXTURE_SKYBOX) {
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



// number of position and velocity coordinates for each joint type
const int nPOS[4] = {7, 4, 1, 1};
const int nVEL[4] = {6, 3, 1, 1};

template <typename T>
static size_t getpathslength(std::vector<T> list) {
  size_t result = 0;
  for (const auto& element : list) {
    if (!element->get_file().empty()) {
      result += element->get_file().length() + 1;
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

  // nq, nv
  for (int i=0; i<njnt; i++) {
    nq += nPOS[joints_[i]->type];
    nv += nVEL[joints_[i]->type];
  }

  // nu, na
  for (int i=0; i<actuators_.size(); i++) {
    nu++;
    na += actuators_[i]->actdim + actuators_[i]->plugin_actdim;
  }

  // nbvh, nbvhstatic, nbvhdynamic
  for (int i=0; i<nbody; i++) {
    nbvhstatic += bodies_[i]->tree.nbvh;
  }
  for (int i=0; i<nmesh; i++) {
    nbvhstatic += meshes_[i]->tree().nbvh;
  }
  for (int i=0; i<nflex; i++) {
    nbvhdynamic += flexes_[i]->tree.nbvh;
  }
  nbvh = nbvhstatic + nbvhdynamic;

  // flex counts
  for (int i=0; i<nflex; i++) {
    nflexvert += flexes_[i]->nvert;
    nflexedge += flexes_[i]->nedge;
    nflexelem += flexes_[i]->nelem;
    nflexelemdata += flexes_[i]->nelem * (flexes_[i]->dim + 1);
    nflexshelldata += (int)flexes_[i]->shell.size();
    nflexevpair += (int)flexes_[i]->evpair.size()/2;
  }

  // mesh counts
  for (int i=0; i<nmesh; i++) {
    nmeshvert += meshes_[i]->nvert();
    nmeshnormal += meshes_[i]->nnormal();
    nmeshface += meshes_[i]->nface();
    nmeshtexcoord += (meshes_[i]->HasTexcoord() ? meshes_[i]->ntexcoord() : 0);
    nmeshgraph += meshes_[i]->szgraph();
  }

  // skin counts
  for (int i=0; i<nskin; i++) {
    nskinvert += skins_[i]->get_vert().size()/3;
    nskintexvert += skins_[i]->get_texcoord().size()/2;
    nskinface += skins_[i]->get_face().size()/3;
    nskinbone += skins_[i]->bodyid.size();
    for (int j=0; j<skins_[i]->bodyid.size(); j++) {
      nskinbonevert += skins_[i]->get_vertid()[j].size();
    }
  }

  // nhfielddata
  for (int i=0; i<nhfield; i++) nhfielddata += hfields_[i]->nrow * hfields_[i]->ncol;

  // ntexdata
  for (int i=0; i<ntex; i++) ntexdata += 3 * textures_[i]->width * textures_[i]->height;

  // nwrap
  for (int i=0; i<ntendon; i++) nwrap += (int)tendons_[i]->path.size();

  // nsensordata
  for (int i=0; i<nsensor; i++) nsensordata += sensors_[i]->dim;

  // nnumericdata
  for (int i=0; i<nnumeric; i++) nnumericdata += numerics_[i]->size;

  // ntextdata
  for (int i=0; i<ntext; i++) ntextdata += (int)texts_[i]->data_.size() + 1;

  // ntupledata
  for (int i=0; i<ntuple; i++) ntupledata += (int)tuples_[i]->objtype_.size();

  // npluginattr
  for (int i=0; i<nplugin; i++) npluginattr += (int)plugins_[i]->flattened_attributes.size();

  // nnames
  nnames = (int)modelname_.size() + 1;
  for (int i=0; i<nbody; i++)    nnames += (int)bodies_[i]->name.length() + 1;
  for (int i=0; i<njnt; i++)     nnames += (int)joints_[i]->name.length() + 1;
  for (int i=0; i<ngeom; i++)    nnames += (int)geoms_[i]->name.length() + 1;
  for (int i=0; i<nsite; i++)    nnames += (int)sites_[i]->name.length() + 1;
  for (int i=0; i<ncam; i++)     nnames += (int)cameras_[i]->name.length() + 1;
  for (int i=0; i<nlight; i++)   nnames += (int)lights_[i]->name.length() + 1;
  for (int i=0; i<nflex; i++)    nnames += (int)flexes_[i]->name.length() + 1;
  for (int i=0; i<nmesh; i++)    nnames += (int)meshes_[i]->name.length() + 1;
  for (int i=0; i<nskin; i++)    nnames += (int)skins_[i]->name.length() + 1;
  for (int i=0; i<nhfield; i++)  nnames += (int)hfields_[i]->name.length() + 1;
  for (int i=0; i<ntex; i++)     nnames += (int)textures_[i]->name.length() + 1;
  for (int i=0; i<nmat; i++)     nnames += (int)materials_[i]->name.length() + 1;
  for (int i=0; i<npair; i++)    nnames += (int)pairs_[i]->name.length() + 1;
  for (int i=0; i<nexclude; i++) nnames += (int)excludes_[i]->name.length() + 1;
  for (int i=0; i<neq; i++)      nnames += (int)equalities_[i]->name.length() + 1;
  for (int i=0; i<ntendon; i++)  nnames += (int)tendons_[i]->name.length() + 1;
  for (int i=0; i<nu; i++)       nnames += (int)actuators_[i]->name.length() + 1;
  for (int i=0; i<nsensor; i++)  nnames += (int)sensors_[i]->name.length() + 1;
  for (int i=0; i<nnumeric; i++) nnames += (int)numerics_[i]->name.length() + 1;
  for (int i=0; i<ntext; i++)    nnames += (int)texts_[i]->name.length() + 1;
  for (int i=0; i<ntuple; i++)   nnames += (int)tuples_[i]->name.length() + 1;
  for (int i=0; i<nkey; i++)     nnames += (int)keys_[i]->name.length() + 1;
  for (int i=0; i<nplugin; i++)  nnames += (int)plugins_[i]->name.length() + 1;

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
  for (int i=0; i<neq; i++) {
    if (equalities_[i]->type==mjEQ_CONNECT) {
      nemax += 3;
    } else if (equalities_[i]->type==mjEQ_WELD) {
      nemax += 7;
    } else {
      nemax += 1;
    }
  }
}



// automatic stiffness and damping computation
void mjCModel::AutoSpringDamper(mjModel* m) {
  // process all joints
  for (int n=0; n<m->njnt; n++) {
    // get joint dof address and number of dimensions
    int adr = m->jnt_dofadr[n];
    int ndim = nVEL[m->jnt_type[n]];

    // get timeconst and dampratio from joint specificatin
    mjtNum timeconst = (mjtNum)joints_[n]->springdamper[0];
    mjtNum dampratio = (mjtNum)joints_[n]->springdamper[1];

    // skip joint if either parameter is non-positive
    if (timeconst<=0 || dampratio<=0) {
      continue;
    }

    // get average inertia (dof_invweight0 in free joint is different for tran and rot)
    mjtNum inertia = 0;
    for (int i=0; i<ndim; i++) {
      inertia += m->dof_invweight0[adr+i];
    }
    inertia = ((mjtNum)ndim) / mju_max(mjMINVAL, inertia);

    // compute stiffness and damping (same as solref computation)
    mjtNum stiffness = inertia / mju_max(mjMINVAL, timeconst*timeconst*dampratio*dampratio);
    mjtNum damping = 2 * inertia / mju_max(mjMINVAL, timeconst);

    // assign
    m->jnt_stiffness[n] = stiffness;
    for (int i=0; i<ndim; i++) {
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

  for (int i=larg->start; i<larg->start+larg->num; i++) {
    if (i<larg->m->nu) {
      if (!mj_setLengthRange(larg->m, larg->data, i, larg->LRopt, larg->error, larg->error_sz)) {
        return NULL;
      }
    }
  }

  return NULL;
}


// compute actuator lengthrange
void mjCModel::LengthRange(mjModel* m, mjData* data) {
  // save options and modify
  mjOption saveopt = m->opt;
  m->opt.disableflags = mjDSBL_FRICTIONLOSS | mjDSBL_CONTACT | mjDSBL_PASSIVE |
                        mjDSBL_GRAVITY | mjDSBL_ACTUATION;
  if (LRopt.timestep>0) {
    m->opt.timestep = LRopt.timestep;
  }

  // number of threads available, max 16
  const int nthread = mjMIN(16, getnumproc()/2);

  // count actuators that need computation
  int cnt = 0;
  for (int i=0; i<m->nu; i++) {
    // skip depending on mode and type
    int ismuscle = (m->actuator_gaintype[i]==mjGAIN_MUSCLE ||
                    m->actuator_biastype[i]==mjBIAS_MUSCLE);
    int isuser = (m->actuator_gaintype[i]==mjGAIN_USER ||
                  m->actuator_biastype[i]==mjBIAS_USER);
    if ((LRopt.mode==mjLRMODE_NONE) ||
        (LRopt.mode==mjLRMODE_MUSCLE && !ismuscle) ||
        (LRopt.mode==mjLRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
      continue;
    }

    // use existing length range if available
    if (LRopt.useexisting &&
        (m->actuator_lengthrange[2*i] < m->actuator_lengthrange[2*i+1])) {
      continue;
    }

    // count
    cnt++;
  }

  // single thread
  if (!usethread || cnt<2 || nthread<2) {
    char err[200];
    for (int i=0; i<m->nu; i++) {
      if (!mj_setLengthRange(m, data, i, &LRopt, err, 200)) {
        throw mjCError(0, "%s", err);
      }
    }
  }

  // multiple threads
  else {
    // allocate mjData for each thread
    char err[16][200];
    mjData* pdata[16] = {data};
    for (int i=1; i<nthread; i++) {
      pdata[i] = mj_makeData(m);
    }

    // number of actuators per thread
    int num = m->nu / nthread;
    while (num*nthread < m->nu) {
      num++;
    }

    // prepare thread function arguments, clear errors
    LRThreadArg arg[16];
    for (int i=0; i<nthread; i++) {
      LRThreadArg temp = {m, pdata[i], i*num, num, &LRopt, err[i], 200};
      arg[i] = temp;
      err[i][0] = 0;
    }

    // use std::thread
#if defined(_WIN32) || defined(__APPLE__)
    // launch threads
    thread th[16];
    for (int i=0; i<nthread; i++) {
      th[i] = thread(LRfunc, arg+i);
    }

    // wait for threads to finish
    for (int i=0; i<nthread; i++) {
      th[i].join();
    }

    // use pthread
#else
    // launch threads
    pthread_t th[16];
    for (int i=0; i<nthread; i++) {
      pthread_create(th+i, NULL, LRfunc, arg+i);
    }

    // wait for threads to finish
    for (int i=0; i<nthread; i++) {
      pthread_join(th[i], NULL);
    }
#endif

    // free mjData allocated here
    for (int i=1; i<nthread; i++) {
      mj_deleteData(pdata[i]);
    }

    // report first error
    for (int i=0; i<nthread; i++) {
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
  for (unsigned int i=0; i<list.size(); i++) {
    // ignore empty strings
    if (list[i]->name.empty()) {
      continue;
    }

    uint64_t j = mj_hashString(list[i]->name.c_str(), map_size);

    // find first empty slot using linear probing
    for (; map[j]!=-1; j=(j+1) % map_size) {}
    map[j] = i;
  }

  for (unsigned int i=0; i<list.size(); i++) {
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
    if (!list[i] || list[i]->get_file().empty()) {
      continue;
    }
    adr = addtolist(list[i]->get_file(), adr, &path_adr[i], paths);
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
  for (int i=0; i<nbody; i++) {
    // get body and parent pointers
    mjCBody* pb = bodies_[i];
    mjCBody* par = bodies_[pb->parentid];

    // set body fields
    m->body_parentid[i] = pb->parentid;
    m->body_weldid[i] = pb->weldid;
    m->body_mocapid[i] = pb->mocapid;
    m->body_jntnum[i] = (int)pb->joints.size();
    m->body_jntadr[i] = (!pb->joints.empty() ? jntadr : -1);
    m->body_dofnum[i] = pb->dofnum;
    m->body_dofadr[i] = (pb->dofnum ? dofadr : -1);
    m->body_geomnum[i] = (int)pb->geoms.size();
    m->body_geomadr[i] = (!pb->geoms.empty() ? pb->geoms[0]->id : -1);
    copyvec(m->body_pos+3*i, pb->pos, 3);
    copyvec(m->body_quat+4*i, pb->quat, 4);
    copyvec(m->body_ipos+3*i, pb->ipos, 3);
    copyvec(m->body_iquat+4*i, pb->iquat, 4);
    m->body_mass[i] = (mjtNum)pb->mass;
    copyvec(m->body_inertia+3*i, pb->inertia, 3);
    m->body_gravcomp[i] = pb->gravcomp;
    copyvec(m->body_user+nuser_body*i, pb->get_userdata().data(), nuser_body);

    m->body_contype[i] = pb->contype;
    m->body_conaffinity[i] = pb->conaffinity;
    m->body_margin[i] = (mjtNum)pb->margin;

    // bounding volume hierarchy
    m->body_bvhadr[i] = pb->tree.nbvh ? bvh_adr : -1;
    m->body_bvhnum[i] = pb->tree.nbvh;
    if (pb->tree.nbvh) {
      memcpy(m->bvh_aabb + 6*bvh_adr, pb->tree.bvh.data(), 6*pb->tree.nbvh*sizeof(mjtNum));
      memcpy(m->bvh_child + 2*bvh_adr, pb->tree.child.data(), 2*pb->tree.nbvh*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pb->tree.level.data(), pb->tree.nbvh*sizeof(int));
      for (int i=0; i<pb->tree.nbvh; i++) {
        m->bvh_nodeid[i + bvh_adr] = pb->tree.nodeid[i] ? *(pb->tree.nodeid[i]) : -1;
      }
    }
    bvh_adr += pb->tree.nbvh;

    // count free joints
    int cntfree = 0;
    for (int j=0; j<(int)pb->joints.size(); j++) {
      cntfree += (pb->joints[j]->type == mjJNT_FREE);
    }

    // check validity of free joint
    if (cntfree>1 || (cntfree==1 && pb->joints.size()>1)) {
      throw mjCError(pb, "free joint can only appear by itself");
    }
    if (cntfree && pb->parentid) {
      throw mjCError(pb, "free joint can only be used on top level");
    }

    // rootid: self if world or child of world, otherwise parent's rootid
    if (i==0 || pb->parentid==0) {
      m->body_rootid[i] = i;
    } else {
      m->body_rootid[i] = m->body_rootid[pb->parentid];
    }

    // init lastdof from parent
    pb->lastdof = par->lastdof;

    // set sameframe
    m->body_sameframe[i] = IsNullPose(m->body_ipos+3*i, m->body_iquat+4*i);

    // init simple: sameframe, and (self-root, or parent is fixed child of world)
    int j = m->body_parentid[i];
    m->body_simple[i] = (m->body_sameframe[i] &&
                         (m->body_rootid[i]==i ||
                          (m->body_parentid[j]==0 &&
                           m->body_dofnum[j]==0)));

    // parent is not simple (unless world)
    if (m->body_parentid[i]>0) {
      m->body_simple[m->body_parentid[i]] = 0;
    }

    // loop over joints for this body
    int rotfound = 0;
    for (int j=0; j<(int)pb->joints.size(); j++) {
      // get pointer and id
      mjCJoint* pj = pb->joints[j];
      int jid = pj->id;

      // set joint fields
      m->jnt_type[jid] = pj->type;
      m->jnt_group[jid] = pj->group;
      m->jnt_limited[jid] = (mjtByte)pj->is_limited();
      m->jnt_actfrclimited[jid] = (mjtByte)pj->is_actfrclimited();
      m->jnt_actgravcomp[jid] = pj->actgravcomp;
      m->jnt_qposadr[jid] = qposadr;
      m->jnt_dofadr[jid] = dofadr;
      m->jnt_bodyid[jid] = pj->body->id;
      copyvec(m->jnt_pos+3*jid, pj->pos, 3);
      copyvec(m->jnt_axis+3*jid, pj->axis, 3);
      m->jnt_stiffness[jid] = (mjtNum)pj->stiffness;
      copyvec(m->jnt_range+2*jid, pj->range, 2);
      copyvec(m->jnt_actfrcrange+2*jid, pj->actfrcrange, 2);
      copyvec(m->jnt_solref+mjNREF*jid, pj->solref_limit, mjNREF);
      copyvec(m->jnt_solimp+mjNIMP*jid, pj->solimp_limit, mjNIMP);
      m->jnt_margin[jid] = (mjtNum)pj->margin;
      copyvec(m->jnt_user+nuser_jnt*jid, pj->get_userdata().data(), nuser_jnt);

      // not simple if: rotation already found, or pos not zero, or mis-aligned axis
      if (rotfound ||
          !IsNullPose(m->jnt_pos+3*jid, NULL) ||
          ((pj->type==mjJNT_HINGE || pj->type==mjJNT_SLIDE) &&
           ((mju_abs(pj->axis[0])>mjEPS) +
            (mju_abs(pj->axis[1])>mjEPS) +
            (mju_abs(pj->axis[2])>mjEPS)) > 1)) {
        m->body_simple[i] = 0;
      }

      // mark rotation
      if (pj->type==mjJNT_BALL || pj->type==mjJNT_HINGE) {
        rotfound = 1;
      }

      // set qpos0 and qpos_spring, check type
      switch (pj->type) {
      case mjJNT_FREE:
        copyvec(m->qpos0+qposadr, pb->pos, 3);
        copyvec(m->qpos0+qposadr+3, pb->quat, 4);
        mju_copy(m->qpos_spring+qposadr, m->qpos0+qposadr, 7);
        break;

      case mjJNT_BALL:
        m->qpos0[qposadr] = 1;
        m->qpos0[qposadr+1] = 0;
        m->qpos0[qposadr+2] = 0;
        m->qpos0[qposadr+3] = 0;
        mju_copy4(m->qpos_spring+qposadr, m->qpos0+qposadr);
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
      for (int j1=0; j1<nVEL[pj->type]; j1++) {
        // set attributes
        m->dof_bodyid[dofadr] = pb->id;
        m->dof_jntid[dofadr] = jid;
        copyvec(m->dof_solref+mjNREF*dofadr, pj->solref_friction, mjNREF);
        copyvec(m->dof_solimp+mjNIMP*dofadr, pj->solimp_friction, mjNIMP);
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
      qposadr += nPOS[pj->type];
    }

    // simple body with sliders and no rotational dofs: promote to simple level 2
    if (m->body_simple[i] && m->body_dofnum[i]) {
      m->body_simple[i] = 2;
      for (int j=0; j<(int)pb->joints.size(); j++) {
        if (pb->joints[j]->type!=mjJNT_SLIDE) {
        m->body_simple[i] = 1;
        break;
        }
      }
    }

    // loop over geoms for this body
    for (int j=0; j<(int)pb->geoms.size(); j++) {
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
      copyvec(m->geom_size+3*gid, pg->size, 3);
      copyvec(m->geom_aabb+6*gid, pg->aabb, 6);
      copyvec(m->geom_pos+3*gid, pg->pos, 3);
      copyvec(m->geom_quat+4*gid, pg->quat, 4);
      copyvec(m->geom_friction+3*gid, pg->friction, 3);
      m->geom_solmix[gid] = (mjtNum)pg->solmix;
      copyvec(m->geom_solref+mjNREF*gid, pg->solref, mjNREF);
      copyvec(m->geom_solimp+mjNIMP*gid, pg->solimp, mjNIMP);
      m->geom_margin[gid] = (mjtNum)pg->margin;
      m->geom_gap[gid] = (mjtNum)pg->gap;
      copyvec(m->geom_fluid+mjNFLUID*gid, pg->fluid, mjNFLUID);
      copyvec(m->geom_user+nuser_geom*gid, pg->get_userdata().data(), nuser_geom);
      copyvec(m->geom_rgba+4*gid, pg->rgba, 4);

      // determine sameframe
      if (IsNullPose(m->geom_pos+3*gid, m->geom_quat+4*gid)) {
        m->geom_sameframe[gid] = 1;
      } else if (pg->pos[0]==pb->ipos[0] &&
                 pg->pos[1]==pb->ipos[1] &&
                 pg->pos[2]==pb->ipos[2] &&
                 pg->quat[0]==pb->iquat[0] &&
                 pg->quat[1]==pb->iquat[1] &&
                 pg->quat[2]==pb->iquat[2] &&
                 pg->quat[3]==pb->iquat[3]) {
        m->geom_sameframe[gid] = 2;
      } else {
        m->geom_sameframe[gid] = 0;
      }

      // compute rbound
      m->geom_rbound[gid] = (mjtNum)pg->GetRBound();
    }

    // loop over sites for this body
    for (int j=0; j<(int)pb->sites.size(); j++) {
      // get pointer and id
      mjCSite* ps = pb->sites[j];
      int sid = ps->id;

      // set site fields
      m->site_type[sid] = ps->type;
      m->site_bodyid[sid] = ps->body->id;
      m->site_matid[sid] = ps->matid;
      m->site_group[sid] = ps->group;
      copyvec(m->site_size+3*sid, ps->size, 3);
      copyvec(m->site_pos+3*sid, ps->pos, 3);
      copyvec(m->site_quat+4*sid, ps->quat, 4);
      copyvec(m->site_user+nuser_site*sid, ps->userdata_.data(), nuser_site);
      copyvec(m->site_rgba+4*sid, ps->rgba, 4);

      // determine sameframe
      if (IsNullPose(m->site_pos+3*sid, m->site_quat+4*sid)) {
        m->site_sameframe[sid] = 1;
      } else if (ps->pos[0]==pb->ipos[0] &&
                 ps->pos[1]==pb->ipos[1] &&
                 ps->pos[2]==pb->ipos[2] &&
                 ps->quat[0]==pb->iquat[0] &&
                 ps->quat[1]==pb->iquat[1] &&
                 ps->quat[2]==pb->iquat[2] &&
                 ps->quat[3]==pb->iquat[3]) {
        m->site_sameframe[sid] = 2;
      } else {
        m->site_sameframe[sid] = 0;
      }
    }

    // loop over cameras for this body
    for (int j=0; j<(int)pb->cameras.size(); j++) {
      // get pointer and id
      mjCCamera* pc = pb->cameras[j];
      int cid = pc->id;

      // set camera fields
      m->cam_bodyid[cid] = pc->body->id;
      m->cam_mode[cid] = pc->mode;
      m->cam_targetbodyid[cid] = pc->targetbodyid;
      copyvec(m->cam_pos+3*cid, pc->pos, 3);
      copyvec(m->cam_quat+4*cid, pc->quat, 4);
      m->cam_fovy[cid] = (mjtNum)pc->fovy;
      m->cam_ipd[cid] = (mjtNum)pc->ipd;
      copyvec(m->cam_resolution+2*cid, pc->resolution, 2);
      copyvec(m->cam_sensorsize+2*cid, pc->sensor_size, 2);
      copyvec(m->cam_intrinsic+4*cid, pc->intrinsic, 4);
      copyvec(m->cam_user+nuser_cam*cid, pc->get_userdata().data(), nuser_cam);
    }

    // loop over lights for this body
    for (int j=0; j<(int)pb->lights.size(); j++) {
      // get pointer and id
      mjCLight* pl = pb->lights[j];
      int lid = pl->id;

      // set light fields
      m->light_bodyid[lid] = pl->body->id;
      m->light_mode[lid] = (int)pl->mode;
      m->light_targetbodyid[lid] = pl->targetbodyid;
      m->light_directional[lid] = (mjtByte)pl->directional;
      m->light_castshadow[lid] = (mjtByte)pl->castshadow;
      m->light_active[lid] = (mjtByte)pl->active;
      copyvec(m->light_pos+3*lid, pl->pos, 3);
      copyvec(m->light_dir+3*lid, pl->dir, 3);
      m->light_bulbradius[lid] = pl->bulbradius;
      copyvec(m->light_attenuation+3*lid, pl->attenuation, 3);
      m->light_cutoff[lid] = pl->cutoff;
      m->light_exponent[lid] = pl->exponent;
      copyvec(m->light_ambient+3*lid, pl->ambient, 3);
      copyvec(m->light_diffuse+3*lid, pl->diffuse, 3);
      copyvec(m->light_specular+3*lid, pl->specular, 3);
    }
  }

  // check number of dof's constructed, SHOULD NOT OCCUR
  if (nv!=dofadr) {
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
  for (int i=0; i<nbody; i++) {
    ngravcomp += (m->body_gravcomp[i] > 0);
  }
  m->ngravcomp = ngravcomp;

  // compute nM and dof_Madr
  nM = 0;
  for (int i=0; i<nv; i++) {
    // set address of this dof
    m->dof_Madr[i] = nM;

    // count ancestor dofs including self
    int j = i;
    while (j>=0) {
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
    bodies_[bodies_[i]->parentid]->subtreedofs += bodies_[i]->subtreedofs;
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
    int j = bodies_[i]->parentid;
    while (j > 0) {
      nB += bodies_[j]->dofnum;
      j = bodies_[j]->parentid;
    }
  }
  m->nB = nB;

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
}



// copy objects outside kinematic tree
void mjCModel::CopyObjects(mjModel* m) {
  int adr, bone_adr, vert_adr, normal_adr, face_adr, texcoord_adr;
  int edge_adr, elem_adr, elemdata_adr, shelldata_adr, evpair_adr;
  int bonevert_adr, graph_adr, data_adr, bvh_adr;

  // sizes outside call to mj_makeModel
  m->nemax = nemax;
  m->njmax = njmax;
  m->nconmax = nconmax;
  m->nsensordata = nsensordata;
  m->nuserdata = nuserdata;
  m->na = na;

  // find bvh_adr after bodies
  bvh_adr = 0;
  for (int i=0; i<nbody; i++) {
    bvh_adr = mjMAX(bvh_adr, m->body_bvhadr[i] + m->body_bvhnum[i]);
  }

  // meshes
  vert_adr = 0;
  normal_adr = 0;
  texcoord_adr = 0;
  face_adr = 0;
  graph_adr = 0;
  for (int i=0; i<nmesh; i++) {
    // get pointer
    mjCMesh* pme = meshes_[i];

    // set fields
    m->mesh_vertadr[i] = vert_adr;
    m->mesh_vertnum[i] = pme->nvert();
    m->mesh_normaladr[i] = normal_adr;
    m->mesh_normalnum[i] = pme->nnormal();
    m->mesh_texcoordadr[i] = (pme->HasTexcoord() ? texcoord_adr : -1);
    m->mesh_texcoordnum[i] = pme->ntexcoord();
    m->mesh_faceadr[i] = face_adr;
    m->mesh_facenum[i] = pme->nface();
    m->mesh_graphadr[i] = (pme->szgraph() ? graph_adr : -1);
    m->mesh_bvhnum[i] = pme->tree().nbvh;
    m->mesh_bvhadr[i] = pme->tree().nbvh ? bvh_adr : -1;
    copyvec(&m->mesh_scale[3 * i], pme->get_scale(), 3);
    copyvec(&m->mesh_pos[3 * i], pme->GetOffsetPosPtr(), 3);
    copyvec(&m->mesh_quat[4 * i], pme->GetOffsetQuatPtr(), 4);

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

    // copy bvh data
    if (pme->tree().nbvh) {
      memcpy(m->bvh_aabb + 6*bvh_adr, pme->tree().bvh.data(), 6*pme->tree().nbvh*sizeof(mjtNum));
      memcpy(m->bvh_child + 2*bvh_adr, pme->tree().child.data(), 2*pme->tree().nbvh*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pme->tree().level.data(), pme->tree().nbvh*sizeof(int));
      for (int j=0; j<pme->tree().nbvh; j++) {
        m->bvh_nodeid[j + bvh_adr] = pme->tree().nodeid[j] ? *(pme->tree().nodeid[j]) : -1;
      }
    }

    // advance counters
    vert_adr += pme->nvert();
    normal_adr += pme->nnormal();
    texcoord_adr += (pme->HasTexcoord() ? pme->ntexcoord() : 0);
    face_adr += pme->nface();
    graph_adr += pme->szgraph();
    bvh_adr += pme->tree().nbvh;
  }

  // flexes
  vert_adr = 0;
  edge_adr = 0;
  elem_adr = 0;
  elemdata_adr = 0;
  shelldata_adr = 0;
  evpair_adr = 0;
  texcoord_adr = 0;
  for (int i=0; i<nflex; i++) {
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
    copyvec(m->flex_solref + mjNREF * i, pfl->solref, mjNREF);
    copyvec(m->flex_solimp + mjNIMP * i, pfl->solimp, mjNIMP);
    m->flex_radius[i] = (mjtNum)pfl->radius;
    copyvec(m->flex_friction + 3 * i, pfl->friction, 3);
    m->flex_margin[i] = (mjtNum)pfl->margin;
    m->flex_gap[i] = (mjtNum)pfl->gap;
    copyvec(m->flex_rgba + 4 * i, pfl->rgba, 4);

    // set fields: mesh-like
    m->flex_dim[i] = pfl->dim;
    m->flex_vertadr[i] = vert_adr;
    m->flex_vertnum[i] = pfl->nvert;
    m->flex_edgeadr[i] = edge_adr;
    m->flex_edgenum[i] = pfl->nedge;
    m->flex_elemadr[i] = elem_adr;
    m->flex_elemdataadr[i] = elemdata_adr;
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
    } else {
      m->flex_texcoordadr[i] = texcoord_adr;
      memcpy(m->flex_texcoord + 2*texcoord_adr,
            pfl->texcoord_.data(), pfl->texcoord_.size()*sizeof(float));
    }
    m->flex_elemnum[i] = pfl->nelem;
    memcpy(m->flex_elem + elemdata_adr, pfl->elem_.data(), pfl->elem_.size()*sizeof(int));
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
    m->flex_bvhnum[i] = pfl->tree.nbvh;
    m->flex_bvhadr[i] = pfl->tree.nbvh ? bvh_adr : -1;

    // find equality constraint referencing this flex
    m->flex_edgeequality[i] = 0;
    for (int k=0; k<(int)equalities_.size(); k++) {
      if (equalities_[k]->type==mjEQ_FLEX && equalities_[k]->name1_==pfl->name) {
        m->flex_edgeequality[i] = 1;
        break;
      }
    }

    // copy bvh data (flex aabb computed dynamically in mjData)
    if (pfl->tree.nbvh) {
      memcpy(m->bvh_child + 2*bvh_adr, pfl->tree.child.data(), 2*pfl->tree.nbvh*sizeof(int));
      memcpy(m->bvh_depth + bvh_adr, pfl->tree.level.data(), pfl->tree.nbvh*sizeof(int));
      for (int i=0; i<pfl->tree.nbvh; i++) {
        m->bvh_nodeid[i+ bvh_adr] = pfl->tree.nodeid[i] ? *(pfl->tree.nodeid[i]) : -1;
      }
    }

    // copy or set vert
    if (pfl->centered) {
      mju_zero(m->flex_vert + 3*vert_adr, 3*pfl->nvert);
    }
    else {
      memcpy(m->flex_vert + 3*vert_adr, pfl->vert_.data(), 3*pfl->nvert*sizeof(mjtNum));
    }

    // copy or set vertbodyid
    if (pfl->rigid) {
      for (int k=0; k<pfl->nvert; k++) {
        m->flex_vertbodyid[vert_adr + k] = pfl->vertbodyid[0];
      }
    }
    else {
      memcpy(m->flex_vertbodyid + vert_adr, pfl->vertbodyid.data(), pfl->nvert*sizeof(int));
    }

    // convert edge pairs to int array, set edge rigid
    for (int k=0; k<pfl->nedge; k++) {
      m->flex_edge[2*(edge_adr+k)] = pfl->edge[k].first;
      m->flex_edge[2*(edge_adr+k)+1] = pfl->edge[k].second;

      if (pfl->rigid) {
        m->flexedge_rigid[edge_adr+k] = 1;
      } else {
        // check if vertex body weldids are the same
        int b1 = pfl->vertbodyid[pfl->edge[k].first];
        int b2 = pfl->vertbodyid[pfl->edge[k].second];
        m->flexedge_rigid[edge_adr+k] = (bodies_[b1]->weldid == bodies_[b2]->weldid);
      }
    }

    // advance counters
    vert_adr += pfl->nvert;
    edge_adr += pfl->nedge;
    elem_adr += pfl->nelem;
    elemdata_adr += (pfl->dim+1) * pfl->nelem;
    shelldata_adr += (int)pfl->shell.size();
    evpair_adr += (int)pfl->evpair.size()/2;
    texcoord_adr += (int)pfl->texcoord_.size()/2;
    bvh_adr += pfl->tree.nbvh;
  }

  // skins
  vert_adr = 0;
  face_adr = 0;
  texcoord_adr = 0;
  bone_adr = 0;
  bonevert_adr = 0;
  for (int i=0; i<nskin; i++) {
    // get pointer
    mjCSkin* psk = skins_[i];

    // set fields
    m->skin_matid[i] = psk->matid;
    m->skin_group[i] = psk->group;
    copyvec(m->skin_rgba+4*i, psk->rgba, 4);
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
    for (int j=0; j<m->skin_bonenum[i]; j++) {
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
  for (int i=0; i<nhfield; i++) {
    // get pointer
    mjCHField* phf = hfields_[i];

    // set fields
    copyvec(m->hfield_size+4*i, phf->size, 4);
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
  for (int i=0; i<ntex; i++) {
    // get pointer
    mjCTexture* ptex = textures_[i];

    // set fields
    m->tex_type[i] = ptex->type;
    m->tex_height[i] = ptex->height;
    m->tex_width[i] = ptex->width;
    m->tex_adr[i] = data_adr;

    // copy rgb data
    memcpy(m->tex_rgb + data_adr, ptex->rgb.data(), 3*ptex->width*ptex->height);

    // advance counter
    data_adr += 3*ptex->width*ptex->height;
  }

  // materials
  for (int i=0; i<nmat; i++) {
    // get pointer
    mjCMaterial* pmat = materials_[i];

    // set fields
    m->mat_texid[i] = pmat->texid;
    m->mat_texuniform[i] = pmat->texuniform;
    copyvec(m->mat_texrepeat+2*i, pmat->texrepeat, 2);
    m->mat_emission[i] = pmat->emission;
    m->mat_specular[i] = pmat->specular;
    m->mat_shininess[i] = pmat->shininess;
    m->mat_reflectance[i] = pmat->reflectance;
    m->mat_metallic[i] = pmat->metallic;
    m->mat_roughness[i] = pmat->roughness;
    copyvec(m->mat_rgba+4*i, pmat->rgba, 4);
  }

  // geom pairs to include
  for (int i=0; i<npair; i++) {
    m->pair_dim[i] = pairs_[i]->condim;
    m->pair_geom1[i] = pairs_[i]->geom1->id;
    m->pair_geom2[i] = pairs_[i]->geom2->id;
    m->pair_signature[i] = pairs_[i]->signature;
    copyvec(m->pair_solref+mjNREF*i, pairs_[i]->solref, mjNREF);
    copyvec(m->pair_solreffriction+mjNREF*i, pairs_[i]->solreffriction, mjNREF);
    copyvec(m->pair_solimp+mjNIMP*i, pairs_[i]->solimp, mjNIMP);
    m->pair_margin[i] = (mjtNum)pairs_[i]->margin;
    m->pair_gap[i] = (mjtNum)pairs_[i]->gap;
    copyvec(m->pair_friction+5*i, pairs_[i]->friction, 5);
  }

  // body pairs to exclude
  for (int i=0; i<nexclude; i++) {
    m->exclude_signature[i] = excludes_[i]->signature;
  }

  // equality constraints
  for (int i=0; i<neq; i++) {
    // get pointer
    mjCEquality* peq = equalities_[i];

    // set fields
    m->eq_type[i] = peq->type;
    m->eq_obj1id[i] = peq->obj1id;
    m->eq_obj2id[i] = peq->obj2id;
    m->eq_active0[i] = peq->active;
    copyvec(m->eq_solref+mjNREF*i, peq->solref, mjNREF);
    copyvec(m->eq_solimp+mjNIMP*i, peq->solimp, mjNIMP);
    copyvec(m->eq_data+mjNEQDATA*i, peq->data, mjNEQDATA);
  }

  // tendons and wraps
  adr = 0;
  for (int i=0; i<ntendon; i++) {
    // get pointer
    mjCTendon* pte = tendons_[i];

    // set fields
    m->tendon_adr[i] = adr;
    m->tendon_num[i] = (int)pte->path.size();
    m->tendon_matid[i] = pte->matid;
    m->tendon_group[i] = pte->group;
    m->tendon_limited[i] = (mjtByte)pte->is_limited();
    m->tendon_width[i] = (mjtNum)pte->width;
    copyvec(m->tendon_solref_lim+mjNREF*i, pte->solref_limit, mjNREF);
    copyvec(m->tendon_solimp_lim+mjNIMP*i, pte->solimp_limit, mjNIMP);
    copyvec(m->tendon_solref_fri+mjNREF*i, pte->solref_friction, mjNREF);
    copyvec(m->tendon_solimp_fri+mjNIMP*i, pte->solimp_friction, mjNIMP);
    m->tendon_range[2*i] = (mjtNum)pte->range[0];
    m->tendon_range[2*i+1] = (mjtNum)pte->range[1];
    m->tendon_margin[i] = (mjtNum)pte->margin;
    m->tendon_stiffness[i] = (mjtNum)pte->stiffness;
    m->tendon_damping[i] = (mjtNum)pte->damping;
    m->tendon_frictionloss[i] = (mjtNum)pte->frictionloss;
    m->tendon_lengthspring[2*i] = (mjtNum)pte->springlength[0];
    m->tendon_lengthspring[2*i+1] = (mjtNum)pte->springlength[1];
    copyvec(m->tendon_user+nuser_tendon*i, pte->get_userdata().data(), nuser_tendon);
    copyvec(m->tendon_rgba+4*i, pte->rgba, 4);

    // set wraps
    for (int j=0; j<(int)pte->path.size(); j++) {
      m->wrap_type[adr+j] = pte->path[j]->type;
      m->wrap_objid[adr+j] = pte->path[j]->obj ? pte->path[j]->obj->id : -1;
      m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->prm;
      if (pte->path[j]->type==mjWRAP_SPHERE || pte->path[j]->type==mjWRAP_CYLINDER) {
        m->wrap_prm[adr+j] = (mjtNum)pte->path[j]->sideid;
      }
    }

    // advance address counter
    adr += (int)pte->path.size();
  }

  // actuators
  adr = 0;
  for (int i=0; i<nu; i++) {
    // get pointer
    mjCActuator* pac = actuators_[i];

    // set fields
    m->actuator_trntype[i] = pac->trntype;
    m->actuator_dyntype[i] = pac->dyntype;
    m->actuator_gaintype[i] = pac->gaintype;
    m->actuator_biastype[i] = pac->biastype;
    m->actuator_trnid[2*i] = pac->trnid[0];
    m->actuator_trnid[2*i+1] = pac->trnid[1];
    m->actuator_actnum[i] = pac->actdim + pac->plugin_actdim;
    m->actuator_actadr[i] = m->actuator_actnum[i] ? adr : -1;
    adr += m->actuator_actnum[i];
    m->actuator_group[i] = pac->group;
    m->actuator_ctrllimited[i] = (mjtByte)pac->is_ctrllimited();
    m->actuator_forcelimited[i] = (mjtByte)pac->is_forcelimited();
    m->actuator_actlimited[i] = (mjtByte)pac->is_actlimited();
    m->actuator_actearly[i] = pac->actearly;
    m->actuator_cranklength[i] = (mjtNum)pac->cranklength;
    copyvec(m->actuator_gear + 6*i, pac->gear, 6);
    copyvec(m->actuator_dynprm + mjNDYN*i, pac->dynprm, mjNDYN);
    copyvec(m->actuator_gainprm + mjNGAIN*i, pac->gainprm, mjNGAIN);
    copyvec(m->actuator_biasprm + mjNBIAS*i, pac->biasprm, mjNBIAS);
    copyvec(m->actuator_ctrlrange + 2*i, pac->ctrlrange, 2);
    copyvec(m->actuator_forcerange + 2*i, pac->forcerange, 2);
    copyvec(m->actuator_actrange + 2*i, pac->actrange, 2);
    copyvec(m->actuator_lengthrange + 2*i, pac->lengthrange, 2);
    copyvec(m->actuator_user+nuser_actuator*i, pac->get_userdata().data(), nuser_actuator);
  }

  // sensors
  adr = 0;
  for (int i=0; i<nsensor; i++) {
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
    copyvec(m->sensor_user+nuser_sensor*i, psen->get_userdata().data(), nuser_sensor);

    // calculate address and advance
    m->sensor_adr[i] = adr;
    adr += psen->dim;
  }

  // numeric fields
  adr = 0;
  for (int i=0; i<nnumeric; i++) {
    // get pointer
    mjCNumeric* pcu = numerics_[i];

    // set fields
    m->numeric_adr[i] = adr;
    m->numeric_size[i] = pcu->size;
    for (int j=0; j<(int)pcu->data_.size(); j++) {
      m->numeric_data[adr+j] = (mjtNum)pcu->data_[j];
    }
    for (int j=(int)pcu->data_.size(); j<(int)pcu->size; j++) {
      m->numeric_data[adr+j] = 0;
    }

    // advance address counter
    adr += m->numeric_size[i];
  }

  // text fields
  adr = 0;
  for (int i=0; i<ntext; i++) {
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
  for (int i=0; i<ntuple; i++) {
    // get pointer
    mjCTuple* ptu = tuples_[i];

    // set fields
    m->tuple_adr[i] = adr;
    m->tuple_size[i] = (int)ptu->objtype_.size();
    for (int j=0; j<m->tuple_size[i]; j++) {
      m->tuple_objtype[adr+j] = (int)ptu->objtype_[j];
      m->tuple_objid[adr+j] = ptu->obj[j]->id;
      m->tuple_objprm[adr+j] = (mjtNum)ptu->objprm_[j];
    }

    // advance address counter
    adr += m->tuple_size[i];
  }

  // copy keyframe data
  for (int i=0; i<nkey; i++) {
    // copy data
    m->key_time[i] = (mjtNum)keys_[i]->time;
    copyvec(m->key_qpos+i*nq, keys_[i]->qpos_.data(), nq);
    copyvec(m->key_qvel+i*nv, keys_[i]->qvel_.data(), nv);
    if (na) {
      copyvec(m->key_act+i*na, keys_[i]->act_.data(), na);
    }
    if (nmocap) {
      copyvec(m->key_mpos + i*3*nmocap, keys_[i]->mpos_.data(), 3*nmocap);
      copyvec(m->key_mquat + i*4*nmocap, keys_[i]->mquat_.data(), 4*nmocap);
    }

    // normalize quaternions in m->key_qpos
    for (int j=0; j<m->njnt; j++) {
      if (m->jnt_type[j]==mjJNT_BALL || m->jnt_type[j]==mjJNT_FREE) {
        mju_normalize4(m->key_qpos+i*nq+m->jnt_qposadr[j]+3*(m->jnt_type[j]==mjJNT_FREE));
      }
    }

    // normalize quaternions in m->key_mquat
    for (int j=0; j<nmocap; j++) {
      mju_normalize4(m->key_mquat+i*4*nmocap+4*j);
    }

    copyvec(m->key_ctrl+i*nu, keys_[i]->ctrl_.data(), nu);
  }

  // save qpos0 in user model (to recognize changed key_qpos in write)
  qpos0.resize(nq);
  mju_copy(qpos0.data(), m->qpos0, nq);
}



//------------------------------- FUSE STATIC ------------------------------------------------------

template <class T>
static void makelistid(std::vector<T*>& dest, std::vector<T*>& source) {
  for (int i=0; i<source.size(); i++) {
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
  for (int i=0; i<body->bodies.size(); i++) {
    body->bodies[i]->parentid = body->id;
    body->bodies[i]->weldid = (!body->bodies[i]->joints.empty() ?
                               body->bodies[i]->id : body->weldid);
  }

  makelistid(joints_, body->joints);
  makelistid(geoms_, body->geoms);
  makelistid(sites_, body->sites);

  // process children recursively
  for (int i=0; i<body->bodies.size(); i++) {
    FuseReindex(body->bodies[i]);
  }
}



// fuse static bodies with their parent
void mjCModel::FuseStatic(void) {
  // skip if model has potential to reference elements with changed ids
  if (!skins_.empty()        ||
      !pairs_.empty()        ||
      !excludes_.empty()     ||
      !equalities_.empty()   ||
      !tendons_.empty()      ||
      !actuators_.empty()    ||
      !sensors_.empty()      ||
      !tuples_.empty()       ||
      !cameras_.empty()      ||
      !lights_.empty()) {
    return;
  }

  // process fusable bodies
  for (int i=1; i<bodies_.size(); i++) {
    // get body and parent
    mjCBody* body = bodies_[i];
    mjCBody* par = bodies_[body->parentid];

    // skip if body has joints or mocap
    if (!body->joints.empty() || body->mocap) {
      continue;
    }

    //------------- add mass and inertia (if parent not world)

    if (body->parentid>0 && body->mass>=mjMINVAL) {
      // body_ipose = body_pose * body_ipose
      changeframe(body->ipos, body->iquat, body->pos, body->quat);

      // organize data
      double mass[2] = {
        par->mass,
        body->mass
      };
      double inertia[2][3] = {
        {par->inertia[0], par->inertia[1], par->inertia[2]},
        {body->inertia[0], body->inertia[1], body->inertia[2]}
      };
      double ipos[2][3] = {
        {par->ipos[0], par->ipos[1], par->ipos[2]},
        {body->ipos[0], body->ipos[1], body->ipos[2]}
      };
      double iquat[2][4] = {
        {par->iquat[0], par->iquat[1], par->iquat[2], par->iquat[3]},
        {body->iquat[0], body->iquat[1], body->iquat[2], body->iquat[3]}
      };

      // compute total mass
      par->mass = 0;
      mjuu_setvec(par->ipos, 0, 0, 0);
      for (int j=0; j<2; j++) {
        par->mass += mass[j];
        par->ipos[0] += mass[j]*ipos[j][0];
        par->ipos[1] += mass[j]*ipos[j][1];
        par->ipos[2] += mass[j]*ipos[j][2];
      }

      // small mass: allow for now, check for errors later
      if (par->mass<mjMINVAL) {
        par->mass = 0;
        mjuu_setvec(par->inertia, 0, 0, 0);
        mjuu_setvec(par->ipos, 0, 0, 0);
        mjuu_setvec(par->iquat, 1, 0, 0, 0);
      }

      // proceed with regular computation
      else {
        // locipos = center-of-mass
        par->ipos[0] /= par->mass;
        par->ipos[1] /= par->mass;
        par->ipos[2] /= par->mass;

        // add inertias
        double toti[6] = {0, 0, 0, 0, 0, 0};
        for (int j=0; j<2; j++) {
          double inertA[6], inertB[6];
          double dpos[3] = {
            ipos[j][0] - par->ipos[0],
            ipos[j][1] - par->ipos[1],
            ipos[j][2] - par->ipos[2]
          };

          mjuu_globalinertia(inertA, inertia[j], iquat[j]);
          mjuu_offcenter(inertB, mass[j], dpos);
          for (int k=0; k<6; k++) {
            toti[k] += inertA[k] + inertB[k];
          }
        }

        // compute principal axes of inertia
        mjuu_copyvec(par->fullinertia, toti, 6);
        const char* err1 = FullInertia(par->iquat, par->inertia, par->fullinertia);
        if (err1) {
          throw mjCError(NULL, "error '%s' in fusing static body inertias", err1);
        }
      }
    }

    //------------- replace body with its children in parent body list

    // change frames of child bodies
    for (int j=0; j<body->bodies.size(); j++)
      changeframe(body->bodies[j]->pos, body->bodies[j]->quat,
                  body->pos, body->quat);

    // find body in parent list, insert children before it
    bool found = false;
    for (auto iter=par->bodies.begin(); iter!=par->bodies.end(); iter++) {
      if (*iter==body) {
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
    for (auto iter=par->bodies.begin(); iter!=par->bodies.end(); iter++) {
      if (*iter==body) {
        par->bodies.erase(iter);
        found = true;
        break;
      }
    }
    if (!found) {
      mju_error("Internal error: FuseStatic: body not found");
    }

    //------------- assign geoms and sites to parent, change frames

    // geoms
    for (int j=0; j<body->geoms.size(); j++) {
      // assign
      body->geoms[j]->body = par;
      par->geoms.push_back(body->geoms[j]);

      // change frame
      changeframe(body->geoms[j]->pos, body->geoms[j]->quat, body->pos, body->quat);
    }

    // sites
    for (int j=0; j<body->sites.size(); j++) {
      // assign
      body->sites[j]->body = par;
      par->sites.push_back(body->sites[j]);

      // change frame
      changeframe(body->sites[j]->pos, body->sites[j]->quat, body->pos, body->quat);
    }

    //------------- remove from global body list, reduce global counts

    // find in global and erase
    found = false;
    for (auto iter=bodies_.begin(); iter!=bodies_.end(); iter++) {
      if (*iter==body) {
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
    for (int j=0; j<bodies_.size(); j++) {
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
      par->margin = mju_max(par->margin, geom->margin);
    }

    // recompute BVH
    int nbvhfuse = body->tree.nbvh + par->tree.nbvh;
    par->ComputeBVH();
    nbvhstatic += par->tree.nbvh - nbvhfuse;
    nbvh += par->tree.nbvh - nbvhfuse;

    //------------- delete body (without deleting children)

    // delete allocation
    body->bodies.clear();
    body->geoms.clear();
    body->sites.clear();
    delete body;

    // check index i again (we have a new body at this index)
    i--;
  }
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
  for (int i=0; i<(int)list.size(); i++) {
    list[i]->id = i;
  }
}


// set ids, check for repeated names
template <class T>
static void processlist(mjListKeyMap& ids, vector<T*>& list,
                        mjtObj type, bool checkrepeat = true) {
  // assign ids for regular elements
  if (type < mjNOBJECT) {
    for (size_t i=0; i < list.size(); i++) {
      // check for incompatible id setting; SHOULD NOT OCCUR
      if (list[i]->id!=-1 && list[i]->id!=i) {
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
        throw mjCError(NULL, "%s", msg.c_str());
      }
    }
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
static thread_local std::jmp_buf error_jmp_buf;
static thread_local char errortext[500] = "";
static void errorhandler(const char* msg) {
  mju::strcpy_arr(errortext, msg);
  std::longjmp(error_jmp_buf, 1);
}


// warning handler for low-level engine
static thread_local char warningtext[500] = "";
static void warninghandler(const char* msg) {
  mju::strcpy_arr(warningtext, msg);
}


// compiler
mjModel* mjCModel::Compile(const mjVFS* vfs) {
  if (compiled) {
    // clear kinematic tree
    for (int i=0; i<bodies_.size(); i++) {
      bodies_[i]->subtreedofs = 0;
    }
    mjCBody* world = bodies_[0];
    Clear();
    bodies_.push_back(world);
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
  mjModel* volatile m = nullptr;
  mjData* volatile data = nullptr;

  // save error and warning handlers
  void (*save_error)(const char*) = _mjPRIVATE__get_tls_error_fn();
  void (*save_warning)(const char*) = _mjPRIVATE__get_tls_warning_fn();

  // install error and warning handlers, clear error and warning
  _mjPRIVATE__set_tls_error_fn(errorhandler);
  _mjPRIVATE__set_tls_warning_fn(warninghandler);

  errInfo = mjCError();
  warningtext[0] = 0;

  // init random number generator, to make textures reproducible
  srand(123);

  try {
    if (setjmp(error_jmp_buf) != 0) {
      // TryCompile resulted in an mju_error which was converted to a longjmp.
      throw mjCError(0, "engine error: %s", errortext);
    }
    TryCompile(*const_cast<mjModel**>(&m), *const_cast<mjData**>(&data), vfs);
  } catch (mjCError err) {
    // deallocate everything allocated in Compile
    mj_deleteModel(m);
    mj_deleteData(data);
    mjCBody* world = bodies_[0];
    Clear();
    bodies_.push_back(world);

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
  return m;
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
  if (bodies_.size()+flexes_.size()>=65534) {
    throw mjCError(0, "number of bodies plus flexes must be less than 65534");
  }

  // append directory separator
  if (!meshdir_.empty()) {
    int n = meshdir_.length();
    if (meshdir_[n-1]!='/' && meshdir_[n-1]!='\\') {
      meshdir_ += '/';
    }
  }
  if (!texturedir_.empty()) {
    int n = texturedir_.length();
    if (texturedir_[n-1]!='/' && texturedir_[n-1]!='\\') {
      texturedir_ += '/';
    }
  }

  // add missing keyframes
  for (int i=keys_.size(); i<nkey; i++) {
    AddKey();
  }

  // make lists of objects created in kinematic tree
  MakeLists(bodies_[0]);

  // fill missing names and check that they are all filled
  SetDefaultNames(meshes_);
  SetDefaultNames(skins_);
  SetDefaultNames(hfields_);
  SetDefaultNames(textures_);
  CheckEmptyNames();

  // set object ids, check for repeated names
  ProcessLists();

  // delete visual assets
  if (discardvisual) {
    DeleteAll(materials_);
    DeleteTexcoord(flexes_);
    DeleteTexcoord(meshes_);
    DeleteAll(textures_);
  }

  // map names to asset references
  IndexAssets(/*discard=*/false);

  // mark meshes that need convex hull
  for (int i=0; i<geoms_.size(); i++) {
    if (geoms_[i]->mesh &&
        (geoms_[i]->spec.type==mjGEOM_MESH || geoms_[i]->spec.type==mjGEOM_SDF) &&
        (geoms_[i]->spec.contype || geoms_[i]->spec.conaffinity)) {
      geoms_[i]->mesh->set_needhull(true);
    }
  }

  // automatically set nuser fields
  SetNuser();

  // compile meshes (needed for geom compilation)
  for (int i=0; i<meshes_.size(); i++) {
    meshes_[i]->Compile(vfs);
  }

  // compile objects in kinematic tree
  for (int i=0; i<bodies_.size(); i++) {
    bodies_[i]->Compile();  // also compiles joints, geoms, sites, cameras, lights, frames
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
  IndexAssets(discardvisual);
  SetSizes();
  // fuse static if enabled
  if (fusestatic) {
    FuseStatic();
  }

  // set nmocap and body.mocapid
  for (mjCBody* body : bodies_) {
    if (body->mocap) {
      body->mocapid = nmocap;
      nmocap++;
    } else {
      body->mocapid = -1;
    }
  }

  // check body mass and inertia
  for (int i=1; i<bodies_.size(); i++) {
    mjCBody* b = bodies_[i];

    // find moving body with small mass or inertia
    if (!b->joints.empty() &&
        (b->mass<mjMINVAL ||
          b->inertia[0]<mjMINVAL ||
          b->inertia[1]<mjMINVAL ||
          b->inertia[2]<mjMINVAL)) {
      // does it have static children with mass and inertia
      bool ok = false;
      for (size_t j=0; j<b->bodies.size(); j++) {
        if (b->bodies[j]->joints.empty() &&
            b->bodies[j]->mass>=mjMINVAL &&
            b->bodies[j]->inertia[0]>=mjMINVAL &&
            b->bodies[j]->inertia[1]>=mjMINVAL &&
            b->bodies[j]->inertia[2]>=mjMINVAL) {
          ok = true;
          break;
        }
      }

      // error
      if (!ok) {
        throw mjCError(b, "mass and inertia of moving bodies must be larger than mjMINVAL");
      }
    }
  }

  // create low-level model
  m = mj_makeModel(nq, nv, nu, na, nbody, nbvh, nbvhstatic, nbvhdynamic, njnt, ngeom, nsite,
                   ncam, nlight, nflex, nflexvert, nflexedge, nflexelem,
                   nflexelemdata, nflexshelldata, nflexevpair, nflextexcoord,
                   nmesh, nmeshvert, nmeshnormal, nmeshtexcoord, nmeshface, nmeshgraph,
                   nskin, nskinvert, nskintexvert, nskinface, nskinbone, nskinbonevert,
                   nhfield, nhfielddata, ntex, ntexdata, nmat, npair, nexclude,
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

  // assign plugin slots and copy plugin config attributes
  {
    int adr = 0;
    for (int i = 0; i < nplugin; ++i) {
      m->plugin[i] = plugins_[i]->spec.plugin_slot;
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
    std::vector<std::vector<int>> plugin_to_actuators(nplugin);
    for (int i = 0; i < nu; ++i) {
      if (actuators_[i]->plugin.active) {
        int actuator_plugin = static_cast<mjCPlugin*>(actuators_[i]->plugin.instance)->id;
        m->actuator_plugin[i] = actuator_plugin;
        plugin_to_actuators[actuator_plugin].push_back(i);
      } else {
        m->actuator_plugin[i] = -1;
      }
    }

    for (int i = 0; i < nbody; ++i) {
      if (bodies_[i]->plugin.active) {
        m->body_plugin[i] = static_cast<mjCPlugin*>(bodies_[i]->plugin.instance)->id;
      } else {
        m->body_plugin[i] = -1;
      }
    }

    for (int i = 0; i < ngeom; ++i) {
      if (geoms_[i]->plugin.active) {
        m->geom_plugin[i] = static_cast<mjCPlugin*>(geoms_[i]->plugin.instance)->id;
      } else {
        m->geom_plugin[i] = -1;
      }
    }

    std::vector<std::vector<int>> plugin_to_sensors(nplugin);
    for (int i = 0; i < nsensor; ++i) {
      if (sensors_[i]->type == mjSENS_PLUGIN) {
        int sensor_plugin = static_cast<mjCPlugin*>(sensors_[i]->plugin.instance)->id;
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
      if ((plugin->capabilityflags & mjPLUGIN_ACTUATOR) && plugin->actuator_actdim) {
        for (int actuator_id : plugin_to_actuators[i]) {
          int plugin_actdim = plugin->actuator_actdim(m, i, actuator_id);
          actuators_[actuator_id]->plugin_actdim = plugin_actdim;
          this->na += plugin_actdim;
        }
      }
    }
    m->npluginstate = stateadr;
  }

  // keyframe compilation needs access to nq, nv, na, nmocap, qpos0
  for (int i=0; i<keys_.size(); i++) {
    keys_[i]->Compile(m);
  }

  // copy objects outsite kinematic tree (including keyframes)
  CopyObjects(m);

  // scale mass
  if (settotalmass>0) {
    mj_setTotalmass(m, settotalmass);
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
  d = mj_makeRawData(m);
  if (!d) {
    mj_deleteModel(m);
    throw mjCError(0, "could not create mjData");
  }
  mj_resetData(m, d);

  // normalize keyframe quaternions
  for (int i=0; i<m->nkey; i++) {
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
  copyvec(center_auto, m->stat.center, 3);

  // override model statistics if defined by user
  if (mjuu_defined(stat.extent)) m->stat.extent = (mjtNum)stat.extent;
  if (mjuu_defined(stat.meaninertia)) m->stat.meaninertia = (mjtNum)stat.meaninertia;
  if (mjuu_defined(stat.meanmass)) m->stat.meanmass = (mjtNum)stat.meanmass;
  if (mjuu_defined(stat.meansize)) m->stat.meansize = (mjtNum)stat.meansize;
  if (mjuu_defined(stat.center[0])) copyvec(m->stat.center, stat.center, 3);

  // assert that model has valid references
  const char* validationerr = mj_validateReferences(m);
  if (validationerr) {  // SHOULD NOT OCCUR
    mj_deleteData(d);
    mj_deleteModel(m);
    throw mjCError(0, "%s", validationerr);
  }

  // delete partial mjData (no plugins), make a complete one
  mj_deleteData(d);
  d = nullptr;
  d = mj_makeData(m);
  if (!d) {
    mj_deleteModel(m);
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
  if (nq!=m->nq || nv!=m->nv || nu!=m->nu || na!=m->na ||
      nbody!=m->nbody ||njnt!=m->njnt || ngeom!=m->ngeom || nsite!=m->nsite ||
      ncam!=m->ncam || nlight != m->nlight || nmesh!=m->nmesh ||
      nskin!=m->nskin || nhfield!=m->nhfield ||
      nmat != m->nmat || ntex != m->ntex || npair!=m->npair || nexclude!=m->nexclude ||
      neq!=m->neq || ntendon!=m->ntendon || nwrap!=m->nwrap || nsensor!=m->nsensor ||
      nnumeric!=m->nnumeric || nnumericdata!=m->nnumericdata || ntext!=m->ntext ||
      ntextdata!=m->ntextdata || nnames!=m->nnames || nM!=m->nM || nD!=m->nD ||
      nB!=m->nB || nemax!=m->nemax || nconmax!=m->nconmax || njmax!=m->njmax ||
      npaths!=m->npaths) {
    errInfo = mjCError(0, "incompatible models in CopyBack");
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
    mju_copy3(stat.center, m->stat.center);
  }

  // qpos0, qpos_spring
  for (int i=0; i<njnt; i++) {
    switch (joints_[i]->type) {
    case mjJNT_FREE:
      copyvec(bodies_[m->jnt_bodyid[i]]->pos, m->qpos0+m->jnt_qposadr[i], 3);
      copyvec(bodies_[m->jnt_bodyid[i]]->quat, m->qpos0+m->jnt_qposadr[i]+3, 4);
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
  mju_copy(qpos0.data(), m->qpos0, m->nq);

  // body
  mjCBody* pb;
  for (int i=0; i<nbody; i++) {
    pb = bodies_[i];

    copyvec(pb->pos, m->body_pos+3*i, 3);
    copyvec(pb->quat, m->body_quat+4*i, 4);
    copyvec(pb->ipos, m->body_ipos+3*i, 3);
    copyvec(pb->iquat, m->body_iquat+4*i, 4);
    pb->mass = (double)m->body_mass[i];
    copyvec(pb->inertia, m->body_inertia+3*i, 3);

    if (nuser_body) {
      copyvec(pb->userdata_.data(), m->body_user + nuser_body*i, nuser_body);
    }
  }

  // joint and dof
  mjCJoint* pj;
  for (int i=0; i<njnt; i++) {
    pj = joints_[i];

    // joint data
    copyvec(pj->pos, m->jnt_pos+3*i, 3);
    copyvec(pj->axis, m->jnt_axis+3*i, 3);
    pj->stiffness = (double)m->jnt_stiffness[i];
    copyvec(pj->range, m->jnt_range+2*i, 2);
    copyvec(pj->solref_limit, m->jnt_solref+mjNREF*i, mjNREF);
    copyvec(pj->solimp_limit, m->jnt_solimp+mjNIMP*i, mjNIMP);
    pj->margin = (double)m->jnt_margin[i];

    if (nuser_jnt) {
      copyvec(pj->userdata_.data(), m->jnt_user + nuser_jnt*i, nuser_jnt);
    }

    // dof data
    int j = m->jnt_dofadr[i];
    copyvec(pj->solref_friction, m->dof_solref+mjNREF*j, mjNREF);
    copyvec(pj->solimp_friction, m->dof_solimp+mjNIMP*j, mjNIMP);
    pj->armature = (double)m->dof_armature[j];
    pj->damping = (double)m->dof_damping[j];
    pj->frictionloss = (double)m->dof_frictionloss[j];
  }

  // geom
  mjCGeom* pg;
  for (int i=0; i<ngeom; i++) {
    pg = geoms_[i];

    copyvec(pg->size, m->geom_size+3*i, 3);
    copyvec(pg->pos, m->geom_pos+3*i, 3);
    copyvec(pg->quat, m->geom_quat+4*i, 4);
    copyvec(pg->friction, m->geom_friction+3*i, 3);
    copyvec(pg->solref, m->geom_solref+mjNREF*i, mjNREF);
    copyvec(pg->solimp, m->geom_solimp+mjNIMP*i, mjNIMP);
    copyvec(pg->rgba, m->geom_rgba+4*i, 4);
    pg->solmix = (double)m->geom_solmix[i];
    pg->margin = (double)m->geom_margin[i];
    pg->gap = (double)m->geom_gap[i];

    if (nuser_geom) {
      copyvec(pg->userdata_.data(), m->geom_user + nuser_geom*i, nuser_geom);
    }
  }

  // mesh
  mjCMesh* pm;
  for (int i=0; i<nmesh; i++) {
    pm = meshes_[i];
    copyvec(pm->GetOffsetPosPtr(), m->mesh_pos+3*i, 3);
    copyvec(pm->GetOffsetQuatPtr(), m->mesh_quat+4*i, 4);
  }

  // heightfield
  mjCHField* phf;
  for (int i=0; i<nhfield; i++) {
    phf = hfields_[i];
    int size = phf->get_userdata().size();
    if (size) {
      int nrow = m->hfield_nrow[i];
      int ncol = m->hfield_ncol[i];
      float* userdata = phf->get_userdata().data();
      float* modeldata = m->hfield_data + m->hfield_adr[i];
      // copy back in reverse row order
      for (int j=0; j<nrow; j++) {
        int flip = nrow-1-j;
        copyvec(userdata + flip*ncol, modeldata+j*ncol, ncol);
      }
    }
  }

  // sites
  for (int i=0; i<nsite; i++) {
    copyvec(sites_[i]->size, m->site_size + 3 * i, 3);
    copyvec(sites_[i]->pos, m->site_pos+3*i, 3);
    copyvec(sites_[i]->quat, m->site_quat+4*i, 4);
    copyvec(sites_[i]->rgba, m->site_rgba+4*i, 4);

    if (nuser_site) {
      copyvec(sites_[i]->userdata_.data(), m->site_user + nuser_site*i, nuser_site);
    }
  }

  // cameras
  for (int i=0; i<ncam; i++) {
    copyvec(cameras_[i]->pos, m->cam_pos+3*i, 3);
    copyvec(cameras_[i]->quat, m->cam_quat+4*i, 4);
    cameras_[i]->fovy = (double)m->cam_fovy[i];
    cameras_[i]->ipd = (double)m->cam_ipd[i];
    copyvec(cameras_[i]->resolution, m->cam_resolution+2*i, 2);
    copyvec(cameras_[i]->intrinsic, m->cam_intrinsic+4*i, 4);

    if (nuser_cam) {
      copyvec(cameras_[i]->userdata_.data(), m->cam_user + nuser_cam*i, nuser_cam);
    }
  }

  // lights
  for (int i=0; i<nlight; i++) {
    copyvec(lights_[i]->pos, m->light_pos+3*i, 3);
    copyvec(lights_[i]->dir, m->light_dir+3*i, 3);
    copyvec(lights_[i]->attenuation, m->light_attenuation+3*i, 3);
    lights_[i]->cutoff = m->light_cutoff[i];
    lights_[i]->exponent = m->light_exponent[i];
    copyvec(lights_[i]->ambient, m->light_ambient+3*i, 3);
    copyvec(lights_[i]->diffuse, m->light_diffuse+3*i, 3);
    copyvec(lights_[i]->specular, m->light_specular+3*i, 3);
  }

  // materials
  for (int i=0; i<nmat; i++) {
    copyvec(materials_[i]->texrepeat, m->mat_texrepeat+2*i, 2);
    materials_[i]->emission = m->mat_emission[i];
    materials_[i]->specular = m->mat_specular[i];
    materials_[i]->shininess = m->mat_shininess[i];
    materials_[i]->reflectance = m->mat_reflectance[i];
    copyvec(materials_[i]->rgba, m->mat_rgba+4*i, 4);
  }

  // pairs
  for (int i=0; i<npair; i++) {
    copyvec(pairs_[i]->solref, m->pair_solref+mjNREF*i, mjNREF);
    copyvec(pairs_[i]->solreffriction, m->pair_solreffriction+mjNREF*i, mjNREF);
    copyvec(pairs_[i]->solimp, m->pair_solimp+mjNIMP*i, mjNIMP);
    pairs_[i]->margin = (double)m->pair_margin[i];
    pairs_[i]->gap = (double)m->pair_gap[i];
    copyvec(pairs_[i]->friction, m->pair_friction+5*i, 5);
  }

  // equality constraints
  for (int i=0; i<neq; i++) {
    copyvec(equalities_[i]->data, m->eq_data+mjNEQDATA*i, mjNEQDATA);
    copyvec(equalities_[i]->solref, m->eq_solref+mjNREF*i, mjNREF);
    copyvec(equalities_[i]->solimp, m->eq_solimp+mjNIMP*i, mjNIMP);
  }

  // tendons
  for (int i=0; i<ntendon; i++) {
    copyvec(tendons_[i]->range, m->tendon_range+2*i, 2);
    copyvec(tendons_[i]->solref_limit, m->tendon_solref_lim+mjNREF*i, mjNREF);
    copyvec(tendons_[i]->solimp_limit, m->tendon_solimp_lim+mjNIMP*i, mjNIMP);
    copyvec(tendons_[i]->solref_friction, m->tendon_solref_fri+mjNREF*i, mjNREF);
    copyvec(tendons_[i]->solimp_friction, m->tendon_solimp_fri+mjNIMP*i, mjNIMP);
    copyvec(tendons_[i]->rgba, m->tendon_rgba+4*i, 4);
    tendons_[i]->width = (double)m->tendon_width[i];
    tendons_[i]->margin = (double)m->tendon_margin[i];
    tendons_[i]->stiffness = (double)m->tendon_stiffness[i];
    tendons_[i]->damping = (double)m->tendon_damping[i];
    tendons_[i]->frictionloss = (double)m->tendon_frictionloss[i];

    if (nuser_tendon) {
      copyvec(tendons_[i]->userdata_.data(), m->tendon_user + nuser_tendon*i, nuser_tendon);
    }
  }

  // actuators
  mjCActuator* pa;
  for (int i=0; i<nu; i++) {
    pa = actuators_[i];

    copyvec(pa->dynprm, m->actuator_dynprm+i*mjNDYN, mjNDYN);
    copyvec(pa->gainprm, m->actuator_gainprm+i*mjNGAIN, mjNGAIN);
    copyvec(pa->biasprm, m->actuator_biasprm+i*mjNBIAS, mjNBIAS);
    copyvec(pa->ctrlrange, m->actuator_ctrlrange+2*i, 2);
    copyvec(pa->forcerange, m->actuator_forcerange+2*i, 2);
    copyvec(pa->actrange, m->actuator_actrange+2*i, 2);
    copyvec(pa->lengthrange, m->actuator_lengthrange+2*i, 2);
    copyvec(pa->gear, m->actuator_gear+6*i, 6);
    pa->cranklength = (double)m->actuator_cranklength[i];

    if (nuser_actuator) {
      copyvec(pa->userdata_.data(), m->actuator_user + nuser_actuator*i, nuser_actuator);
    }
  }

  // sensors
  for (int i=0; i<nsensor; i++) {
    sensors_[i]->cutoff = (double)m->sensor_cutoff[i];
    sensors_[i]->noise = (double)m->sensor_noise[i];

    if (nuser_sensor) {
      copyvec(sensors_[i]->userdata_.data(), m->sensor_user + nuser_sensor*i, nuser_sensor);
    }
  }

  // numeric data
  for (int i=0; i<nnumeric; i++) {
    for (int j=0; j<m->numeric_size[i]; j++) {
      numerics_[i]->data_[j] = (double)m->numeric_data[m->numeric_adr[i]+j];
    }
  }

  // tuple data
  for (int i=0; i<ntuple; i++) {
    for (int j=0; j<m->tuple_size[i]; j++) {
      tuples_[i]->objprm_[j] = (double)m->tuple_objprm[m->tuple_adr[i]+j];
    }
  }

  // keyframes
  for (int i=0; i<m->nkey; i++) {
    mjCKey* pk = keys_[i];

    pk->time = (double)m->key_time[i];
    copyvec(pk->qpos_.data(), m->key_qpos + i*nq, nq);
    copyvec(pk->qvel_.data(), m->key_qvel + i*nv, nv);
    if (na) {
      copyvec(pk->act_.data(), m->key_act + i*na, na);
    }
    if (nmocap) {
      copyvec(pk->mpos_.data(), m->key_mpos + i*3*nmocap, 3*nmocap);
      copyvec(pk->mquat_.data(), m->key_mquat + i*4*nmocap, 4*nmocap);
    }
    if (nu) {
      copyvec(pk->ctrl_.data(), m->key_ctrl + i*nu, nu);
    }
  }

  return true;
}

void mjCModel::ResolvePlugin(mjCBase* obj, const std::string& plugin_name,
                             const std::string& plugin_instance_name, mjCPlugin** plugin_instance) {
  // if plugin_name is specified, check if it is in the list of active plugins
  // (in XML, active plugins are those declared as <required>)
  int plugin_slot = -1;
  if (!plugin_name.empty()) {
    for (int i = 0; i < active_plugins_.size(); ++i) {
      if (active_plugins_[i].first->name == plugin_name) {
        plugin_slot = active_plugins_[i].second;
        break;
      }
    }
    if (plugin_slot == -1) {
      throw mjCError(obj, "unrecognized plugin '%s'", plugin_name.c_str());
    }
  }

  // implicit plugin instance
  if (*plugin_instance && (*plugin_instance)->spec.plugin_slot == -1) {
      (*plugin_instance)->spec.plugin_slot = plugin_slot;
      (*plugin_instance)->parent = obj;
  }

  // explicit plugin instance, look up existing mjCPlugin by instance name
  else if (!*plugin_instance) {
    *plugin_instance =
        static_cast<mjCPlugin*>(FindObject(mjOBJ_PLUGIN, plugin_instance_name));
    if (!*plugin_instance) {
      throw mjCError(
          obj, "unrecognized name '%s' for plugin instance", plugin_instance_name.c_str());
    }
    if (plugin_slot != -1 && plugin_slot != (*plugin_instance)->spec.plugin_slot) {
      throw mjCError(
          obj, "'plugin' attribute does not match that of the instance");
    }
    plugin_slot = (*plugin_instance)->spec.plugin_slot;
  }
}
