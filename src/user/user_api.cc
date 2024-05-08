// Copyright 2024 DeepMind Technologies Limited
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

#include "user/user_api.h"

#include <cstddef>
#include <functional>
#include <map>
#include <new>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "user/user_cache.h"
#include "xml/xml_util.h"

// global cache size in bytes (default 500MB)
static constexpr std::size_t kGlobalCacheSize = 500 * (1 << 20);


// prepend prefix
template <typename T>
static T& operator+(std::string_view prefix, T& base) {
  base.prefix = std::string(prefix);
  return base;
}



// append suffix
template <typename T>
static T& operator+(T& base, std::string_view suffix) {
  base.suffix = std::string(suffix);
  return base;
}



// create model
mjSpec* mjs_createSpec() {
  mjCModel* modelC = new mjCModel;
  return &modelC->spec;
}



// copy model
mjSpec* mjs_copySpec(const mjSpec* s) {
  mjCModel* modelC = new mjCModel(*static_cast<mjCModel*>(s->element));
  return &modelC->spec;
}



// copy back model
void mjs_copyBack(mjSpec* s, const mjModel* m) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  modelC->CopyBack(m);
}



// compile model
mjModel* mjs_compile(mjSpec* s, const mjVFS* vfs) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->Compile(vfs);
}



// attach body to a frame of the parent
int mjs_attachBody(mjsFrame* parent, const mjsBody* child,
                   const char* prefix, const char* suffix) {
  mjCFrame* frame_parent = static_cast<mjCFrame*>(parent->element);
  mjCBody* child_body = static_cast<mjCBody*>(child->element);
  *frame_parent += std::string(prefix) + *child_body + std::string(suffix);
  return 0;
}



// attach frame to a parent body
int mjs_attachFrame(mjsBody* parent, const mjsFrame* child,
                    const char* prefix, const char* suffix) {
  mjCBody* body_parent = static_cast<mjCBody*>(parent->element);
  mjCFrame* child_frame = static_cast<mjCFrame*>(child->element);
  *body_parent += std::string(prefix) + *child_frame + std::string(suffix);
  return 0;
}



// get error message from model
const char* mjs_getError(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->GetError().message;
}



// Detach body from mjSpec, return 0 if success.
int mjs_detachBody(mjSpec* s, mjsBody* b) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  mjCBody* body = static_cast<mjCBody*>(b->element);
  *model -= *body;
  mjs_deleteBody(b);
  return 0;
}



// check if model has warnings
int mjs_isWarning(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  return modelC->GetError().warning;
}



// delete model
void mjs_deleteSpec(mjSpec* s) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  delete model;
}



// delete body
void mjs_deleteBody(mjsBody* b) {
  mjCBody* body = static_cast<mjCBody*>(b->element);
  delete body;
}



// add child body to body, return child spec
mjsBody* mjs_addBody(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element)->AddBody(def);
  return &body->spec;
}



// add site to body, return site spec
mjsSite* mjs_addSite(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCSite* site = body->AddSite(def);
  return &site->spec;
}



// add joint to body
mjsJoint* mjs_addJoint(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddJoint(def);
  return &joint->spec;
}



// add free joint to body
mjsJoint* mjs_addFreeJoint(mjsBody* bodyspec) {
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddFreeJoint();
  return &joint->spec;
}



// add geom to body
mjsGeom* mjs_addGeom(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCGeom* geom = body->AddGeom(def);
  return &geom->spec;
}



// add camera to body
mjsCamera* mjs_addCamera(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCCamera* camera = body->AddCamera(def);
  return &camera->spec;
}



// add light to body
mjsLight* mjs_addLight(mjsBody* bodyspec, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCLight* light = body->AddLight(def);
  return &light->spec;
}



// add flex to model
mjsFlex* mjs_addFlex(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCFlex* flex = modelC->AddFlex();
  return &flex->spec;
}



// add frame to body
mjsFrame* mjs_addFrame(mjsBody* bodyspec, mjsFrame* parentframe) {
  mjCFrame* parentframeC = 0;
  if (parentframe) {
    parentframeC = static_cast<mjCFrame*>(parentframe->element);
  }
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCFrame* frameC = body->AddFrame(parentframeC);
  frameC->SetParent(body);
  return &frameC->spec;
}



// add mesh to model
mjsMesh* mjs_addMesh(mjSpec* s, mjsDefault* defspec) {
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCMesh* mesh = modelC->AddMesh(def);
  return &mesh->spec;
}



// add height field to model
mjsHField* mjs_addHField(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCHField* heightField = modelC->AddHField();
  return &heightField->spec;
}



// add skin to model
mjsSkin* mjs_addSkin(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCSkin* skin = modelC->AddSkin();
  return &skin->spec;
}



// add texture to model
mjsTexture* mjs_addTexture(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCTexture* texture = modelC->AddTexture();
  return &texture->spec;
}



// add material to model
mjsMaterial* mjs_addMaterial(mjSpec* s, mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCMaterial* material = modelC->AddMaterial(def);
  return &material->spec;
}



// add pair to model
mjsPair* mjs_addPair(mjSpec* s, mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCPair* pair = modelC->AddPair(def);
  return &pair->spec;
}



// add pair exclusion to model
mjsExclude* mjs_addExclude(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCBodyPair* bodypair = modelC->AddExclude();
  return &bodypair->spec;
}



// add equality to model
mjsEquality* mjs_addEquality(mjSpec* s, mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCEquality* equality = modelC->AddEquality(def);
  return &equality->spec;
}



// add tendon to model
mjsTendon* mjs_addTendon(mjSpec* s, mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCTendon* tendon = modelC->AddTendon(def);
  return &tendon->spec;
}



// wrap site using tendon
MJAPI mjsWrap* mjs_wrapSite(mjsTendon* tendonspec, const char* name) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapSite(name);
  return &tendon->path.back()->spec;
}



// wrap geom using tendon
mjsWrap* mjs_wrapGeom(mjsTendon* tendonspec, const char* name, const char* sidesite) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapGeom(name, sidesite);
  return &tendon->path.back()->spec;
}



// wrap joint using tendon
mjsWrap* mjs_wrapJoint(mjsTendon* tendonspec, const char* name, double coef) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapJoint(name, coef);
  return &tendon->path.back()->spec;
}



// wrap pulley using tendon
mjsWrap* mjs_wrapPulley(mjsTendon* tendonspec, double divisor) {
  mjCTendon* tendon = static_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapPulley(divisor);
  return &tendon->path.back()->spec;
}



// add actuator to model
mjsActuator* mjs_addActuator(mjSpec* s, mjsDefault* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = defspec ? static_cast<mjCDef*>(defspec->element) : 0;
  mjCActuator* actuator = modelC->AddActuator(def);
  return &actuator->spec;
}



// add sensor to model
mjsSensor* mjs_addSensor(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCSensor* sensor = modelC->AddSensor();
  return &sensor->spec;
}



// add numeric to model
mjsNumeric* mjs_addNumeric(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCNumeric* numeric = modelC->AddNumeric();
  return &numeric->spec;
}



// add text to model
mjsText* mjs_addText(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCText* text = modelC->AddText();
  return &text->spec;
}



// add tuple to model
mjsTuple* mjs_addTuple(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCTuple* tuple = modelC->AddTuple();
  return &tuple->spec;
}



// add keyframe to model
mjsKey* mjs_addKey(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCKey* key = modelC->AddKey();
  return &key->spec;
}



// add plugin to model
mjsPlugin* mjs_addPlugin(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCPlugin* plugin = modelC->AddPlugin();
  plugin->spec.instance = static_cast<mjElement*>(plugin);
  return &plugin->spec;
}



// add default to model
mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, int parentid, int* id) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  *id = (int)modelC->Defaults().size();
  mjCDef* def = modelC->AddDefault(classname, parentid);
  if (def) {
    return &def->spec;
  } else {
    return nullptr;
  }
}



// get objects
mjSpec* mjs_getSpec(mjsBody* body) {
  return &(static_cast<mjCBody*>(body->element)->model->spec);
}



// get default
mjsDefault* mjs_getDefault(mjElement* element) {
  return &(static_cast<mjCBase*>(element)->def->spec);
}



// Find default with given name in model.
mjsDefault* mjs_findDefault(mjSpec* s, const char* classname) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* cdef = modelC->FindDefault(classname);
  if (!cdef) {
    return nullptr;
  }
  return &cdef->spec;
}



// get default[0] from model
mjsDefault* mjs_getSpecDefault(mjSpec* s) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  mjCDef* def = modelC->Defaults()[0];
  if (!def) {
    return nullptr;
  }
  return &def->spec;
}



// find body in model by name
mjsBody* mjs_findBody(mjSpec* s, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  mjCBase* body = 0;
  if (model->IsCompiled()) {
    body = model->FindObject(mjOBJ_BODY, std::string(name));  // fast lookup
  } else {
    body = model->FindBody(model->GetWorld(), std::string(name));  // recursive search
  }
  return body ? &(static_cast<mjCBody*>(body)->spec) : nullptr;
}



// find child of a body by name
mjsBody* mjs_findChild(mjsBody* bodyspec, const char* name) {
  mjCBody* body = static_cast<mjCBody*>(bodyspec->element);
  mjCBase* child = body->FindObject(mjOBJ_BODY, std::string(name));
  return child ? &(static_cast<mjCBody*>(child)->spec) : nullptr;
}



// find mesh by name
mjsMesh* mjs_findMesh(mjSpec* s, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  mjCMesh* mesh = (mjCMesh*)model->FindObject(mjOBJ_MESH, std::string(name));
  return mesh ? &(static_cast<mjCMesh*>(mesh)->spec) : nullptr;
}



// find frame by name
mjsFrame* mjs_findFrame(mjSpec* s, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(s->element);
  mjCFrame* frame = (mjCFrame*)model->FindFrame(model->GetWorld(), std::string(name));
  return frame ? &(static_cast<mjCFrame*>(frame)->spec) : nullptr;
}



// set frame
void mjs_setFrame(mjElement* dest, mjsFrame* frame) {
  if (!frame) {
    return;
  }
  mjCFrame* frameC = static_cast<mjCFrame*>(frame->element);
  mjCBase* baseC = static_cast<mjCBase*>(dest);
  baseC->SetFrame(frameC);
}



// Resolve alternative orientations.
const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
                                   const mjsOrientation* orientation) {
  return ResolveOrientation(quat, degree, sequence, *orientation);
}



// get id
int mjs_getId(mjElement* element) {
  return static_cast<mjCBase*>(element)->id;
}



// set default
void mjs_setDefault(mjElement* element, mjsDefault* defspec) {
  mjCBase* baseC = static_cast<mjCBase*>(element);
  baseC->def = static_cast<mjCDef*>(defspec->element);
}



// set string
void mjs_setString(mjString dest, const char* text) {
  std::string* str = reinterpret_cast<std::string*>(dest);
  *str = std::string(text);
}



// Set specific entry in destination string vector.
mjtByte mjs_setInStringVec(mjStringVec dest, int i, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  if (v->size() <= i) {
    mju_error("Requested index in mjs_setInStringVec is out of bounds");
    return 0;
  }
  v->at(i) = std::string(text);
  return 1;
}



// split text and copy into string array
void mjs_setStringVec(mjStringVec dest, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  *v = mjXUtil::String2Vector<std::string>(text);
}



// add text entry to destination string vector
void mjs_appendString(mjStringVec dest, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  v->push_back(std::string(text));
}


// copy int array to vector
void mjs_setInt(mjIntVec dest, const int* array, int size) {
  std::vector<int>* v = reinterpret_cast<std::vector<int>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// append int array to vector of arrays
void mjs_appendIntVec(mjIntVecVec dest, const int* array, int size) {
  std::vector<std::vector<int>>* v = reinterpret_cast<std::vector<std::vector<int>>*>(dest);
  v->push_back(std::vector<int>(array, array + size));
}



// copy float array to vector
void mjs_setFloat(mjFloatVec dest, const float* array, int size) {
  std::vector<float>* v = reinterpret_cast<std::vector<float>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}




// append float array to vector of arrays
void mjs_appendFloatVec(mjFloatVecVec dest, const float* array, int size) {
  std::vector<std::vector<float>>* v = reinterpret_cast<std::vector<std::vector<float>>*>(dest);
  v->push_back(std::vector<float>(array, array + size));
}



// copy double array to vector
void mjs_setDouble(mjDoubleVec dest, const double* array, int size) {
  std::vector<double>* v = reinterpret_cast<std::vector<double>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// get string
const char* mjs_getString(const mjString source) {
  std::string* str = reinterpret_cast<std::string*>(source);
  if (!str) {
    return nullptr;
  }
  return str->c_str();
}



// get double array
const double* mjs_getDouble(const mjDoubleVec source, int* size) {
  std::vector<double>* v = reinterpret_cast<std::vector<double>*>(source);
  if (size) {
    *size = v->size();
  }
  return v->data();
}



// set plugin attributes
void mjs_setPluginAttributes(mjsPlugin* plugin, void* attributes) {
  mjCPlugin* pluginC = static_cast<mjCPlugin*>(plugin->instance);
  std::map<std::string, std::string, std::less<>>* config_attribs =
      reinterpret_cast<std::map<std::string, std::string, std::less<>>*>(attributes);
  pluginC->config_attribs = std::move(*config_attribs);
}



// Set active plugins.
void mjs_setActivePlugins(mjSpec* s, void* activeplugins) {
  mjCModel* modelC = static_cast<mjCModel*>(s->element);
  std::vector<std::pair<const mjpPlugin*, int>>* active_plugins =
      reinterpret_cast<std::vector<std::pair<const mjpPlugin*, int>>*>(activeplugins);
  modelC->SetActivePlugins(std::move(*active_plugins));
}



// compute full inertia
const char* mjs_fullInertia(double quat[4], double inertia[3], const double fullinertia[6]) {
  return FullInertia(quat, inertia, fullinertia);
}



// -------------------------- GLOBAL ASSET CACHE -------------------------------

void mj_setCacheSize(mjCache cache, std::size_t size) {
  mjCCache* ccache = reinterpret_cast<mjCCache*>(cache);
  if (ccache) {
    ccache->SetMaxSize(size);
  }
}



mjCache mj_globalCache() {
  // mjCCache is not trivially destructible and so the global cache needs to
  // allocated on the heap
  if constexpr (kGlobalCacheSize != 0) {
    static mjCCache* cache = new(std::nothrow) mjCCache(kGlobalCacheSize);
    return (mjCache) cache;
  } else {
    return NULL;
  }
}
