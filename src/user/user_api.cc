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
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <mujoco/mujoco.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "xml/xml_util.h"



// create model
mjmModel* mjm_createModel() {
  mjCModel* modelC = new mjCModel();
  return &modelC->spec;
}



// delete model
void mjm_deleteModel(mjmModel* modelspec) {
  mjCModel* model = reinterpret_cast<mjCModel*>(modelspec->element);
  delete model;
}



// add child body to body, return child spec
mjmBody* mjm_addBody(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element)->AddBody(def);
  return &body->spec;
}



// add site to body, return site spec
mjmSite* mjm_addSite(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCSite* site = body->AddSite(def);
  return &site->spec;
}



// add joint to body
mjmJoint* mjm_addJoint(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddJoint(def);
  return &joint->spec;
}



// add free joint to body
mjmJoint* mjm_addFreeJoint(mjmBody* bodyspec) {
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddFreeJoint();
  return &joint->spec;
}



// add geom to body
mjmGeom* mjm_addGeom(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCGeom* geom = body->AddGeom(def);
  return &geom->spec;
}



// add camera to body
mjmCamera* mjm_addCamera(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCCamera* camera = body->AddCamera(def);
  return &camera->spec;
}



// add light to body
mjmLight* mjm_addLight(mjmBody* bodyspec, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCLight* light = body->AddLight(def);
  return &light->spec;
}



// add flex to model
mjmFlex* mjm_addFlex(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCFlex* flex = modelC->AddFlex();
  return &flex->spec;
}



// add frame to body
mjmFrame* mjm_addFrame(mjmBody* bodyspec, mjmFrame* parentframe) {
  mjCFrame* parentframeC = 0;
  if (parentframe) {
    parentframeC = reinterpret_cast<mjCFrame*>(parentframe->element);
  }
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCFrame* frameC = body->AddFrame(parentframeC);
  return &frameC->spec;
}



// add mesh to model
mjmMesh* mjm_addMesh(mjmModel* model, mjmDefault* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCMesh* mesh = modelC->AddMesh(def);
  return &mesh->spec;
}



// add height field to model
mjmHField* mjm_addHField(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCHField* heightField = modelC->AddHField();
  return &heightField->spec;
}



// add skin to model
mjmSkin* mjm_addSkin(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCSkin* skin = modelC->AddSkin();
  return &skin->spec;
}



// add texture to model
mjmTexture* mjm_addTexture(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCTexture* texture = modelC->AddTexture();
  return &texture->spec;
}



// add material to model
mjmMaterial* mjm_addMaterial(mjmModel* model, mjmDefault* defspec) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCMaterial* material = modelC->AddMaterial(def);
  return &material->spec;
}



// add pair to model
mjmPair* mjm_addPair(mjmModel* model, mjmDefault* defspec) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCPair* pair = modelC->AddPair(def);
  return &pair->spec;
}



// add pair exclusion to model
mjmExclude* mjm_addExclude(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCBodyPair* bodypair = modelC->AddExclude();
  return &bodypair->spec;
}



// add equality to model
mjmEquality* mjm_addEquality(mjmModel* model, mjmDefault* defspec) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCEquality* equality = modelC->AddEquality(def);
  return &equality->spec;
}



// add tendon to model
mjmTendon* mjm_addTendon(mjmModel* model, mjmDefault* defspec) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCTendon* tendon = modelC->AddTendon(def);
  return &tendon->spec;
}



// wrap site using tendon
MJAPI mjmWrap* mjm_wrapSite(mjmTendon* tendonspec, const char* name) {
  mjCTendon* tendon = reinterpret_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapSite(name);
  return &tendon->path.back()->spec;
}



// wrap geom using tendon
mjmWrap* mjm_wrapGeom(mjmTendon* tendonspec, const char* name, const char* sidesite) {
  mjCTendon* tendon = reinterpret_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapGeom(name, sidesite);
  return &tendon->path.back()->spec;
}



// wrap joint using tendon
mjmWrap* mjm_wrapJoint(mjmTendon* tendonspec, const char* name, double coef) {
  mjCTendon* tendon = reinterpret_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapJoint(name, coef);
  return &tendon->path.back()->spec;
}



// wrap pulley using tendon
mjmWrap* mjm_wrapPulley(mjmTendon* tendonspec, double divisor) {
  mjCTendon* tendon = reinterpret_cast<mjCTendon*>(tendonspec->element);
  tendon->WrapPulley(divisor);
  return &tendon->path.back()->spec;
}



// add actuator to model
mjmActuator* mjm_addActuator(mjmModel* model, mjmDefault* defspec) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec->element) : 0;
  mjCActuator* actuator = modelC->AddActuator(def);
  return &actuator->spec;
}



// add sensor to model
mjmSensor* mjm_addSensor(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCSensor* sensor = modelC->AddSensor();
  return &sensor->spec;
}



// add numeric to model
mjmNumeric* mjm_addNumeric(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCNumeric* numeric = modelC->AddNumeric();
  return &numeric->spec;
}



// add text to model
mjmText* mjm_addText(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCText* text = modelC->AddText();
  return &text->spec;
}



// add tuple to model
mjmTuple* mjm_addTuple(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCTuple* tuple = modelC->AddTuple();
  return &tuple->spec;
}



// add keyframe to model
mjmKey* mjm_addKey(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCKey* key = modelC->AddKey();
  return &key->spec;
}



// add plugin to model
mjmPlugin* mjm_addPlugin(mjmModel* model) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCPlugin* plugin = modelC->AddPlugin();
  plugin->spec.instance = (mjElement)plugin;
  return &plugin->spec;
}



// add default to model
mjmDefault* mjm_addDefault(mjmModel* model, const char* classname, int parentid) {
  mjCModel* modelC = reinterpret_cast<mjCModel*>(model->element);
  mjCDef* def = modelC->AddDef(classname, parentid);
  if (def) {
    return &def->spec;
  } else {
    return nullptr;
  }
}



// get objects
mjmModel* mjm_getModel(mjmBody* bodyspec) {
  return &(reinterpret_cast<mjCBody*>(bodyspec->element)->model->spec);
}



// get default
mjmDefault* mjm_getDefault(mjElement element) {
  return &(reinterpret_cast<mjCBase*>(element)->def->spec);
}



// find body in model by name
mjmBody* mjm_findBody(mjmModel* modelspec, const char* name) {
  mjCModel* model = reinterpret_cast<mjCModel*>(modelspec->element);
  mjCBase* body = model->FindObject(mjOBJ_BODY, std::string(name));
  if (!body) {
    return 0;
  }
  return &(static_cast<mjCBody*>(body)->spec);
}



// find child of a body by name
mjmBody* mjm_findChild(mjmBody* bodyspec, const char* name) {
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCBase* child = body->FindObject(mjOBJ_BODY, std::string(name));
  if (!child) {
    return 0;
  }
  return &(static_cast<mjCBody*>(child)->spec);
}



// find mesh by name
mjmMesh* mjm_findMesh(mjmModel* modelspec, const char* name) {
  mjCModel* model = reinterpret_cast<mjCModel*>(modelspec->element);
  mjCMesh* mesh = (mjCMesh*)model->FindObject(mjOBJ_MESH, std::string(name));
  if (!mesh) {
    return nullptr;
  }
  return &(static_cast<mjCMesh*>(mesh)->spec);
}



// set frame
void mjm_setFrame(mjElement dest, mjmFrame* frame) {
  if (!frame) {
    return;
  }
  mjCFrame* frameC = reinterpret_cast<mjCFrame*>(frame->element);
  mjCBase* baseC = reinterpret_cast<mjCBase*>(dest);
  baseC->SetFrame(frameC);
}



// get id
int mjm_getId(mjElement element) {
  return reinterpret_cast<mjCBase*>(element)->id;
}



// set default
void mjm_setDefault(mjElement element, mjmDefault* defspec) {
  mjCBase* baseC = reinterpret_cast<mjCBase*>(element);
  baseC->def = reinterpret_cast<mjCDef*>(defspec->element);
}



// set string
void mjm_setString(mjString dest, const char* text) {
  std::string* str = reinterpret_cast<std::string*>(dest);
  *str = std::string(text);
}



// Set specific entry in destination string vector.
mjtByte mjm_setInStringVec(mjStringVec dest, int i, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  if (v->size() <= i) {
    mju_error("Requested index in mjm_setInStringVec is out of bounds");
    return 0;
  }
  v->at(i) = std::string(text);
  return 1;
}



// split text and copy into string array
void mjm_setStringVec(mjStringVec dest, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  mjXUtil::String2Vector(text, *v);
}



// add text entry to destination string vector
void mjm_appendString(mjStringVec dest, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  v->push_back(std::string(text));
}


// copy int array to vector
void mjm_setInt(mjIntVec dest, const int* array, int size) {
  std::vector<int>* v = reinterpret_cast<std::vector<int>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// append int array to vector of arrays
void mjm_appendIntVec(mjIntVecVec dest, const int* array, int size) {
  std::vector<std::vector<int>>* v = reinterpret_cast<std::vector<std::vector<int>>*>(dest);
  v->push_back(std::vector<int>(array, array + size));
}



// copy float array to vector
void mjm_setFloat(mjFloatVec dest, const float* array, int size) {
  std::vector<float>* v = reinterpret_cast<std::vector<float>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}




// append float array to vector of arrays
void mjm_appendFloatVec(mjFloatVecVec dest, const float* array, int size) {
  std::vector<std::vector<float>>* v = reinterpret_cast<std::vector<std::vector<float>>*>(dest);
  v->push_back(std::vector<float>(array, array + size));
}



// copy double array to vector
void mjm_setDouble(mjDoubleVec dest, const double* array, int size) {
  std::vector<double>* v = reinterpret_cast<std::vector<double>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// get string
const char* mjm_getString(const mjString source) {
  std::string* str = reinterpret_cast<std::string*>(source);
  return str->c_str();
}



// get double array
const double* mjm_getDouble(const mjDoubleVec source, int* size) {
  std::vector<double>* v = reinterpret_cast<std::vector<double>*>(source);
  if (size) {
    *size = v->size();
  }
  return v->data();
}



// set plugin attributes
void mjm_setPluginAttributes(mjmPlugin* plugin, void* attributes) {
  mjCPlugin* pluginC = reinterpret_cast<mjCPlugin*>(plugin->instance);
  std::map<std::string, std::string, std::less<>>* config_attribs =
      reinterpret_cast<std::map<std::string, std::string, std::less<>>*>(attributes);
  pluginC->config_attribs = std::move(*config_attribs);
}



// compute full inertia
const char* mjm_setFullInertia(mjmBody* bodyspec, double quat[4], double inertia[3]) {
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  return body->FullInertia(quat, inertia);
}


