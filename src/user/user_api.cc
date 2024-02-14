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
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "user/user_model.h"
#include "user/user_objects.h"
#include "xml/xml_util.h"



// create model
void* mjm_createModel() {
  mjCModel* modelC = new mjCModel();
  return modelC;
}



// delete model
void mjm_deleteModel(void* modelspec) {
  mjCModel* model = static_cast<mjCModel*>(modelspec);
  delete model;
}



// copy spec into private attributes
MJAPI void mjm_finalize(mjElement object) {
  mjCBase* baseC = reinterpret_cast<mjCBase*>(object);
  baseC->CopyFromSpec();
}



// add child body to body, return child spec
mjmBody* mjm_addBody(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element)->AddBody(def);
  return &body->spec;
}



// add site to body, return site spec
mjmSite* mjm_addSite(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCSite* site = body->AddSite(def);
  return &site->spec;
}



// add joint to body
mjmJoint* mjm_addJoint(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
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
mjmGeom* mjm_addGeom(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCGeom* geom = body->AddGeom(def);
  return &geom->spec;
}



// add camera to body
mjmCamera* mjm_addCamera(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCCamera* camera = body->AddCamera(def);
  return &camera->spec;
}



// add light to body
mjmLight* mjm_addLight(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCLight* light = body->AddLight(def);
  return &light->spec;
}



// add flex to model
mjmFlex* mjm_addFlex(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
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
mjmMesh* mjm_addMesh(void* model, void* defspec) {
  mjCDef* def = defspec ? reinterpret_cast<mjCDef*>(defspec) : 0;
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCMesh* mesh = modelC->AddMesh(def);
  return &mesh->spec;
}



// add height field to model
mjmHField* mjm_addHField(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCHField* heightField = modelC->AddHField();
  return &heightField->spec;
}



// add texture to model
mjmTexture* mjm_addTexture(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCTexture* texture = modelC->AddTexture();
  return &texture->spec;
}



// add material to model
mjmMaterial* mjm_addMaterial(void* model, void* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCMaterial* material = modelC->AddMaterial(def);
  return &material->spec;
}



// add pair to model
mjmPair* mjm_addPair(void* model, void* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCPair* pair = modelC->AddPair(def);
  return &pair->spec;
}



// add equality to model
mjmEquality* mjm_addEquality(void* model, void* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCEquality* equality = modelC->AddEquality(def);
  return &equality->spec;
}



// add tendon to model
mjmTendon* mjm_addTendon(void* model, void* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCDef* def = static_cast<mjCDef*>(defspec);
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
mjmActuator* mjm_addActuator(void* model, void* defspec) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCActuator* actuator = modelC->AddActuator(def);
  return &actuator->spec;
}



// add sensor to model
mjmSensor* mjm_addSensor(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCSensor* sensor = modelC->AddSensor();
  return &sensor->spec;
}



// add numeric to model
mjmNumeric* mjm_addNumeric(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCNumeric* numeric = modelC->AddNumeric();
  return &numeric->spec;
}



// add text to model
mjmText* mjm_addText(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCText* text = modelC->AddText();
  return &text->spec;
}



// add tuple to model
mjmTuple* mjm_addTuple(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCTuple* tuple = modelC->AddTuple();
  return &tuple->spec;
}



// add keyframe to model
mjmKey* mjm_addKey(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCKey* key = modelC->AddKey();
  return &key->spec;
}



// Add plugin to model.
mjElement mjm_addPlugin(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  mjCPlugin* plugin = modelC->AddPlugin();
  return (mjElement)plugin;
}



// get objects
void* mjm_getModel(mjmBody* bodyspec) {
  return reinterpret_cast<mjCBody*>(bodyspec->element)->model;
}



// get default
void* mjm_getDefault(mjElement element) {
  return reinterpret_cast<mjCBase*>(element)->def;
}



// find body in model by name
mjmBody* mjm_findBody(void* modelspec, const char* name) {
  mjCModel* model = static_cast<mjCModel*>(modelspec);
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
void mjm_setDefault(mjElement element, void* defspec) {
  mjCBase* baseC = reinterpret_cast<mjCBase*>(element);
  baseC->def = static_cast<mjCDef*>(defspec);
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
void mjm_addToStringVec(mjStringVec dest, const char* text) {
  std::vector<std::string>* v = reinterpret_cast<std::vector<std::string>*>(dest);
  v->push_back(std::string(text));
}


// set int array
void mjm_setInt(mjIntVec dest, const int* array, int size) {
  std::vector<int>* v = reinterpret_cast<std::vector<int>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// set float array
void mjm_setFloat(mjFloatVec dest, const float* array, int size) {
  std::vector<float>* v = reinterpret_cast<std::vector<float>*>(dest);
  v->assign(size, 0.0);
  for (int i = 0; i < size; ++i) {
    (*v)[i] = array[i];
  }
}



// set double array
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



// compute full inertia
const char* mjm_setFullInertia(mjmBody* bodyspec, double quat[4], double inertia[3]) {
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  return body->FullInertia(quat, inertia);
}


