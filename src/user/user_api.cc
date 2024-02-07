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
void* mjm_addJoint(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddJoint(def);
  return joint;
}



// add free joint to body
void* mjm_addFreeJoint(mjmBody* bodyspec) {
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCJoint* joint = body->AddFreeJoint();
  return joint;
}



// add geom to body
void* mjm_addGeom(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCGeom* geom = body->AddGeom(def);
  return geom;
}



// add camera to body
void* mjm_addCamera(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCCamera* camera = body->AddCamera(def);
  return camera;
}



// add light to body
void* mjm_addLight(mjmBody* bodyspec, void* defspec) {
  mjCDef* def = static_cast<mjCDef*>(defspec);
  mjCBody* body = reinterpret_cast<mjCBody*>(bodyspec->element);
  mjCLight* light = body->AddLight(def);
  return light;
}



// add frame to body
void* mjm_addFrame(mjmBody* bodyspec, void* parentframe) {
  mjCFrame* parentframeC = static_cast<mjCFrame*>(parentframe);
  mjCFrame* frameC = reinterpret_cast<mjCBody*>(bodyspec->element)->AddFrame(parentframeC);
  return frameC;
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
void mjm_setFrame(mjElement dest, void* frame) {
  mjCFrame* frameC = static_cast<mjCFrame*>(frame);
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



// set double array
void mjm_setDouble(mjDouble dest, const double* array, int size) {
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
const double* mjm_getDouble(const mjDouble source, int* size) {
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


