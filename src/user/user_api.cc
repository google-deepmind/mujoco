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
void mjm_deleteModel(void* model) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  delete modelC;
}



// add body to body
void* mjm_addBody(void* body, void* def) {
  mjCDef* defC = static_cast<mjCDef*>(def);
  mjCBody* bodyC = static_cast<mjCBody*>(body);
  return bodyC->AddBody(defC);
}



// add site to body
mjmSite* mjm_addSite(void* body, void* def) {
  mjCDef* defC = static_cast<mjCDef*>(def);
  mjCSite* siteC = static_cast<mjCBody*>(body)->AddSite(defC);
  return &siteC->spec;
}



// get object of given type
void* mjm_findObject(void* model, mjtObj type, const char* name) {
  mjCModel* modelC = static_cast<mjCModel*>(model);
  return modelC->FindObject(type, std::string(name));
}



// set parent frame of dest
void mjm_setFrame(void* dest, void* frame) {
  mjCFrame* frameC = static_cast<mjCFrame*>(frame);
  mjCBase* baseC = static_cast<mjCBase*>(dest);
  baseC->SetFrame(frameC);
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
