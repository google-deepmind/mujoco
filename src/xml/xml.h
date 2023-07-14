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

#ifndef MUJOCO_SRC_XML_XML_H_
#define MUJOCO_SRC_XML_XML_H_

#include <string>

#include <mujoco/mjmodel.h>
#include "user/user_model.h"

// Top level API

// Main writer function
std::string mjWriteXML(mjCModel* model, char* error, int error_sz);

// Main parser function
mjCModel* mjParseXML(const char* filename, int vfs_provider, char* error, int error_sz);


#endif  // MUJOCO_SRC_XML_XML_H_
