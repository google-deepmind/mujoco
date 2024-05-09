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

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include "user/user_api.h"

// Top level API

// Main writer function
std::string mjWriteXML(mjSpec* spec, char* error, int error_sz);

// Main parser function
MJAPI mjSpec* mjParseXML(const char* filename, const mjVFS* vfs, char* error, int error_sz);

// Returns a newly-allocated mjSpec, loaded from the contents of xml.
// On failure returns nullptr and populates the error array if present.
MJAPI mjSpec* ParseSpecFromString(std::string_view xml, char* error = nullptr,
                                  int error_size = 0, mjVFS* vfs = nullptr);


#endif  // MUJOCO_SRC_XML_XML_H_
