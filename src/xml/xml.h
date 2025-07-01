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
#include <string_view>

#include <mujoco/mjmodel.h>
#include <mujoco/mjspec.h>

// Main parser function
mjSpec* ParseXML(const char* filename, const mjVFS* vfs, char* error, int nerror);

// Returns a newly-allocated mjSpec, loaded from the contents of xml.
// On failure returns nullptr and populates the error array if present.
mjSpec* ParseSpecFromString(std::string_view xml, const mjVFS* vfs = nullptr,
                            char* error = nullptr, int nerror = 0);

// Main writer function
std::string WriteXML(const mjModel* m, mjSpec* spec, char* error, int nerror);

#endif  // MUJOCO_SRC_XML_XML_H_
