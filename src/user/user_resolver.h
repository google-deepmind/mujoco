// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_USER_USER_RESOLVER_H_
#define MUJOCO_SRC_USER_USER_RESOLVER_H_

#include <string>

#include <mujoco/mjspec.h>

namespace mujoco {

// Resolves global attribute conflicts between parent and child specs.
// Returns true on success (no errors). If errors are found, mutations are not
// applied, and 'error_msg' will contain the summary. If warnings are generated
// during resolution, 'warning_subject' and 'warning_body' will be populated.
bool ResolveConflicts(mjSpec* parent, const mjSpec* child, mjtConflict mode,
                      std::string* error_msg, std::string* warning_subject,
                      std::string* warning_body);

}  // namespace mujoco

#endif  // MUJOCO_SRC_USER_USER_RESOLVER_H_
