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

#ifndef MUJOCO_TEST_COMPARE_MODEL_H_
#define MUJOCO_TEST_COMPARE_MODEL_H_

#include <string>

#include <mujoco/mujoco.h>

namespace mujoco {

// Compares all fields of two mjModels.
// Returns the name of the different field and the max difference.
mjtNum CompareModel(const mjModel* m1, const mjModel* m2, std::string& field);

}  // namespace mujoco

#endif  // MUJOCO_TEST_COMPARE_MODEL_H_
