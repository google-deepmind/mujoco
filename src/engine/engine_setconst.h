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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SETCONST_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SETCONST_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// Set constant fields of mjModel, corresponding to qpos0 configuration.
MJAPI void mj_setConst(mjModel* m, mjData* d);

// Set actuator_lengthrange for specified actuator; return 1 if ok, 0 if error.
MJAPI int mj_setLengthRange(mjModel* m, mjData* d, int index,
                            const mjLROpt* opt, char* error, int error_sz);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SETCONST_H_
