// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_PYTHON_PRIVATE_H_
#define MUJOCO_PYTHON_PRIVATE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>

// DO NOT USE THESE FUNCTIONS ELSEWHERE.
// They should be regarded as part of MuJoCo's internal implementation detail.
extern "C" {
MJAPI void _mjPRIVATE__set_tls_error_fn(void (*h)(const char*));
MJAPI void* mj_arenaAllocByte(mjData* d, int bytes, int alignment);
}

#endif  // MUJOCO_PYTHON_PRIVATE_H_
