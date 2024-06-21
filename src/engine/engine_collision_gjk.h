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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjtnum.h>
#include "engine/engine_collision_convex.h"

#ifdef __cplusplus
extern "C" {
#endif

// internal struct with settings for GJK
struct _mjGjkConfig {
  mjtNum max_iterations;
  mjtNum tolerance;
};
typedef struct _mjGjkConfig mjGjkConfig;

// Returns the distance between the two objects given an initial guess x0.
MJAPI mjtNum mj_gjk(const mjGjkConfig* config, mjtCCObj* obj1, mjtCCObj* obj2, const mjtNum x0[3]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
