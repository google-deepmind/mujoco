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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_IPC_H_
#define MUJOCO_SRC_ENGINE_ENGINE_IPC_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// IPC variational integrator (mjINT_IPC): owns the full step. Minimizes the per-step incremental
// potential (inertia; elasticity rides the native edge-equality efc rows -- the solver carries no
// elastic energy, and FEM elasticity is rejected at load) with penetration-free contact by a
// barrier-free augmented-Lagrangian method (no log barrier). Supports flex self-contact
// (vertex-triangle + edge-edge) and flex-vs-geom; falls back to Euler if the model has no 2D flex.
MJAPI void mj_IPC(const mjModel* m, mjData* d);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_IPC_H_
