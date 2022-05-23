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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_INVERSE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_INVERSE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// inverse dynamics
MJAPI void mj_inverse(const mjModel* m, mjData* d);

// Inverse dynamics with skip; skipstage is mjtStage.
MJAPI void mj_inverseSkip(const mjModel* m, mjData* d,
                          int skipstage, int skipsensor);

// position-dependent computations
MJAPI void mj_invPosition(const mjModel* m, mjData* d);

// velocity-dependent computations
MJAPI void mj_invVelocity(const mjModel* m, mjData* d);

// inverse constraint solver
MJAPI void mj_invConstraint(const mjModel* m, mjData* d);

// compare forward and inverse dynamics, without changing results of forward dynamics
MJAPI void mj_compareFwdInv(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_INVERSE_H_
