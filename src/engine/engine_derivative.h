// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - qfrc_bias) / d qvel
MJAPI void mjd_smooth_vel(const mjModel* m, mjData* d);

// centered finite difference approximation to mjd_smooth_vel
MJAPI void mjd_smooth_velFD(const mjModel* m, mjData* d, mjtNum eps);

// add (d qfrc_passive / d qvel) to DfDv
MJAPI void mjd_passive_vel(const mjModel* m, mjData* d, mjtNum* DfDv);

// add forward finite difference approximation of (d qfrc_passive / d qvel) to DfDv
MJAPI void mjd_passive_velFD(const mjModel* m, mjData* d, mjtNum eps, mjtNum* DfDv);

// advance simulation using control callback, skipstage is mjtStage
MJAPI void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);

// finite differenced transition matrices (control theory notation)
MJAPI void mjd_transitionFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte centered,
                            mjtNum* A, mjtNum* B, mjtNum* C, mjtNum* D);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
