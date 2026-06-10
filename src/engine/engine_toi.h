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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_TOI_H_
#define MUJOCO_SRC_ENGINE_ENGINE_TOI_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_gjk.h"

#ifdef __cplusplus
extern "C" {
#endif

// compute time of impact between two convex geoms within [0, horizon], assuming constant
// world-frame velocities taken from d:
//   x_c(t) = x_c(0) + t*v,   R(t) = exp(hat(w)*t) * R(0)
// return TOI in [0, horizon]; 0 if already touching/penetrating; -1 if no impact
// on impact, fromto holds witness points on geom1/geom2 at the impact poses
// supported geom types: sphere, capsule, ellipsoid, cylinder, box, mesh
// requires up-to-date poses and velocities (call after mj_forward)
// Nullable: fromto
MJAPI mjtNum mj_geomTOI(const mjModel* m, mjData* d, int geom1, int geom2, mjtNum horizon,
                        mjtNum fromto[6]);

// conservative-advancement core: obj poses (pos/mat) hold the t = 0 poses on entry and are
// mutated to the final-iterate poses on return; vel = [rot(3); lin(3)] in the world frame at
// the object center (mj_objectVelocity convention with flg_local=0); rbound = bounding-sphere
// radius about the object center; the motion model is rotation about the moving object center
// plus linear center motion (not a constant spatial twist); config->buffer must be allocated
// via mjc_ccdSize and config->dist_cutoff is managed internally
// return TOI in [0, horizon] or -1 if no impact
MJAPI mjtNum mjc_toi(const mjCCDConfig* config, mjCCDStatus* status,
                     mjCCDObj* obj1, const mjtNum vel1[6], mjtNum rbound1,
                     mjCCDObj* obj2, const mjtNum vel2[6], mjtNum rbound2,
                     mjtNum horizon, mjtNum tolerance);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_TOI_H_
