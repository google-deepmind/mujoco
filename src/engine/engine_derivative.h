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

// derivatives of mju_subQuat w.r.t inputs
MJAPI void mjd_subQuat(const mjtNum qa[4], const mjtNum qb[4], mjtNum Da[9], mjtNum Db[9]);

// derivatives of mju_quatIntegrate w.r.t inputs
MJAPI void mjd_quatIntegrate(const mjtNum vel[3], mjtNum scale,
                             mjtNum Dquat[9], mjtNum Dvel[9], mjtNum Dscale[3]);

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - [qfrc_bias]) / d qvel
MJAPI void mjd_smooth_vel(const mjModel* m, mjData* d, int flg_bias);

// add (d qfrc_actuator / d qvel) to qDeriv
MJAPI void mjd_actuator_vel(const mjModel* m, mjData* d);

// add (d qfrc_passive / d qvel) to qDeriv
MJAPI void mjd_passive_vel(const mjModel* m, mjData* d);

// subtract (d qfrc_bias / d qvel) from qDeriv (dense version)
MJAPI void mjd_rne_vel_dense(const mjModel* m, mjData* d);

// compute res += (s1 + s2*damping) * J'*K*J * vec, for all interpolated flexes
//   K_rot_cache: if non-NULL, use pre-cached K_rot (same layout as m->flex_stiffness)
MJAPI void mjd_flexInterp_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                              mjtNum s1, mjtNum s2, const mjtNum* K_rot_cache);

// precompute unscaled K_rot for all elements into cache (same layout as m->flex_stiffness)
MJAPI void mjd_flexInterp_cacheKrot(const mjModel* m, mjData* d, mjtNum* K_rot_out);

// compute res += scale * K_bend * vec for standard (non-interp) flex bending
//   scale = s1 + s2 * flex_damping[f]  per flex
MJAPI void mjd_flexBend_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                            mjtNum s1, mjtNum s2);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
