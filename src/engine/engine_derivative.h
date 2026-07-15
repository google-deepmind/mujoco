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

// 6x6 block B = d qfrc_bias / d qvel for the free joint of a standalone body
MJAPI void mjd_freeBias_vel(const mjModel* m, const mjData* d, int jnt,
                            mjtNum B[36]);

// 6x6 block A = M - h * (d qfrc_smooth / d qvel) for the free joint of a standalone body
//   returns 1 and writes A if jnt is the free joint of a standalone awake body, 0 otherwise
//   requires valid d->qDeriv rows for the block, computed with flg_bias = 0
MJAPI int mjd_freeMhat(const mjModel* m, const mjData* d, int jnt, mjtNum h, mjtNum A[36]);

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

// compute res += scale * K_stretch * vec for standard (non-interp) flex stretch,
// K_stretch the Gauss-Newton Hessian of the passive stretch force at the current state
//   scale = s1 + s2 * flex_damping[f]  per flex
MJAPI void mjd_flexStretch_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                               mjtNum s1, mjtNum s2);

// assemble the standard-flex implicit stiffness (s1 + s2*damping)*(K_bend + K_stretch) into
// dof-level CSR; phase 1 (colind==NULL) fills rownnz/rowadr and returns total nnz, phase 2
// fills colind/val. Interp flexes are assembled iff Krot (mjd_flexInterp_cacheKrot cache) is
// non-NULL and the centered fast path applies (check mjd_flexInterpAssemblable first).
MJAPI int mjd_flexStiff_assemble(const mjModel* m, mjData* d, int* rownnz, int* rowadr,
                                 int* colind, mjtNum* val, mjtNum s1, mjtNum s2,
                                 int flg_bend, int flg_stretch, const mjtNum* Krot);

// can all interp flexes be assembled to dof-level CSR? (centered fast path everywhere)
MJAPI mjtBool mjd_flexInterpAssemblable(const mjModel* m);

// does any flex contribute assemblable implicit stiffness? (existence check)
MJAPI mjtBool mjd_flexStiff_any(const mjModel* m, int flg_interp);

// implicit effective metric Mtilde = M + (h^2+h*d)*K: per-step arena object (see mjdata.h efm_*)
// build (or deactivate, active==0); the gate decision belongs to the caller
MJAPI void mjd_effBuild(const mjModel* m, mjData* d, int active, int flg_factor);

// refresh the metric's smooth-force shift c = h*K*qvel (values only, velocity stage)
MJAPI void mjd_effShift(const mjModel* m, mjData* d);

// res += B*vec (the stiffness part of the metric; caller supplies the M part)
MJAPI void mjd_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec);

// x = (M + B)^-1 b to 1e-10 relative; x = M^-1 b when the metric is inactive
MJAPI void mjd_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
