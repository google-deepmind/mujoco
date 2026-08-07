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

// return 1 if body is a standalone free body: a free joint with no children
mjtBool mj_isFreeBody(const mjModel* m, int body);

// 6x6 block B = d qfrc_bias / d qvel for the free joint of a standalone body
MJAPI void mjd_freeBias_vel(const mjModel* m, const mjData* d, int jnt, mjtNum B[36]);

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
// Passive contact stiffness (omega^2 * m_min); force and Hessian must use the same value.
// res += scale * K_contact * vec (shift counterpart of the contact stiffness in the metric).
MJAPI void mjd_flexContact_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                               mjtNum scale);

MJAPI mjtNum mjd_flexContactStiffness(const mjModel* m, const mjData* d, const mjContact* con);

// build the passive flex contact pair list and publish it in d->efm_contact (position stage, after
// mj_makeConstraint has marked the passive contacts and before the metric is sized against them)
MJAPI void mjd_flexContactPairs(const mjModel* m, mjData* d);

MJAPI int mjd_flexStiff_assemble(const mjModel* m, mjData* d, int* rownnz, int* rowadr,
                                 int* colind, mjtNum* val, mjtNum s1, mjtNum s2,
                                 int flg_bend, int flg_stretch, int flg_contact, const mjtNum* Krot);

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
// Contact expressed as a FORCE plus a STIFFNESS, in the one form every consumer of the metric
// reads. Pair c couples npt[c] participating vertices, each occupying one contiguous dof triple
// starting at base[]; w[] is that vertex's weight vector. The pair costs
//   0.5*D[c]*(sum_p w_p . x[base_p] - ref[c])^2
// split into the two pieces the solvers consume: a constant force D[c]*ref[c]*w_p summed into f,
// and a stiffness D[c]*w_p w_q^T.
//
// CONVENTION: D and w are in the metric's units -- D*w w^T is the contribution to the EFFECTIVE
// stiffness h^2*K, not to K. Producers scale accordingly (passive flex contact folds the h^2 into
// D; the IPC integrator carries it as D=k/h^2 with w=h^2*J, which is the same product). Get this
// wrong and the contact is off by h^2 with no test failing loudly.
//
// Consumers, and the ordering that keeps them from double-counting: a producer publishing BEFORE
// mjd_effBuild is folded into the metric CSR and read by mjd_effShift, then cleared -- that is the
// passive flex contact path. A producer publishing DURING the solve is applied matrix-free by
// mj_extraStiffMulAdd -- that is the IPC integrator. Never both for one pair.
// max participating vertices in one pair: the IPC integrator's primitives are point-triangle (1+3)
// and edge-edge (2+2), but a passive contact between two surface ELEMENTS reaches 6. Silently
// exceeding this would drop a pair's tail vertices from base/w.
#define mjNEXTRAPT 8

typedef struct mjExtraStiff_ {
  int npair;             // number of contact pairs
  const int* npt;        // (npair)              participating vertices per pair, <= mjNEXTRAPT
  const int* base;       // (npair*mjNEXTRAPT)   first dof of each participating vertex
  const mjtNum* w;       // (npair*mjNEXTRAPT*3) weight vector per participating vertex
  const mjtNum* D;       // (npair)              stiffness
  const mjtNum* f;       // (nv)                 constant force D*ref*w, summed over pairs
} mjExtraStiff;

// The producer publishes its pair list by pointing d->efm_contact at it for the duration of the
// solve and clearing it afterwards. It lives in mjData rather than in a file-static because two
// threads stepping two mjData share one mjModel but must not share a contact set.

// res += (sum_c D_c J_c^T J_c) * vec, for d's contact stiffness (no-op if none)
MJAPI void mj_extraStiffMulAdd(const mjData* d, mjtNum* res, const mjtNum* vec);

// number of published contact pairs (0 if none): lets mj_fwdConstraint solve when nefc==0
MJAPI int mj_extraPrimalRows(const mjData* d);

// Fold the registered contact stiffness and the efc rows into the metric preconditioner blocks,
// writing the contact-folded factor to L (9*nefmdof). The efc side is passed as a view because it
// lives in the solver's working state; everything else is metric state. Returns 0 if there is
// nothing covered to fold into, in which case L is untouched and the caller should use the shared
// blocks.
MJAPI int mjd_effPrecContact(const mjModel* m, mjData* d, mjtNum* L,
                             int nefc, const int* efc_state, const mjtNum* efc_D, int is_sparse,
                             const mjtNum* J, const int* J_rownnz, const int* J_rowadr,
                             const int* J_colind);

// res += (K + contact)*vec, the metric's non-inertial part. flg_contact selects whether the
// published contact stiffness is included: the linear solves want it, an energy evaluation that
// accounts for contact separately does not.
MJAPI void mjd_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                         int flg_contact);

// solve (M + B) x = b by PCG preconditioned with mjd_effPrec, to opt.tolerance on the relative
// residual; x = M^-1 b when the metric is inactive. Warns (mjWARN_INERTIA) if the iteration cap
// is reached before convergence, in which case x is returned under-converged.
MJAPI void mjd_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b);

// apply the metric preconditioner: x ~= (M + B)^-1 b, a cheap fixed linear operator, NOT a solve.
// Exact only when the metric is inactive (x = M^-1 b); otherwise approximate by construction.
MJAPI void mjd_effPrec(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b);

// build+factor the metric blocks with an extra per-covered-vertex 3x3 term folded in (9*nefmdof)
MJAPI void mjd_effPrecFold(const mjModel* m, mjData* d, const mjtNum* Badd, mjtNum* L);

// apply the metric preconditioner using caller-supplied factored blocks
MJAPI void mjd_effPrecBlocks(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b,
                             const mjtNum* L);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
