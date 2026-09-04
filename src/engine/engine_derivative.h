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
MJAPI int mjd_freeMhat(const mjModel* m, const mjData* d, int jnt, mjtNum h, mjtNum A[36],
                       int flg_discrete);

// can this standalone free joint take the local gyroscopic treatment under discrete
int mjd_freeGyroPossible(const mjModel* m, const mjData* d, int jnt);

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
// The flex-contact law, shared by the passive penalty and the IPC contact mode. A pair with
// normal row `row` and stiffness `scale` costs
//
//   0.5 * scale * d^2,   d = gap - s - lam/k,   s = max(0, gap - lam/k)
//
// giving a force -scale*d along the row and a curvature scale*row'row in the metric. `gap` is the
// pair's signed distance in the producer's own convention (surface distance for the penalty,
// midsurface distance less the standoff for IPC), `lam` the augmented-Lagrangian multiplier and
// `k` the stiffness it is defined against. The passive penalty is the case lam = 0, scale = k.
// The slack is frozen across an inner solve: computed once per outer iteration, passed back in.
// Convention: scale*row'row must equal h^2 * k * J'J, however the producer splits the factors.
MJAPI mjtNum mjd_flexContactSlack(mjtNum k, mjtNum gap, mjtNum lam);
MJAPI mjtNum mjd_flexContactResidual(mjtNum k, mjtNum gap, mjtNum s, mjtNum lam);

// A published contact row in mjData.efm_con_ind / efm_con_val is a two-slot header and the entries:
//
//   ind = [nnz, conid, colind...]      val = [scale, force, val...]
//
// scale is the curvature the metric applies, force the pair's force along the row, conid the
// contact it came from or -1. Apply the published forces: res += force * row over the rows.
MJAPI void mjd_effContactForce(const mjData* d, mjtNum* res);

// natural frequency of the law: pair stiffness = mjFLEXCONTACT_OMEGA2 * min nonzero vertex mass
#define mjFLEXCONTACT_OMEGA2 5e7

// point mass of global flex vertex gv, 0 when it has no 3-dof body of its own (pinned)
MJAPI mjtNum mjd_flexVertMass(const mjModel* m, const mjData* d, int gv);

// passive contact stiffness of a pair; force and Hessian must use the same value
MJAPI mjtNum mjd_flexContactStiffness(const mjModel* m, const mjData* d, const mjContact* con);

MJAPI int mjd_flexStiff_assemble(const mjModel* m, mjData* d, int* rownnz, int* rowadr,
                                 int* colind, mjtNum* val, mjtNum s1, mjtNum s2,
                                 int flg_bend, int flg_stretch, const mjtNum* Krot);

// can all interp flexes be assembled to dof-level CSR? (centered fast path everywhere)
MJAPI mjtBool mjd_flexInterpAssemblable(const mjModel* m);

// does any flex contribute assemblable implicit stiffness? (existence check)
MJAPI mjtBool mjd_flexStiff_any(const mjModel* m, int flg_interp);

// actuation-stage refresh of the metric: actuator gains, their shift, the backbone factor
void mjd_effActuation(const mjModel* m, mjData* d);

// one rank-1 term of the metric: term = scale * val' * val over the sparse row
typedef struct {
  const mjtNum* val;   // sparse row values
  const int* colind;   // sparse row column indices
  int nnz;             // row nonzeros
  mjtNum scale;        // rank-1 scale
} mjEffRank1;

// iteration cursor over the metric's rank-1 producers (internal layout)
typedef struct {
  int cls;             // producer class
  int i;               // index within class
  int k;               // sub-row within index (multi-output actuators)
} mjEffRank1Iter;

// yield the next live rank-1 term of the metric; init the cursor to {0}, returns 0 when
// exhausted. Producers: tendons, then actuator output rows; zero entries are skipped.
// A new metric class becomes a new case here, invisible to every consumer.
// flg_contact selects the passive-contact class, for callers that account for contact separately
int mjd_effRank1Next(const mjModel* m, const mjData* d, mjEffRank1Iter* it,
                     mjEffRank1* e, int flg_contact);

// island-local metric product res += S*vec, vectors in island-local dof coordinates
void mjd_effMulAddIsland(const mjModel* m, const mjData* d, mjtNum* res,
                         const mjtNum* vec, int island);

// implicit effective metric Mtilde = M + (h^2+h*d)*K: per-step arena object (see mjdata.h efm_*)
// build (or deactivate, active==0); the gate decision belongs to the caller
MJAPI void mjd_effBuild(const mjModel* m, mjData* d, int active, int flg_factor);

// refresh the metric's smooth-force shift c = h*K*qvel (values only, velocity stage)
MJAPI void mjd_effShift(const mjModel* m, mjData* d);

// res += B*vec (the stiffness part of the metric; caller supplies the M part).
// flg_contact selects the passive-contact class, for callers that account for contact
// energy separately
MJAPI void mjd_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                         int flg_contact);

// solve (M + B) x = b by PCG preconditioned with mjd_effPrec, to opt.tolerance on the relative
// residual; x = M^-1 b when the metric is inactive. Warns (mjWARN_INERTIA) if the iteration cap
// is reached before convergence, in which case x is returned under-converged.
MJAPI void mjd_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b);

// apply the metric preconditioner: x ~= (M + B)^-1 b, a cheap fixed linear operator, NOT a solve.
// Exact only when the metric is inactive (x = M^-1 b); otherwise approximate by construction.
MJAPI void mjd_effPrec(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b);

// fold the rank-1 classes and the efc rows (quadratic zone) into a copy of the preconditioner
// blocks L (9*nefmdof), factored; returns 0 if nothing is covered, leaving L untouched
MJAPI int mjd_effPrecFold(const mjModel* m, mjData* d, mjtNum* L,
                          int nefc, const int* efc_state, const mjtNum* efc_D, int is_sparse,
                          const mjtNum* J, const int* J_rownnz, const int* J_rowadr,
                          const int* J_colind);

// apply the metric preconditioner using caller-supplied factored blocks
MJAPI void mjd_effPrecBlocks(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b,
                             const mjtNum* L);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_DERIVATIVE_H_
