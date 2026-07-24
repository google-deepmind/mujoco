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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SOLVER_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SOLVER_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

//------------------------------ monolithic solvers ------------------------------------------------

// PGS solver
void mj_solPGS(const mjModel* m, mjData* d, int maxiter);

// No Slip solver (modified PGS)
void mj_solNoSlip(const mjModel* m, mjData* d, int maxiter);

// CG solver
void mj_solCG(const mjModel* m, mjData* d, int maxiter);

// Newton solver
void mj_solNewton(const mjModel* m, mjData* d, int maxiter);


//------------------------------ per-island solvers ------------------------------------------------

// PGS solver (one island, no dualFinish — caller handles it)
void mj_solPGS_island(const mjModel* m, mjData* d, int island, int maxiter);

// NoSlip solver (one island, no dualFinish — caller handles it)
void mj_solNoSlip_island(const mjModel* m, mjData* d, int island, int maxiter);

// CG solver
void mj_solCG_island(const mjModel* m, mjData* d, int island, int maxiter);

// Newton entry point
void mj_solNewton_island(const mjModel* m, mjData* d, int island, int maxiter);

// map efc_force to joint space (used after dual island dispatch)
void mj_dualFinish(const mjModel* m, mjData* d);


//------------------------------ extra primal term (IPC contact add-on) ----------------------------

#ifdef __cplusplus
extern "C" {
#endif

// A set of extra convex penalty rows added to the primal objective, bypassing the efc / impedance /
// arena machinery. Row t contributes cost 0.5*D[t]*r^2 to the primal objective minimized over qacc,
// with residual r = (sum_k val[k]*qacc[colind[k]]) - ref[t] over its nonzeros. onesided[t]!=0 makes it
// a one-sided contact penalty (active only when r < 0); else it is a two-sided (equality) penalty.
// Purpose: inject the barrier-free-AL flex-flex contact energy into MuJoCo's CG solver as an add-on,
// instead of replicating MuJoCo's constraint model in a separate integrator. NOT thread-safe (a single
// file-scope pointer); set it immediately before a monolithic mj_solCG call and clear (NULL) after.
// The island solver path ignores it (mj_flexCG models force the monolithic path).
typedef struct mjExtraPrimal_ {
  int nrow;              // number of extra rows
  const int* rownnz;     // (nrow)  nonzeros per row
  const int* rowadr;     // (nrow)  offset of each row into colind/val
  const int* colind;     // (nnz)   dof index of each nonzero
  const mjtNum* val;     // (nnz)   d(residual)/d(qacc) for each nonzero
  const mjtNum* ref;     // (nrow)  residual offset
  const mjtNum* D;       // (nrow)  penalty stiffness
  const int* onesided;   // (nrow)  1: active iff residual < 0; 0: always active
} mjExtraPrimal;

// register (or clear, with NULL) the extra primal term consumed by the next monolithic primal solve
MJAPI void mj_setExtraPrimal(const mjExtraPrimal* extra);

// number of active extra-primal rows (0 if none): lets mj_fwdConstraint solve when nefc==0
MJAPI int mj_extraPrimalRows(void);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SOLVER_H_
