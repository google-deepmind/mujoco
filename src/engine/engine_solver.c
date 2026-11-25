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

#include "engine/engine_solver.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjtype.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_inline.h"
#include "engine/engine_memory.h"
#include "engine/engine_metric.h"
#include "engine/engine_simd.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"


//---------------------------------- utility functions ---------------------------------------------

// zero 2D vector
static inline void zero2(mjtNum* restrict res) {
  res[0] = 0;
  res[1] = 0;
}


// copy 2D vector
static inline void copy2(mjtNum* restrict res, const mjtNum* vec) {
  res[0] = vec[0];
  res[1] = vec[1];
}


// 2D vector dot-product
static inline mjtNum dot2(const mjtNum vec1[2], const mjtNum vec2[2]) {
  return vec1[0]*vec2[0] + vec1[1]*vec2[1];
}


// save solver statistics
static void saveStats(const mjModel* m, mjData* d, int island, int iter,
                      mjtNum improvement, mjtNum gradient, mjtNum lineslope,
                      int nactive, int nchange, int neval, int nupdate) {
  // if island out of range, return
  if (island >= mjNISLAND) {
    return;
  }

  // if no islands, use first island
  island = mjMAX(0, island);

  // if iter out of range, return
  if (iter >= mjNSOLVER) {
    return;
  }

  // get mjSolverStat pointer
  mjSolverStat* stat = d->solver + island*mjNSOLVER + iter;

  // save stats
  stat->improvement = improvement;
  stat->gradient = gradient;
  stat->lineslope = lineslope;
  stat->nactive = nactive;
  stat->nchange = nchange;
  stat->neval = neval;
  stat->nupdate = nupdate;
}


// finalize dual solver: map to joint space
static void dualFinish(const mjModel* m, mjData* d) {
  // map constraint force to joint space
  mj_mulJacTVec(m, d, d->qfrc_constraint, d->efc_force);

  // compute constrained acceleration in joint space, in the solve metric: under the
  // effective metric, qacc_smooth and the constraint response both live in Mtilde
  // (mj_effSolve is the plain M solve when the metric is inactive)
  mj_effSolve(m, d, d->qacc, d->qfrc_constraint);
  mju_addTo(d->qacc, d->qacc_smooth, m->nv);
}


// PGS: map efc_force to joint space
void mj_dualFinish(const mjModel* m, mjData* d) {
  dualFinish(m, d);
}


// compute 1/diag(AR)
//   res[c] = 1 / AR[efclist[c], efclist[c]] for c = 0..nefc-1
//   efclist is NULL for monolithic (sequential) iteration
static void ARdiaginv(const mjModel* m, const mjData* d, mjtNum* restrict res,
                      int nefc, const int* restrict efclist, int flg_subR) {
  const mjtNum* restrict AR = d->efc_AR;
  const mjtNum* restrict R = d->efc_R;

  // sparse
  if (mj_isSparse(m)) {
    const int* restrict rowadr = d->efc_AR_rowadr;
    const int* restrict rownnz = d->efc_AR_rownnz;
    const int* restrict colind = d->efc_AR_colind;

    for (int c=0; c < nefc; c++) {
      int i = efclist ? efclist[c] : c;
      int nnz = rownnz[i];
      for (int j=0; j < nnz; j++) {
        int adr = rowadr[i] + j;
        if (i == colind[adr]) {
          res[c] = 1 / (flg_subR ? mju_max(mjMINVAL, AR[adr] - R[i]) : AR[adr]);
          break;
        }
      }
    }
  }

  // dense
  else {
    int d_nefc = d->nefc;  // global nefc
    for (int c=0; c < nefc; c++) {
      int i = efclist ? efclist[c] : c;
      int adr = i * (d_nefc + 1);
      res[c] = 1 / (flg_subR ? mju_max(mjMINVAL, AR[adr] - R[i]) : AR[adr]);
    }
  }
}


// extract diagonal block from AR, clamp diag to 1e-10 if flg_subR
static void extractBlock(const mjModel* m, const mjData* d, mjtNum* Ac,
                         int start, int n, int flg_subR) {
  int nefc = d->nefc;
  const mjtNum *AR = d->efc_AR;

  // sparse
  if (mj_isSparse(m)) {
    const int* rownnz = d->efc_AR_rownnz;
    const int* rowadr = d->efc_AR_rowadr;
    const int* colind = d->efc_AR_colind;
    /*
            // GENERAL CASE
            mju_zero(Ac, n*n);
            for( j=0; j<n; j++ )
                for( k=0; k<rownnz[start+j]; k++ )
                {
                    int col = colind[rowadr[start+j]+k];
                    if( col>=start && col<start+n )
                        Ac[j*n+col-start] = AR[rowadr[start+j]+k];
                }
     */
    // assume full sub-matrix, find starting k: same for all rows
    int k;
    for (k=0; k < rownnz[start]; k++) {
      if (colind[rowadr[start]+k] == start) {
        break;
      }
    }

    // SHOULD NOT OCCUR
    if (k >= rownnz[start]) {
      mjERROR("internal error");
    }

    // copy rows
    for (int j=0; j < n; j++) {
      mju_copy(Ac+j*n, AR+rowadr[start+j]+k, n);
    }
  }

  // dense
  else {
    for (int j=0; j < n; j++) {
      mju_copy(Ac+j*n, AR+start+(start+j)*nefc, n);
    }
  }

  // subtract R from diagonal, clamp to 1e-10 from below
  if (flg_subR) {
    const mjtNum *R = d->efc_R;
    for (int j=0; j < n; j++) {
      Ac[j*(n+1)] -= R[start+j];
      Ac[j*(n+1)] = mju_max(1e-10, Ac[j*(n+1)]);
    }
  }
}


// compute residual for one block
static void residual(const mjModel* m, const mjData* d, mjtNum* res, int i, int dim, int flg_subR) {
  int nefc = d->nefc;

  // sparse
  if (mj_isSparse(m)) {
    for (int j=0; j < dim; j++) {
      res[j] = d->efc_b[i+j] + mju_dotSparse(d->efc_AR + d->efc_AR_rowadr[i+j],
                                             d->efc_force,
                                             d->efc_AR_rownnz[i+j],
                                             d->efc_AR_colind + d->efc_AR_rowadr[i+j]);
    }
  }

  // dense
  else {
    for (int j=0; j < dim; j++) {
      res[j] = d->efc_b[i+j] + mju_dot(d->efc_AR+(i+j)*nefc, d->efc_force, nefc);
    }
  }

  if (flg_subR) {
    for (int j=0; j < dim; j++) {
      res[j] -= d->efc_R[i+j]*d->efc_force[i+j];
    }
  }
}


// compute cost change
static mjtNum costChange(const mjtNum* A, mjtNum* force, const mjtNum* oldforce,
                         const mjtNum* res, int dim) {
  mjtNum change;

  // compute change
  if (dim == 1) {
    mjtNum delta = force[0] - oldforce[0];
    change = 0.5*delta*delta*A[0] + delta*res[0];
  } else if (dim == 2) {
    mjtNum delta[2] = {force[0] - oldforce[0], force[1] - oldforce[1]};
    mjtNum vmv = 0;
    vmv += delta[0] * dot2(A, delta);
    vmv += delta[1] * dot2(A+2, delta);
    change = 0.5*vmv + dot2(delta, res);
  } else if (dim == 3) {
    mjtNum delta[3] = {force[0] - oldforce[0], force[1] - oldforce[1], force[2] - oldforce[2]};
    mjtNum vmv = 0;
    vmv += delta[0] * mju_dot3(A, delta);
    vmv += delta[1] * mju_dot3(A+3, delta);
    vmv += delta[2] * mju_dot3(A+6, delta);
    change = 0.5*vmv + mju_dot3(delta, res);
  } else {
    mjtNum delta[6];
    mju_sub(delta, force, oldforce, dim);
    change = 0.5*mju_mulVecMatVec(delta, A, delta, dim) + mju_dot(delta, res, dim);
  }

  // positive change: restore force
  if (change > 1e-10) {
    if (dim == 1) {
      force[0] = oldforce[0];
    } else if (dim == 2) {
      copy2(force, oldforce);
    } else if (dim == 3) {
      mji_copy3(force, oldforce);
    } else {
      mju_copy(force, oldforce, dim);
    }
    change = 0;
  }

  return change;
}


// PCG32 random number generator state
typedef struct {
  uint64_t state;
  uint64_t inc;
} pcg32_state;


// generate next 32-bit pseudorandom integer
static uint32_t pcg32_next(pcg32_state* rng) {
  uint64_t oldstate = rng->state;
  rng->state = oldstate * 6364136223846793005ULL + (rng->inc | 1);
  uint32_t xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
  uint32_t rot = oldstate >> 59u;
  return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}


// Fisher-Yates shuffle of integer array
static void shuffle_int(int* array, int n, pcg32_state* rng) {
  for (int i = n - 1; i > 0; i--) {
    uint32_t j = pcg32_next(rng) % (i + 1);
    int temp = array[i];
    array[i] = array[j];
    array[j] = temp;
  }
}


// set efc_state to dual constraint state; return nactive
//   iterates over efclist (or sequentially if NULL), classifies by ne/nf ranges
static int dualState(const mjData* d, int* state,
                     int ne, int nf, int nefc, const int* efclist) {
  const mjtNum* force = d->efc_force;
  const mjtNum* floss = d->efc_frictionloss;

  // equality and friction always active
  int nactive = ne + nf;

  // equality
  for (int c=0; c < ne; c++) {
    int i = efclist ? efclist[c] : c;
    state[i] = mjCNSTRSTATE_QUADRATIC;
  }

  // friction
  for (int c=ne; c < ne+nf; c++) {
    int i = efclist ? efclist[c] : c;
    if (force[i] <= -floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARPOS;  // opposite of primal
    } else if (force[i] >= floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARNEG;
    } else {
      state[i] = mjCNSTRSTATE_QUADRATIC;
    }
  }

  // limit and contact
  for (int c=ne+nf; c < nefc; c++) {
    int i = efclist ? efclist[c] : c;

    // non-negative
    if (d->efc_type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
      if (force[i] <= 0) {
        state[i] = mjCNSTRSTATE_SATISFIED;
      } else {
        state[i] = mjCNSTRSTATE_QUADRATIC;
        nactive++;
      }
    }

    // elliptic
    else {
      // get contact dimensionality, friction, mu
      mjContact* con = d->contact + d->efc_id[i];
      int dim = con->dim, result = 0;
      mjtNum mu = con->mu, f[6];

      // f = map force to regular-cone space
      f[0] = force[i]/mu;
      for (int j=1; j < dim; j++) {
        f[j] = force[i+j]/con->friction[j-1];
      }

      // N = normal, T = norm of tangent vector
      mjtNum N = f[0];
      mjtNum T;
      if (dim == 3) {
        T = mju_sqrt(dot2(f+1, f+1));
      } else if (dim == 4) {
        T = mju_norm3(f+1);
      } else {
        T = mju_norm(f+1, dim-1);
      }

      // top zone
      if (mu*N >= T) {
        result = mjCNSTRSTATE_SATISFIED;
      }

      // bottom zone
      else if (N+mu*T <= 0) {
        result = mjCNSTRSTATE_QUADRATIC;
        nactive += dim;
      }

      // middle zone
      else {
        result = mjCNSTRSTATE_CONE;
        nactive += dim;
      }

      // replicate state in all cone dimensions
      mju_fillInt(state+i, result, dim);

      // advance
      c += (dim-1);
    }
  }

  return nactive;
}


// update constraint state, return nactive and nchange
static int dualStateChange(const mjData* d, int* state, int* oldstate,
                           int ne, int nf, int nefc,
                           const int* efclist, int* nchange) {
  // save old state
  for (int c=0; c < nefc; c++) {
    int i = efclist ? efclist[c] : c;
    oldstate[c] = state[i];
  }

  // update state
  int nactive = dualState(d, state, ne, nf, nefc, efclist);

  // count state changes
  *nchange = 0;
  for (int c=0; c < nefc; c++) {
    int i = efclist ? efclist[c] : c;
    *nchange += (oldstate[c] != state[i]);
  }

  return nactive;
}


// project onto friction ellipsoid, write to force[i+1..i+dim-1]
// project tangential force onto friction ellipsoid: sum(f_t[j]^2/mu[j]^2) <= f_n^2
// if feasible is true, only scale down if outside the ellipsoid
// if feasible is false, always scale to the boundary
static void projectEllipsoid(mjtNum* friction, mjtNum normal, const mjtNum* mu,
                             int dim, int feasible) {
  mjtNum s = 0;
  for (int j=0; j < dim-1; j++) {
    s += friction[j]*friction[j] / (mu[j]*mu[j]);
  }

  mjtNum normal2 = normal*normal;
  if (!feasible || s > normal2) {
    mjtNum scl = mju_sqrt(normal2 / mju_max(mjMINVAL, s));
    for (int j=0; j < dim-1; j++) {
      friction[j] *= scl;
    }
  }
}


// solve QCQP and project onto friction ellipsoid, write to force[i+1..i+dim-1]
static void solveQCQP(mjtNum* force, int i, int dim,
                      mjtNum* Ac, mjtNum* bc, const mjtNum* mu) {
  int flg_active;
  mjtNum v[6];

  // solve
  if (dim == 3) {
    flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
  } else if (dim == 4) {
    flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
  } else {  // dim == 5
    flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);
  }

  // on constraint: put v on ellipsoid, in case QCQP is approximate
  if (flg_active) {
    projectEllipsoid(v, force[i], mu, dim, /*feasible=*/0);
  }

  // assign
  if (dim == 3) {
    copy2(force+i+1, v);
  } else {
    mju_copy(force+i+1, v, dim-1);
  }
}


// project contact force block onto friction cone (pyramidal or elliptic)
static void projectCone(mjtNum* force, const mjtNum* mu, int dim, int type) {
  // elliptic cone: project onto friction ellipsoid
  if (type == mjCNSTR_CONTACT_ELLIPTIC) {
    // clamp normal force
    if (force[0] < 0) {
      mju_zero(force, dim);
    } else {
      projectEllipsoid(force+1, force[0], mu, dim, /*feasible=*/1);
    }
  }

  // pyramidal or scalar: clamp to non-negative
  else {
    if (force[0] < 0) {
      force[0] = 0;
    }
  }
}


// global variable to toggle Nesterov momentum (for benchmarks/tests)
mjTHREADLOCAL int mj_nesterov_momentum = 1;


// inverse trace of the inertia matrix, over all dofs (island < 0) or over those of an island
static mjtNum inertiaScale(const mjModel* m, const mjData* d, int island) {
  int nv = island < 0 ? m->nv : d->island_nv[island];
  const int* dofs = island < 0 ? NULL : d->map_idof2dof + d->island_idofadr[island];
  mjtNum trace = 0;
  for (int i=0; i < nv; i++) {
    int dof = dofs ? dofs[i] : i;
    trace += d->M[m->M_rowadr[dof] + m->M_rownnz[dof] - 1];
  }
  return 1 / mju_max(mjMINVAL, trace);
}


//---------------------------- PGS solver ----------------------------------------------------------


// core PGS solver: iterates over constraints specified by efclist
//   island: island index for stats (use -1 for monolithic, mapped to 0)
//   ne, nf, nefc: constraint type counts
//   efclist: maps list position c to monolithic efc index (NULL for sequential)
static void solPGS(const mjModel* m, mjData* d, int island,
                   int ne, int nf, int nefc,
                   const int* efclist, int maxiter) {
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mj_markStack(d);
  mjtNum* ARinv = mjSTACKALLOC(d, nefc, mjtNum);
  int* oldstate = mjSTACKALLOC(d, 2*nefc, int);
  int* blockstart = oldstate + nefc;

  // Nesterov momentum
  mjtBool nesterov = (mj_nesterov_momentum != 0);
  mjtNum* force_prev = NULL;
  mjtNum* force_momentum = NULL;
  if (nesterov) {
    force_prev = mjSTACKALLOC(d, nefc, mjtNum);
    force_momentum = mjSTACKALLOC(d, nefc, mjtNum);
    mju_gather(force_prev, force, efclist, nefc);
  }

  int island_stat = mjMAX(0, island);  // island index for diagnostic stats
  mjtNum scale = inertiaScale(m, d, island);

  // precompute inverse diagonal of AR
  ARdiaginv(m, d, ARinv, nefc, efclist, 0);

  // initial constraint state
  dualState(d, d->efc_state, ne, nf, nefc, efclist);

  // build block-index array: one entry per constraint block
  int nblocks = 0;
  for (int c=0; c < nefc; ) {
    blockstart[nblocks++] = c;
    int i = efclist ? efclist[c] : c;
    if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
      c += d->contact[d->efc_id[i]].dim;
    } else {
      c++;
    }
  }

  // seed PCG32 RNG with a fixed seed
  pcg32_state rng;
  rng.state = 0;
  rng.inc = 1;
  pcg32_next(&rng);

  // main iteration
  int iter = 0;
  int nesterov_k = 0;  // Nesterov counter (resets on adaptive restart)
  while (iter < maxiter) {
    // Nesterov momentum extrapolation
    if (nesterov) {
      mjtNum beta = 0;
      if (iter > 0) {
        beta = (mjtNum)(nesterov_k - 1) / (mjtNum)(nesterov_k + 2);
      }

      // update with momentum, save pre-extrapolation value
      if (beta > 0) {
        for (int c=0; c < nefc; c++) {
          int i = efclist ? efclist[c] : c;
          mjtNum f_save = force[i];
          force[i] += beta*(force[i] - force_prev[c]);
          force_prev[c] = f_save;
        }

        // friction loss: project onto bounds
        for (int c=ne; c < ne+nf; c++) {
          int i = efclist ? efclist[c] : c;
          force[i] = mju_clip(force[i], -floss[i], floss[i]);
        }

        // contact force: project onto friction cone
        for (int c=ne+nf; c < nefc; ) {
          int i = efclist ? efclist[c] : c;
          int dim = 1;
          int type = d->efc_type[i];
          const mjtNum* mu = NULL;

          if (type == mjCNSTR_CONTACT_ELLIPTIC) {
            dim = d->contact[d->efc_id[i]].dim;
            mu = d->contact[d->efc_id[i]].friction;
          }

          projectCone(force+i, mu, dim, type);
          c += dim;
        }
      }

      // iter == 0 or beta <= 0 (nesterov_k <= 1): just save current force
      else {
        mju_gather(force_prev, force, efclist, nefc);
      }

      // save extrapolated point for gradient restart check
      mju_gather(force_momentum, force, efclist, nefc);
    }

    // clear improvement
    mjtNum improvement = 0;

    // shuffle constraint visitation order
    shuffle_int(blockstart, nblocks, &rng);

    // perform one sweep over constraint blocks
    for (int bi=0; bi < nblocks; bi++) {
      int c = blockstart[bi];
      int i = efclist ? efclist[c] : c;

      // get constraint dimensionality
      int dim;
      if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
        dim = d->contact[d->efc_id[i]].dim;
      } else {
        dim = 1;
      }

      // compute residual for this constraint
      mjtNum res[6];
      residual(m, d, res, i, dim, 0);

      // save old force
      mjtNum oldforce[6];
      if (dim == 1) {
        oldforce[0] = force[i];
      } else if (dim == 3) {
        mji_copy3(oldforce, force+i);
      } else {
        mju_copy(oldforce, force+i, dim);
      }

      // allocate AR submatrix, required later for costChage
      mjtNum Athis[36];

      // simple constraint
      if (d->efc_type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
        // unconstrained minimum
        force[i] -= res[0]*ARinv[c];

        // impose interval and inequality constraints
        if (c >= ne && c < ne+nf) {
          if (force[i] < -floss[i]) {
            force[i] = -floss[i];
          } else if (force[i] > floss[i]) {
            force[i] = floss[i];
          }
        } else if (c >= ne+nf) {
          if (force[i] < 0) {
            force[i] = 0;
          }
        }
      }

      // elliptic cone constraint
      else {
        // get friction
        mjtNum *mu = d->contact[d->efc_id[i]].friction;

        //-------------------- perform normal or ray update

        // Athis = AR(this,this)
        extractBlock(m, d, Athis, i, dim, 0);

        // normal force too small: normal update
        if (force[i] < mjMINVAL) {
          // unconstrained minimum
          force[i] -= res[0]*ARinv[c];

          // clamp
          if (force[i] < 0) {
            force[i] = 0;
          }

          // clear friction (just in case)
          if (dim == 3) {
            zero2(force+i+1);
          } else {
            mju_zero(force+i+1, dim-1);
          }
        }

        // ray update
        else {
          // v = ray, denom = v' * AR(this,this) * v
          mjtNum v[6], v1[6], denom;
          if (dim == 3) {
            mji_copy3(v, force+i);
            mji_mulMatVec3(v1, Athis, v);
            denom = mju_dot3(v, v1);
          } else {
            mju_copy(v, force+i, dim);
            mju_mulMatVec(v1, Athis, v, dim, dim);
            denom = mju_dot(v, v1, dim);
          }

          // avoid division by 0
          if (denom >= mjMINVAL) {
            // x = v' * res / denom
            mjtNum x = (dim == 3 ? -mju_dot3(v, res) : -mju_dot(v, res, dim)) / denom;

            // make sure normal is non-negative
            if (force[i]+x*v[0] < 0) {
              x = -v[0]/force[i];
            }

            //  add x*v to f
            for (int j=0; j < dim; j++) {
              force[i+j] += x*v[j];
            }
          }
        }

        //-------------------- perform friction update, keep normal fixed

        // Ac = AR-submatrix; bc = b-subvector + Ac,rest * f_rest
        mjtNum bc[5], Ac[25];
        if (dim == 3) {
          mjtNum df0 = force[i] - oldforce[0];
          Ac[0] = Athis[4];
          Ac[1] = Athis[5];
          Ac[2] = Athis[7];
          Ac[3] = Athis[8];
          bc[0] = (res[1] - dot2(Ac, oldforce+1)) + Athis[3]*df0;
          bc[1] = (res[2] - dot2(Ac+2, oldforce+1)) + Athis[6]*df0;
        } else {
          mju_copy(bc, res+1, dim-1);
          for (int j=0; j < dim-1; j++) {
            mju_copy(Ac+j*(dim-1), Athis+(j+1)*dim+1, dim-1);
            bc[j] -= (dim == 4) ? mju_dot3(Ac+j*3, oldforce+1)
                                : mju_dot(Ac+j*(dim-1), oldforce+1, dim-1);
            bc[j] += Athis[(j+1)*dim]*(force[i]-oldforce[0]);
          }
        }

        // guard for f_normal==0
        if (force[i] < mjMINVAL) {
          if (dim == 3) {
            zero2(force+i+1);
          } else {
            mju_zero(force+i+1, dim-1);
          }
        }

        // QCQP
        else {
          solveQCQP(force, i, dim, Ac, bc, mu);
        }
      }

      // accumulate improvement
      if (dim == 1) {
        Athis[0] = 1/ARinv[c];
      }
      improvement -= costChange(Athis, force+i, oldforce, res, dim);
    }

    // update constraint state
    int nchange;
    int nactive = dualStateChange(d, d->efc_state, oldstate, ne, nf, nefc, efclist, &nchange);

    // scale improvement, save stats
    improvement *= scale;
    saveStats(m, d, island_stat, iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // Nesterov gradient restart (O'Donoghue-Candès): reset when correction opposes extrapolation
    if (nesterov) {
      mjtBool restart = false;
      if (iter > 0) {
        mjtNum dot_corr_extr = 0;
        for (int c=0; c < nefc; c++) {
          int i = efclist ? efclist[c] : c;
          mjtNum correction = force[i] - force_momentum[c];
          mjtNum extrapolation = force_momentum[c] - force_prev[c];
          dot_corr_extr += correction * extrapolation;
        }
        restart = (dot_corr_extr < 0);
      }

      if (restart) {
        nesterov_k = 0;
      } else {
        nesterov_k++;
      }
    }

    // increment iteration count
    iter++;

    // terminate
    if (improvement < m->opt.tolerance) {
      break;
    }
  }

  // finalize statistics
  if (island_stat < mjNISLAND) {
    // update solver iterations
    d->solver_niter[island_stat] += iter;

    // set nnz
    if (mj_isSparse(m)) {
      d->solver_nnz[island_stat] = 0;
      for (int c=0; c < nefc; c++) {
        d->solver_nnz[island_stat] += d->efc_AR_rownnz[efclist ? efclist[c] : c];
      }
    } else {
      d->solver_nnz[island_stat] = nefc*nefc;
    }
  }

  mj_freeStack(d);
}


// PGS entry point (monolithic, no dualFinish — caller handles it)
void mj_solPGS(const mjModel* m, mjData* d, int maxiter) {
  solPGS(m, d, /*island=*/-1, d->ne, d->nf, d->nefc, /*efclist=*/NULL, maxiter);
}


// PGS entry point (one island)
void mj_solPGS_island(const mjModel* m, mjData* d, int island, int maxiter) {
  int ne = d->island_ne[island];
  int nf = d->island_nf[island];
  int nefc = d->island_nefc[island];
  int iefcadr = d->island_iefcadr[island];

  solPGS(m, d, island, ne, nf, nefc, d->map_iefc2efc + iefcadr, maxiter);
}


//---------------------------- NoSlip solver -------------------------------------------------------

// core NoSlip solver: iterates over constraints specified by efclist
//   island: island index for stats (use -1 for monolithic, mapped to 0)
//   ne, nf, nefc: constraint type counts
//   efclist: maps list position c to monolithic efc index (NULL for sequential)
static void solNoSlip(const mjModel* m, mjData* d, int island,
                      int ne, int nf, int nefc,
                      const int* efclist, int maxiter) {
  int dim, iter = 0;
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mjtNum *mu, improvement;
  mjtNum Ac[25], bc[5], res[5], oldforce[5], delta[5], mid, y, K0, K1;
  mjContact* con;
  mj_markStack(d);
  mjtNum* ARinv = mjSTACKALLOC(d, nefc, mjtNum);
  int* oldstate = mjSTACKALLOC(d, nefc, int);

  int island_stat = mjMAX(0, island);
  mjtNum scale = inertiaScale(m, d, island);

  // precompute inverse diagonal of A
  ARdiaginv(m, d, ARinv, nefc, efclist, 1);

  // initial constraint state
  dualState(d, d->efc_state, ne, nf, nefc, efclist);

  // main iteration
  while (iter < maxiter) {
    // clear improvement
    improvement = 0;

    // correct for cost change at iter 0
    if (iter == 0) {
      for (int c=0; c < nefc; c++) {
        int i = efclist ? efclist[c] : c;
        improvement += 0.5*force[i]*force[i]*d->efc_R[i];
      }
    }

    // perform one sweep: dry friction
    for (int c=ne; c < ne+nf; c++) {
      int i = efclist ? efclist[c] : c;

      // compute residual, save old
      residual(m, d, res, i, 1, 1);
      oldforce[0] = force[i];

      // unconstrained minimum
      force[i] -= res[0]*ARinv[c];

      // impose interval constraints
      if (force[i] < -floss[i]) {
        force[i] = -floss[i];
      } else if (force[i] > floss[i]) {
        force[i] = floss[i];
      }

      // add to improvement
      delta[0] = force[i] - oldforce[0];
      improvement -= 0.5*delta[0]*delta[0]/ARinv[c] + delta[0]*res[0];
    }

    // perform one sweep: contact friction
    for (int c=ne+nf; c < nefc; c++) {
      int i = efclist ? efclist[c] : c;

      // pyramidal contact
      if (d->efc_type[i] == mjCNSTR_CONTACT_PYRAMIDAL) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // loop over pairs of opposing pyramid edges
        for (int j=i; j < i+2*(dim-1); j+=2) {
          // compute residual, save old
          residual(m, d, res, j, 2, 1);
          copy2(oldforce, force+j);

          // Ac = AR-submatirx
          extractBlock(m, d, Ac, j, 2, 1);

          // bc = b-subvector + Ac,rest * f_rest
          bc[0] = res[0] - dot2(Ac, oldforce);
          bc[1] = res[1] - dot2(Ac+2, oldforce);

          // f0 = mid+y, f1 = mid-y
          mid = 0.5*(force[j]+force[j+1]);
          y = 0.5*(force[j]-force[j+1]);

          // K1 = A00 + A11 - 2*A01,  K0 = mid*A00 - mid*A11 + b0 - b1
          K1 = Ac[0] + Ac[3] - Ac[1] - Ac[2];
          K0 = mid*(Ac[0] - Ac[3]) + bc[0] - bc[1];

          // guard against Ac==0
          if (K1 < mjMINVAL) {
            force[j] = force[j+1] = mid;
          }

          // otherwise minimize over y \in [-mid, mid]
          else {
            // unconstrained minimum
            y = -K0/K1;

            // clamp and assign
            if (y < -mid) {
              force[j] = 0;
              force[j+1] = 2*mid;
            } else if (y > mid) {
              force[j] = 2*mid;
              force[j+1] = 0;
            } else {
              force[j] = mid+y;
              force[j+1] = mid-y;
            }
          }

          // accumulate improvement
          improvement -= costChange(Ac, force+j, oldforce, res, 2);
        }

        // skip the rest of this contact
        c += 2*(dim-1)-1;
      }

      // elliptic contact
      else if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // compute residual, save old
        residual(m, d, res, i+1, dim-1, 1);
        if (dim == 3) {
          copy2(oldforce, force+i+1);
        } else {
          mju_copy(oldforce, force+i+1, dim-1);
        }

        // Ac = AR-submatrix
        extractBlock(m, d, Ac, i+1, dim-1, 1);

        // bc = b-subvector + Ac,rest * f_rest
        if (dim == 3) {
          bc[0] = res[0] - dot2(Ac, oldforce);
          bc[1] = res[1] - dot2(Ac+2, oldforce);
        } else {
          mju_copy(bc, res, dim-1);
          for (int j=0; j < dim-1; j++) {
            bc[j] -= (dim == 4) ? mju_dot3(Ac+j*3, oldforce)
                                : mju_dot(Ac+j*(dim-1), oldforce, dim-1);
          }
        }

        // guard for f_normal==0
        if (force[i] < mjMINVAL) {
          if (dim == 3) {
            zero2(force+i+1);
          } else {
            mju_zero(force+i+1, dim-1);
          }
        }

        // QCQP
        else {
          solveQCQP(force, i, dim, Ac, bc, mu);
        }

        // accumulate improvement
        improvement -= costChange(Ac, force+i+1, oldforce, res, dim-1);

        // skip the rest of this contact
        c += (dim-1);
      }
    }

    // update constraint state
    int nchange;
    int nactive = dualStateChange(d, d->efc_state, oldstate, ne, nf, nefc, efclist, &nchange);

    // scale improvement, save stats
    improvement *= scale;

    // save noslip stats after all the entries from regular solver
    if (island_stat < mjNISLAND) {
      int stats_iter = iter + d->solver_niter[island_stat];
      saveStats(m, d, island_stat, stats_iter, improvement, 0, 0, nactive, nchange, 0, 0);
    }

    // increment iteration count
    iter++;

    // terminate
    if (improvement < m->opt.noslip_tolerance) {
      break;
    }
  }

  // update solver iterations
  if (island_stat < mjNISLAND) {
    d->solver_niter[island_stat] += iter;
  }

  mj_freeStack(d);
}


// NoSlip entry point (monolithic, no dualFinish — caller handles it)
void mj_solNoSlip(const mjModel* m, mjData* d, int maxiter) {
  solNoSlip(m, d, /*island=*/-1, d->ne, d->nf, d->nefc, /*efclist=*/NULL, maxiter);
}


// NoSlip entry point (one island)
void mj_solNoSlip_island(const mjModel* m, mjData* d, int island, int maxiter) {
  int ne = d->island_ne[island];
  int nf = d->island_nf[island];
  int nefc = d->island_nefc[island];
  int iefcadr = d->island_iefcadr[island];

  solNoSlip(m, d, island, ne, nf, nefc, d->map_iefc2efc + iefcadr, maxiter);
}


//------------------------- Primal solvers ---------------------------------------------------------

// Primal context
typedef struct {
  int is_sparse;          // 1: sparse, 0: dense
  int is_elliptic;        // 1: elliptic, 0: pyramidal
  int island;             // current island index, -1 if monolithic

  // sizes
  int nv;                 // number of dofs
  int ne;                 // number of equalities
  int nf;                 // number of friction constraints
  int nefc;               // number of all constraints
  int nJ;                 // number of nonzeros in Jacobian

  // contact array
  mjContact* contact;

  // dof arrays
  const mjtNum* qfrc_smooth;
  const mjtNum* qacc_smooth;
  mjtNum* qfrc_constraint;
  mjtNum* qacc;

  // inertia
  int* M_rownnz;
  int* M_rowadr;
  int* M_colind;
  mjtNum* M;
  mjtNum* qLD;
  mjtNum* qLDiagInv;

  // efc arrays
  const mjtNum* efc_D;
  const mjtNum* efc_R;
  const mjtNum* efc_frictionloss;
  const mjtNum* efc_aref;
  const int* efc_id;
  const int* efc_type;
  mjtNum* efc_force;
  int* efc_state;

  // Jacobians
  int* J_rownnz;
  int* J_rowadr;
  int* J_rowsuper;
  int* J_colind;
  mjtNum* J;
  int* JT_rownnz;
  int* JT_rowadr;
  int* JT_rowsuper;
  int* JT_colind;
  mjtNum* JT;

  // common arrays (PrimalAllocate)
  mjtNum* Jaref;          // Jac*qacc - aref                              (nefc x 1)
  mjtNum* Jv;             // Jac*search                                   (nefc x 1)
  mjtNum* Ma;             // M*qacc                                       (nv x 1)
  mjtNum* Mv;             // M*search                                     (nv x 1)
  mjtNum* grad;           // gradient of master cost                      (nv x 1)
  mjtNum* Mgrad;          // M\grad or H\grad                             (nv x 1)
  mjtNum* search;         // linesearch vector                            (nv x 1)
  mjtNum* quad;           // quadratic polynomials for constraint costs   (nefc x 3)
  int* oldstate;          // previous constraint state                    (nefc x 1)
  int efc_cone_start;     // first elliptic contact row, or nefc if none
  int efc_cone3_start;    // start of 4-wide contiguous dim=3 elliptic contacts
  int efc_cone3_end;      // end of 4-wide contiguous dim=3 elliptic contacts

  // CG arrays (PrimalAllocate, CG only)
  mjtNum* gradold;        // previous gradient                            (nv x 1)
  mjtNum* Mgradold;       // previous preconditioned gradient             (nv x 1)
  mjtNum* graddif;        // grad - gradold                               (nv x 1)
  mjtNum* Mgraddif;       // M\(grad - gradold)                           (nv x 1)

  // Newton arrays, known-size (PrimalAllocate)
  mjtNum* D;              // constraint inertia                           (nefc x 1)
  mjtNum* cholupd;        // scratch for rank-1 Cholesky updates          (nv x 1)
  mjtNum* cholscratch;    // dense accumulator for the sparse kernels     (nv x 1)
  mjtNum* LTJ;            // V*J for cone Cholesky updates                (5 x nv)
  int* H_rowadr;          // Hessian row addresses                        (nv x 1)
  int* H_rownnz;          // Hessian row nonzeros                         (nv x 1)
  int* HT_rownnz;         // Hessian transpose row nonzeros               (nv x 1)
  int* HT_rowadr;         // Hessian transpose row addresses              (nv x 1)
  int* L_rownnz;          // Hessian factor row nonzeros                  (nv x 1)
  int* L_rowadr;          // Hessian factor row addresses                 (nv x 1)
  int* LT_rownnz;         // Hessian factor transpose row nonzeros        (nv x 1)
  int* LT_rowadr;         // Hessian factor transpose row addresses       (nv x 1)

  // Newton arrays, computed-size (MakeHessian)
  int nH;                 // number of nonzeros in Hessian H
  int* H_colind;          // Hessian column indices                       (nH x 1)
  int* HT_colind;         // Hessian transpose column indices             (nH x 1)
  mjtNum* H;              // Hessian                                      (nH x 1)
  int nL;                 // number of nonzeros in Cholesky factor L
  int* L_colind;          // Cholesky factor column indices               (nL x 1)
  int* LT_colind;         // Cholesky factor transpose column indices     (nL x 1)
  int* LT_map;            // CSC-to-CSR index mapping                     (nL x 1)
  int* L_super;           // Cholesky factor supernode sizes              (nv x 1)
  mjtNum* L;              // Cholesky factor                              (nL x 1)
  mjtNum* Lcone;          // Cholesky factor with cone contributions      (nL x 1)

  // Newton sparse fill-reducing ordering (MakeHessian): NULL when natural order is kept
  int* perm;              // DOF permutation old -> new                   (nv x 1)
  int* HJ_colind;         // J column indices in Hessian order            (nJ x 1)
  mjtNum* HJ;             // J values in Hessian order                    (nJ x 1)
  int* HJT_rownnz;        // J' row nonzeros in Hessian order             (nv x 1)
  int* HJT_rowadr;        // J' row addresses in Hessian order            (nv x 1)
  int* HJT_rowsuper;      // J' supernodes in Hessian order               (nv x 1)
  int* HJT_colind;        // J' column indices in Hessian order           (nJ x 1)
  mjtNum* HJT;            // J' values in Hessian order                   (nJ x 1)
  int* HM_rownnz;         // M lower-triangle row nonzeros, Hessian order (nv x 1)
  int* HM_rowadr;         // M lower-triangle row addresses, Hessian ord  (nv x 1)
  int* HM_colind;         // M lower-triangle column indices              (nM x 1)
  mjtNum* HM;             // M lower-triangle values                      (nM x 1)

  // implicit effective metric (built per step in mj_fwdPosition): when active, ctx.qfrc_smooth
  // is pre-shifted by +c and Ma/Mv carry the metric terms, so the stock objective/gradient/
  // linesearch formulas below operate in the Mtilde = M + h*D + h^2*K metric unchanged
  int flg_metric;         // effective metric active for this solve
  const mjModel* fm;      // model, for the metric calls
  mjData* fd;             // data, for the metric calls
  mjtNum* epL;            // CG: per-dof diagonal with efc and contact rows folded in
                          // (mj_effPrecFold), mj_effFoldDiagSize, or NULL

  // Newton: lower-triangle view of the metric matrix S = diag(h*D+h^2*K) + K_csr, built by
  // MakeMetricLower on the solver stack, merged into the Hessian alongside M
  int nS;                 // total nonzeros of the view
  int S_tcap;             // capacity of the rank-1 contribution, set by PrimalAllocate
  int* S_rownnz;          // view row nonzeros                            (nv x 1)
  int* S_rowadr;          // view row addresses                           (nv x 1)
  int* S_colind;          // view column indices                          (nS x 1)
  mjtNum* S_val;          // view values                                  (nS x 1)

  // globals
  mjtNum cost;            // constraint + Gauss cost
  mjtNum quadGauss[3];    // quadratic polynomial for Gauss cost
  mjtNum scale;           // scaling factor for improvement and gradient
  int nactive;            // number of active constraints
  int ncone;              // number of contacts in cone state
  int nupdate;            // number of Cholesky updates

  // cone fold gate (sparse and dense)
  int* pathcost;          // reverse-etree path costs of L rows               (nv x 1)
  int* dense_cols;        // nonzero column scratch for dense outer products  (nv x 1)
  mjtSize Lflops;         // sum of squared L row nonzeros (2x factorization MACs)
  int cone_fold;          // cached fold decision (-1: recompute)
  int flg_Lvalid;         // 1 if L holds chol(H), 0 if deferred while folding
  int flg_Hdirty;         // 1 if H has incremental downdates since full build
  int ndowndate;          // accumulated H downdates since full build

  // cone fold workspace, allocated on the first fold (elliptic sparse)
  mjtNum* Hcone;          // cone-inclusive Hessian                           (nH x 1)

  // linesearch diagnostics
  int LSiter;             // number of linesearch iterations
  int LSresult;           // linesearch result
  mjtNum LSslope;         // linesearch slope at solution
} mjPrimalContext;


// return 1 if 4 contiguous dim=3 elliptic contacts start at pos
static inline int isFourDim3Elliptic(const mjPrimalContext* ctx, int pos) {
  return pos + 12 <= ctx->nefc &&
         ctx->efc_type[pos+0] == mjCNSTR_CONTACT_ELLIPTIC &&
         ctx->contact[ctx->efc_id[pos+0]].dim == 3 &&
         ctx->efc_type[pos+3] == mjCNSTR_CONTACT_ELLIPTIC &&
         ctx->contact[ctx->efc_id[pos+3]].dim == 3 &&
         ctx->efc_type[pos+6] == mjCNSTR_CONTACT_ELLIPTIC &&
         ctx->contact[ctx->efc_id[pos+6]].dim == 3 &&
         ctx->efc_type[pos+9] == mjCNSTR_CONTACT_ELLIPTIC &&
         ctx->contact[ctx->efc_id[pos+9]].dim == 3;
}


// set sizes and pointers to mjData arrays in mjPrimalContext
static void PrimalPointers(const mjModel* m, const mjData* d, mjPrimalContext* ctx, int island) {
  // clear everything
  memset(ctx, 0, sizeof(mjPrimalContext));

  // globals
  ctx->is_sparse = mj_isSparse(m);
  ctx->is_elliptic = (m->opt.cone == mjCONE_ELLIPTIC);
  ctx->contact = d->contact;
  ctx->island = island;
  ctx->cone_fold = -1;

  // set sizes and pointers (monolithic)
  if (island < 0) {
    // sizes
    ctx->nv               = m->nv;
    ctx->ne               = d->ne;
    ctx->nf               = d->nf;
    ctx->nefc             = d->nefc;
    ctx->nJ               = d->nJ;

    // dof arrays
    ctx->qfrc_smooth      = d->qfrc_smooth;
    ctx->qfrc_constraint  = d->qfrc_constraint;
    ctx->qacc_smooth      = d->qacc_smooth;
    ctx->qacc             = d->qacc;

    // inertia
    ctx->M_rownnz         = m->M_rownnz;
    ctx->M_rowadr         = m->M_rowadr;
    ctx->M_colind         = m->M_colind;
    ctx->M                = d->M;
    ctx->qLD              = d->qLD;
    ctx->qLDiagInv        = d->qLDiagInv;

    // efc arrays
    ctx->efc_D            = d->efc_D;
    ctx->efc_R            = d->efc_R;
    ctx->efc_frictionloss = d->efc_frictionloss;
    ctx->efc_aref         = d->efc_aref;
    ctx->efc_id           = d->efc_id;
    ctx->efc_type         = d->efc_type;
    ctx->efc_force        = d->efc_force;
    ctx->efc_state        = d->efc_state;

    // Jacobians
    ctx->J                = d->efc_J;
    if (ctx->is_sparse) {
      ctx->J_rownnz       = d->efc_J_rownnz;
      ctx->J_rowadr       = d->efc_J_rowadr;
      ctx->J_rowsuper     = d->efc_J_rowsuper;
      ctx->J_colind       = d->efc_J_colind;
    }
  }

  // set sizes and pointers (per-island)
  else {
    // sizes
    ctx->nv               = d->island_nv[island];
    ctx->ne               = d->island_ne[island];
    ctx->nf               = d->island_nf[island];
    ctx->nefc             = d->island_nefc[island];

    // dof arrays
    int idofadr           = d->island_idofadr[island];
    ctx->qfrc_smooth      = d->ifrc_smooth       + idofadr;
    ctx->qfrc_constraint  = d->ifrc_constraint   + idofadr;
    ctx->qacc_smooth      = d->iacc_smooth       + idofadr;
    ctx->qacc             = d->iacc              + idofadr;

    // efc arrays
    int iefcadr           = d->island_iefcadr[island];
    ctx->efc_D            = d->iefc_D            + iefcadr;
    ctx->efc_R            = d->iefc_R            + iefcadr;
    ctx->efc_frictionloss = d->iefc_frictionloss + iefcadr;
    ctx->efc_aref         = d->iefc_aref         + iefcadr;
    ctx->efc_id           = d->iefc_id           + iefcadr;
    ctx->efc_type         = d->iefc_type         + iefcadr;
    ctx->efc_force        = d->iefc_force        + iefcadr;
    ctx->efc_state        = d->iefc_state        + iefcadr;

    // single island covering all DOFs: mj_island fills map_dof2idof and map_efc2iefc in
    // ascending order per island, so both maps are the identity and monolithic M, qLD, and J
    // can be reused directly without copying
    if (d->nisland == 1 && d->nidof == m->nv) {
      int use_qH = mj_effUseQH(d);
      ctx->nJ               = d->nJ;
      ctx->M_rownnz         = m->M_rownnz;
      ctx->M_rowadr         = m->M_rowadr;
      ctx->M_colind         = m->M_colind;
      ctx->M                = d->M;
      ctx->qLD              = use_qH ? d->qH : d->qLD;
      ctx->qLDiagInv        = use_qH ? d->qHDiagInv : d->qLDiagInv;
      ctx->J                = d->efc_J;
      if (ctx->is_sparse) {
        ctx->J_rownnz       = d->efc_J_rownnz;
        ctx->J_rowadr       = d->efc_J_rowadr;
        ctx->J_rowsuper     = d->efc_J_rowsuper;
        ctx->J_colind       = d->efc_J_colind;
      }
    }
  }

  // check whether any elliptic contact constraint is actually present and find 4-wide dim=3 span
  ctx->efc_cone_start = ctx->nefc;
  ctx->efc_cone3_start = ctx->nefc;
  ctx->efc_cone3_end = ctx->nefc;
  if (ctx->is_elliptic) {
    int has_elliptic = 0;
    if (d->ncon > 0) {
      for (int i = ctx->ne + ctx->nf; i < ctx->nefc; i++) {
        if (ctx->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
          ctx->efc_cone_start = i;
          has_elliptic = 1;
          break;
        }
      }
    }
    ctx->is_elliptic = has_elliptic;
    if (has_elliptic) {
      int pos = ctx->efc_cone_start;
      int start = pos, end = pos;
      while (pos + 12 <= ctx->nefc) {
        if (isFourDim3Elliptic(ctx, pos)) {
          start = pos;
          do {
            pos += 12;
          } while (isFourDim3Elliptic(ctx, pos));
          end = pos;
          break;
        }
        pos += (ctx->efc_type[pos] == mjCNSTR_CONTACT_ELLIPTIC)
                   ? ctx->contact[ctx->efc_id[pos]].dim : 1;
      }
      ctx->efc_cone3_start = start;
      ctx->efc_cone3_end = end;
    }
  }
}


// extract island M, qLD (or qH), and qLDiagInv using contiguous tree-DOF runs
static void primalExtractIslandM(const mjModel* m, const mjData* d, mjPrimalContext* ctx,
                                 const mjtNum* gLD, const mjtNum* gLDiagInv) {
  int nv = ctx->nv;
  int idofadr = d->island_idofadr[ctx->island];
  const int* restrict idof2dof = d->map_idof2dof + idofadr;

  // M and qLD are block-diagonal by tree, and each tree's DOFs are contiguous in both global and
  // island order, so every column in a contiguous run shifts by the constant delta = k0 - r
  int adr = 0;
  for (int r=0; r < nv; ) {
    int k0 = idof2dof[r];
    int delta = k0 - r;
    int len = 1;
    while (r + len < nv && idof2dof[r+len] == k0 + len) {
      len++;
    }

    int src0 = m->M_rowadr[k0];
    int run_nnz = (m->M_rowadr[k0+len-1] + m->M_rownnz[k0+len-1]) - src0;
    mju_copyInt(ctx->M_rownnz + r, m->M_rownnz + k0, len);
    int row_shift = src0 - adr;
    for (int i=0; i < len; i++) {
      ctx->M_rowadr[r+i] = m->M_rowadr[k0+i] - row_shift;
    }
    if (delta == 0) {
      mju_copyInt(ctx->M_colind + adr, m->M_colind + src0, run_nnz);
    } else {
      const int* restrict src_col = m->M_colind + src0;
      int* restrict dst_col = ctx->M_colind + adr;
      for (int j=0; j < run_nnz; j++) {
        dst_col[j] = src_col[j] - delta;
      }
    }
    mju_copy(ctx->qLD + adr, gLD + src0, run_nnz);
    mju_copy(ctx->M + adr, d->M + src0, run_nnz);
    mju_copy(ctx->qLDiagInv + r, gLDiagInv + k0, len);

    adr += run_nnz;
    r += len;
  }
}


// fused island sparse J extraction, J_rowsuper construction, and JT transpose
static void primalExtractIslandJAndTranspose(const mjData* d, mjPrimalContext* ctx) {
  int nv = ctx->nv;
  int nefc = ctx->nefc;
  int island = ctx->island;
  int idofadr = d->island_idofadr[island];
  int iefcadr = d->island_iefcadr[island];
  const int* restrict idof2dof = d->map_idof2dof + idofadr;
  const int* restrict dof2idof = d->map_dof2idof;
  const int* restrict iefc2efc = d->map_iefc2efc + iefcadr;

  int* restrict J_rownnz = ctx->J_rownnz;
  int* restrict J_rowadr = ctx->J_rowadr;
  int* restrict J_rowsuper = ctx->J_rowsuper;
  int* restrict J_colind = ctx->J_colind;
  mjtNum* restrict J = ctx->J;

  int* restrict JT_rownnz = ctx->JT_rownnz;
  int* restrict JT_rowadr = ctx->JT_rowadr;
  int* restrict JT_rowsuper = ctx->JT_rowsuper;

  // initialize JT counts and supernode flags
  mju_zeroInt(JT_rownnz, nv);
  for (int i=0; i < nv-1; i++) {
    JT_rowsuper[i] = 1;
  }
  JT_rowsuper[nv-1] = 0;

  // contiguous island DOF slice allows constant subtraction instead of dof2idof gather
  int contiguous_dof = (idof2dof[nv-1] - idof2dof[0] == nv - 1);
  int dof0 = idof2dof[0];

  // pass 1: extract J using d->efc_J_rowsuper runs, detect J_rowsuper boundaries, count JT
  int j_adr = 0;
  int prev_nnz = -1;
  const int* prev_col = NULL;
  for (int r=0; r < nefc; ) {
    int k = iefc2efc[r];
    int nnz = d->efc_J_rownnz[k];

    // reuse maximal contiguous prefix of global J supernode present in this island
    int s = d->efc_J_rowsuper[k];
    int ns = 1;
    if (s > 0) {
      if (r + s < nefc && iefc2efc[r+s] == k + s) {
        ns = s + 1;
      } else {
        while (ns <= s && r + ns < nefc && iefc2efc[r+ns] == k + ns) {
          ns++;
        }
      }
    }

    for (int t=0; t < ns; t++) {
      J_rownnz[r+t] = nnz;
      J_rowadr[r+t] = j_adr + t*nnz;
      J_rowsuper[r+t] = 1;
    }

    int* restrict dst_col = J_colind + j_adr;
    if (nnz > 0) {
      int src_adr0 = d->efc_J_rowadr[k];
      const int* restrict src_col = d->efc_J_colind + src_adr0;

      // remap colind once for the supernode
      if (contiguous_dof) {
        for (int j=0; j < nnz; j++) {
          dst_col[j] = src_col[j] - dof0;
        }
      } else {
        for (int j=0; j < nnz; j++) {
          dst_col[j] = dof2idof[src_col[j]] - idofadr;
        }
      }

      // count JT_rownnz and mark non-supernode boundaries in JT
      int c_prev = dst_col[0];
      JT_rownnz[c_prev] += ns;
      if (c_prev > 0) {
        JT_rowsuper[c_prev-1] = 0;
      }
      for (int j=1; j < nnz; j++) {
        int c = dst_col[j];
        JT_rownnz[c] += ns;
        if (c != c_prev + 1) {
          JT_rowsuper[c-1] = 0;
        }
        c_prev = c;
      }

      // replicate remapped column indices across remaining rows in the supernode
      for (int t=1; t < ns; t++) {
        mju_copyInt(dst_col + t*nnz, dst_col, nnz);
      }

      // copy J values (bulk copy when contiguous in d->efc_J)
      if (ns == 1 || d->efc_J_rowadr[k+ns-1] == src_adr0 + (ns-1)*nnz) {
        mju_copy(J + j_adr, d->efc_J + src_adr0, ns*nnz);
      } else {
        for (int t=0; t < ns; t++) {
          mju_copy(J + j_adr + t*nnz, d->efc_J + d->efc_J_rowadr[k+t], nnz);
        }
      }
    }

    // check supernode boundary with previous run
    if (r > 0) {
      J_rowsuper[r-1] = (nnz == prev_nnz && (nnz == 0 || mju_compare(prev_col, dst_col, nnz)));
    }
    prev_nnz = nnz;
    prev_col = dst_col;
    j_adr += ns * nnz;
    r += ns;
  }

  // finalize J_rowsuper
  J_rowsuper[nefc-1] = 0;
  for (int i=nefc-2; i >= 0; i--) {
    if (J_rowsuper[i]) {
      J_rowsuper[i] += J_rowsuper[i+1];
    }
  }

  // finalize JT_rowsuper and compute JT_rowadr
  for (int i=0; i < nv-1; i++) {
    JT_rowsuper[i] &= (JT_rownnz[i] == JT_rownnz[i+1]);
  }
  for (int i=nv-2; i >= 0; i--) {
    if (JT_rowsuper[i]) {
      JT_rowsuper[i] += JT_rowsuper[i+1];
    }
  }
  JT_rowadr[0] = 0;
  for (int i=1; i < nv; i++) {
    JT_rowadr[i] = JT_rowadr[i-1] + JT_rownnz[i-1];
  }

  // pass 2: fill JT_colind and JT using precomputed JT_rowadr and finalized J_rowsuper
  mju_transposeSparse(ctx->JT, ctx->J, nefc, nv,
                      NULL, ctx->JT_rowadr, ctx->JT_colind, NULL,
                      ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind, ctx->J_rowsuper);
}


// allocate fixed-size arrays in mjPrimalContext
//  mj_{mark/free}Stack in calling function!
static void PrimalAllocate(const mjModel* m, mjData* d, mjPrimalContext* ctx, int flg_Newton) {
  // local sizes and flags
  int nv = ctx->nv;
  int nefc = ctx->nefc;
  int is_sparse = ctx->is_sparse;
  int is_elliptic = ctx->is_elliptic;
  int nJ = is_sparse ? d->nJ : 0;
  int copy_island = (ctx->island >= 0 && !ctx->M);

  // compute island matrix sizes if needed
  int nC = 0;
  if (copy_island) {
    // count nC: number of nonzeros in M block of island (always sparse)
    int island = ctx->island;
    int idofadr = d->island_idofadr[island];
    for (int i = 0; i < nv; i++) {
      int dof = d->map_idof2dof[idofadr + i];
      nC += m->M_rownnz[dof];
    }

    // count nJ: number of nonzeros in J block of island (sparse or dense)
    if (is_sparse) {
      nJ = 0;
      int iefcadr = d->island_iefcadr[island];
      for (int i = 0; i < nefc; i++) {
        int efc = d->map_iefc2efc[iefcadr + i];
        nJ += d->efc_J_rownnz[efc];
      }
    } else {
      nJ = nefc * nv;
    }
    ctx->nJ = nJ;
  }

  // discrete metric: size the lower-triangle view S_* (filled by MakeMetricLower)
  int nScap = 0;
  if (d->efm_active && flg_Newton) {
    int tcap = 0, tlow = 0;
    mjEffRank1Iter it = {0};
    mjEffRank1 e;
    while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
      if (ctx->island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != ctx->island) {
        continue;
      }
      tcap += e.nnz * e.nnz;
      tlow += e.nnz * (e.nnz + 1) / 2;
    }
    ctx->S_tcap = tcap;

    // the view is lower-triangle: at most half the symmetric CSR, the lower wedge of
    // each rank-1 outer product, the strictly-lower fluid pattern, one diagonal per row
    int kcap = d->nefmK / 2;
    if (ctx->island >= 0 && d->nefmK) {
      int ksum = 0;
      int idofadr = d->island_idofadr[ctx->island];
      for (int i = 0; i < nv; i++) {
        ksum += d->efm_K_rownnz[d->map_idof2dof[idofadr + i]];
      }
      kcap = ksum / 2;
    }
    nScap = kcap + tlow + nv + (d->efm_fluid ? m->nC - nv : 0);
  }

  // carve the mjtNum and int blocks: pass 0 totals the sizes, pass 1 hands out the
  // pointers, so a new array cannot be carved without also being counted
  size_t nNum = 0, nInt = 0;
  mjtNum* numblock = NULL;
  int* intblock = NULL;
  mjtNum* qfrc_eff = NULL;

#define CARVE_NUM(lval, count) \
  do {                         \
    if (pass) {                \
      (lval) = numblock;       \
      numblock += (count);     \
    } else                     \
      nNum += (count);         \
  } while (0)

#define CARVE_INT(lval, count) \
  do {                         \
    if (pass) {                \
      (lval) = intblock;       \
      intblock += (count);     \
    } else                     \
      nInt += (count);         \
  } while (0)

  // CG: per-dof folded diagonal, for mj_effPrecFold
  int nep = !flg_Newton ? mj_effFoldDiagSize(m, d, ctx->island) : 0;

  for (int pass = 0; pass < 2; pass++) {
    if (pass) {
      // keep the folded diagonal only if the stack holds it with the other arrays and the
      // deepest frame the solve takes on top: the preconditioner or the metric product
      if (nep) {
        size_t scratch = mjMAX(mj_effPrecScratch(m, d), mj_effMulAddScratch(m, d));
        size_t need = mj_stackBytes(sizeof(mjtNum)*nNum, _Alignof(mjtNum)) +
                      mj_stackBytes(sizeof(int)*nInt, _Alignof(int)) + scratch;
        if (need > mj_stackBytesAvailable(d)) {
          nNum -= nep;
          nep = 0;
          mj_warning(d, mjWARN_CNSTRFULL, d->narena);
        }
      }
      numblock = mjSTACKALLOC(d, nNum, mjtNum);
      intblock = mjSTACKALLOC(d, nInt, int);
    }

    // island matrices
    if (copy_island) {
      // inertia
      CARVE_INT(ctx->M_rownnz,  nv);
      CARVE_INT(ctx->M_rowadr,  nv);
      CARVE_INT(ctx->M_colind,  nC);
      CARVE_NUM(ctx->M,         nC);
      CARVE_NUM(ctx->qLD,       nC);
      CARVE_NUM(ctx->qLDiagInv, nv);

      // Jacobians
      CARVE_NUM(ctx->J, nJ);
      if (is_sparse) {
        CARVE_INT(ctx->J_rownnz,   nefc);
        CARVE_INT(ctx->J_rowadr,   nefc);
        CARVE_INT(ctx->J_rowsuper, nefc);
        CARVE_INT(ctx->J_colind,   nJ);
      }
    }

    // common arrays
    CARVE_NUM(ctx->Jaref,  nefc);
    CARVE_NUM(ctx->Jv,     nefc);
    CARVE_NUM(ctx->Ma,     nv);
    CARVE_NUM(ctx->Mv,     nv);
    CARVE_NUM(ctx->grad,   nv);
    CARVE_NUM(ctx->Mgrad,  nv);
    CARVE_NUM(ctx->search, nv);
    if (is_elliptic) {
      CARVE_NUM(ctx->quad, 3*nefc);
    }
    if (is_sparse) {
      CARVE_NUM(ctx->JT, nJ);
    }

    // solver-specific arrays
    if (flg_Newton) {
      CARVE_NUM(ctx->D,       nefc);
      CARVE_NUM(ctx->cholupd, nv);
      if (is_elliptic) {
        CARVE_NUM(ctx->LTJ, 5*nv);
      }
      if (is_sparse) {
        _Static_assert(sizeof(mjtNum) >= sizeof(int), "cholscratch must hold nv ints");
        CARVE_NUM(ctx->cholscratch, nv);
      } else {
        ctx->nL = nv*nv;
        CARVE_NUM(ctx->L, ctx->nL);
        if (is_elliptic) {
          ctx->nH = nv*nv;
          CARVE_NUM(ctx->H, ctx->nH);
          CARVE_NUM(ctx->Lcone, ctx->nL);
          CARVE_INT(ctx->dense_cols, nv);
        }
      }
    } else {
      CARVE_NUM(ctx->gradold,  nv);
      CARVE_NUM(ctx->Mgradold, nv);
      CARVE_NUM(ctx->graddif,  nv);
      CARVE_NUM(ctx->Mgraddif, nv);
    }

    // constraint state and Jacobian transpose (sparse)
    CARVE_INT(ctx->oldstate, nefc);
    if (is_sparse) {
      CARVE_INT(ctx->JT_rownnz,   nv);
      CARVE_INT(ctx->JT_rowadr,   nv);
      CARVE_INT(ctx->JT_rowsuper, nv);
      CARVE_INT(ctx->JT_colind,   nJ);
    }

    // Newton sparse Hessian and factor sparsity
    if (flg_Newton && is_sparse) {
      CARVE_INT(ctx->H_rowadr,  nv);
      CARVE_INT(ctx->H_rownnz,  nv);
      CARVE_INT(ctx->HT_rownnz, nv);
      CARVE_INT(ctx->HT_rowadr, nv);
      CARVE_INT(ctx->L_rownnz,  nv);
      CARVE_INT(ctx->L_rowadr,  nv);
      CARVE_INT(ctx->LT_rownnz, nv);
      CARVE_INT(ctx->LT_rowadr, nv);
    }

    // discrete metric arrays
    if (d->efm_active) {
      if (flg_Newton) {
        CARVE_INT(ctx->S_rownnz, nv);
        CARVE_INT(ctx->S_rowadr, nv);
        CARVE_INT(ctx->S_colind, nScap);
        CARVE_NUM(ctx->S_val,    nScap);
      }
      CARVE_NUM(qfrc_eff, nv);
      ctx->epL = NULL;
      if (nep) {
        CARVE_NUM(ctx->epL, nep);
      }
    }
  }

#undef CARVE_NUM
#undef CARVE_INT

  // populate island matrices if needed
  if (copy_island) {
    int island = ctx->island;
    int idofadr = d->island_idofadr[island];
    int iefcadr = d->island_iefcadr[island];

    // under the discrete metric, gather the qH backbone factor of M + diag in place of
    // qLD: both are block-diagonal by tree, so the island block of the global factor is
    // the factor of the island block
    int use_qH = mj_effUseQH(d);
    const mjtNum* gLD = use_qH ? d->qH : d->qLD;
    const mjtNum* gLDiagInv = use_qH ? d->qHDiagInv : d->qLDiagInv;
    primalExtractIslandM(m, d, ctx, gLD, gLDiagInv);

    if (!is_sparse) {
      mju_block(ctx->J, d->efc_J, m->nv, nv, nefc,
                d->map_iefc2efc + iefcadr, d->map_idof2dof + idofadr);
    } else if (nefc) {
      primalExtractIslandJAndTranspose(d, ctx);
    } else {
      // no constraints: valid empty transpose structures
      mju_zeroInt(ctx->JT_rownnz, nv);
      mju_zeroInt(ctx->JT_rowadr, nv);
      mju_zeroInt(ctx->JT_rowsuper, nv);
    }
  }

  // sparse without island copy: compute Jacobian transpose using J supernodes
  else if (is_sparse) {
    if (nefc) {
      int offset = ctx->J_rowadr[0];
      mju_transposeSparse(ctx->JT, ctx->J + offset, nefc, nv,
                          ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper,
                          ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind + offset, ctx->J_rowsuper);
    } else {
      // no constraints (reachable via the flex CG dispatch): valid empty transpose structures
      mju_zeroInt(ctx->JT_rownnz, nv);
      mju_zeroInt(ctx->JT_rowadr, nv);
      mju_zeroInt(ctx->JT_rowsuper, nv);
    }
  }

  // implicit effective metric (built in mj_fwdPosition): route Ma/Mv/Mgrad through the
  // metric operators and shift the smooth force, so the stock objective/gradient/linesearch
  // formulas operate in the Mtilde = M + h*D + h^2*K metric; Newton additionally merges the
  // metric matrix into its Hessian (MakeHessian/FactorizeHessian). Island contexts see the
  // island-local view: gathered shift, island-mapped products, and the backbone factor of
  // M + diag gathered in place of qLD; flex never reaches the island path (monolithic solve
  // is forced for flex with metric terms)
  if (d->efm_active) {
    ctx->flg_metric = 1;
    ctx->fm = m;
    ctx->fd = d;
    if (ctx->island < 0) {
      mju_add(qfrc_eff, ctx->qfrc_smooth, d->efm_c, nv);
      if (d->efm_ca) {
        mju_addTo(qfrc_eff, d->efm_ca, nv);
      }
    } else {
      const int* idof2dof = d->map_idof2dof + d->island_idofadr[ctx->island];
      for (int k=0; k < nv; k++) {
        qfrc_eff[k] = ctx->qfrc_smooth[k] + d->efm_c[idof2dof[k]] +
                      (d->efm_ca ? d->efm_ca[idof2dof[k]] : 0);
      }
    }
    ctx->qfrc_smooth = qfrc_eff;
  }
}


// update efc_force, qfrc_constraint, cost-related
static void PrimalUpdateConstraint(mjPrimalContext* ctx) {
  int nefc = ctx->nefc, nv = ctx->nv;

  // update constraints
  mj_constraintUpdate_impl(ctx->ne, ctx->nf, ctx->nefc, ctx->efc_D, ctx->efc_R,
                           ctx->efc_frictionloss, ctx->Jaref, ctx->efc_type, ctx->efc_id,
                           ctx->is_elliptic ? ctx->contact : NULL,
                           ctx->efc_state, ctx->efc_force,
                           &(ctx->cost));

  // compute qfrc_constraint (dense or sparse)
  if (!ctx->is_sparse) {
    mju_mulMatTVec(ctx->qfrc_constraint, ctx->J, ctx->efc_force, nefc, nv);
  } else {
    mju_mulMatVecSparse(ctx->qfrc_constraint, ctx->JT, ctx->efc_force, nv,
                        ctx->JT_rownnz, ctx->JT_rowadr, ctx->JT_colind, ctx->JT_rowsuper);
  }

  // count active and cone
  ctx->nactive = 0;
  ctx->ncone = 0;
  if (!ctx->is_elliptic) {
    for (int i=0; i < nefc; i++) {
      ctx->nactive += (ctx->efc_state[i] != mjCNSTRSTATE_SATISFIED);
    }
  } else {
    for (int i=0; i < nefc; i++) {
      ctx->nactive += (ctx->efc_state[i] != mjCNSTRSTATE_SATISFIED);
      ctx->ncone += (ctx->efc_state[i] == mjCNSTRSTATE_CONE);
    }
  }

  // add Gauss cost, set in quadratic[0]
  mjtNum Gauss = 0;
  for (int i=0; i < nv; i++) {
    Gauss += 0.5 * (ctx->Ma[i] - ctx->qfrc_smooth[i]) * (ctx->qacc[i] - ctx->qacc_smooth[i]);
  }

  ctx->quadGauss[0] = Gauss;
  ctx->cost += Gauss;
}


// update grad = M*qacc - qfrc_smooth - qfrc_constraint
static void PrimalUpdateGrad(mjPrimalContext* ctx) {
  int nv = ctx->nv;
  for (int i=0; i < nv; i++) {
    ctx->grad[i] = ctx->Ma[i] - ctx->qfrc_smooth[i] - ctx->qfrc_constraint[i];
  }
}


// update Mgrad;  Newton: Mgrad = H \ grad,  CG: Mgrad = M \ grad
static void PrimalUpdateMgrad(mjPrimalContext* ctx, int flg_Newton) {
  int nv = ctx->nv;

  // Newton: Mgrad = H \ grad
  if (flg_Newton) {
    const mjtNum* L = ctx->ncone ? ctx->Lcone : ctx->L;
    if (ctx->is_sparse) {
      if (ctx->perm) {
        mjtNum* buf = ctx->cholscratch;
        for (int i=0; i < nv; i++) {
          buf[ctx->perm[i]] = ctx->grad[i];
        }
        mju_cholSolveSparse(buf, L, buf, nv, ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind);
        for (int i=0; i < nv; i++) {
          ctx->Mgrad[i] = buf[ctx->perm[i]];
        }
      } else {
        mju_cholSolveSparse(ctx->Mgrad, L, ctx->grad, nv,
                            ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind);
      }
    } else {
      mju_cholSolve(ctx->Mgrad, L, ctx->grad, nv);
    }
  }

  // CG monolithic: Mgrad = Mtilde \ grad (approximate metric preconditioner)
  else if (ctx->flg_metric && ctx->island < 0) {
    if (ctx->epL) {
      mj_effPrecBlocks(ctx->fm, ctx->fd, ctx->Mgrad, ctx->grad, ctx->epL);
    } else {
      mj_effPrec(ctx->fm, ctx->fd, ctx->Mgrad, ctx->grad);
    }
  }

  // CG island with metric: island-local metric preconditioner
  else if (ctx->flg_metric) {
    mj_effPrecIsland(ctx->fm, ctx->fd, ctx->Mgrad, ctx->grad, ctx->island, ctx->epL,
                     ctx->qLD, ctx->qLDiagInv, ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind);
  }

  // CG without metric: Mgrad = M \ grad
  else {
    mju_copy(ctx->Mgrad, ctx->grad, nv);
    mj_solveLD(ctx->Mgrad, ctx->qLD, ctx->qLDiagInv, nv, 1,
               ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind, NULL);
  }
}


// update grad, Mgrad
static void PrimalUpdateGradient(mjPrimalContext* ctx, int flg_Newton) {
  PrimalUpdateGrad(ctx);
  PrimalUpdateMgrad(ctx, flg_Newton);
}


// 36-element SoA layout in ctx->quad + 3*i for 4 contiguous dim=3 elliptic contacts (4*3*3 = 36)
enum {
  CONE4_Q0 = 0,  CONE4_Q1 = 4,  CONE4_Q2 = 8,
  CONE4_U0 = 12, CONE4_T0 = 16, CONE4_DM = 20,
  CONE4_MU = 24, CONE4_F0 = 28, CONE4_F1 = 32
};


// 4-wide batched preparation for contiguous dim=3 elliptic contacts
static void PrimalPrepareElliptic4(mjPrimalContext* ctx) {
  mjtNum4 vhalf = v4_splat(0.5);
  mjtNum4 vone = v4_splat(1);
  for (int i = ctx->efc_cone3_start; i < ctx->efc_cone3_end; i += 12) {
    mjtNum4 J0, J1, J2, K0, K1, K2, D0, D1, D2;
    v4_load4x3(ctx->Jaref + i, &J0, &J1, &J2);
    v4_load4x3(ctx->Jv + i, &K0, &K1, &K2);
    v4_load4x3(ctx->efc_D + i, &D0, &D1, &D2);

    const mjContact* c0 = ctx->contact + ctx->efc_id[i+0];
    const mjContact* c1 = ctx->contact + ctx->efc_id[i+3];
    const mjContact* c2 = ctx->contact + ctx->efc_id[i+6];
    const mjContact* c3 = ctx->contact + ctx->efc_id[i+9];

    mjtNum4 vmu = v4_set(c0->mu, c1->mu, c2->mu, c3->mu);
    mjtNum4 vf0 = v4_set(c0->friction[0], c1->friction[0], c2->friction[0], c3->friction[0]);
    mjtNum4 vf1 = v4_set(c0->friction[1], c1->friction[1], c2->friction[1], c3->friction[1]);

    // bottom-zone quadratic coefficients (q0, q1, q2)
    mjtNum4 DJ0 = v4_mul(D0, J0);
    mjtNum4 DJ1 = v4_mul(D1, J1);
    mjtNum4 DJ2 = v4_mul(D2, J2);
    mjtNum4 vq0 = v4_mul(vhalf, v4_add(v4_add(v4_mul(J0, DJ0), v4_mul(J1, DJ1)),
                                       v4_mul(J2, DJ2)));
    mjtNum4 vq1 = v4_add(v4_add(v4_mul(K0, DJ0), v4_mul(K1, DJ1)), v4_mul(K2, DJ2));
    mjtNum4 vq2 = v4_mul(vhalf, v4_add(v4_add(v4_mul(v4_mul(K0, D0), K0),
                                              v4_mul(v4_mul(K1, D1), K1)),
                                       v4_mul(v4_mul(K2, D2), K2)));

    // cone quantities at alpha=0 (normal U0, tangential norm T0, middle-zone metric Dm)
    mjtNum4 vU0 = v4_mul(J0, vmu);
    mjtNum4 vU1 = v4_mul(J1, vf0);
    mjtNum4 vU2 = v4_mul(J2, vf1);
    mjtNum4 vT0 = v4_sqrt(v4_add(v4_mul(vU1, vU1), v4_mul(vU2, vU2)));
    mjtNum4 mu2 = v4_mul(vmu, vmu);
    mjtNum4 vDm = v4_div(D0, v4_mul(mu2, v4_add(vone, mu2)));

    mjtNum* w = ctx->quad + 3*i;
    v4_store(w + CONE4_Q0, vq0);
    v4_store(w + CONE4_Q1, vq1);
    v4_store(w + CONE4_Q2, vq2);
    v4_store(w + CONE4_U0, vU0);
    v4_store(w + CONE4_T0, vT0);
    v4_store(w + CONE4_DM, vDm);
    v4_store(w + CONE4_MU, vmu);
    v4_store(w + CONE4_F0, vf0);
    v4_store(w + CONE4_F1, vf1);
  }
}


// prepare quadratic polynomials and contact cone quantities
static void PrimalPrepare(mjPrimalContext* ctx) {
  int nv = ctx->nv, ne = ctx->ne, nefc = ctx->nefc;
  const mjtNum* v = ctx->search;

  // Gauss: alpha^2*0.5*v'*M*v + alpha*v'*(Ma-qfrc_smooth) + 0.5*(a-qacc_smooth)'*(Ma-qfrc_smooth)
  //  quadGauss[0] already computed in PrimalUpdateConstraint
  mjtNum qg1 = mju_dot(v, ctx->Ma, nv) - mju_dot(ctx->qfrc_smooth, v, nv);
  mjtNum qg2 = 0.5*mju_dot(v, ctx->Mv, nv);

  // fold alpha-invariant equality rows directly into quadGauss[1..2] once per linesearch
  if (ne > 0) {
    mjtNum4 vhalf = v4_splat(0.5);
    mjtNum4 eq1_v = v4_splat(0), eq2_v = v4_splat(0);
    int i = 0;
    for (; i + 4 <= ne; i += 4) {
      mjtNum4 vj0 = v4_load(ctx->Jaref + i);
      mjtNum4 vjv = v4_load(ctx->Jv + i);
      mjtNum4 vd  = v4_load(ctx->efc_D + i);
      eq1_v = v4_add(eq1_v, v4_mul(vjv, v4_mul(vd, vj0)));
      eq2_v = v4_add(eq2_v, v4_mul(vhalf, v4_mul(v4_mul(vjv, vd), vjv)));
    }
    qg1 += v4_sum(eq1_v);
    qg2 += v4_sum(eq2_v);
    for (; i < ne; i++) {
      mjtNum Jv = ctx->Jv[i];
      mjtNum D = ctx->efc_D[i];
      qg1 += Jv*(D*ctx->Jaref[i]);
      qg2 += 0.5*(Jv*D*Jv);
    }
  }
  ctx->quadGauss[1] = qg1;
  ctx->quadGauss[2] = qg2;

  // fast path: no elliptic cones (unilateral rows are evaluated directly from Jaref, Jv, efc_D)
  if (!ctx->is_elliptic) {
    return;
  }

  // 4-wide batched preparation for contiguous dim=3 elliptic contacts
  PrimalPrepareElliptic4(ctx);

  // process remaining elliptic cones outside 4-wide dim=3 span
  for (int i = ctx->efc_cone_start; i < nefc; i++) {
    if (i == ctx->efc_cone3_start) {
      i = ctx->efc_cone3_end;
      if (i >= nefc) {
        break;
      }
    }
    if (ctx->efc_type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
      continue;
    }

    // pointers to numeric data
    const mjtNum* Jv = ctx->Jv + i;
    const mjtNum* Jaref = ctx->Jaref + i;
    const mjtNum* D = ctx->efc_D + i;
    mjtNum* quad = ctx->quad + 3*i;

    // init with normal row quadratic
    mjtNum DJ0 = D[0]*Jaref[0];
    quad[0] = Jaref[0]*DJ0;
    quad[1] = Jv[0]*DJ0;
    quad[2] = Jv[0]*D[0]*Jv[0];

    // extract contact info
    const mjContact* con = ctx->contact + ctx->efc_id[i];
    int dim = con->dim;
    mjtNum UU = 0, mu = con->mu;
    const mjtNum* friction = con->friction;

    // complete vector quadratic (for bottom zone) and tangential norm at alpha=0
    for (int j=1; j < dim; j++) {
      mjtNum DJj = D[j]*Jaref[j];
      quad[0] += Jaref[j]*DJj;
      quad[1] += Jv[j]*DJj;
      quad[2] += Jv[j]*D[j]*Jv[j];
      mjtNum Uj = Jaref[j]*friction[j-1];
      UU += Uj*Uj;
    }

    // store cone quantities in quad[3-6], using the fact that dim>=3
    quad[3] = Jaref[0]*mu;
    quad[4] = Jv[0]*mu;
    quad[5] = mju_sqrt(UU);
    quad[6] = D[0] / ((mu*mu) * (1 + (mu*mu)));

    // apply scaling and advance to next constraint
    quad[0] *= 0.5;
    quad[2] *= 0.5;
    i += (dim-1);
  }
}


// linesearch evaluation point
struct _mjPrimalPnt {
  mjtNum alpha;
  mjtNum cost;
  mjtNum deriv[2];
};
typedef struct _mjPrimalPnt mjPrimalPnt;


// Huber cost of a single friction constraint at a given point x
static mjtNum frictionCost(mjtNum x, mjtNum f, mjtNum Rf, mjtNum D) {
  // -bound < x < bound : quadratic
  if (-Rf < x && x < Rf) return 0.5*D*x*x;

  // x < -bound : linear negative
  else if (x <= -Rf) return f*(-0.5*Rf - x);

  // bound < x : linear positive
  else return f*(-0.5*Rf + x);
}


// Huber cost difference of a single friction constraint: cost(x) - cost(start)
static mjtNum frictionCostDif(mjtNum start, mjtNum x, mjtNum f, mjtNum Rf, mjtNum D) {
  int state_start = (-Rf < start && start < Rf) ? 0 : (start <= -Rf ? -1 : 1);
  int state_x = (-Rf < x && x < Rf) ? 0 : (x <= -Rf ? -1 : 1);

  // both quadratic
  if (state_start == 0 && state_x == 0) {
    return 0.5*D*(x - start)*(x + start);
  }

  // both linear negative
  if (state_start == -1 && state_x == -1) {
    return f*(start - x);
  }

  // both linear positive
  if (state_start == 1 && state_x == 1) {
    return f*(x - start);
  }

  // otherwise different zones: compute absolute costs and subtract
  return frictionCost(x, f, Rf, D) - frictionCost(start, f, Rf, D);
}


// compute cost difference of an elliptic cone at a given alpha: cost(alpha) - cost(0)
static mjtNum ellipticCostDif(const mjtNum* quad, mjtNum alpha, mjtNum mu,
                              mjtNum N, mjtNum T, mjtNum Tsqr_delta) {
  mjtNum U0 = quad[3], V0 = quad[4], T0 = quad[5], Dm = quad[6];

  // determine zone at alpha=0
  int zone0 = 0;
  if (T0 <= 0) {
    zone0 = (U0 < 0) ? 2 : 1;
  } else if (U0 >= mu*T0) {
    zone0 = 1;  // top zone
  } else if (mu*U0 + T0 <= 0) {
    zone0 = 2;  // bottom zone
  } else {
    zone0 = 3;  // middle zone
  }

  // determine zone at alpha
  int zone_alpha = 0;
  if (T <= 0) {
    zone_alpha = (N < 0) ? 2 : 1;  // bottom or top zone
  } else if (N >= mu*T) {
    zone_alpha = 1;  // top zone
  } else if (mu*N + T <= 0) {
    zone_alpha = 2;  // bottom zone
  } else {
    zone_alpha = 3;  // middle zone
  }

  // both top zone
  if (zone0 == 1 && zone_alpha == 1) {
    return 0;
  }

  // both bottom zone
  if (zone0 == 2 && zone_alpha == 2) {
    return alpha*alpha*quad[2] + alpha*quad[1];
  }

  // both middle zone: apply rationalized formula to avoid cancellation
  if (zone0 == 3 && zone_alpha == 3) {
    mjtNum T_delta = Tsqr_delta / (T + T0);
    mjtNum r_delta = alpha*V0 - mu*T_delta;
    mjtNum r0 = U0 - mu*T0;
    return 0.5*Dm*r_delta*(2*r0 + r_delta);
  }

  // CONE -> QUADRATIC (3 -> 2)
  if (zone0 == 3 && zone_alpha == 2) {
    mjtNum dq = alpha*(alpha*quad[2] + quad[1]);
    mjtNum boundary0 = mu*U0 + T0;
    mjtNum gap0 = 0.5*Dm*boundary0*boundary0;
    return dq + gap0;
  }

  // QUADRATIC -> CONE (2 -> 3)
  if (zone0 == 2 && zone_alpha == 3) {
    mjtNum dq = alpha*(alpha*quad[2] + quad[1]);
    mjtNum boundary = mu*N + T;
    mjtNum gap = 0.5*Dm*boundary*boundary;
    return dq - gap;
  }

  // SATISFIED -> QUADRATIC (1 -> 2)
  if (zone0 == 1 && zone_alpha == 2) {
    return alpha*alpha*quad[2] + alpha*quad[1] + quad[0];
  }

  // SATISFIED -> CONE (1 -> 3)
  if (zone0 == 1 && zone_alpha == 3) {
    mjtNum r = N - mu*T;
    return 0.5*Dm*r*r;
  }

  // CONE -> SATISFIED (3 -> 1)
  if (zone0 == 3 && zone_alpha == 1) {
    mjtNum r0 = U0 - mu*T0;
    return -0.5*Dm*r0*r0;
  }

  // QUADRATIC -> SATISFIED (2 -> 1)
  if (zone0 == 2 && zone_alpha == 1) {
    return -quad[0];
  }

  return 0;
}


// branchless 4-wide evaluation for unilateral prefix [ne+nf, efc_cone_start)
//  uses factored difference-of-squares cost to avoid cancellation on active-set crossings
static void PrimalEvalUnilateral(const mjPrimalContext* ctx, mjtNum alpha,
                                 mjtNum* qg1, mjtNum* qg2, mjtNum* cost, mjtNum deriv[2]) {
  int u0 = ctx->ne + ctx->nf;
  int nuni = ctx->efc_cone_start - u0;
  if (nuni <= 0) {
    return;
  }

  const mjtNum* Jaref_u = ctx->Jaref + u0;
  const mjtNum* Jv_u = ctx->Jv + u0;
  const mjtNum* D_u = ctx->efc_D + u0;
  int k = 0;

  if (nuni >= 4) {
    mjtNum qg_d0 = 0.25 * (2*alpha*(*qg2) + (*qg1));
    mjtNum4 valpha = v4_splat(alpha);
    mjtNum4 vhalf = v4_splat(0.5);
    mjtNum4 vzero = v4_splat(0);
    mjtNum4 acc_c = vzero;
    mjtNum4 acc_d0 = v4_splat(qg_d0);
    mjtNum4 acc_d1 = vzero;

    for (; k + 4 <= nuni; k += 4) {
      mjtNum4 vstart = v4_load(Jaref_u + k);
      mjtNum4 vjv    = v4_load(Jv_u + k);
      mjtNum4 vd     = v4_load(D_u + k);
      mjtNum4 vstep  = v4_mul(valpha, vjv);
      mjtNum4 vx     = v4_add(vstart, vstep);
      mjtMask4 m_s   = v4_cmplt(vstart, vzero);
      mjtMask4 m_x   = v4_cmplt(vx, vzero);

      mjtNum4 vdjv = v4_mul(vd, vjv);
      acc_d0 = v4_add(acc_d0, v4_select(m_x, v4_mul(vx, vdjv), vzero));
      acc_d1 = v4_add(acc_d1, v4_select(m_x, v4_mul(vjv, vdjv), vzero));

      // cost delta = 0.5*d*va*vb: step*(start+x) if both active, x^2 if entering, -start^2 if leaving
      mjtNum4 va = v4_select(m_x, v4_select(m_s, vstep, vx), v4_sub(vzero, vstart));
      mjtNum4 vb = v4_select(m_x, v4_select(m_s, v4_add(vstart, vx), vx), vstart);
      mjtMask4 m_any = v4_or(m_s, m_x);
      acc_c = v4_add(acc_c, v4_select(m_any, v4_mul(vhalf, v4_mul(vd, v4_mul(va, vb))), vzero));
    }

    *cost += v4_sum(acc_c) + alpha*(alpha*(*qg2) + (*qg1));
    deriv[0] += v4_sum(acc_d0);
    deriv[1] += v4_sum(acc_d1) + 2*(*qg2);
    *qg1 = 0;
    *qg2 = 0;
  }

  for (; k < nuni; k++) {
    mjtNum start = Jaref_u[k];
    mjtNum jv = Jv_u[k];
    mjtNum d = D_u[k];
    mjtNum step = alpha*jv;
    mjtNum x = start + step;
    if (x < 0) {
      mjtNum djv = d*jv;
      deriv[0] += x*djv;
      deriv[1] += jv*djv;
      *cost += (start < 0) ? 0.5*d*step*(start + x) : 0.5*d*x*x;
    } else if (start < 0) {
      *cost -= 0.5*d*start*start;
    }
  }
}


// 4-wide batched evaluation for contiguous dim=3 elliptic cones [efc_cone3_start, efc_cone3_end)
static void PrimalEvalElliptic4(const mjPrimalContext* ctx, mjtNum alpha,
                                mjtNum* cost, mjtNum deriv[2]) {
  if (ctx->efc_cone3_start >= ctx->efc_cone3_end) {
    return;
  }

  mjtNum4 valpha = v4_splat(alpha);
  mjtNum4 vzero = v4_splat(0);
  mjtNum4 acc_cost = vzero, acc_d0 = vzero, acc_d1 = vzero;

  for (int i = ctx->efc_cone3_start; i < ctx->efc_cone3_end; i += 12) {
    const mjtNum* w = ctx->quad + 3*i;

    mjtNum4 J0, J1, J2, K0, K1, K2;
    v4_load4x3(ctx->Jaref + i, &J0, &J1, &J2);
    v4_load4x3(ctx->Jv + i, &K0, &K1, &K2);
    (void)J0;  // J0 already scaled into CONE4_U0

    // normal (N) and tangential (W1, W2, T) cone coordinates at alpha
    mjtNum4 vf0 = v4_load(w + CONE4_F0);
    mjtNum4 vf1 = v4_load(w + CONE4_F1);
    mjtNum4 vU1 = v4_mul(J1, vf0);
    mjtNum4 vV1 = v4_mul(K1, vf0);
    mjtNum4 vW1 = v4_add(vU1, v4_mul(valpha, vV1));

    mjtNum4 vU2 = v4_mul(J2, vf1);
    mjtNum4 vV2 = v4_mul(K2, vf1);
    mjtNum4 vW2 = v4_add(vU2, v4_mul(valpha, vV2));

    mjtNum4 vmu = v4_load(w + CONE4_MU);
    mjtNum4 vU0 = v4_load(w + CONE4_U0);
    mjtNum4 vV0 = v4_mul(K0, vmu);
    mjtNum4 vN  = v4_add(vU0, v4_mul(valpha, vV0));

    mjtNum4 vTsqr = v4_add(v4_mul(vW1, vW1), v4_mul(vW2, vW2));
    mjtNum4 vT = v4_sqrt(vTsqr);
    mjtNum4 vT0 = v4_load(w + CONE4_T0);

    // zone masks at alpha=0 and alpha (for T >= 0 and mu >= 0, U >= mu*T covers T == 0)
    mjtMask4 m_top0 = v4_cmpge(vU0, v4_mul(vmu, vT0));
    mjtMask4 m_bot0 = v4_andnot(m_top0, v4_or(v4_cmple(vT0, vzero),
                                              v4_cmple(v4_add(v4_mul(vmu, vU0), vT0), vzero)));
    mjtMask4 m_mid0 = v4_not(v4_or(m_top0, m_bot0));

    mjtMask4 m_top = v4_cmpge(vN, v4_mul(vmu, vT));
    mjtMask4 m_bot = v4_andnot(m_top, v4_or(v4_cmple(vTsqr, vzero),
                                            v4_cmple(v4_add(v4_mul(vmu, vN), vT), vzero)));
    mjtMask4 m_mid = v4_not(v4_or(m_top, m_bot));

    // bottom-zone quadratic cost delta and derivatives
    mjtNum4 vq1 = v4_load(w + CONE4_Q1);
    mjtNum4 vq2 = v4_load(w + CONE4_Q2);
    mjtNum4 vdq = v4_mul(valpha, v4_add(v4_mul(valpha, vq2), vq1));
    mjtNum4 vc  = v4_select(v4_and(m_bot0, m_bot), vdq, vzero);
    mjtNum4 v2q2 = v4_add(vq2, vq2);
    mjtNum4 vd0 = v4_select(m_bot, v4_add(v4_mul(valpha, v2q2), vq1), vzero);
    mjtNum4 vd1 = v4_select(m_bot, v2q2, vzero);

    // middle -> middle: factored cost delta avoiding sqrt cancellation
    mjtMask4 m_mid0_mid = v4_and(m_mid0, m_mid);
    if (v4_any(m_mid0_mid)) {
      mjtNum4 vDm = v4_load(w + CONE4_DM);
      mjtNum4 vTsqr_delta = v4_add(v4_mul(v4_mul(valpha, vV1), v4_add(vU1, vW1)),
                                   v4_mul(v4_mul(valpha, vV2), v4_add(vU2, vW2)));
      mjtNum4 safe_sum = v4_select(m_mid0_mid, v4_add(vT, vT0), v4_splat(1));
      mjtNum4 vT_delta = v4_div(v4_select(m_mid0_mid, vTsqr_delta, vzero), safe_sum);
      mjtNum4 vr_delta = v4_sub(v4_mul(valpha, vV0), v4_mul(vmu, vT_delta));
      mjtNum4 vr0 = v4_sub(vU0, v4_mul(vmu, vT0));
      mjtNum4 vc_mid = v4_mul(v4_splat(0.5),
                              v4_mul(vDm, v4_mul(vr_delta, v4_add(v4_add(vr0, vr0), vr_delta))));
      vc = v4_add(vc, v4_select(m_mid0_mid, vc_mid, vzero));
    }

    // zone crossing: exact cost delta for all 6 transitions between {top, mid, bot}
    mjtMask4 m_cross = v4_or(v4_xor(m_top0, m_top), v4_xor(m_bot0, m_bot));
    if (v4_any(m_cross)) {
      mjtNum4 vq0  = v4_load(w + CONE4_Q0);
      mjtNum4 vDm  = v4_load(w + CONE4_DM);
      mjtNum4 vhalf = v4_splat(0.5);
      mjtNum4 bnd0 = v4_add(v4_mul(vmu, vU0), vT0);
      mjtNum4 gap0 = v4_mul(vhalf, v4_mul(vDm, v4_mul(bnd0, bnd0)));
      mjtNum4 bnd  = v4_add(v4_mul(vmu, vN), vT);
      mjtNum4 gap  = v4_mul(vhalf, v4_mul(vDm, v4_mul(bnd, bnd)));
      mjtNum4 vr0  = v4_sub(vU0, v4_mul(vmu, vT0));
      mjtNum4 vr   = v4_sub(vN, v4_mul(vmu, vT));
      mjtNum4 cm0  = v4_mul(vhalf, v4_mul(vDm, v4_mul(vr0, vr0)));
      mjtNum4 cm   = v4_mul(vhalf, v4_mul(vDm, v4_mul(vr, vr)));

      vc = v4_add(vc, v4_select(v4_and(m_mid0, m_bot), v4_add(vdq, gap0), vzero));
      vc = v4_add(vc, v4_select(v4_and(m_bot0, m_mid), v4_sub(vdq, gap), vzero));
      vc = v4_add(vc, v4_select(v4_and(m_top0, m_bot), v4_add(vdq, vq0), vzero));
      vc = v4_add(vc, v4_select(v4_and(m_top0, m_mid), cm, vzero));
      vc = v4_sub(vc, v4_select(v4_and(m_mid0, m_top), cm0, vzero));
      vc = v4_sub(vc, v4_select(v4_and(m_bot0, m_top), vq0, vzero));
    }
    acc_cost = v4_add(acc_cost, vc);

    // middle-zone derivatives (skip division if no lane is in middle zone)
    if (v4_any(m_mid)) {
      mjtNum4 vDm    = v4_load(w + CONE4_DM);
      mjtNum4 safe_T = v4_select(m_mid, vT, v4_splat(1));
      mjtNum4 vTinv  = v4_select(m_mid, v4_div(v4_splat(1), safe_T), vzero);
      mjtNum4 vWV    = v4_add(v4_mul(vW1, vV1), v4_mul(vW2, vV2));
      mjtNum4 What1  = v4_mul(vW1, vTinv);
      mjtNum4 What2  = v4_mul(vW2, vTinv);
      mjtNum4 d01    = v4_sub(v4_mul(What1, vV2), v4_mul(What2, vV1));
      mjtNum4 det    = v4_mul(d01, d01);
      mjtNum4 vT1    = v4_mul(vWV, vTinv);
      mjtNum4 vNT    = v4_mul(vN, vTinv);
      mjtNum4 dNmT   = v4_sub(vV0, v4_mul(vmu, vT1));
      mjtNum4 mid_d0 = v4_mul(vDm, v4_mul(v4_sub(vN, v4_mul(vmu, vT)), dNmT));
      mjtNum4 mid_d1 = v4_mul(vDm, v4_sub(v4_mul(dNmT, dNmT),
                                          v4_mul(v4_sub(vNT, vmu), v4_mul(vmu, det))));
      vd0 = v4_add(vd0, v4_select(m_mid, mid_d0, vzero));
      vd1 = v4_add(vd1, v4_select(m_mid, mid_d1, vzero));
    }
    acc_d0 = v4_add(acc_d0, vd0);
    acc_d1 = v4_add(acc_d1, vd1);
  }

  *cost += v4_sum(acc_cost);
  deriv[0] += v4_sum(acc_d0);
  deriv[1] += v4_sum(acc_d1);
}


// evaluate shifted linesearch cost: cost(alpha) - cost(0), and derivatives
static void PrimalEval(mjPrimalContext* ctx, mjPrimalPnt* p) {
  int ne = ctx->ne, nf = ctx->nf, nefc = ctx->nefc;

  // clear result
  mjtNum cost = 0, alpha = p->alpha;
  mjtNum deriv[2] = {0, 0};

  // init quad with Gauss + equality (already folded in PrimalPrepare), shifted: drop quadGauss[0]
  mjtNum qg1 = ctx->quadGauss[1], qg2 = ctx->quadGauss[2];

  // friction: compute cost(alpha) - cost(0) directly
  for (int i=ne; i < ne + nf; i++) {
    // search point, friction loss, bound (Rf)
    mjtNum start = ctx->Jaref[i];
    mjtNum dir = ctx->Jv[i];
    mjtNum x = start + alpha*dir;
    mjtNum f = ctx->efc_frictionloss[i];
    mjtNum D = ctx->efc_D[i];
    mjtNum Rf = ctx->efc_R[i]*f;

    // cost delta
    cost += frictionCostDif(start, x, f, Rf, D);

    // -bound < x < bound : quadratic
    if (-Rf < x && x < Rf) {
      deriv[0] += D*x*dir;
      deriv[1] += D*dir*dir;
    }

    // x < -bound : linear negative
    else if (x <= -Rf) {
      deriv[0] += -f*dir;
    }

    // bound < x : linear positive
    else {
      deriv[0] += f*dir;
    }
  }

  // unilateral prefix [ne+nf, efc_cone_start)
  PrimalEvalUnilateral(ctx, alpha, &qg1, &qg2, &cost, deriv);

  // elliptic cones
  if (ctx->is_elliptic) {
    // 4-wide batched evaluation for contiguous dim=3 elliptic cones
    PrimalEvalElliptic4(ctx, alpha, &cost, deriv);

    // general path for remaining / non-dim3 constraints
    for (int i = ctx->efc_cone_start; i < nefc; i++) {
      if (i == ctx->efc_cone3_start) {
        i = ctx->efc_cone3_end;
        if (i >= nefc) {
          break;
        }
      }
      if (ctx->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {       // elliptic cone
        // extract contact info
        const mjContact* con = ctx->contact + ctx->efc_id[i];
        const mjtNum* Jaref = ctx->Jaref + i;
        const mjtNum* Jv = ctx->Jv + i;
        const mjtNum* friction = con->friction;
        mjtNum* quad = ctx->quad + 3*i;
        int dim = con->dim;
        mjtNum mu = con->mu;

        // unpack quad
        mjtNum U0 = quad[3];
        mjtNum V0 = quad[4];
        mjtNum Dm = quad[6];

        // normal and tangential components at alpha (formed componentwise before squaring)
        mjtNum N = U0 + alpha*V0;
        mjtNum V[5], W[5];
        mjtNum Tsqr = 0, WV = 0, Tsqr_delta = 0;
        for (int j=0; j < dim-1; j++) {
          mjtNum Uj = Jaref[j+1]*friction[j];
          mjtNum Vj = Jv[j+1]*friction[j];
          mjtNum Wj = Uj + alpha*Vj;
          V[j] = Vj;
          W[j] = Wj;
          Tsqr += Wj*Wj;
          WV += Wj*Vj;
          Tsqr_delta += alpha*Vj*(Uj + Wj);
        }
        mjtNum T = mju_sqrt(Tsqr);

        // shifted cost
        cost += ellipticCostDif(quad, alpha, mu, N, T, Tsqr_delta);

        // no tangential force : top or bottom zone
        if (Tsqr <= 0) {
          // bottom zone: quadratic derivatives
          if (N < 0) {
            deriv[0] += 2*alpha*quad[2] + quad[1];
            deriv[1] += 2*quad[2];
          }

          // top zone: nothing to do
        }

        // otherwise regular processing
        else {
          // N>=mu*T : top zone
          if (N >= mu*T) {
            // nothing to do
          }

          // mu*N+T<=0 : bottom zone
          else if (mu*N+T <= 0) {
            deriv[0] += 2*alpha*quad[2] + quad[1];
            deriv[1] += 2*quad[2];
          }

          // otherwise middle zone
          else {
            // unit tangent What = W / T
            mjtNum What[5];
            for (int j=0; j < dim-1; j++) {
              What[j] = W[j] / T;
            }

            // cross-product norm squared ||What ^ V||^2 via Lagrange identity to avoid cancellation
            mjtNum det = 0;
            for (int j=0; j < dim-1; j++) {
              for (int k=j+1; k < dim-1; k++) {
                mjtNum djk = What[j]*V[k] - What[k]*V[j];
                det += djk*djk;
              }
            }

            // derivatives
            mjtNum N1 = V0;
            mjtNum T1 = WV / T;
            deriv[0] += Dm*(N-mu*T)*(N1-mu*T1);
            deriv[1] += Dm*((N1-mu*T1)*(N1-mu*T1) + (N/T - mu)*(-mu*det));
          }
        }

        // advance to next constraint
        i += (dim-1);
      } else {                                                  // inequality
        // search point
        mjtNum start = ctx->Jaref[i];
        mjtNum jv = ctx->Jv[i];
        mjtNum d = ctx->efc_D[i];
        mjtNum step = alpha*jv;
        mjtNum x = start + step;

        // active
        if (x < 0) {
          mjtNum djv = d*jv;
          deriv[0] += x*djv;
          deriv[1] += jv*djv;
          cost += (start < 0) ? 0.5*d*step*(start + x) : 0.5*d*x*x;
        } else if (start < 0) {
          cost -= 0.5*d*start*start;
        }
      }
    }
  }

  // add remaining Gauss + equality quadratic
  cost += alpha*alpha*qg2 + alpha*qg1;
  deriv[0] += 2*alpha*qg2 + qg1;
  deriv[1] += 2*qg2;

  // check for convexity; SHOULD NOT OCCUR
  if (deriv[1] <= 0) {
    mju_warning("Linesearch objective is not convex");
    deriv[1] = mjMINVAL;
  }

  // assign and count
  p->cost = cost;
  p->deriv[0] = deriv[0];
  p->deriv[1] = deriv[1];
  ctx->LSiter++;
}


// update bracket point given 3 candidate points
static int updateBracket(mjPrimalContext* ctx,
                         mjPrimalPnt* p, const mjPrimalPnt candidates[3], mjPrimalPnt* pnext) {
  int flag = 0;
  for (int i=0; i < 3; i++) {
    // negative deriv
    if (p->deriv[0] < 0 && candidates[i].deriv[0] < 0 && p->deriv[0] < candidates[i].deriv[0]) {
      *p = candidates[i];
      flag = 1;
    }

    // positive deriv
    else if (p->deriv[0] > 0 &&
             candidates[i].deriv[0] > 0 &&
             p->deriv[0] > candidates[i].deriv[0]) {
      *p = candidates[i];
      flag = 2;
    }
  }

  // compute next point if updated
  if (flag) {
    pnext->alpha = p->alpha - p->deriv[0]/p->deriv[1];
    PrimalEval(ctx, pnext);
  }

  return flag;
}


// line search
static mjtNum PrimalSearch(mjPrimalContext* ctx, mjtNum tolerance, mjtNum ls_iterations,
                           mjtNum* improvement) {
  int nv = ctx->nv, nefc = ctx->nefc;
  mjPrimalPnt p0, p1, p2, pmid, p1next, p2next;

  // clear results
  ctx->LSiter = 0;
  ctx->LSresult = 0;
  ctx->LSslope = 1;       // means not computed
  *improvement = 0;

  // save search vector length, check
  mjtNum snorm = mju_norm(ctx->search, nv);
  if (snorm < mjMINVAL) {
    ctx->LSresult = 1;                          // search vector too small
    return 0;
  }

  // compute scaled gradtol and slope scaling
  mjtNum gtol = tolerance * snorm / ctx->scale;
  mjtNum slopescl = ctx->scale / snorm;

  // compute Mv = Mtilde * v
  mju_mulSymVecSparse(ctx->Mv, ctx->M, ctx->search, nv,
                      ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind);
  if (ctx->flg_metric) {
    if (ctx->island < 0) {
      mj_effMulAdd(ctx->fm, ctx->fd, ctx->Mv, ctx->search, /*flg_contact=*/1);
    } else {
      mj_effMulAddIsland(ctx->fm, ctx->fd, ctx->Mv, ctx->search, ctx->island);
    }
  }

  // compute Jv = J * search  (dense or sparse)
  if (!ctx->is_sparse) {
    mju_mulMatVec(ctx->Jv, ctx->J, ctx->search, nefc, nv);
  } else {
    mju_mulMatVecSparse(ctx->Jv, ctx->J, ctx->search, nefc,
                        ctx->J_rownnz, ctx->J_rowadr, ctx->J_colind, ctx->J_rowsuper);
  }

  // prepare quadratics and cones
  PrimalPrepare(ctx);

  // init at alpha = 0, save
  p0.alpha = 0;
  PrimalEval(ctx, &p0);

  // always attempt one Newton step
  p1.alpha = p0.alpha - p0.deriv[0]/p0.deriv[1];
  PrimalEval(ctx, &p1);

  // check for initial convergence
  if (mju_abs(p1.deriv[0]) < gtol && (p1.alpha == 0 || p1.cost < 0)) {
    if (p1.alpha == 0) {
      ctx->LSresult = 2;  // no improvement, initial convergence
    } else {
      ctx->LSresult = 0;  // SUCCESS
    }
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    *improvement = -p1.cost;
    return p1.alpha;
  }

  // save direction
  int dir = (p1.deriv[0] < 0 ? +1 : -1);

  // SANITY CHECKS
  /*
     // descent direction
     if( mju_dot(ctx->grad, ctx->search, m->nv)>=0 )
      printf("NOT A DESCENT:  grad %g   search %g   dot %g\n",
          mju_norm(ctx->grad, m->nv),
          mju_norm(ctx->search, m->nv),
          mju_dot(ctx->grad, ctx->search, m->nv));

     // 2nd derivative for Newton cone
     if( ctx->flg_Newton && ctx->ncone )
     {
      mjtNum dd = -p0.deriv[0]/p0.deriv[1];

      if( mju_abs(dd-1)>1e-6 )
          printf("2nd DERIVATIVE FAIL: d0  %g   d1  %g   alpha %g\n",
              p0.deriv[0], p0.deriv[1], dd);
     }

     // cost and gradient at 0: full-space vs. linesearch
     mjtNum grd = mju_dot(ctx->grad, ctx->search, m->nv);
     if( mju_abs(p0.cost-ctx->cost)/mjMAX(mjMINVAL,mju_abs(p0.cost+ctx->cost)) > 1e-6 ||
      mju_abs(p0.deriv[0]-grd)/mjMAX(mjMINVAL,mju_abs(p0.deriv[0]+grd)) > 1e-6 )
     {
      printf("LSiter = %d:\n", ctx->LSiter);
      printf("COST:  %g  %g  %g\n",
          p0.cost, ctx->cost,
          mju_abs(p0.cost-ctx->cost)/mjMAX(mjMINVAL,mju_abs(p0.cost+ctx->cost)));
      printf("GRAD:  %g  %g  %g\n",
          p0.deriv[0], grd,
          mju_abs(p0.deriv[0]-grd)/mjMAX(mjMINVAL,mju_abs(p0.deriv[0]+grd)));
     }
   */

  // one-sided search
  p2 = p0;
  int p2update = 1;
  while (p1.deriv[0]*dir <= -gtol && ctx->LSiter < ls_iterations) {
    // save current
    p2 = p1;
    p2update = 1;

    // move to Newton point w.r.t current
    p1.alpha -= p1.deriv[0]/p1.deriv[1];
    PrimalEval(ctx, &p1);

    // check for convergence
    if (mju_abs(p1.deriv[0]) < gtol && p1.cost < 0) {
      ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
      *improvement = -p1.cost;
      return p1.alpha;                          // SUCCESS
    }
  }

  // check for failure to bracket
  if (ctx->LSiter >= ls_iterations) {
    ctx->LSresult = 3;                          // could not bracket
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    *improvement = -p1.cost;
    return p1.alpha;
  }

  // check for p2 update; SHOULD NOT OCCUR
  if (!p2update) {
    ctx->LSresult = 6;                          // no p2 update
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    *improvement = -p1.cost;
    return p1.alpha;
  }

  // compute next-points for bracket
  p2next = p1;
  p1next.alpha = p1.alpha - p1.deriv[0]/p1.deriv[1];
  PrimalEval(ctx, &p1next);

  // bracketed search
  while (ctx->LSiter < ls_iterations) {
    // evaluate at midpoint
    pmid.alpha = 0.5*(p1.alpha + p2.alpha);
    PrimalEval(ctx, &pmid);

    // make list of candidates
    mjPrimalPnt candidates[3] = {p1next, p2next, pmid};

    // check candidates for convergence
    mjtNum bestcost = 0;
    int bestind = -1;
    for (int i=0; i < 3; i++) {
      if (mju_abs(candidates[i].deriv[0]) < gtol &&
          (bestind == -1 || candidates[i].cost < bestcost)) {
        bestcost = candidates[i].cost;
        bestind = i;
      }
    }
    if (bestind >= 0) {
      ctx->LSslope = mju_abs(candidates[bestind].deriv[0])*slopescl;
      *improvement = -candidates[bestind].cost;
      return candidates[bestind].alpha;       // SUCCESS
    }

    // update brackets
    int b1 = updateBracket(ctx, &p1, candidates, &p1next);
    int b2 = updateBracket(ctx, &p2, candidates, &p2next);

    // no update possible: numerical accuracy reached, use midpoint
    if (!b1 && !b2) {
      if (pmid.cost < 0) {
        ctx->LSresult = 0;  // SUCCESS
      } else {
        ctx->LSresult = 7;  // no improvement, could not bracket
      }

      ctx->LSslope = mju_abs(pmid.deriv[0])*slopescl;
      *improvement = -pmid.cost;
      return pmid.alpha;
    }
  }

  // choose bracket with best cost
  if (p1.cost <= p2.cost && p1.cost < 0) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    *improvement = -p1.cost;
    return p1.alpha;
  } else if (p2.cost <= p1.cost && p2.cost < 0) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p2.deriv[0])*slopescl;
    *improvement = -p2.cost;
    return p2.alpha;
  } else {
    ctx->LSresult = 5;                          // no improvement
    return 0;
  }
}


// build the lower-triangle view of the discrete metric matrix
// S = diag(h*D+h^2*K) + K_csr + tendon outer products: Newton consumes the metric
// explicitly inside the Hessian. The efm CSR stores the full symmetric pattern with
// sorted columns; the view keeps entries with col < row, folds all diagonals into one
// entry, and produces sorted unique columns per row. For island contexts rows and columns
// are island-local (the maps are monotone, so mapped columns stay sorted); the flex CSR
// never appears there (flex with metric terms forces a monolithic solve)
static void MakeMetricLower(mjData* d, mjPrimalContext* ctx) {
  const mjModel* m = ctx->fm;
  int nv = ctx->nv, island = ctx->island;
  int idofadr = island >= 0 ? d->island_idofadr[island] : 0;
  const int* idof2dof = island >= 0 ? d->map_idof2dof + idofadr : NULL;

  // the view S_* was carved by PrimalAllocate (which also sized the rank-1 contribution)
  // and lives for the whole solve: FactorizeHessian re-merges it on active-set recomputes.
  // Scratch below is freed before returning
  int tcap = ctx->S_tcap;
  mjEffRank1Iter it = {0};
  mjEffRank1 e;

  mj_markStack(d);

  // bucket the rank-1 entries by (local) row: count, carve, fill
  int* tint = mjSTACKALLOC(d, 2*nv + (tcap > 0 ? tcap : 1), int);
  mjtNum* tval = mjSTACKALLOC(d, tcap > 0 ? tcap : 1, mjtNum);
  int* trownnz = tint;
  int* trowadr = tint + nv;
  int* tcol = tint + 2*nv;
  mju_zeroInt(trownnz, nv);

  // count rank-1 entries per (local) row
  it = (mjEffRank1Iter){0};
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    for (int j=0; j < e.nnz; j++) {
      int r = e.colind[j];
      int lr = island >= 0 ? d->map_dof2idof[r] - idofadr : r;
      if (lr < 0 || lr >= nv) {
        // island discovery unions metric couplings and the sleep policy keeps coupled
        // trees awake together, so a rank-1 term cannot straddle islands or sleep
        mjERROR("metric rank-1 term spans islands on dof %d", r);  // SHOULD NOT OCCUR
      }
      trownnz[lr] += e.nnz;
    }
  }
  // carve row addresses
  int adr = 0;
  for (int i=0; i < nv; i++) {
    trowadr[i] = adr;
    adr += trownnz[i];
    trownnz[i] = 0;
  }

  // fill the buckets
  it = (mjEffRank1Iter){0};
  while (mj_effRank1Next(m, d, &it, &e, /*flg_contact=*/1)) {
    if (island >= 0 && d->tree_island[m->dof_treeid[e.colind[0]]] != island) {
      continue;
    }
    for (int j=0; j < e.nnz; j++) {
      int r = e.colind[j];
      int lr = island >= 0 ? d->map_dof2idof[r] - idofadr : r;
      mjtNum sJr = e.scale * e.val[j];
      for (int a=0; a < e.nnz; a++) {
        int lc = island >= 0 ? d->map_dof2idof[e.colind[a]] - idofadr : e.colind[a];
        int w = trowadr[lr] + trownnz[lr]++;
        tcol[w] = lc;
        tval[w] = sJr * e.val[a];
      }
    }
  }

  // sort each bucketed row (insertion sort: per-tendon runs are already sorted and rows
  // are short)
  for (int i=0; i < nv; i++) {
    int start = trowadr[i], end = start + trownnz[i];
    for (int a=start+1; a < end; a++) {
      int c = tcol[a];
      mjtNum v = tval[a];
      int b = a - 1;
      while (b >= start && tcol[b] > c) {
        tcol[b+1] = tcol[b];
        tval[b+1] = tval[b];
        b--;
      }
      tcol[b+1] = c;
      tval[b+1] = v;
    }
  }

  // merge flex CSR, tendon buckets and diagonals into the lower view
  adr = 0;
  for (int i=0; i < nv; i++) {
    ctx->S_rowadr[i] = adr;
    int gdof = island >= 0 ? idof2dof[i] : i;
    mjtNum diag = d->efm_diag ? d->efm_diag[gdof] : 0;

    // three-way merge of the sorted flex row, the sorted tendon/actuator row and the
    // fluid M-pattern row, summing duplicates. Island dof->idof maps preserve order
    // across DOFs in the island, so mapped flex and fluid columns stay sorted
    int fa = 0, fn = 0;
    const int* fcol = NULL;
    const mjtNum* fval = NULL;
    if (d->nefmK) {
      fa = d->efm_K_rowadr[gdof];
      fn = d->efm_K_rownnz[gdof];
      fcol = d->efm_K_colind;
      fval = d->efm_K_val;
    }
    int fe = fa + fn;
    int ta = trowadr[i], te = ta + trownnz[i];
    int ga = 0, ge = 0;
    if (d->efm_fluid) {
      ga = m->M_rowadr[gdof];
      ge = ga + m->M_rownnz[gdof];
    }
    while (fa < fe || ta < te || ga < ge) {
      int col;
      mjtNum val = 0;
      int cf = fa < fe ? (island >= 0 ? d->map_dof2idof[fcol[fa]] - idofadr
                                      : fcol[fa]) : nv;
      int ct = ta < te ? tcol[ta] : nv;
      int cg = ga < ge ? (island >= 0 ? d->map_dof2idof[m->M_colind[ga]] - idofadr
                                      : m->M_colind[ga]) : nv;
      col = cf < ct ? cf : ct;
      col = cg < col ? cg : col;
      while (fa < fe &&
             (island >= 0 ? d->map_dof2idof[fcol[fa]] - idofadr
                          : fcol[fa]) == col) {
        val += fval[fa++];
      }
      while (ta < te && tcol[ta] == col) {
        val += tval[ta++];
      }
      while (ga < ge &&
             (island >= 0 ? d->map_dof2idof[m->M_colind[ga]] - idofadr
                          : m->M_colind[ga]) == col) {
        val += d->efm_fluid[ga++];
      }
      if (col < i) {
        ctx->S_colind[adr] = col;
        ctx->S_val[adr] = val;
        adr++;
      } else if (col == i) {
        diag += val;
        break;  // columns are strictly sorted: nothing at or before the diagonal remains
      } else {
        break;
      }
    }

    if (diag) {
      ctx->S_colind[adr] = i;
      ctx->S_val[adr] = diag;
      adr++;
    }
    ctx->S_rownnz[i] = adr - ctx->S_rowadr[i];
  }
  ctx->nS = adr;

  mj_freeStack(d);
}


// scratch stack bytes needed by primalSqrSymbolic + mju_sqrMatTDSparseNumeric
static size_t primalSqrScratchBytes(int nv, int nefc) {
  size_t sym_bytes = mj_stackFrameBytes() +
                     2 * mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int)) +
                     mj_stackBytes((size_t)nefc * sizeof(int), _Alignof(int));
  size_t num_bytes = mj_stackFrameBytes() +
                     mj_stackBytes(8 * (size_t)nv * sizeof(mjtNum), _Alignof(mjtNum));
  return mjMAX(sym_bytes, num_bytes);
}


// per-iteration scratch stack bytes needed by the primal solver main loop
static size_t primalLoopScratchBytes(const mjPrimalContext* ctx, const mjData* d) {
  size_t sqr_bytes = ctx->is_sparse ? primalSqrScratchBytes(ctx->nv, ctx->nefc) : 0;
  size_t mul_bytes = ctx->flg_metric ? mj_effMulAddScratch(ctx->fm, d) : 0;
  size_t prec_bytes = ctx->flg_metric ? mj_effPrecScratch(ctx->fm, d) : 0;
  return mjMAX(sqr_bytes, mjMAX(mul_bytes, prec_bytes));
}


// minimum-degree reordering thresholds:
//   MINNV:   skip small systems where natural-order Cholesky overhead is negligible
//   MAXBLK:  cap nb so the nb x words bitset stays bounded (<= 8 MB)
//   MINFLOP: skip nearly tree-structured graphs where natural Cholesky is already cheap
//   MINSAVE: minimum Cholesky flop reduction to amortize ordering and (J, M, S) permutation
#define PRIMAL_MD_MINNV   64
#define PRIMAL_MD_MAXBLK  8192
#define PRIMAL_MD_MINFLOP 50000
#define PRIMAL_MD_MINSAVE 25000


// group contiguous DOFs of the same body into supernode blocks of size <= 3;
// if bstart is NULL, only count and return nb
static int primalDofBlocks(const mjModel* m, const mjData* d, const mjPrimalContext* ctx,
                           int* bstart, int* bsz, int* dof2blk) {
  int nv = ctx->nv;
  const int* idof2dof = ctx->island >= 0 ? d->map_idof2dof+d->island_idofadr[ctx->island] : NULL;
  int nb = 0;
  for (int i=0; i < nv; ) {
    int gdof = idof2dof ? idof2dof[i] : i;
    int body = m->dof_bodyid[gdof];
    int s = 1;
    while (s < 3 && i + s < nv) {
      int gdof_s = idof2dof ? idof2dof[i+s] : i + s;
      if (m->dof_bodyid[gdof_s] != body) {
        break;
      }
      s++;
    }
    if (bstart) {
      bstart[nb] = i;
      bsz[nb] = s;
      for (int a=0; a < s; a++) {
        dof2blk[i+a] = nb;
      }
    }
    nb++;
    i += s;
  }
  return nb;
}


// count reverse-Cholesky nL and flops on block graph adj under block permutation bperm
// (or natural order if bperm is NULL) via the block elimination tree in O(nL_blk)
static mjtSize primalBlockElimFlops(int nb, int words, const uint64_t* restrict adj,
                                    const int* restrict bsz, const int* restrict bperm,
                                    int* restrict order, int* restrict parent,
                                    int* restrict flag, int* restrict wsum,
                                    size_t* nH_upper_out, size_t* nL_out, int* fill_out) {
  if (bperm) {
    for (int b=0; b < nb; b++) {
      order[bperm[b]] = b;
    }
  }
  size_t nH_upper = 0, nL = 0;
  int fill_blks = 0;
  for (int q=nb-1; q >= 0; q--) {
    int bq = bperm ? order[q] : q;
    parent[q] = -1;
    flag[q] = q;
    wsum[q] = 0;
    int sq = bsz[bq];
    nH_upper += (size_t)(sq * (sq + 1) / 2);
    const uint64_t* aq = adj + (size_t)bq*words;
    int w0 = bperm ? 0 : (q >> 6);
    for (int w=w0; w < words; w++) {
      uint64_t bits = aq[w];
      if (!bperm && w == w0) {
        bits &= (~0ULL << (q & 63)) << 1;
      }
      while (bits) {
        int bu = (w << 6) + mju_ctz64(bits);
        bits &= bits - 1;
        int p = bperm ? bperm[bu] : bu;
        if (p <= q) {
          continue;
        }
        nH_upper += (size_t)(sq * bsz[bu]);
        int hop = 0;
        while (flag[p] != q) {
          flag[p] = q;
          wsum[p] += sq;
          fill_blks += hop;
          hop = 1;
          int par = parent[p];
          if (par == -1) {
            parent[p] = q;
            break;
          }
          p = par;
        }
      }
    }
  }
  mjtSize flops = 0;
  for (int p=0; p < nb; p++) {
    int bp = bperm ? order[p] : p;
    int w = wsum[p], s = bsz[bp];
    for (int a=1; a <= s; a++) {
      mjtSize lnnz = w + a;
      nL += (size_t)lnnz;
      flops += lnnz * lnnz;
    }
  }
  if (nH_upper_out) *nH_upper_out = nH_upper;
  if (nL_out) *nL_out = nL;
  if (fill_out) *fill_out = fill_blks;
  return flops;
}


// try seeding block permutation from model-level static ordering m->efmC_perm
static int primalSeedModelPerm(const mjModel* m, const mjData* d, const mjPrimalContext* ctx,
                               int nb, const int* bstart, const int* bsz, const int* dof2blk,
                               int* bperm) {
  if (!m->nefmCvert || m->efmC_perm[0] < 0) {
    return 0;
  }
  int nv = ctx->nv, island = ctx->island;
  int idofadr = island >= 0 ? d->island_idofadr[island] : 0;
  for (int b=0; b < nb; b++) {
    bperm[b] = -1;
  }
  int nseeded = 0;
  for (int t=0; t < m->nefmCvert && m->efmC_perm[t] >= 0; t++) {
    int gdof = m->efmC_perm[t];
    if (gdof >= m->nv) {
      continue;
    }
    if (island >= 0 && d->tree_island[m->dof_treeid[gdof]] != island) {
      continue;
    }
    int ldof = island >= 0 ? d->map_dof2idof[gdof] - idofadr : gdof;
    if (ldof >= 0 && ldof < nv) {
      int b = dof2blk[ldof];
      if (bstart[b] == ldof && bsz[b] == 3 && bperm[b] < 0) {
        bperm[b] = nseeded++;
      }
    }
  }
  if (nseeded < 4) {
    return 0;
  }
  int unseeded = nb - nseeded;
  if (unseeded > 0) {
    int u = 0;
    for (int b=0; b < nb; b++) {
      if (bperm[b] >= 0) {
        bperm[b] += unseeded;
      } else {
        bperm[b] = u++;
      }
    }
  }
  return 1;
}


// compute minimum-degree DOF ordering on supernode blocks and adopt it if Cholesky flops drop
static void primalTryReorder(const mjModel* m, mjData* d, mjPrimalContext* ctx,
                             size_t cg_bytes, size_t loop_bytes) {
  int nv = ctx->nv, nefc = ctx->nefc;
  if (!m->nflex || nv < PRIMAL_MD_MINNV || nv > 3*PRIMAL_MD_MAXBLK || (!ctx->nS && nefc < 16)) {
    return;
  }
  int nb = primalDofBlocks(m, d, ctx, NULL, NULL, NULL);
  if (nb <= 4 || nb > PRIMAL_MD_MAXBLK) {
    return;
  }

  int words = (nb + 63) / 64;
  size_t perm_bytes = mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int));
  size_t md_int_cnt = 10*(size_t)nb + (size_t)words + (size_t)nv;
  size_t md_bit_cnt = (size_t)(nb + 2) * (size_t)words;
  size_t md_scratch =
      mj_stackFrameBytes() +
      mj_stackBytes(md_int_cnt * sizeof(int), _Alignof(int)) +
      mj_stackBytes(md_bit_cnt * sizeof(uint64_t), _Alignof(uint64_t));
  if (mj_stackBytesAvailable(d) < perm_bytes + mjMAX(md_scratch, cg_bytes + loop_bytes)) {
    return;
  }

  int* perm = mjSTACKALLOC(d, nv, int);
  size_t nH_blk_upper = 0;
  size_t nL_perm = 0;
  mjtSize Lflops_nat = 0;
  mjtSize Lflops_perm = 0;
  int md_ok = 0;

  // build supernode block graph, count natural Cholesky flops, and compute block ordering
  mj_markStack(d);
  {
    int* md_ints       = mjSTACKALLOC(d, md_int_cnt, int);
    uint64_t* adj      = mjSTACKALLOC(d, md_bit_cnt, uint64_t);
    uint64_t* row_bits = adj + (size_t)nb*words;
    uint64_t* nz_bits  = row_bits + words;
    int* bstart        = md_ints;
    int* bsz           = bstart + nb;
    int* bperm         = bsz + nb;
    int* order         = bperm + nb;
    int* row_blks      = order + nb;
    int* deg_scratch   = row_blks + nb;
    int* dof2blk       = deg_scratch + 5*nb + words;

    primalDofBlocks(m, d, ctx, bstart, bsz, dof2blk);
    memset(adj, 0, md_bit_cnt * sizeof(uint64_t));

    // add undirected block edges from M, discrete metric S, and unique supernode rows of J
    for (int r=0; r < nv; r++) {
      int br = dof2blk[r];
      int end = ctx->M_rowadr[r] + ctx->M_rownnz[r];
      for (int j=ctx->M_rowadr[r]; j < end; j++) {
        int c = ctx->M_colind[j];
        if (c >= r) {
          break;
        }
        int bc = dof2blk[c];
        if (bc != br) {
          adj[(size_t)br*words+(bc>>6)] |= 1ULL << (bc & 63);
          adj[(size_t)bc*words+(br>>6)] |= 1ULL << (br & 63);
        }
      }
    }
    if (ctx->nS) {
      for (int r=0; r < nv; r++) {
        int br = dof2blk[r];
        int end = ctx->S_rowadr[r] + ctx->S_rownnz[r];
        for (int j=ctx->S_rowadr[r]; j < end; j++) {
          int c = ctx->S_colind[j];
          if (c >= r) {
            break;
          }
          int bc = dof2blk[c];
          if (bc != br) {
            adj[(size_t)br*words+(bc>>6)] |= 1ULL << (bc & 63);
            adj[(size_t)bc*words+(br>>6)] |= 1ULL << (br & 63);
          }
        }
      }
    }
    int has_dyn_edges = 0;
    for (int i=0; i < nefc; i++) {
      int adr = ctx->J_rowadr[i], nnz = ctx->J_rownnz[i];
      int k = 0, prev = -1;
      for (int j=0; j < nnz; j++) {
        int b = dof2blk[ctx->J_colind[adr+j]];
        if (b != prev) {
          row_blks[k++] = b;
          prev = b;
        }
      }
      if (k > 1) {
        int check_new = (i >= ctx->ne || k > 2);
        if (k <= 16) {
          for (int a=0; a < k; a++) {
            int ba = row_blks[a];
            uint64_t* aa = adj + (size_t)ba*words;
            for (int c=a+1; c < k; c++) {
              int bc = row_blks[c];
              uint64_t mask = 1ULL << (bc & 63);
              if (check_new && !(aa[bc>>6] & mask)) {
                has_dyn_edges = 1;
              }
              aa[bc>>6] |= mask;
              adj[(size_t)bc*words+(ba>>6)] |= 1ULL << (ba & 63);
            }
          }
        } else {
          if (check_new) {
            has_dyn_edges = 1;
          }
          memset(row_bits, 0, words * sizeof(uint64_t));
          for (int a=0; a < k; a++) {
            int ba = row_blks[a];
            row_bits[ba>>6] |= 1ULL << (ba & 63);
          }
          for (int a=0; a < k; a++) {
            int ba = row_blks[a];
            uint64_t* aa = adj + (size_t)ba*words;
            for (int w=0; w < words; w++) {
              aa[w] |= row_bits[w];
            }
            aa[ba>>6] &= ~(1ULL << (ba & 63));
          }
        }
      }
      if (ctx->J_rowsuper) {
        i += ctx->J_rowsuper[i];
      }
    }

    // count natural-order reverse-Cholesky flops via block elimination tree in O(nL_blk)
    int* parent = deg_scratch;
    int* flag   = parent + nb;
    int* wsum   = flag + nb;
    int fill_blks = 0;
    Lflops_nat = primalBlockElimFlops(nb, words, adj, bsz, NULL, NULL, parent, flag, wsum,
                                      &nH_blk_upper, NULL, &fill_blks);

    // reuse model-level efmC_perm when no dynamic inter-block constraint edges exist, else run MD
    if (fill_blks && Lflops_nat >= PRIMAL_MD_MINFLOP) {
      if (!has_dyn_edges && primalSeedModelPerm(m, d, ctx, nb, bstart, bsz, dof2blk, bperm)) {
        Lflops_perm = primalBlockElimFlops(nb, words, adj, bsz, bperm, order, parent, flag, wsum,
                                           NULL, &nL_perm, NULL);
        md_ok = (5 * Lflops_perm < 4 * Lflops_nat);
      }
      if (!md_ok) {
        md_ok = mju_minDegree(bperm, adj, nb, bsz, &nL_perm, &Lflops_perm, Lflops_nat,
                              deg_scratch, nz_bits);
      }
      if (md_ok) {
        for (int b=0; b < nb; b++) {
          order[bperm[b]] = b;
        }
        int cursor = 0;
        for (int pos=0; pos < nb; pos++) {
          int b = order[pos];
          int s0 = bstart[b], sv = bsz[b];
          for (int a=0; a < sv; a++) {
            perm[s0+a] = cursor++;
          }
        }
      }
    }
  }
  mj_freeStack(d);

  // require >= 20% flop reduction and absolute savings covering permutation overhead
  int nM = ctx->M_rowadr[nv-1] + ctx->M_rownnz[nv-1];
  int nJ = ctx->nJ;
  mjtSize saved = Lflops_nat - Lflops_perm;
  if (!md_ok || saved < PRIMAL_MD_MINSAVE || saved <= 16 * (mjtSize)(nJ + nM + ctx->nS)) {
    return;
  }

  // check stack space for permuted (HJ, HJT, HM), S scratch, H upper bound, and permuted L
  size_t nP_int = 2*(size_t)nJ + 5*(size_t)nv + (size_t)nM;
  size_t nP_num = 2*(size_t)nJ + (size_t)nM;
  size_t P_bytes =
      mj_stackBytes(nP_int * sizeof(int), _Alignof(int)) +
      mj_stackBytes(nP_num * sizeof(mjtNum), _Alignof(mjtNum));
  size_t S_scratch = ctx->nS ?
      (mj_stackFrameBytes() +
       mj_stackBytes((2*(size_t)nv + (size_t)ctx->nS) * sizeof(int), _Alignof(int)) +
       mj_stackBytes((size_t)ctx->nS * sizeof(mjtNum), _Alignof(mjtNum))) : 0;
  size_t J_scratch = nefc ?
      (mj_stackFrameBytes() +
       mj_stackBytes(((size_t)nv + (size_t)nefc) * sizeof(int), _Alignof(int))) : 0;
  size_t nH_cap = nH_blk_upper + (size_t)nM + (size_t)ctx->nS;
  size_t H_cap_bytes =
      mj_stackBytes(nH_cap * sizeof(mjtNum), _Alignof(mjtNum)) +
      mj_stackBytes(2 * nH_cap * sizeof(int), _Alignof(int));
  size_t nLp_int = 3*nL_perm + (size_t)nv;
  size_t nLp_num = ctx->is_elliptic ? 2*nL_perm : nL_perm;
  size_t Lp_bytes =
      mj_stackBytes(nLp_int * sizeof(int), _Alignof(int)) +
      mj_stackBytes(nLp_num * sizeof(mjtNum), _Alignof(mjtNum));
  size_t chol_count_bytes =
      mj_stackFrameBytes() + 2 * mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int));
  size_t chol_fill_bytes =
      mj_stackFrameBytes() + 3 * mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int));
  size_t after_H = mjMAX(cg_bytes + mjMAX(chol_count_bytes, loop_bytes),
                         Lp_bytes + mjMAX(chol_fill_bytes, loop_bytes));
  size_t after_P = mjMAX(mjMAX(S_scratch, J_scratch), H_cap_bytes + after_H);
  if (mj_stackBytesAvailable(d) < P_bytes + mjMAX(after_P, cg_bytes + loop_bytes)) {
    return;
  }

  int* P_int = mjSTACKALLOC(d, nP_int, int);
  mjtNum* P_num = mjSTACKALLOC(d, nP_num, mjtNum);
  ctx->perm         = perm;
  ctx->HJ_colind    = P_int;  P_int += nJ;
  ctx->HJT_rownnz   = P_int;  P_int += nv;
  ctx->HJT_rowadr   = P_int;  P_int += nv;
  ctx->HJT_rowsuper = P_int;  P_int += nv;
  ctx->HJT_colind   = P_int;  P_int += nJ;
  ctx->HM_rownnz    = P_int;  P_int += nv;
  ctx->HM_rowadr    = P_int;  P_int += nv;
  ctx->HM_colind    = P_int;
  ctx->HJ           = P_num;  P_num += nJ;
  ctx->HJT          = P_num;  P_num += nJ;
  ctx->HM           = P_num;

  // permute rows of JT into HJT and scatter into sorted HJ in a single pass
  if (nefc) {
    mj_markStack(d);
    int* scratch = mjSTACKALLOC(d, nv + nefc, int);
    int* invperm = scratch;
    int* hj_pos = scratch + nv;
    for (int r=0; r < nv; r++) {
      invperm[perm[r]] = r;
    }
    mju_copyInt(hj_pos, ctx->J_rowadr, nefc);

    int hjt_adr = 0;
    for (int r=0; r < nv; r++) {
      int r_old = invperm[r];
      int nnz = ctx->JT_rownnz[r_old];
      int src_adr = ctx->JT_rowadr[r_old];
      ctx->HJT_rownnz[r] = nnz;
      ctx->HJT_rowadr[r] = hjt_adr;
      const int* restrict src_col = ctx->JT_colind + src_adr;
      const mjtNum* restrict src_val = ctx->JT + src_adr;
      int* restrict dst_col = ctx->HJT_colind + hjt_adr;
      mjtNum* restrict dst_val = ctx->HJT + hjt_adr;
      for (int k=0; k < nnz; k++) {
        int i = src_col[k];
        mjtNum v = src_val[k];
        dst_col[k] = i;
        dst_val[k] = v;
        int w = hj_pos[i]++;
        ctx->HJ_colind[w] = r;
        ctx->HJ[w] = v;
      }
      hjt_adr += nnz;
    }

    ctx->HJT_rowsuper[nv-1] = 0;
    for (int r=nv-2; r >= 0; r--) {
      int match = 0;
      int r_old = invperm[r];
      if (invperm[r+1] == r_old + 1) {
        match = (ctx->JT_rowsuper[r_old] > 0);
      } else if (ctx->HJT_rownnz[r] == ctx->HJT_rownnz[r+1]) {
        int nnz = ctx->HJT_rownnz[r];
        const int* c0 = ctx->HJT_colind + ctx->HJT_rowadr[r];
        const int* c1 = ctx->HJT_colind + ctx->HJT_rowadr[r+1];
        match = 1;
        for (int k=0; k < nnz; k++) {
          if (c0[k] != c1[k]) {
            match = 0;
            break;
          }
        }
      }
      ctx->HJT_rowsuper[r] = match ? ctx->HJT_rowsuper[r+1] + 1 : 0;
    }
    mj_freeStack(d);
  } else {
    mju_zeroInt(ctx->HJT_rownnz, nv);
    mju_zeroInt(ctx->HJT_rowadr, nv);
    mju_zeroInt(ctx->HJT_rowsuper, nv);
  }

  // permute lower-triangular M and discrete metric S
  mju_permuteLowerCSR(nv, perm, ctx->HM_rownnz, ctx->HM_rowadr, ctx->HM_colind, ctx->HM,
                      ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind, ctx->M);
  if (ctx->nS) {
    mj_markStack(d);
    size_t nSp_int = 2*(size_t)nv + (size_t)ctx->nS;
    int* Sp_int = mjSTACKALLOC(d, nSp_int, int);
    mjtNum* Sp_val = mjSTACKALLOC(d, ctx->nS, mjtNum);
    int* Sp_rownnz = Sp_int;
    int* Sp_rowadr = Sp_int + nv;
    int* Sp_colind = Sp_int + 2*nv;
    mju_permuteLowerCSR(nv, perm, Sp_rownnz, Sp_rowadr, Sp_colind, Sp_val,
                        ctx->S_rownnz, ctx->S_rowadr, ctx->S_colind, ctx->S_val);
    mju_copyInt(ctx->S_rownnz, Sp_rownnz, nv);
    mju_copyInt(ctx->S_rowadr, Sp_rowadr, nv);
    mju_copyInt(ctx->S_colind, Sp_colind, ctx->nS);
    mju_copy(ctx->S_val, Sp_val, ctx->nS);
    mj_freeStack(d);
  }
}


// allocate and compute Hessian given efc_state; return 1 on success, 0 if arena is exhausted
//  mj_{mark/free}Stack in caller function!
static int MakeHessian(const mjModel* m, mjData* d, mjPrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;

  // default Hessian inputs to natural DOF order
  ctx->perm         = NULL;
  ctx->HJ_colind    = ctx->J_colind;
  ctx->HJ           = ctx->J;
  ctx->HJT_rownnz   = ctx->JT_rownnz;
  ctx->HJT_rowadr   = ctx->JT_rowadr;
  ctx->HJT_rowsuper = ctx->JT_rowsuper;
  ctx->HJT_colind   = ctx->JT_colind;
  ctx->HJT          = ctx->JT;
  ctx->HM_rownnz    = ctx->M_rownnz;
  ctx->HM_rowadr    = ctx->M_rowadr;
  ctx->HM_colind    = ctx->M_colind;
  ctx->HM           = ctx->M;

  // per-iteration scratch needed by the solver main loop after MakeHessian returns
  size_t cg_bytes =
      ctx->LTJ ? 0 : mj_stackBytes(2 * (size_t)nv * sizeof(mjtNum), _Alignof(mjtNum));
  size_t loop_bytes = primalLoopScratchBytes(ctx, d);

  // compute constraint inertia
  for (int i=0; i < nefc; i++) {
    ctx->D[i] = ctx->efc_state[i] == mjCNSTRSTATE_QUADRATIC ? ctx->efc_D[i] : 0;
  }

  // discrete metric: build the lower-triangle view of S for merging into H
  ctx->nS = 0;
  if (ctx->flg_metric) {
    int tcap = ctx->S_tcap > 0 ? ctx->S_tcap : 1;
    size_t metric_bytes =
        mj_stackFrameBytes() +
        mj_stackBytes((2 * (size_t)nv + (size_t)tcap) * sizeof(int), _Alignof(int)) +
        mj_stackBytes((size_t)tcap * sizeof(mjtNum), _Alignof(mjtNum));
    if (mj_stackBytesAvailable(d) < mjMAX(metric_bytes, cg_bytes + loop_bytes)) {
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);
      return 0;
    }
    MakeMetricLower(d, ctx);
  }

  // sparse
  if (ctx->is_sparse) {
    if (mj_stackBytesAvailable(d) < cg_bytes + loop_bytes) {
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);
      return 0;
    }

    // adopt supernode minimum-degree ordering when it substantially reduces Cholesky flops
    primalTryReorder(m, d, ctx, cg_bytes, loop_bytes);

    // count Hessian nonzeros, initialize rowadr, rownnz
    ctx->nH = mju_sqrMatTDSparseSymbolic(
        ctx->H_rownnz, ctx->H_rowadr, NULL, NULL,
        nefc, nv, ctx->J_rownnz, ctx->J_rowadr, ctx->HJ_colind, ctx->J_rowsuper,
        ctx->HJT_rownnz, ctx->HJT_rowadr, ctx->HJT_colind, ctx->HJT_rowsuper, d);

    // add M nonzeros to Hessian total (unavoidable overcounting since H_colind is still unknown)
    ctx->nH += ctx->HM_rowadr[nv-1] + ctx->HM_rownnz[nv-1];

    // add discrete-metric nonzeros
    ctx->nH += ctx->nS;

    // check stack space for H, H_colind, HT_colind plus symbolic Cholesky counting scratch
    size_t H_bytes =
        mj_stackBytes((size_t)ctx->nH * sizeof(mjtNum), _Alignof(mjtNum)) +
        mj_stackBytes(2 * (size_t)ctx->nH * sizeof(int), _Alignof(int));
    size_t chol_count_bytes =
        mj_stackFrameBytes() + 2 * mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int));
    if (mj_stackBytesAvailable(d) < H_bytes + cg_bytes + mjMAX(chol_count_bytes, loop_bytes)) {
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);
      return 0;
    }

    // nH is known: allocate H, H_colind, HT_colind
    ctx->H          = mjSTACKALLOC(d, ctx->nH, mjtNum);
    int* H_intblock = mjSTACKALLOC(d, 2*ctx->nH, int);
    ctx->H_colind   = H_intblock;
    ctx->HT_colind  = H_intblock + ctx->nH;

    // shift H row addresses to make room for M and the discrete metric
    int shift = 0;
    for (int r=0; r < nv-1; r++) {
      shift += ctx->HM_rownnz[r] + (ctx->nS ? ctx->S_rownnz[r] : 0);
      ctx->H_rowadr[r+1] += shift;
    }

    // compute H = J'*D*J: symbolic phase
    mju_sqrMatTDSparseSymbolic(
        ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, NULL,
        nefc, nv, ctx->J_rownnz, ctx->J_rowadr, ctx->HJ_colind, ctx->J_rowsuper,
        ctx->HJT_rownnz, ctx->HJT_rowadr, ctx->HJT_colind, ctx->HJT_rowsuper, d);

    // compute H = J'*D*J: numeric phase
    mju_sqrMatTDSparseNumeric(
        ctx->H, nv, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, NULL,
        ctx->HJ, ctx->J_rownnz, ctx->J_rowadr, ctx->HJ_colind,
        ctx->HJT, ctx->HJT_rownnz, ctx->HJT_rowadr, ctx->HJT_colind,
        ctx->HJT_rowsuper, ctx->D, d);

    // add mass matrix: H = J'*D*J + M
    mju_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                       ctx->HM, ctx->HM_rownnz, ctx->HM_rowadr, ctx->HM_colind);

    // add discrete metric: H = J'*D*J + Mtilde
    if (ctx->nS) {
      mju_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                         ctx->S_val, ctx->S_rownnz, ctx->S_rowadr, ctx->S_colind);
    }

    // compute H' sparse structure (upper triangle, required for symbolic Cholesky)
    mju_transposeSparse(NULL, NULL, nv, nv, ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind, NULL,
                        ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, NULL);

    // count total and row non-zeros of reverse-Cholesky factors L and LT
    ctx->nL = mju_cholFactorSymbolic(NULL, ctx->L_rownnz, ctx->L_rowadr, NULL,
                                     ctx->LT_rownnz, ctx->LT_rowadr, NULL, NULL,
                                     ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind,
                                     nv, d);

    // nL is known: check stack space, then allocate L_colind, LT_colind, LT_map, L, Lcone
    // L_colind + LT_colind + LT_map + pathcost + L_super
    size_t nL_int = 3*(size_t)ctx->nL + 2*(size_t)nv;
    size_t nL_num = ctx->is_elliptic ? 2*(size_t)ctx->nL : (size_t)ctx->nL;  // L + Lcone
    size_t L_bytes =
        mj_stackBytes(nL_int * sizeof(int), _Alignof(int)) +
        mj_stackBytes(nL_num * sizeof(mjtNum), _Alignof(mjtNum));
    size_t chol_fill_bytes =
        mj_stackFrameBytes() + 3 * mj_stackBytes((size_t)nv * sizeof(int), _Alignof(int));
    if (mj_stackBytesAvailable(d) < L_bytes + mjMAX(chol_fill_bytes, loop_bytes)) {
      mj_warning(d, mjWARN_CNSTRFULL, d->narena);
      return 0;
    }
    int* L_intblock    = mjSTACKALLOC(d, nL_int, int);
    mjtNum* L_numblock = mjSTACKALLOC(d, nL_num, mjtNum);
    ctx->L_colind      = L_intblock;
    ctx->LT_colind     = L_intblock + ctx->nL;
    ctx->LT_map        = L_intblock + 2*ctx->nL;
    ctx->pathcost      = L_intblock + 3*ctx->nL;
    ctx->L_super       = L_intblock + 3*ctx->nL + nv;
    ctx->L             = L_numblock;
    ctx->Lcone         = ctx->is_elliptic ? L_numblock + ctx->nL : NULL;

    // symbolic Cholesky: populate L_colind and LT structures
    mju_cholFactorSymbolic(ctx->L_colind, ctx->L_rownnz, ctx->L_rowadr,
                           ctx->LT_colind, ctx->LT_rownnz, ctx->LT_rowadr, ctx->LT_map,
                           ctx->L_super, ctx->HT_rownnz, ctx->HT_rowadr, ctx->HT_colind,
                           nv, d);

    // fold gate inputs, fixed for the solve since the L pattern is fixed:
    // pathcost[r] totals the L row nonzeros along the reverse-etree path from
    // row r (parent(r) = largest off-diagonal column of L row r), the cost of
    // a rank-1 update starting at column r; Lflops totals squared row nonzeros
    ctx->Lflops = 0;
    for (int r=0; r < nv; r++) {
      int nnz = ctx->L_rownnz[r];
      int parent = nnz >= 2 ? ctx->L_colind[ctx->L_rowadr[r]+nnz-2] : -1;
      ctx->pathcost[r] = nnz + (parent >= 0 ? ctx->pathcost[parent] : 0);
      ctx->Lflops += (mjtSize)nnz * nnz;
    }
  }

  // dense
  else {
    // compute H = M + J'*D*J (into H for elliptic cones, directly into L otherwise)
    mjtNum* target = ctx->H ? ctx->H : ctx->L;
    mju_sqrMatTD_impl(target, ctx->J, ctx->D, nefc, nv, /*flg_upper=*/ 0);
    mju_addToSymSparse(target, ctx->M, ctx->nv,
                       ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                       /*flg_upper=*/ 0);

    // add discrete metric (the view is lower+diagonal by construction)
    if (ctx->nS) {
      mju_addToSymSparse(target, ctx->S_val, nv,
                         ctx->S_rownnz, ctx->S_rowadr, ctx->S_colind,
                         /*flg_upper=*/ 0);
    }

    // sum_{r=0}^{nv-1} (r+1)^2 = 2x dense Cholesky MAC count (matches sparse Lflops)
    ctx->Lflops = (mjtSize)nv * (nv + 1) * (2*nv + 1) / 6;
  }

  return 1;
}


// forward declarations (for readability)
static void HessianCone(mjData* d, mjPrimalContext* ctx);
static void HessianConeUpdate(mjData* d, mjPrimalContext* ctx);
static int HessianConeFold(mjData* d, mjPrimalContext* ctx, mjtSize cost_L);


// compute rank-(dim-1) factor V of elliptic cone Hessian H = V'*V; return number of rows
static int primalConeFactor(mjtNum V[30], const mjContact* con, mjtNum D0, const mjtNum* jar) {
  int dim = con->dim;
  int m = dim - 1;
  mjtNum mu = con->mu;
  const mjtNum* friction = con->friction;

  // ray coordinates u = diag(mu, friction) * jar
  mjtNum N = jar[0] * mu;
  mjtNum u[5];
  for (int j=0; j < m; j++) {
    u[j] = jar[j+1] * friction[j];
  }

  // tangential norm and cone scaling
  mjtNum T = mju_norm(u, m);
  mjtNum Dm = D0 / (mu*mu * (1 + mu*mu));
  if (T <= 0 || Dm <= 0) {
    return 0;
  }
  mjtNum sqrtDm = mju_sqrt(Dm);
  for (int j=0; j < m; j++) {
    u[j] /= T;
  }

  // row 0: sqrt(Dm) * mu * (1, -u .* friction)
  mjtNum w0 = sqrtDm * mu;
  V[0] = w0;
  for (int j=0; j < m; j++) {
    V[j+1] = -w0 * u[j] * friction[j];
  }

  // rows 1..m-1: tangential curvature
  mjtNum s = mu*mu - mu*(N/T);
  if (s <= 0) {
    return 1;
  }
  mjtNum wt = sqrtDm * mju_sqrt(s);
  if (m == 2) {
    V[3] = 0;
    V[4] = -wt * u[1] * friction[0];
    V[5] =  wt * u[0] * friction[1];
    return 2;
  }

  // dim == 4 or 6: Householder reflector I - 2*(u*u')/(u'*u)
  u[0] += (u[0] >= 0 ? 1 : -1);
  mjtNum inv_uu = 2 / mju_dot(u, u, m);
  for (int k=1; k < m; k++) {
    mjtNum scale = inv_uu * u[k];
    mjtNum* row = V + k*dim;
    row[0] = 0;
    for (int j=0; j < m; j++) {
      row[j+1] = wt * ((j == k) - scale*u[j]) * friction[j];
    }
  }
  return m;
}


// compute res (nrow x nc) = V (nrow x dim) * J (dim x nc, stride jstride),
// indexing J at cols[k] when cols != NULL (otherwise k); uses V[r*dim] == 0 for r > 0;
// mjALWAYS_INLINE constant-folds cols at each call site and fuses common condim=3,6 cases
static mjALWAYS_INLINE void primalConeMulJ(mjtNum* restrict res, const mjtNum V[30],
                                           const mjtNum* restrict J, int nrow, int dim,
                                           int nc, int jstride, const int* restrict cols) {
  if (dim == 3 && nrow == 2) {
    const mjtNum* restrict j0 = J, * restrict j1 = J + jstride, * restrict j2 = J + 2*jstride;
    mjtNum* restrict l0 = res, * restrict l1 = res + nc;
    mjtNum v00 = V[0], v01 = V[1], v02 = V[2], v11 = V[4], v12 = V[5];
    for (int k=0; k < nc; k++) {
      int col = cols ? cols[k] : k;
      mjtNum x1 = j1[col], x2 = j2[col];
      l0[k] = v00*j0[col] + v01*x1 + v02*x2;
      l1[k] =               v11*x1 + v12*x2;
    }
  } else if (dim == 6 && nrow == 5) {
    const mjtNum* restrict j0 = J,             * restrict j1 = J + jstride;
    const mjtNum* restrict j2 = J + 2*jstride, * restrict j3 = J + 3*jstride;
    const mjtNum* restrict j4 = J + 4*jstride, * restrict j5 = J + 5*jstride;
    mjtNum* restrict l0 = res,        * restrict l1 = res + nc;
    mjtNum* restrict l2 = res + 2*nc, * restrict l3 = res + 3*nc;
    mjtNum* restrict l4 = res + 4*nc;
    for (int k=0; k < nc; k++) {
      int col = cols ? cols[k] : k;
      mjtNum x0 = j0[col], x1 = j1[col], x2 = j2[col],
             x3 = j3[col], x4 = j4[col], x5 = j5[col];
      l0[k] = V[0]*x0 + V[1]*x1  + V[2]*x2  + V[3]*x3  + V[4]*x4  + V[5]*x5;
      l1[k] =           V[7]*x1  + V[8]*x2  + V[9]*x3  + V[10]*x4 + V[11]*x5;
      l2[k] =           V[13]*x1 + V[14]*x2 + V[15]*x3 + V[16]*x4 + V[17]*x5;
      l3[k] =           V[19]*x1 + V[20]*x2 + V[21]*x3 + V[22]*x4 + V[23]*x5;
      l4[k] =           V[25]*x1 + V[26]*x2 + V[27]*x3 + V[28]*x4 + V[29]*x5;
    }
  } else if (!cols) {
    for (int r=0; r < nrow; r++) {
      const mjtNum* v = V + r*dim;
      int c0 = (r > 0);
      mju_scl(res + r*nc, J + c0*jstride, v[c0], nc);
      for (int c=c0+1; c < dim; c++) {
        mju_addToScl(res + r*nc, J + c*jstride, v[c], nc);
      }
    }
  } else {
    for (int k=0; k < nc; k++) {
      int col = cols[k];
      for (int r=0; r < nrow; r++) {
        const mjtNum* v = V + r*dim;
        int c0 = (r > 0);
        mjtNum s = v[c0] * J[c0*jstride+col];
        for (int c=c0+1; c < dim; c++) {
          s += v[c] * J[c*jstride+col];
        }
        res[r*nc+k] = s;
      }
    }
  }
}


// add scl * val' * val (lower triangle) into sparse symmetric matrix H
static void primalAddRowOuterSparse(mjtNum* restrict H, const int* restrict H_rowadr,
                                    const int* restrict H_colind, int nnz,
                                    const int* restrict cols, const mjtNum* restrict val,
                                    mjtNum scl) {
  for (int a=0; a < nnz; a++) {
    mjtNum va = scl * val[a];
    if (!va) {
      continue;
    }
    int r = cols[a];
    int adr = H_rowadr[r];
    const int* restrict hcol = H_colind + adr;
    mjtNum* restrict hval = H + adr;
    int k = 0;
    for (int b=0; b <= a; b++) {
      int cb = cols[b];
      while (hcol[k] < cb) {
        k++;
      }
      hval[k] += va * val[b];
    }
  }
}


// add LTJ' * LTJ (nrow x nnz, lower triangle) into sparse symmetric matrix H
static void primalAddConeOuterSparse(mjtNum* restrict H, const int* restrict H_rowadr,
                                     const int* restrict H_colind, int nrow, int nnz,
                                     const int* restrict cols, const mjtNum* restrict LTJ) {
  for (int a=0; a < nnz; a++) {
    int r = cols[a];
    int adr = H_rowadr[r];
    const int* restrict hcol = H_colind + adr;
    mjtNum* restrict hval = H + adr;
    int k = 0;
    if (nrow == 2) {
      mjtNum la0 = LTJ[a], la1 = LTJ[nnz+a];
      const mjtNum* restrict l0 = LTJ;
      const mjtNum* restrict l1 = LTJ + nnz;
      for (int b=0; b <= a; b++) {
        int cb = cols[b];
        while (hcol[k] < cb) {
          k++;
        }
        hval[k] += la0*l0[b] + la1*l1[b];
      }
    } else {
      mjtNum la[5];
      for (int c=0; c < nrow; c++) {
        la[c] = LTJ[c*nnz+a];
      }
      for (int b=0; b <= a; b++) {
        int cb = cols[b];
        while (hcol[k] < cb) {
          k++;
        }
        mjtNum s = 0;
        for (int c=0; c < nrow; c++) {
          s += la[c] * LTJ[c*nnz+b];
        }
        hval[k] += s;
      }
    }
  }
}


// add scl * row' * row (lower triangle) into dense symmetric matrix H (nv x nv)
static void primalAddRowOuterDense(mjtNum* restrict H, const mjtNum* restrict row,
                                   mjtNum scl, int nv) {
  int lo = -1;
  for (int r=0; r < nv; r++) {
    if (row[r]) {
      if (lo < 0) {
        lo = r;
      }
      mju_addToScl(H + r*nv + lo, row + lo, scl*row[r], r - lo + 1);
    }
  }
}


// compute row a (b=0..a) of LTJ'*LTJ (nrow x nnz) into out (add if flg_add, set if 0);
// mjALWAYS_INLINE constant-folds flg_add and fuses common nrow=2,5 cases into one pass
static mjALWAYS_INLINE void primalConeOuterRow(mjtNum* restrict out, const mjtNum* restrict LTJ,
                                               int nrow, int nnz, int a, int flg_add) {
  if (nrow == 2) {
    const mjtNum* restrict l0 = LTJ, * restrict l1 = LTJ + nnz;
    mjtNum la0 = l0[a], la1 = l1[a];
    for (int b=0; b <= a; b++) {
      mjtNum s = la0*l0[b] + la1*l1[b];
      out[b] = flg_add ? out[b] + s : s;
    }
  } else if (nrow == 5) {
    const mjtNum* restrict l0 = LTJ, * restrict l1 = LTJ + nnz, * restrict l2 = LTJ + 2*nnz;
    const mjtNum* restrict l3 = LTJ + 3*nnz, * restrict l4 = LTJ + 4*nnz;
    mjtNum la0 = l0[a], la1 = l1[a], la2 = l2[a], la3 = l3[a], la4 = l4[a];
    for (int b=0; b <= a; b++) {
      mjtNum s = la0*l0[b] + la1*l1[b] + la2*l2[b] + la3*l3[b] + la4*l4[b];
      out[b] = flg_add ? out[b] + s : s;
    }
  } else {
    if (!flg_add) {
      mju_zero(out, a+1);
    }
    for (int r=0; r < nrow; r++) {
      const mjtNum* restrict lr = LTJ + r*nnz;
      mju_addToScl(out, lr, lr[a], a+1);
    }
  }
}


// add (V*J)' * (V*J) (lower triangle) into dense symmetric matrix H (nv x nv)
static void primalAddConeOuterDense(mjtNum* restrict H, const mjtNum* restrict J,
                                    const mjtNum V[30], int nrow, int dim, int nv,
                                    mjtNum* restrict LTJ, mjtNum* restrict rowbuf,
                                    int* restrict cols) {
  // collect nonzero columns across the contact's dim Jacobian rows
  int nnz = 0;
  for (int k=0; k < nv; k++) {
    for (int c=0; c < dim; c++) {
      if (J[c*nv+k]) {
        cols[nnz++] = k;
        break;
      }
    }
  }
  if (!nnz) {
    return;
  }

  // compute compact LTJ (nrow x nnz) = V * J[:, cols] and add lower triangle of LTJ' * LTJ into H
  int lo = cols[0];
  int contiguous = (cols[nnz-1] - lo + 1 == nnz);
  if (contiguous) {
    primalConeMulJ(LTJ, V, J + lo, nrow, dim, nnz, nv, NULL);
    for (int a=0; a < nnz; a++) {
      primalConeOuterRow(H + (lo+a)*nv + lo, LTJ, nrow, nnz, a, /*flg_add=*/1);
    }
  } else {
    primalConeMulJ(LTJ, V, J, nrow, dim, nnz, nv, cols);
    for (int a=0; a < nnz; a++) {
      primalConeOuterRow(rowbuf, LTJ, nrow, nnz, a, /*flg_add=*/0);
      mjtNum* restrict dst = H + cols[a]*nv;
      for (int b=0; b <= a; b++) {
        dst[cols[b]] += rowbuf[b];
      }
    }
  }
}


// factorize Hessian H into L (dense allows H == L for in-place); return rank
static int primalFactorH(const mjPrimalContext* ctx, mjtNum* L, const mjtNum* H) {
  if (ctx->is_sparse) {
    return mju_cholFactorNumeric(
        L, ctx->nv, mjMINVAL,
        ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind,
        ctx->LT_rownnz, ctx->LT_rowadr, ctx->LT_colind, ctx->LT_map, ctx->L_super,
        H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, (int*)ctx->cholscratch);
  } else {
    if (H != L) {
      mju_copy(L, H, ctx->nH);
    }
    return mju_cholFactor(L, ctx->nv, mjMINVAL);
  }
}


// rank-1 Cholesky update flop estimate for row i (or nrows consecutive rows in dense mode)
static mjtSize primalRowUpdateCost(const mjPrimalContext* ctx, int i, int nrows) {
  if (ctx->is_sparse) {
    int nnz = ctx->J_rownnz[i];
    return nnz > 0 ? ctx->pathcost[ctx->HJ_colind[ctx->J_rowadr[i]+nnz-1]] : 0;
  }
  int nv = ctx->nv;
  const mjtNum* J = ctx->J + i*nv;
  for (int k=0; k < nv; k++) {
    for (int r=0; r < nrows; r++) {
      if (J[r*nv+k]) {
        mjtSize rem = nv - k;
        return rem * (rem + 1) / 2;
      }
    }
  }
  return 0;
}


// factorize Hessian: L = chol(H), maybe (re)compute H given efc_state
static void FactorizeHessian(mjData* d, mjPrimalContext* ctx, int flg_recompute) {
  int nv = ctx->nv, nefc = ctx->nefc;

  // maybe recompute H = Mtilde + J'*D*J
  if (flg_recompute) {
    for (int i=0; i < nefc; i++) {
      ctx->D[i] = ctx->efc_state[i] == mjCNSTRSTATE_QUADRATIC ? ctx->efc_D[i] : 0;
    }

    // sparse (sparsity pattern already in H_rownnz, H_rowadr, H_colind)
    if (ctx->is_sparse) {
      // compute H = J'*D*J: numeric phase
      mju_sqrMatTDSparseNumeric(
          ctx->H, nv, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, NULL,
          ctx->HJ, ctx->J_rownnz, ctx->J_rowadr, ctx->HJ_colind,
          ctx->HJT, ctx->HJT_rownnz, ctx->HJT_rowadr, ctx->HJT_colind,
          ctx->HJT_rowsuper, ctx->D, d);

      // add mass matrix: H = J'*D*J + C
      mju_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                         ctx->HM, ctx->HM_rownnz, ctx->HM_rowadr, ctx->HM_colind);

      // add discrete metric: H = J'*D*J + Mtilde
      if (ctx->nS) {
        mju_addToMatSparse(ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv,
                           ctx->S_val, ctx->S_rownnz, ctx->S_rowadr, ctx->S_colind);
      }
    }

    // dense
    else {
      mjtNum* target = ctx->H ? ctx->H : ctx->L;
      mju_sqrMatTD_impl(target, ctx->J, ctx->D, nefc, nv, /*flg_upper=*/ 0);
      mju_addToSymSparse(target, ctx->M, ctx->nv,
                         ctx->M_rownnz, ctx->M_rowadr, ctx->M_colind,
                         /*flg_upper=*/ 0);
      if (ctx->nS) {
        mju_addToSymSparse(target, ctx->S_val, nv,
                           ctx->S_rownnz, ctx->S_rowadr, ctx->S_colind,
                           /*flg_upper=*/ 0);
      }
    }
    ctx->flg_Hdirty = 0;
    ctx->ndowndate = 0;
  }

  // factorize H unless cone folding will factor Hcone directly.
  // Pass cost_L = 0 so L is deferred only when cone rank-1 updates alone exceed a full
  // factorization; passing cost_L = 3*Lflops would fold even a single cone contact, leaving
  // flg_Lvalid = 0 and forcing a full refactor_L + re-fold on the next iteration
  ctx->cone_fold = -1;
  ctx->flg_Lvalid = 0;
  if (!HessianConeFold(d, ctx, 0)) {
    primalFactorH(ctx, ctx->L, ctx->H ? ctx->H : ctx->L);
    ctx->flg_Lvalid = 1;
  }

  // add cones to factor if present
  if (ctx->ncone) {
    HessianCone(d, ctx);
  }

  // mark full update
  ctx->nupdate = nefc;
}


// elliptic case with many cones: rebuild Lcone with one factorization
static void HessianConeFolded(mjData* d, mjPrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;
  mjtNum* Hcone = ctx->is_sparse ? ctx->Hcone : ctx->Lcone;

  // start from quadratic Hessian H = M + S + J_quad'*D_quad*J_quad
  mju_copy(Hcone, ctx->H, ctx->nH);

  // scatter cone contact Hessians Jc'*Hc*Jc into Hcone, where Hc = V'*V
  for (int i=0; i < nefc; i++) {
    if (ctx->efc_state[i] == mjCNSTRSTATE_CONE) {
      const mjContact* con = ctx->contact + ctx->efc_id[i];
      int dim = con->dim;
      mjtNum V[30];
      int nrow = primalConeFactor(V, con, ctx->efc_D[i], ctx->Jaref+i);
      if (nrow > 0) {
        if (ctx->is_sparse) {
          int nnz = ctx->J_rownnz[i];
          if (nnz > 0) {
            primalConeMulJ(ctx->LTJ, V, ctx->HJ+ctx->J_rowadr[i], nrow, dim, nnz, nnz, NULL);
            primalAddConeOuterSparse(Hcone, ctx->H_rowadr, ctx->H_colind, nrow, nnz,
                                     ctx->HJ_colind+ctx->J_rowadr[i], ctx->LTJ);
            ctx->nupdate += nrow;
          }
        } else {
          primalAddConeOuterDense(Hcone, ctx->J+i*nv, V, nrow, dim, nv,
                                  ctx->LTJ, ctx->cholupd, ctx->dense_cols);
          ctx->nupdate += nrow;
        }
      }
      i += (dim-1);
    }
  }

  // Lcone = chol(Hcone)
  int rank = primalFactorH(ctx, ctx->Lcone, Hcone);
  if (rank < nv && ctx->flg_Hdirty) {
    FactorizeHessian(d, ctx, /*flg_recompute=*/1);
  }
}


// elliptic case, update path: Lcone = L, then (dim-1) rank-1 updates per cone contact
static void HessianConeUpdate(mjData* d, mjPrimalContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc;
  mjtNum* LTJ = ctx->LTJ;

  // start with Hcone = H
  mju_copy(ctx->Lcone, ctx->L, ctx->nL);

  // add contributions
  for (int i=0; i < nefc; i++) {
    if (ctx->efc_state[i] == mjCNSTRSTATE_CONE) {
      const mjContact* con = ctx->contact + ctx->efc_id[i];
      int dim = con->dim;
      mjtNum V[30];
      int nrow = primalConeFactor(V, con, ctx->efc_D[i], ctx->Jaref+i);

      // sparse
      if (ctx->is_sparse) {
        // get nnz for row i (same for all rows in contact)
        const int nnz = ctx->J_rownnz[i];
        if (nrow > 0 && nnz > 0) {
          primalConeMulJ(LTJ, V, ctx->HJ+ctx->J_rowadr[i], nrow, dim, nnz, nnz, NULL);
          for (int r=0; r < nrow; r++) {
            mju_cholUpdateSparse(ctx->Lcone, LTJ+r*nnz, nv, 1,
                                 ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz,
                                 ctx->HJ_colind+ctx->J_rowadr[i], ctx->cholscratch);
          }
          ctx->nupdate += nrow;
        }
      }

      // dense
      else if (nrow > 0) {
        primalConeMulJ(LTJ, V, ctx->J+i*nv, nrow, dim, nv, nv, NULL);
        for (int r=0; r < nrow; r++) {
          mju_cholUpdate(ctx->Lcone, LTJ+r*nv, nv, 1);
        }
        ctx->nupdate += nrow;
      }

      // advance to next constraint
      i += (dim-1);
    }
  }
}


// predict whether folding is faster than rank-1 updates for the current set, comparing flop counts:
// sparse update path: cost_L + 10*U where U sums (dim-1)*pathcost[lastcol] over cone contacts
// sparse folded path: Lflops for supernodal factorization plus 2*S for cone outer products on H
// dense update path:  cost_L + 10*U where U sums (dim-1)*(nv-firstcol)*(nv-firstcol+1)/2
//                     (mju_cholUpdate skips leading zeros k < firstcol and fills in above it)
// dense folded path:  vectorized mju_cholFactor (4*Lflops including copy/scatter)
static int HessianConeFoldFaster(const mjPrimalContext* ctx, mjtSize cost_L) {
  int nefc = ctx->nefc;
  mjtSize U = 0, S = 0;

  for (int i=0; i < nefc; i++) {
    if (ctx->efc_state[i] == mjCNSTRSTATE_CONE) {
      int dim = ctx->contact[ctx->efc_id[i]].dim;
      if (ctx->is_sparse) {
        int nnz = ctx->J_rownnz[i];
        U += (mjtSize)(dim - 1) * primalRowUpdateCost(ctx, i, 1);
        S += (mjtSize)(dim - 1) * nnz * nnz;
      } else {
        U += (mjtSize)(dim - 1) * primalRowUpdateCost(ctx, i, dim);
      }
      i += dim - 1;
    }
  }

  // compare update cost against folded refactorization
  mjtSize fold_cost = ctx->is_sparse ? ctx->Lflops + 2*S : 4*ctx->Lflops;
  return cost_L + 10*U > fold_cost;
}


// evaluate and cache whether cone folding is faster and fits on the solver stack
static int HessianConeFold(mjData* d, mjPrimalContext* ctx, mjtSize cost_L) {
  if (!ctx->ncone) {
    return 0;
  }
  if (ctx->cone_fold < 0) {
    ctx->cone_fold = HessianConeFoldFaster(ctx, cost_L);
    if (ctx->cone_fold && ctx->is_sparse && !ctx->Hcone) {
      size_t fold_bytes = mj_stackBytes((size_t)ctx->nH * sizeof(mjtNum), _Alignof(mjtNum));
      size_t loop_bytes = primalLoopScratchBytes(ctx, d);
      if (mj_stackBytesAvailable(d) < fold_bytes + loop_bytes) {
        mj_warning(d, mjWARN_CNSTRFULL, d->narena);
        ctx->cone_fold = 0;
      } else {
        ctx->Hcone = mjSTACKALLOC(d, ctx->nH, mjtNum);
      }
    }
  }
  return ctx->cone_fold;
}


// elliptic case: Hcone = H + cone_contributions
static void HessianCone(mjData* d, mjPrimalContext* ctx) {
  // many sliding contacts: one refactorization can beat sum(dim-1) rank-1
  // updates per cone contact. The decision depends only on sparsity patterns
  // and efc_state, so it is cached until a state change.
  if (HessianConeFold(d, ctx, 0)) {
    HessianConeFolded(d, ctx);
    return;
  }

  HessianConeUpdate(d, ctx);
}


// maximum accumulated Hessian downdates before rebuilding H from M + S + J'*D*J
#ifdef mjUSESINGLE
  #define PRIMAL_MAX_DOWNDATE 8
#else
  #define PRIMAL_MAX_DOWNDATE 32
#endif


// incremental update to Hessian factor due to changes in efc_state
static void HessianIncremental(mjData* d, mjPrimalContext* ctx, const int* oldstate) {
  int rank, nv = ctx->nv, nefc = ctx->nefc;
  mjtNum* cholupd = ctx->cholupd;

  // clear update counter
  ctx->nupdate = 0;

  // scan state changes: invalidate cached cone fold decision and accumulate quadratic update cost
  mjtSize U_quad = 0;
  int ndowndate_step = 0;
  for (int i=0; i < nefc; i++) {
    if (oldstate[i] != ctx->efc_state[i]) {
      ctx->cone_fold = -1;
      if (ctx->H &&
          ((oldstate[i] == mjCNSTRSTATE_QUADRATIC) !=
           (ctx->efc_state[i] == mjCNSTRSTATE_QUADRATIC))) {
        if (oldstate[i] == mjCNSTRSTATE_QUADRATIC) {
          ndowndate_step++;
        }
        U_quad += primalRowUpdateCost(ctx, i, 1);
      }
    }
  }

  // bound accumulated H downdates to prevent cancellation drift across iterations
  if (ndowndate_step > 0 && ctx->ndowndate + ndowndate_step >= PRIMAL_MAX_DOWNDATE) {
    FactorizeHessian(d, ctx, /*flg_recompute=*/1);
    return;
  }

  // decide whether L is cheaper to refactorize from H than to rank-1 update, and whether cone
  // folding beats updating L plus rank-1 updating Lcone
  mjtSize factor_cost = ctx->is_sparse ? ctx->Lflops : 3*ctx->Lflops;
  int refactor_L = ctx->H && (!ctx->flg_Lvalid || 10*U_quad > factor_cost);
  mjtSize cost_L = refactor_L ? factor_cost : 10*U_quad;
  int use_fold = HessianConeFold(d, ctx, cost_L);
  if (refactor_L || use_fold) {
    ctx->flg_Lvalid = 0;
  }

  // update H factorization
  for (int i=0; i < nefc; i++) {
    int flag_update = -1;

    // add quad
    if (oldstate[i] != mjCNSTRSTATE_QUADRATIC && ctx->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
      flag_update = 1;
    }

    // subtract quad
    else if (oldstate[i] == mjCNSTRSTATE_QUADRATIC && ctx->efc_state[i] != mjCNSTRSTATE_QUADRATIC) {
      flag_update = 0;
    }

    // perform update if flagged
    if (flag_update != -1) {
      // keep H synchronized with quadratic constraints (sparse or dense elliptic)
      if (ctx->H) {
        mjtNum scl = flag_update ? ctx->efc_D[i] : -ctx->efc_D[i];
        if (ctx->is_sparse) {
          int nnz = ctx->J_rownnz[i], adr = ctx->J_rowadr[i];
          primalAddRowOuterSparse(ctx->H, ctx->H_rowadr, ctx->H_colind, nnz,
                                  ctx->HJ_colind+adr, ctx->HJ+adr, scl);
        } else {
          primalAddRowOuterDense(ctx->H, ctx->J+i*nv, scl, nv);
        }
        if (!flag_update) {
          ctx->flg_Hdirty = 1;
          ctx->ndowndate++;
        }

        // skip L rank-1 update when L will be refactorized or cone folding is active
        if (!ctx->flg_Lvalid) {
          ctx->nupdate++;
          continue;
        }
      }

      // update with cholupd = J(i,:)*sqrt(D[i]))
      if (ctx->is_sparse) {
        // get nnz and adr of row i
        const int nnz = ctx->J_rownnz[i], adr = ctx->J_rowadr[i];

        // scale cholupd
        mju_scl(cholupd, ctx->HJ+adr, mju_sqrt(ctx->efc_D[i]), nnz);

        // sparse update or downdate
        rank = mju_cholUpdateSparse(ctx->L, cholupd, nv, flag_update,
                                    ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz,
                                    ctx->HJ_colind+adr, ctx->cholscratch);
      } else {
        mju_scl(cholupd, ctx->J+i*nv, mju_sqrt(ctx->efc_D[i]), nv);
        rank = mju_cholUpdate(ctx->L, cholupd, nv, flag_update);
      }
      ctx->nupdate++;

      // recompute H directly if accuracy lost
      if (rank < nv) {
        FactorizeHessian(d, ctx, /*flg_recompute=*/1);

        // nothing else to do
        return;
      }
    }
  }

  // factor L from H when L was deferred or quadratic updates exceeded refactorization cost
  if (!use_fold && !ctx->flg_Lvalid) {
    rank = primalFactorH(ctx, ctx->L, ctx->H);
    ctx->flg_Lvalid = 1;
    if (rank < nv && ctx->flg_Hdirty) {
      FactorizeHessian(d, ctx, /*flg_recompute=*/1);
      return;
    }
  }

  // add cones if present
  if (ctx->ncone) {
    HessianCone(d, ctx);
  }
}


// driver
static void mj_solPrimal(const mjModel* m, mjData* d, int island, int maxiter, int flg_Newton) {
  int iter = 0;
  mjtNum alpha, beta;
  mjPrimalContext ctx;
  mj_markStack(d);

  // make context
  PrimalPointers(m, d, &ctx, island);

  // unconstrained metric island: mj_fwdAcceleration already solved
  // (M + S)*qacc_smooth = qfrc_smooth - qfrc_bias via mj_effSolve (which includes all metric
  // terms S), so with nefc == 0 the primal cost 1/2*(a - qacc_smooth)^T*Mtilde*(a - qacc_smooth)
  // is minimized directly at qacc = qacc_smooth
  if (island >= 0 && ctx.nefc == 0) {
    mju_copy(ctx.qacc, ctx.qacc_smooth, ctx.nv);
    mju_zero(ctx.qfrc_constraint, ctx.nv);
    if (island < mjNISLAND) {
      d->solver_niter[island] = 0;
    }
    mj_freeStack(d);
    return;
  }

  PrimalAllocate(m, d, &ctx, flg_Newton);

  // local copies
  int nv   = ctx.nv;
  int nefc = ctx.nefc;
  int* oldstate = ctx.oldstate;

  // compute Ma = Mtilde * qacc
  mju_mulSymVecSparse(ctx.Ma, ctx.M, ctx.qacc, nv,
                      ctx.M_rownnz, ctx.M_rowadr, ctx.M_colind);
  if (ctx.flg_metric) {
    if (ctx.island < 0) {
      mj_effMulAdd(m, d, ctx.Ma, ctx.qacc, /*flg_contact=*/1);
    } else {
      mj_effMulAddIsland(m, d, ctx.Ma, ctx.qacc, ctx.island);
    }
  }


  // compute Jaref = J * qacc - aref  (dense or sparse)
  if (!ctx.is_sparse) {
    mju_mulMatVec(ctx.Jaref, ctx.J, ctx.qacc, nefc, nv);
  } else {
    mju_mulMatVecSparse(ctx.Jaref, ctx.J, ctx.qacc, nefc,
                        ctx.J_rownnz, ctx.J_rowadr, ctx.J_colind, ctx.J_rowsuper);
  }
  mju_subFrom(ctx.Jaref, ctx.efc_aref, nefc);

  // first update
  PrimalUpdateConstraint(&ctx);
  PrimalUpdateGrad(&ctx);

  // compute and save scaling factor: inverse trace of the inertia matrix
  mjtNum scale = inertiaScale(m, d, island);
  ctx.scale = scale;

  // CG: fold the efc rows and the rank-1 classes into the solver's copy of the metric blocks, so
  // the preconditioner sees the curvature the operator carries on the flex dofs
  if (ctx.epL && !mj_effPrecFold(m, d, ctx.epL, island, nefc, ctx.efc_D, ctx.is_sparse, ctx.J,
                                 ctx.J_rownnz, ctx.J_rowadr, ctx.J_colind)) {
    ctx.epL = NULL;
  }

  // Mgrad = M \ grad: the CG preconditioned gradient, also the convergence certificate
  PrimalUpdateMgrad(&ctx, /*flg_Newton=*/0);

  // convergence certificate: when !ctx.flg_metric the cost is strongly convex in the M-norm,
  // bounding suboptimality by duality gap at current constraint forces:
  //   cost(qacc) - cost* <= 0.5 * grad'*M^-1*grad
  // under discrete metric Mgrad = P^-1 grad where P does not bound Hessian, so gap bound
  // only holds when !ctx.flg_metric. if below tolerance (e.g. good warmstart), skip main loop
  int flg_gap = mju_max(0, 0.5*scale*mju_dot(ctx.grad, ctx.Mgrad, nv)) < m->opt.tolerance;

  // the gap bounds the *cost* suboptimality; on stiff constraints this permits force
  // errors of order sqrt(2*gap*stiffness). Newton solutions are characteristically
  // force-accurate, so Newton zero-iteration exits also require the gradient criterion;
  // CG solutions are characteristically cost-accurate and exit on the gap alone;
  // under discrete metric, gating on ctx.flg_metric is deliberately conservative:
  // preconditioner P != Mtilde when off-diagonal terms couple, so zero-iteration exits
  // also require the gradient criterion (remaining sound when P is diagonal)
  int flg_gradient = scale*mju_norm(ctx.grad, nv) < m->opt.tolerance;
  int flg_certificate = flg_gap && (!(flg_Newton || ctx.flg_metric) || flg_gradient);
  int flg_done = flg_certificate;

  // Newton: compute and factorize Hessian, Mgrad = H \ grad
  if (!flg_done && flg_Newton) {
    if (MakeHessian(m, d, &ctx)) {
      FactorizeHessian(d, &ctx, /*flg_recompute=*/0);
      PrimalUpdateMgrad(&ctx, /*flg_Newton=*/1);

      // Newton decrement already below tolerance: converged, skip the first line search
      // (gradient-gated like the certificate: H^-1 suppresses stiff-direction force errors)
      flg_done = flg_gradient &&
                 mju_max(0, 0.5*scale*mju_dot(ctx.grad, ctx.Mgrad, nv)) < m->opt.tolerance;
    } else {
      // arena exhausted during Hessian/Cholesky setup: fall back to CG
      // (conservative discrete metric gate: P != Mtilde with off-diagonal coupling)
      flg_Newton = 0;
      flg_done = flg_gap && (!ctx.flg_metric || flg_gradient);
      if (!flg_done) {
        // PrimalAllocate unconditionally carves cholupd[nv], cholscratch[nv] (sparse) /
        // L[nv*nv] (dense), and LTJ[5*nv] (elliptic) when flg_Newton is set; reuse them
        // for the 4 CG vectors now that Newton is disabled
        ctx.gradold  = ctx.cholupd;
        ctx.Mgradold = ctx.is_sparse ? ctx.cholscratch : ctx.L;
        mjtNum* cg_buf = ctx.LTJ ? ctx.LTJ : mjSTACKALLOC(d, 2*nv, mjtNum);
        ctx.graddif  = cg_buf;
        ctx.Mgraddif = cg_buf + nv;
      }
    }
  }

  // start both with preconditioned gradient
  if (!flg_done) {
    mju_scl(ctx.search, ctx.Mgrad, -1, nv);
  }

  // main loop
  while (!flg_done && iter < maxiter) {
    // perform linesearch
    mjtNum ls_improvement;
    alpha = PrimalSearch(&ctx, m->opt.tolerance * m->opt.ls_tolerance, m->opt.ls_iterations,
                         &ls_improvement);

    // no improvement: done
    if (alpha == 0) {
      break;
    }

    // move to new solution
    mju_addToScl(ctx.qacc, ctx.search, alpha, nv);
    mju_addToScl(ctx.Ma, ctx.Mv, alpha, nv);
    mju_addToScl(ctx.Jaref, ctx.Jv, alpha, nefc);

    // save old
    if (!flg_Newton) {
      mju_copy(ctx.gradold, ctx.grad, nv);
      mju_copy(ctx.Mgradold, ctx.Mgrad, nv);
    }
    mju_copyInt(oldstate, ctx.efc_state, nefc);

    // update
    PrimalUpdateConstraint(&ctx);
    if (flg_Newton) {
      HessianIncremental(d, &ctx, oldstate);
    }
    PrimalUpdateGradient(&ctx, flg_Newton);

    // count state changes
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (ctx.efc_state[i] != oldstate[i]);
    }

    // scale improvement, gradient, save stats
    mjtNum improvement = scale * ls_improvement;
    mjtNum gradient = scale * mju_norm(ctx.grad, nv);
    saveStats(m, d, island, iter, improvement, gradient, ctx.LSslope,
              ctx.nactive, nchange, ctx.LSiter, ctx.nupdate);

    // Newton decrement: 0.5*grad'*H^-1*grad, the model's predicted improvement of the
    // next step; clamp to 0 so that tolerance == 0 keeps early termination disabled
    mjtNum decrement = flg_Newton ? mju_max(0, 0.5*scale*mju_dot(ctx.grad, ctx.Mgrad, nv)) : 0;

    // increment iteration count
    iter++;

    // termination
    if ((improvement > 0 && improvement < m->opt.tolerance) ||
         gradient < m->opt.tolerance ||
         (flg_Newton && decrement < m->opt.tolerance)) {
      break;
    }

    // update direction
    if (flg_Newton) {
      mju_scl(ctx.search, ctx.Mgrad, -1, nv);
    } else {
#ifndef mjCG_PRP
      // Hager-Zhang conjugate direction update
      mjtNum d_dot_y, y_dot_My, y_dot_Mgrad, d_dot_grad;
      mjtNum beta_hz;
      mjtNum d_norm, grad_norm, eta_k;
      const mjtNum eta = 0.01;

      // graddif = grad - gradold, Mgraddif = Mgrad - Mgradold
      mju_sub(ctx.graddif, ctx.grad, ctx.gradold, nv);
      mju_sub(ctx.Mgraddif, ctx.Mgrad, ctx.Mgradold, nv);

      // compute d'*y; restart to steepest descent if conjugacy is lost
      d_dot_y = mju_dot(ctx.search, ctx.graddif, nv);
      if (d_dot_y < mjMINVAL) {
        beta = 0;
      } else {
        // compute remaining inner products for the HZ formula
        y_dot_My = mju_dot(ctx.graddif, ctx.Mgraddif, nv);
        y_dot_Mgrad = mju_dot(ctx.graddif, ctx.Mgrad, nv);
        d_dot_grad = mju_dot(ctx.search, ctx.grad, nv);

        // primary Hager-Zhang beta coefficient
        beta_hz = (y_dot_Mgrad - 2*(y_dot_My/d_dot_y)*d_dot_grad) / d_dot_y;

        // dynamic truncation threshold to ensure d is not orthogonal to grad
        d_norm = mju_norm(ctx.search, nv);
        grad_norm = mju_norm(ctx.grad, nv);
        eta_k = -1.0 / mju_max(mjMINVAL, d_norm * mju_min(eta, grad_norm));

        // apply lower bound
        beta = mju_max(eta_k, beta_hz);
      }
#else
      // Polak-Ribiere-Plus conjugate direction update
      mju_sub(ctx.Mgraddif, ctx.Mgrad, ctx.Mgradold, nv);
      beta = mju_dot(ctx.grad, ctx.Mgraddif, nv) /
             mju_max(mjMINVAL, mju_dot(ctx.gradold, ctx.Mgradold, nv));

      // reset if negative
      if (beta < 0) {
        beta = 0;
      }
#endif

      // update
      for (int i=0; i < nv; i++) {
        ctx.search[i] = -ctx.Mgrad[i] + beta*ctx.search[i];
      }
    }
  }

  // finalize statistics
  if (island < mjNISLAND) {
    // if island is -1 (monolithic), clamp to 0
    int island_stat = island < 0 ? 0 : island;

    // update solver iterations
    d->solver_niter[island_stat] += iter;

    // set solver_nnz; if the certificate fired, no Hessian was built: report Jacobian nnz
    if (flg_Newton && !flg_certificate) {
      if (mj_isSparse(m)) {
        // two L factors if Lcone is present
        int num_factors = 1 + (ctx.Lcone != NULL);
        d->solver_nnz[island_stat] = num_factors * ctx.nL + ctx.nH;
      } else {
        d->solver_nnz[island_stat] = nv*nv;
      }
    } else {
      d->solver_nnz[island_stat] = ctx.nJ;
    }
  }

  mj_freeStack(d);
}


// CG entry point
void mj_solCG(const mjModel* m, mjData* d, int maxiter) {
  mj_solPrimal(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/0);
}


// CG entry point (one island)
void mj_solCG_island(const mjModel* m, mjData* d, int island, int maxiter) {
  mj_solPrimal(m, d, island, maxiter, /*flg_Newton=*/0);
}


// Newton entry point
void mj_solNewton(const mjModel* m, mjData* d, int maxiter) {
  mj_solPrimal(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/1);
}


// Newton entry point (one island)
void mj_solNewton_island(const mjModel* m, mjData* d, int island, int maxiter) {
  mj_solPrimal(m, d, island, maxiter, /*flg_Newton=*/1);
}
