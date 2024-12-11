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
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_io.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"


//---------------------------------- utility functions ---------------------------------------------

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
// TODO: b/295296178 - add island support to Dual solvers
static void dualFinish(const mjModel* m, mjData* d) {
  // map constraint force to joint space
  mj_mulJacTVec(m, d, d->qfrc_constraint, d->efc_force);

  // compute constrained acceleration in joint space
  mj_solveM(m, d, d->qacc, d->qfrc_constraint, 1);
  mju_addTo(d->qacc, d->qacc_smooth, m->nv);
}



// compute 1/diag(AR)
// TODO: b/295296178 - add island support to Dual solvers
static void ARdiaginv(const mjModel* m, const mjData* d, mjtNum* res, int flg_subR) {
  int nefc = d->nefc;
  const mjtNum *AR = d->efc_AR;
  const mjtNum *R = d->efc_R;

  // sparse
  if (mj_isSparse(m)) {
    const int *rowadr = d->efc_AR_rowadr;
    const int *rownnz = d->efc_AR_rownnz;
    const int *colind = d->efc_AR_colind;

    for (int i=0; i < nefc; i++) {
      int nnz = rownnz[i];
      for (int j=0; j < nnz; j++) {
        int adr = rowadr[i] + j;
        if (i == colind[adr]) {
          res[i] = 1 / (flg_subR ? mju_max(mjMINVAL, AR[adr] - R[i]) : AR[adr]);
          break;
        }
      }
    }
  }

  // dense
  else {
    for (int i=0; i < nefc; i++) {
      int adr = i * (nefc + 1);
      res[i] = 1 / (flg_subR ? mju_max(mjMINVAL, AR[adr] - R[i]) : AR[adr]);
    }
  }
}



// extract diagonal block from AR, clamp diag to 1e-10 if flg_subR
// TODO: b/295296178 - add island support to Dual solvers
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
// TODO: b/295296178 - add island support to Dual solvers
static void residual(const mjModel* m, const mjData* d, mjtNum* res, int i, int dim, int flg_subR) {
  int nefc = d->nefc;

  // sparse
  if (mj_isSparse(m)) {
    for (int j=0; j < dim; j++) {
      res[j] = d->efc_b[i+j] + mju_dotSparse(d->efc_AR + d->efc_AR_rowadr[i+j],
                                             d->efc_force,
                                             d->efc_AR_rownnz[i+j],
                                             d->efc_AR_colind + d->efc_AR_rowadr[i+j],
                                             /*flg_unc1=*/0);
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
// TODO: b/295296178 - add island support to Dual solvers
static mjtNum costChange(const mjtNum* A, mjtNum* force, const mjtNum* oldforce,
                         const mjtNum* res, int dim) {
  mjtNum change;

  // compute change
  if (dim == 1) {
    mjtNum delta = force[0] - oldforce[0];
    change = 0.5*delta*delta*A[0] + delta*res[0];
  } else {
    mjtNum delta[6];
    mju_sub(delta, force, oldforce, dim);
    change = 0.5*mju_mulVecMatVec(delta, A, delta, dim) + mju_dot(delta, res, dim);
  }

  // positive change: restore force
  if (change > 1e-10) {
    mju_copy(force, oldforce, dim);
    change = 0;
  }

  return change;
}



// set efc_state to dual constraint state; return nactive
// TODO: b/295296178 - add island support to Dual solvers
static int dualState(const mjModel* m, const mjData* d, int* state) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum* force = d->efc_force;
  const mjtNum* floss = d->efc_frictionloss;

  // equality and friction always active
  int nactive = ne + nf;

  // equality
  for (int i=0; i < ne; i++) {
    state[i] = mjCNSTRSTATE_QUADRATIC;
  }

  // friction
  for (int i=ne; i < ne+nf; i++) {
    if (force[i] <= -floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARPOS;  // opposite of primal
    } else if (force[i] >= floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARNEG;
    } else {
      state[i] = mjCNSTRSTATE_QUADRATIC;
    }
  }

  // limit and contact
  for (int i=ne+nf; i < nefc; i++) {
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
      mjtNum T = mju_norm(f+1, dim-1);

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
      for (int j=0; j < dim; j++) {
        state[i+j] = result;
      }

      // advance
      i += (dim-1);
    }
  }

  return nactive;
}



//---------------------------- PGS solver ----------------------------------------------------------

// TODO: b/295296178 - add island support to Dual solvers
void mj_solPGS(const mjModel* m, mjData* d, int maxiter) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mj_markStack(d);
  mjtNum* ARinv = mjSTACKALLOC(d, nefc, mjtNum);
  int* oldstate = mjSTACKALLOC(d, nefc, int);

  // TODO: b/295296178 - Use island index (currently hardcoded to 0)
  int island = 0;
  mjtNum scale = 1 / (m->stat.meaninertia * mjMAX(1, m->nv));

  // precompute inverse diagonal of AR
  ARdiaginv(m, d, ARinv, 0);

  // initial constraint state
  dualState(m, d, d->efc_state);

  // main iteration
  int iter = 0;
  while (iter < maxiter) {
    // clear improvement
    mjtNum improvement = 0;

    // perform one sweep
    for (int i=0; i < nefc; i++) {
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
      mju_copy(oldforce, force+i, dim);

      // allocate AR submatrix, required later for costChage
      mjtNum Athis[36];

      // simple constraint
      if (d->efc_type[i] != mjCNSTR_CONTACT_ELLIPTIC) {
        // unconstrained minimum
        force[i] -= res[0]*ARinv[i];

        // impose interval and inequality constraints
        if (i >= ne && i < ne+nf) {
          if (force[i] < -floss[i]) {
            force[i] = -floss[i];
          } else if (force[i] > floss[i]) {
            force[i] = floss[i];
          }
        } else if (i >= ne+nf) {
          if (force[i] < 0) {
            force[i] = 0;
          }
        }
      }

      // elliptic cone constraint
      else {
        // get friction
        mjtNum *mu =  d->contact[d->efc_id[i]].friction;

        //-------------------- perform normal or ray update

        // Athis = AR(this,this)
        extractBlock(m, d, Athis, i, dim, 0);

        // normal force too small: normal update
        if (force[i] < mjMINVAL) {
          // unconstrained minimum
          force[i] -= res[0]*ARinv[i];

          // clamp
          if (force[i] < 0) {
            force[i] = 0;
          }

          // clear friction (just in case)
          mju_zero(force+i+1, dim-1);
        }

        // ray update
        else {
          // v = ray
          mjtNum v[6];
          mju_copy(v, force+i, dim);

          // denom = v' * AR(this,this) * v
          mjtNum v1[6];
          mju_mulMatVec(v1, Athis, v, dim, dim);
          mjtNum denom = mju_dot(v, v1, dim);

          // avoid division by 0
          if (denom >= mjMINVAL) {
            // x = v' * res / denom
            mjtNum x = -mju_dot(v, res, dim) / denom;

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
        mju_copy(bc, res+1, dim-1);
        for (int j=0; j < dim-1; j++) {
          mju_copy(Ac+j*(dim-1), Athis+(j+1)*dim+1, dim-1);
          bc[j] -= mju_dot(Ac+j*(dim-1), oldforce+1, dim-1);
          bc[j] += Athis[(j+1)*dim]*(force[i]-oldforce[0]);
        }

        // guard for f_normal==0
        if (force[i] < mjMINVAL) {
          mju_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active;
          mjtNum v[6];

          // solve
          if (dim == 3) {
            flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim == 4) {
            flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            mjtNum s = 0;
            for (int j=0; j < dim-1; j++) {
              s += v[j]*v[j] / (mu[j]*mu[j]);
            }
            s = mju_sqrt(force[i]*force[i] / mju_max(mjMINVAL, s));
            for (int j=0; j < dim-1; j++) {
              v[j] *= s;
            }
          }

          // assign
          mju_copy(force+i+1, v, dim-1);
        }
      }

      // accumulate improvement
      if (dim == 1) {
        Athis[0] = 1/ARinv[i];
      }
      improvement -= costChange(Athis, force+i, oldforce, res, dim);

      // skip the rest of this constraint
      i += (dim-1);
    }

    // process state
    mju_copyInt(oldstate, d->efc_state, nefc);
    int nactive = dualState(m, d, d->efc_state);
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (oldstate[i] != d->efc_state[i]);
    }

    // scale improvement, save stats
    improvement *= scale;
    saveStats(m, d, island, iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // increment iteration count
    iter++;


    // terminate
    if (improvement < m->opt.tolerance) {
      break;
    }
  }

  // finalize statistics
  if (island < mjNISLAND) {
    // update solver iterations
    d->solver_niter[island] += iter;

    // set nnz
    if (mj_isSparse(m)) {
      d->solver_nnz[island] = 0;
      for (int i=0; i < nefc; i++) {
        d->solver_nnz[island] += d->efc_AR_rownnz[i];
      }
    } else {
      d->solver_nnz[island] = nefc*nefc;
    }
  }

  // map to joint space
  dualFinish(m, d);

  mj_freeStack(d);
}



//---------------------------- NoSlip solver -------------------------------------------------------

// TODO: b/295296178 - add island support to Dual solvers
void mj_solNoSlip(const mjModel* m, mjData* d, int maxiter) {
  int dim, iter = 0, ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mjtNum *mu, improvement;
  mjtNum v[5], Ac[25], bc[5], res[5], oldforce[5], delta[5], mid, y, K0, K1;
  mjContact* con;
  mj_markStack(d);
  mjtNum* ARinv = mjSTACKALLOC(d, nefc, mjtNum);
  int* oldstate = mjSTACKALLOC(d, nefc, int);

  // TODO: b/295296178 - Use island index (currently hardcoded to 0)
  int island = 0;
  mjtNum scale = 1 / (m->stat.meaninertia * mjMAX(1, m->nv));

  // precompute inverse diagonal of A
  ARdiaginv(m, d, ARinv, 1);

  // initial constraint state
  dualState(m, d, d->efc_state);

  // main iteration
  while (iter < maxiter) {
    // clear improvement
    improvement = 0;

    // correct for cost change at iter 0
    if (iter == 0) {
      for (int i=0; i < nefc; i++) {
        improvement += 0.5*force[i]*force[i]*d->efc_R[i];
      }
    }

    // perform one sweep: dry friction
    for (int i=ne; i < ne+nf; i++) {
      // compute residual, save old
      residual(m, d, res, i, 1, 1);
      oldforce[0] = force[i];

      // unconstrained minimum
      force[i] -= res[0]*ARinv[i];

      // impose interval constraints
      if (force[i] < -floss[i]) {
        force[i] = -floss[i];
      } else if (force[i] > floss[i]) {
        force[i] = floss[i];
      }

      // add to improvement
      delta[0] = force[i] - oldforce[0];
      improvement -= 0.5*delta[0]*delta[0]/ARinv[i] + delta[0]*res[0];
    }

    // perform one sweep: contact friction
    for (int i=ne+nf; i < nefc; i++) {
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
          mju_copy(oldforce, force+j, 2);

          // Ac = AR-submatirx
          extractBlock(m, d, Ac, j, 2, 1);

          // bc = b-subvector + Ac,rest * f_rest
          mju_copy(bc, res, 2);
          for (int k=0; k < 2; k++) {
            bc[k] -= mju_dot(Ac+k*2, oldforce, 2);
          }

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
        i += 2*(dim-1)-1;
      }

      // elliptic contact
      else if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // compute residual, save old
        residual(m, d, res, i+1, dim-1, 1);
        mju_copy(oldforce, force+i+1, dim-1);

        // Ac = AR-submatrix
        extractBlock(m, d, Ac, i+1, dim-1, 1);

        // bc = b-subvector + Ac,rest * f_rest
        mju_copy(bc, res, dim-1);
        for (int j=0; j < dim-1; j++) {
          bc[j] -= mju_dot(Ac+j*(dim-1), oldforce, dim-1);
        }

        // guard for f_normal==0
        if (force[i] < mjMINVAL) {
          mju_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active = 0;

          // solve
          if (dim == 3) {
            flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim == 4) {
            flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            mjtNum s = 0;
            for (int j=0; j < dim-1; j++) {
              s += v[j]*v[j]/(mu[j]*mu[j]);
            }
            s = mju_sqrt(force[i]*force[i] / mju_max(mjMINVAL, s));
            for (int j=0; j < dim-1; j++) {
              v[j] *= s;
            }
          }

          // assign
          mju_copy(force+i+1, v, dim-1);
        }

        // accumulate improvement
        improvement -= costChange(Ac, force+i+1, oldforce, res, dim-1);

        // skip the rest of this contact
        i += (dim-1);
      }
    }

    // process state
    mju_copyInt(oldstate, d->efc_state, nefc);
    int nactive = dualState(m, d, d->efc_state);
    int nchange = 0;
    for (int i=0; i < nefc; i++) {
      nchange += (oldstate[i] != d->efc_state[i]);
    }

    // scale improvement, save stats
    improvement *= scale;

    // save noslip stats after all the entries from regular solver
    int stats_iter = iter + d->solver_niter[island];
    saveStats(m, d, island, stats_iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // increment iteration count
    iter++;

    // terminate
    if (improvement < m->opt.noslip_tolerance) {
      break;
    }
  }

  // update solver iterations
  d->solver_niter[island] += iter;

  // map to joint space
  dualFinish(m, d);

  mj_freeStack(d);
}



//------------------------- CG and Newton solver  --------------------------------------------------

// CG context
struct _mjCGContext {
  int flg_Newton;         // 1: Newton, 0: CG

  // island-related
  int island;             // current island index, -1 if monolithic
  int nv;                 // number of dofs
  int nefc;               // number of constraints
  int* dofind;            // dof indices of this island, NULL if monolithic
  int* efcind;            // constraint indices of this island, NULL if monolithic

  // common arrays (CGallocate)
  mjtNum* Jaref;          // Jac*qacc - aref                              (nefc x 1)
  mjtNum* Jv;             // Jac*search                                   (nefc x 1)
  mjtNum* Ma;             // M*qacc                                       (nv x 1)
  mjtNum* Mv;             // M*search                                     (nv x 1)
  mjtNum* grad;           // gradient of master cost                      (nv x 1)
  mjtNum* Mgrad;          // M\grad or H\grad                             (nv x 1)
  mjtNum* search;         // linesearch vector                            (nv x 1)
  mjtNum* quad;           // quadratic polynomials for constraint costs   (nefc x 3)

  // Newton arrays, known-size (CGallocate)
  mjtNum* D;              // constraint inertia                           (nefc x 1)
  mjtNum* C;              // reduced sparse inertia matrix                (nC x 1)
  int* H_rowadr;          // Hessian row addresses                        (nv x 1)
  int* H_rownnz;          // Hessian row nonzeros                         (nv x 1)
  int* H_lowernnz;        // Hessian lower triangle row nonzeros          (nv x 1)
  int* L_rownnz;          // Hessian factor row nonzeros                  (nv x 1)
  int* L_rowadr;          // Hessian factor row addresses                 (nv x 1)

  // Newton arrays, computed-size (HessianMake)
  int nH;                 // number of nonzeros in Hessian H
  int* H_colind;          // Hessian column indices                       (nH x 1)
  mjtNum* H;              // Hessian                                      (nH x 1)
  int nL;                 // number of nonzeros in Cholesky factor L
  int* L_colind;          // Cholesky factor column indices               (nL x 1)
  mjtNum* L;              // Cholesky factor                              (nL x 1)
  mjtNum* Lcone;          // Cholesky factor with cone contributions      (nL x 1)

  // globals
  mjtNum cost;            // constraint + Gauss cost
  mjtNum quadGauss[3];    // quadratic polynomial for Gauss cost
  mjtNum scale;           // scaling factor for improvement and gradient
  int nactive;            // number of active constraints
  int ncone;              // number of contacts in cone state
  int nupdate;            // number of Cholesky updates

  // linesearch diagnostics
  int LSiter;             // number of linesearch iterations
  int LSresult;           // linesearch result
  mjtNum LSslope;         // linesearch slope at solution
};
typedef struct _mjCGContext mjCGContext;


// allocate fixed-size arrays in mjCGContext
//  mj_{mark/free}Stack in calling function!
static void CGallocate(const mjModel* m, mjData* d, mjCGContext* ctx,
                       int island, int flg_Newton) {
  // clear everything
  memset(ctx, 0, sizeof(mjCGContext));

  // get sizes
  int nv      = island < 0 ? m->nv   : d->island_dofnum[island];
  int nefc    = island < 0 ? d->nefc : d->island_efcnum[island];

  // island-related
  ctx->island = island;
  ctx->nv     = nv;
  ctx->nefc   = nefc;
  ctx->dofind = island < 0 ? NULL : d->island_dofind + d->island_dofadr[island];
  ctx->efcind = island < 0 ? NULL : d->island_efcind + d->island_efcadr[island];

  // common arrays
  ctx->Jaref  = mjSTACKALLOC(d, nefc, mjtNum);
  ctx->Jv     = mjSTACKALLOC(d, nefc, mjtNum);
  ctx->Ma     = mjSTACKALLOC(d, nv, mjtNum);
  ctx->Mv     = mjSTACKALLOC(d, nv, mjtNum);
  ctx->grad   = mjSTACKALLOC(d, nv, mjtNum);
  ctx->Mgrad  = mjSTACKALLOC(d, nv, mjtNum);
  ctx->search = mjSTACKALLOC(d, nv, mjtNum);
  ctx->quad   = mjSTACKALLOC(d, nefc*3, mjtNum);

  // Newton only, known-size arrays
  ctx->flg_Newton = flg_Newton;
  if (flg_Newton) {
    ctx->D    = mjSTACKALLOC(d, nefc, mjtNum);

    // sparse Newton only
    if (mj_isSparse(m)) {
      ctx->C          = mjSTACKALLOC(d, m->nC, mjtNum);
      ctx->H_rowadr   = mjSTACKALLOC(d, nv, int);
      ctx->H_rownnz   = mjSTACKALLOC(d, nv, int);
      ctx->H_lowernnz = mjSTACKALLOC(d, nv, int);
      ctx->L_rownnz   = mjSTACKALLOC(d, nv, int);
      ctx->L_rowadr   = mjSTACKALLOC(d, nv, int);
    }
  }
}



// update efc_force, qfrc_constraint, cost-related
static void CGupdateConstraint(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nefc = ctx->nefc, nv = ctx->nv;
  const int* dofind = ctx->dofind;
  const int* efcind = ctx->efcind;

  // update constraints
  mj_constraintUpdate_island(m, d, ctx->Jaref, &(ctx->cost), ctx->flg_Newton, ctx->island);

  // count active and cone
  ctx->nactive = 0;
  ctx->ncone = 0;
  for (int c=0; c < nefc; c++) {
    int i = efcind ? efcind[c] : c;
    ctx->nactive += (d->efc_state[i] != mjCNSTRSTATE_SATISFIED);
    ctx->ncone += (d->efc_state[i] == mjCNSTRSTATE_CONE);
  }

  // add Gauss cost, set in quadratic[0]
  mjtNum Gauss = 0;
  for (int c=0; c < nv; c++) {
    int i = dofind ? dofind[c] : c;
    Gauss += 0.5 * (ctx->Ma[c] - d->qfrc_smooth[i]) * (d->qacc[i] - d->qacc_smooth[i]);
  }

  ctx->quadGauss[0] = Gauss;
  ctx->cost += Gauss;
}



// update grad, Mgrad
static void CGupdateGradient(const mjModel* m, const mjData* d, mjCGContext* ctx) {
  int nv = ctx->nv;
  const int* dofind = ctx->dofind;

  // grad = M*qacc - qfrc_smooth - qfrc_constraint
  for (int c=0; c < nv; c++) {
    int i = dofind ? dofind[c] : c;
    ctx->grad[c] = ctx->Ma[c] - d->qfrc_smooth[i] - d->qfrc_constraint[i];
  }

  // Newton: Mgrad = H \ grad
  // TODO: b/295296178 - add island support to Newton solver
  if (ctx->flg_Newton) {
    if (mj_isSparse(m)) {
      mju_cholSolveSparse(ctx->Mgrad, (ctx->ncone ? ctx->Lcone : ctx->L),
                          ctx->grad, nv, ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind);
    } else {
      mju_cholSolve(ctx->Mgrad, (ctx->ncone ? ctx->Lcone : ctx->L), ctx->grad, nv);
    }
  }

  // CG: Mgrad = M \ grad
  else {
    mju_copy(ctx->Mgrad, ctx->grad, nv);
    mj_solveM_island(m, d, ctx->Mgrad, ctx->island);
  }
}



// prepare quadratic polynomials and contact cone quantities
static void CGprepare(const mjModel* m, const mjData* d, mjCGContext* ctx) {
  int nv = ctx->nv, nefc = ctx->nefc, island = ctx->island;
  const int* dofind = ctx->dofind;
  const int* efcind = ctx->efcind;
  const mjtNum* v = ctx->search;

  // Gauss: alpha^2*0.5*v'*M*v + alpha*v'*(Ma-qfrc_smooth) + 0.5*(a-qacc_smooth)'*(Ma-qfrc_smooth)
  //  quadGauss[0] already computed in CGupdateConstraint
  mjtNum v_dot_smooth;
  if (island < 0) {
    v_dot_smooth = mju_dot(d->qfrc_smooth, v, nv);
  } else {
    v_dot_smooth = 0;
    for (int c=0; c < nv; c++) {
      v_dot_smooth += d->qfrc_smooth[dofind[c]] * v[c];
    }
  }
  ctx->quadGauss[1] = mju_dot(v, ctx->Ma, nv) - v_dot_smooth;
  ctx->quadGauss[2] = 0.5*mju_dot(v, ctx->Mv, nv);

  // process constraints
  for (int c=0; c < nefc; c++) {
    int i = efcind ? efcind[c] : c;

    // pointers to numeric data
    const mjtNum* Jv = ctx->Jv + c;
    const mjtNum* Jaref = ctx->Jaref + c;
    const mjtNum* D = d->efc_D + i;

    // pointer to this quadratic
    mjtNum* quad = ctx->quad + 3*c;

    // init with scalar quadratic
    mjtNum DJ0 = D[0]*Jaref[0];
    quad[0] = Jaref[0]*DJ0;
    quad[1] = Jv[0]*DJ0;
    quad[2] = Jv[0]*D[0]*Jv[0];

    // elliptic cone: extra processing
    if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {
      // extract contact info
      mjContact* con = d->contact + d->efc_id[i];
      int dim = con->dim;
      mjtNum U[6], V[6], UU = 0, UV = 0, VV = 0, mu = con->mu;
      mjtNum* friction = con->friction;

      // complete vector quadratic (for bottom zone)
      for (int j=1; j < dim; j++) {
        mjtNum DJj = D[j]*Jaref[j];
        quad[0] += Jaref[j]*DJj;
        quad[1] += Jv[j]*DJj;
        quad[2] += Jv[j]*D[j]*Jv[j];
      }

      // rescale to make primal cone circular
      U[0] = Jaref[0]*mu;
      V[0] = Jv[0]*mu;
      for (int j=1; j < dim; j++) {
        U[j] = Jaref[j]*friction[j-1];
        V[j] = Jv[j]*friction[j-1];
      }

      // accumulate sums of squares
      for (int j=1; j < dim; j++) {
        UU += U[j]*U[j];
        UV += U[j]*V[j];
        VV += V[j]*V[j];
      }

      // store in quad[3-8], using the fact that dim>=3
      quad[3] = U[0];
      quad[4] = V[0];
      quad[5] = UU;
      quad[6] = UV;
      quad[7] = VV;
      quad[8] = D[0] / ((mu*mu) * (1 + (mu*mu)));

      // advance to next constraint
      c += (dim-1);
    }

    // apply scaling
    quad[0] *= 0.5;
    quad[2] *= 0.5;
  }
}



// linesearch evaluation point
struct _mjCGPnt {
  mjtNum alpha;
  mjtNum cost;
  mjtNum deriv[2];
};
typedef struct _mjCGPnt mjCGPnt;



// evaluate linesearch cost, return first and second derivatives
static void CGeval(const mjModel* m, const mjData* d, mjCGContext* ctx, mjCGPnt* p) {
  int ne = d->ne, nf = d->nf, nefc = ctx->nefc;
  const int* efcind = ctx->efcind;

  // clear result
  mjtNum cost = 0, alpha = p->alpha;
  mjtNum deriv[2] = {0, 0};

  // init quad with Gauss
  mjtNum quadTotal[3];
  mju_copy3(quadTotal, ctx->quadGauss);

  // process constraints
  for (int c=0; c < nefc; c++) {
    int i = efcind ? efcind[c] : c;

    // equality
    if (i < ne) {
      mju_addTo3(quadTotal, ctx->quad+3*c);
      continue;
    }

    // friction
    if (i < ne + nf) {
      // search point, friction loss, bound (Rf)
      mjtNum start = ctx->Jaref[c], dir = ctx->Jv[c];
      mjtNum x = start + alpha*dir;
      mjtNum f = d->efc_frictionloss[i];
      mjtNum Rf = d->efc_R[i]*f;

      // -bound < x < bound : quadratic
      if (-Rf < x && x < Rf) {
        mju_addTo3(quadTotal, ctx->quad+3*c);
      }

      // x < -bound : linear negative
      else if (x <= -Rf) {
        mjtNum qf[3] = {f*(-0.5*Rf-start), -f*dir, 0};
        mju_addTo3(quadTotal, qf);
      }

      // bound < x : linear positive
      else {
        mjtNum qf[3] = {f*(-0.5*Rf+start), f*dir, 0};
        mju_addTo3(quadTotal, qf);
      }
      continue;
    }

    // limit and contact
    if (d->efc_type[i] == mjCNSTR_CONTACT_ELLIPTIC) {         // elliptic cone
      // extract contact info
      mjContact* con = d->contact + d->efc_id[i];
      mjtNum* quad = ctx->quad + 3*c;
      int dim = con->dim;
      mjtNum mu = con->mu;

      // unpack quad
      mjtNum U0 = quad[3];
      mjtNum V0 = quad[4];
      mjtNum UU = quad[5];
      mjtNum UV = quad[6];
      mjtNum VV = quad[7];
      mjtNum Dm = quad[8];

      // compute N, Tsqr
      mjtNum N = U0 + alpha*V0;
      mjtNum Tsqr = UU + alpha*(2*UV + alpha*VV);

      // no tangential force : top or bottom zone
      if (Tsqr <= 0) {
        // bottom zone: quadratic cost
        if (N < 0) {
          mju_addTo3(quadTotal, quad);
        }

        // top zone: nothing to do
      }

      // otherwise regular processing
      else {
        // tangential force
        mjtNum T = mju_sqrt(Tsqr);

        // N>=mu*T : top zone
        if (N >= mu*T) {
          // nothing to do
        }

        // mu*N+T<=0 : bottom zone
        else if (mu*N+T <= 0) {
          mju_addTo3(quadTotal, quad);
        }

        // otherwise middle zone
        else {
          // derivatives
          mjtNum N1 = V0;
          mjtNum T1 = (UV + alpha*VV)/T;
          mjtNum T2 = VV/T - (UV + alpha*VV)*T1/(T*T);

          // add to cost
          cost += 0.5*Dm*(N-mu*T)*(N-mu*T);
          deriv[0] += Dm*(N-mu*T)*(N1-mu*T1);
          deriv[1] += Dm*((N1-mu*T1)*(N1-mu*T1) + (N-mu*T)*(-mu*T2));
        }
      }

      // advance to next constraint
      c += (dim-1);
    } else {                                                  // inequality
      // search point
      mjtNum x = ctx->Jaref[c] + alpha*ctx->Jv[c];

      // active
      if (x < 0) {
        mju_addTo3(quadTotal, ctx->quad+3*c);
      }
    }
  }

  // add total quadratic
  cost += alpha*alpha*quadTotal[2] + alpha*quadTotal[1] + quadTotal[0];
  deriv[0] += 2*alpha*quadTotal[2] + quadTotal[1];
  deriv[1] += 2*quadTotal[2];

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
static int updateBracket(const mjModel* m, const mjData* d, mjCGContext* ctx,
                         mjCGPnt* p, const mjCGPnt candidates[3], mjCGPnt* pnext) {
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
    CGeval(m, d, ctx, pnext);
  }

  return flag;
}



// line search
static mjtNum CGsearch(const mjModel* m, const mjData* d, mjCGContext* ctx) {
  int nv = ctx->nv;
  mjCGPnt p0, p1, p2, pmid, p1next, p2next;

  // clear results
  ctx->LSiter = 0;
  ctx->LSresult = 0;
  ctx->LSslope = 1;       // means not computed

  // save search vector length, check
  mjtNum snorm = mju_norm(ctx->search, nv);
  if (snorm < mjMINVAL) {
    ctx->LSresult = 1;                          // search vector too small
    return 0;
  }

  // compute scaled gradtol and slope scaling
  mjtNum gtol = m->opt.tolerance * m->opt.ls_tolerance * snorm / ctx->scale;
  mjtNum slopescl = ctx->scale / snorm;

  // compute Mv, Jv
  mj_mulM_island(m, d, ctx->Mv, ctx->search, ctx->island, /*flg_vecunc=*/0);
  mj_mulJacVec_island(m, d, ctx->Jv, ctx->search, ctx->island, /*flg_resunc=*/0, /*flg_vecunc=*/0);

  // prepare quadratics and cones
  CGprepare(m, d, ctx);

  // init at alpha = 0, save
  p0.alpha = 0;
  CGeval(m, d, ctx, &p0);

  // always attempt one Newton step
  p1.alpha = p0.alpha - p0.deriv[0]/p0.deriv[1];
  CGeval(m, d, ctx, &p1);
  if (p0.cost < p1.cost) {
    p1 = p0;
  }

  // check for initial convergence
  if (mju_abs(p1.deriv[0]) < gtol) {
    if (p1.alpha == 0) {
      ctx->LSresult = 2;  // no improvement, initial convergence
    } else {
      ctx->LSresult = 0;  // SUCCESS
    }
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
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
  int p2update = 0;
  while (p1.deriv[0]*dir <= -gtol && ctx->LSiter < m->opt.ls_iterations) {
    // save current
    p2 = p1;
    p2update = 1;

    // move to Newton point w.r.t current
    p1.alpha -= p1.deriv[0]/p1.deriv[1];
    CGeval(m, d, ctx, &p1);

    // check for convergence
    if (mju_abs(p1.deriv[0]) < gtol) {
      ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
      return p1.alpha;                          // SUCCESS
    }
  }

  // check for failure to bracket
  if (ctx->LSiter >= m->opt.ls_iterations) {
    ctx->LSresult = 3;                          // could not bracket
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // check for p2 update; SHOULD NOT OCCUR
  if (!p2update) {
    ctx->LSresult = 6;                          // no p2 update
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // compute next-points for bracket
  p2next = p1;
  p1next.alpha = p1.alpha - p1.deriv[0]/p1.deriv[1];
  CGeval(m, d, ctx, &p1next);

  // bracketed search
  while (ctx->LSiter < m->opt.ls_iterations) {
    // evaluate at midpoint
    pmid.alpha = 0.5*(p1.alpha + p2.alpha);
    CGeval(m, d, ctx, &pmid);

    // make list of candidates
    mjCGPnt candidates[3] = {p1next, p2next, pmid};

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
      return candidates[bestind].alpha;       // SUCCESS
    }

    // update brackets
    int b1 = updateBracket(m, d, ctx, &p1, candidates, &p1next);
    int b2 = updateBracket(m, d, ctx, &p2, candidates, &p2next);

    // no update possible: numerical accuracy reached, use midpoint
    if (!b1 && !b2) {
      if (pmid.cost < p0.cost) {
        ctx->LSresult = 0;  // SUCCESS
      } else {
        ctx->LSresult = 7;  // no improvement, could not bracket
      }

      ctx->LSslope = mju_abs(pmid.deriv[0])*slopescl;
      return pmid.alpha;
    }
  }

  // choose bracket with best cost
  if (p1.cost <= p2.cost && p1.cost < p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  } else if (p2.cost <= p1.cost && p2.cost < p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p2.deriv[0])*slopescl;
    return p2.alpha;
  } else {
    ctx->LSresult = 5;                          // no improvement
    return 0;
  }
}



// allocate and compute Hessian given efc_state
//  mj_{mark/free}Stack in caller function!
static void MakeHessian(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nv = m->nv, nefc = d->nefc;

  // compute constraint inertia
  for (int i=0; i < nefc; i++) {
    ctx->D[i] = d->efc_state[i] == mjCNSTRSTATE_QUADRATIC ? d->efc_D[i] : 0;
  }

  // sparse
  if (mj_isSparse(m)) {
    // copy values of reduced sparse inertia matrix C
    for (int i=0; i < m->nC; i++) {
      ctx->C[i] = d->qM[d->mapM2C[i]];
    }

    // initialize Hessian rowadr, rownnz
    mju_sqrMatTDSparseInit(ctx->H_rownnz, ctx->H_rowadr, nv,
                           d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind,
                           d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind, d->efc_JT_rowsuper,
                           d);

    // add nC to Hessian total nonzeros (unavoidable overcounting since H_colind is still unknown)
    ctx->nH = m->nC + ctx->H_rowadr[nv - 1] + ctx->H_rownnz[nv - 1];

    // shift H row adresses to make room for C
    int shift = 0;
    for (int r = 0; r < nv - 1; r++) {
      shift += d->C_rownnz[r];
      ctx->H_rowadr[r + 1] += shift;
    }

    // allocate H_colind and H
    ctx->H_colind = mjSTACKALLOC(d, ctx->nH, int);
    ctx->H = mjSTACKALLOC(d, ctx->nH, mjtNum);

    // compute H = J'*D*J
    mju_sqrMatTDSparse(ctx->H, d->efc_J, d->efc_JT, ctx->D, nefc, nv,
                       ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                       d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind, NULL,
                       d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind, d->efc_JT_rowsuper,
                       d);

    // add mass matrix: H = J'*D*J + C
    mj_addMSparse(m, d, ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                  ctx->C, d->C_rownnz, d->C_rowadr, d->C_colind);

    // count total and row non-zeros of reverse-Cholesky factor L
    ctx->nL = mju_cholFactorNNZ(ctx->L_rownnz, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind, nv, d);

    // compute L row adresses: rowadr = cumsum(rownnz)
    ctx->L_rowadr[0] = 0;
    for (int r=1; r < nv; r++) {
      ctx->L_rowadr[r] = ctx->L_rowadr[r-1] + ctx->L_rownnz[r-1];
    }

    // allocate L_colind, L, Lcone
    ctx->L_colind = mjSTACKALLOC(d, ctx->nL, int);
    ctx->L = mjSTACKALLOC(d, ctx->nL, mjtNum);
    if (m->opt.cone == mjCONE_ELLIPTIC) {
      ctx->Lcone = mjSTACKALLOC(d, ctx->nL, mjtNum);
    }

    // count nonzeros in rows of H lower triangle
    for (int r = 0; r < nv; r++) {
      const int* colind = ctx->H_colind + ctx->H_rowadr[r];
      int rownnz = ctx->H_rownnz[r];

      // count nonzeros up to diagonal (inclusive) for row r
      int nnz = 1;
      while (nnz < rownnz && colind[nnz - 1] < r) {
        nnz++;
      }

      // last row element is not the diagonal; SHOULD NOT OCCUR
      if (colind[nnz - 1] != r) {
        mjERROR("Newton solver Hessian has zero diagonal on row %d", r);
      }

      // save row nonzeros
      ctx->H_lowernnz[r] = nnz;
    }
  }

  // dense
  else {
    // allocate L, Lcone
    ctx->nL = nv*nv;
    ctx->L = mjSTACKALLOC(d, ctx->nL, mjtNum);
    if (m->opt.cone == mjCONE_ELLIPTIC) {
      ctx->Lcone = mjSTACKALLOC(d, ctx->nL, mjtNum);
    }

    // compute H = M + J'*D*J
    mju_sqrMatTD(ctx->L, d->efc_J, ctx->D, nefc, nv);
    mj_addMDense(m, d, ctx->L);
  }
}



// forward declaration of HessianCone (readability)
static void HessianCone(const mjModel* m, mjData* d, mjCGContext* ctx);

// factorize Hessian: L = chol(H), maybe (re)compute H given efc_state
static void FactorizeHessian(const mjModel* m, mjData* d, mjCGContext* ctx,
                             int flg_recompute) {
  int nv = m->nv, nefc = d->nefc;

  // maybe compute constraint inertia
  if (flg_recompute) {
    for (int i=0; i < nefc; i++) {
      ctx->D[i] = d->efc_state[i] == mjCNSTRSTATE_QUADRATIC ? d->efc_D[i] : 0;
    }
  }

  // sparse
  if (mj_isSparse(m)) {
    // maybe compute H = M + J'*D*J
    if (flg_recompute) {
      // compute H = J'*D*J
      mju_sqrMatTDSparse(ctx->H, d->efc_J, d->efc_JT, ctx->D, nefc, nv,
                        ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                        d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind, NULL,
                        d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind, d->efc_JT_rowsuper,
                        d);

      // add mass matrix: H = J'*D*J + C
      mj_addMSparse(m, d, ctx->H, ctx->H_rownnz, ctx->H_rowadr, ctx->H_colind,
                    ctx->C, d->C_rownnz, d->C_rowadr, d->C_colind);
    }

    // copy H lower-triangle into L, fill-in already accounted for
    for (int r = 0; r < nv; r++) {
      int nnz = ctx->H_lowernnz[r];
      mju_copy(ctx->L + ctx->L_rowadr[r], ctx->H + ctx->H_rowadr[r], nnz);
      mju_copyInt(ctx->L_colind + ctx->L_rowadr[r], ctx->H_colind + ctx->H_rowadr[r], nnz);
      ctx->L_rownnz[r] = nnz;
    }

    // in-place sparse factorization: L = chol(H)
    int rank = mju_cholFactorSparse(ctx->L, nv, mjMINVAL,
                                    ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, d);

    // rank-deficient; SHOULD NOT OCCUR
    if (rank != nv) {
      mjERROR("rank-deficient sparse Hessian");
    }

    // pre-counted nL does not match post-factorization nL; SHOULD NOT OCCUR
    if (ctx->nL !=  ctx->L_rowadr[nv-1] + ctx->L_rownnz[nv-1]) {
      mjERROR("mismatch between pre-counted and post-factorization L nonzeros");
    }
  }

  // dense
  else {
    // maybe compute H = M + J'*D*J
    if (flg_recompute) {
      mju_sqrMatTD(ctx->L, d->efc_J, ctx->D, nefc, nv);
      mj_addMDense(m, d, ctx->L);
    }

    // factorize H
    mju_cholFactor(ctx->L, nv, mjMINVAL);
  }

  // add cones to factor if present
  if (ctx->ncone) {
    HessianCone(m, d, ctx);
  }

  // mark full update
  ctx->nupdate = nefc;
}



// elliptic case: Hcone = H + cone_contributions
static void HessianCone(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nv = m->nv, nefc = d->nefc;
  mjtNum local[36];

  // start with Hcone = H
  mju_copy(ctx->Lcone, ctx->L, ctx->nL);

  mj_markStack(d);

  // storage for L'*J
  mjtNum* LTJ = mjSTACKALLOC(d, 6*nv, mjtNum);
  mjtNum* LTJ_row = mjSTACKALLOC(d, nv, mjtNum);
  int* LTJ_ind = mjSTACKALLOC(d, nv, int);

  // add contributions
  for (int i=0; i < nefc; i++) {
    if (d->efc_state[i] == mjCNSTRSTATE_CONE) {
      mjContact* con = d->contact + d->efc_id[i];
      int dim = con->dim;

      // Cholesky of local Hessian
      mju_copy(local, con->H, dim*dim);
      mju_cholFactor(local, dim, mjMINVAL);

      // sparse
      if (mj_isSparse(m)) {
        // get nnz for row i (same for all rows in contact)
        const int nnz = d->efc_J_rownnz[i];

        // compute LTJ = L'*J for this contact
        mju_zero(LTJ, dim*nnz);
        for (int r=0; r < dim; r++) {
          for (int c=0; c <= r; c++) {
            mju_addToScl(LTJ+c*nnz, d->efc_J+d->efc_J_rowadr[i+r], local[r*dim+c], nnz);
          }
        }

        // update
        for (int r=0; r < dim; r++) {
          // copy data for this row
          mju_copy(LTJ_row, LTJ+r*nnz, nnz);
          mju_copyInt(LTJ_ind, d->efc_J_colind+d->efc_J_rowadr[i+r], nnz);

          // update
          mju_cholUpdateSparse(ctx->Lcone, LTJ_row, nv, 1,
                               ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz, LTJ_ind, d);
        }
      }

      // dense
      else {
        // compute LTJ = L'*J for this contact row
        mju_zero(LTJ, dim*nv);
        for (int r=0; r < dim; r++) {
          for (int c=0; c <= r; c++) {
            mju_addToScl(LTJ+c*nv, d->efc_J+(i+r)*nv, local[r*dim+c], nv);
          }
        }

        // update
        for (int r=0; r < dim; r++) {
          mju_cholUpdate(ctx->Lcone, LTJ+r*nv, nv, 1);
        }
      }

      // count updates
      ctx->nupdate += dim;

      // advance to next constraint
      i += (dim-1);
    }
  }

  mj_freeStack(d);
}



// incremental update to Hessian factor due to changes in efc_state
static void HessianIncremental(const mjModel* m, mjData* d, mjCGContext* ctx, const int* oldstate) {
  int rank, nv = m->nv, nefc = d->nefc;
  mj_markStack(d);

  // local space
  mjtNum* vec = mjSTACKALLOC(d, nv, mjtNum);
  int* vec_ind = mjSTACKALLOC(d, nv, int);

  // clear update counter
  ctx->nupdate = 0;

  // update H factorization
  for (int i=0; i < nefc; i++) {
    int flag_update = -1;

    // add quad
    if (oldstate[i] != mjCNSTRSTATE_QUADRATIC && d->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
      flag_update = 1;
    }

    // subtract quad
    else if (oldstate[i] == mjCNSTRSTATE_QUADRATIC && d->efc_state[i] != mjCNSTRSTATE_QUADRATIC) {
      flag_update = 0;
    }

    // perform update if flagged
    if (flag_update != -1) {
      // update with vec = J(i,:)*sqrt(D[i]))
      if (mj_isSparse(m)) {
        // get nnz and adr of row i
        const int nnz = d->efc_J_rownnz[i], adr = d->efc_J_rowadr[i];

        // scale vec, copy colind
        mju_scl(vec, d->efc_J+adr, mju_sqrt(d->efc_D[i]), nnz);
        mju_copyInt(vec_ind, d->efc_J_colind+adr, nnz);

        // sparse update or downdate
        rank = mju_cholUpdateSparse(ctx->L, vec, nv, flag_update,
                                    ctx->L_rownnz, ctx->L_rowadr, ctx->L_colind, nnz, vec_ind,
                                    d);
      } else {
        mju_scl(vec, d->efc_J+i*nv, mju_sqrt(d->efc_D[i]), nv);
        rank = mju_cholUpdate(ctx->L, vec, nv, flag_update);
      }
      ctx->nupdate++;

      // recompute H directly if accuracy lost
      if (rank < nv) {
        mj_freeStack(d);
        FactorizeHessian(m, d, ctx, /*flg_recompute=*/1);

        // nothing else to do
        return;
      }
    }
  }

  // add cones if present
  if (ctx->ncone) {
    HessianCone(m, d, ctx);
  }

  mj_freeStack(d);
}



// driver
static void mj_solCGNewton(const mjModel* m, mjData* d, int island, int maxiter, int flg_Newton) {
  int iter = 0;
  mjtNum alpha, beta;
  mjtNum *gradold = NULL, *Mgradold = NULL, *Mgraddif = NULL;
  mjCGContext ctx;
  mj_markStack(d);

  // allocate context
  CGallocate(m, d, &ctx, island, flg_Newton);

  // local copies
  int nv   = ctx.nv;
  int nefc = ctx.nefc;
  const int* dofind = ctx.dofind;
  const int* efcind = ctx.efcind;

  // allocate local storage
  if (!flg_Newton) {
    gradold     = mjSTACKALLOC(d, nv, mjtNum);
    Mgradold    = mjSTACKALLOC(d, nv, mjtNum);
    Mgraddif    = mjSTACKALLOC(d, nv, mjtNum);
  }
  int* oldstate = mjSTACKALLOC(d, nefc, int);

  // initialize matrix-vector products
  int flg_vecunc = 1;  // d->qacc is uncompressed
  mj_mulM_island(m, d, ctx.Ma, d->qacc, island, flg_vecunc);
  int flg_resunc = 0;  // ctx.Jaref is compressed
  mj_mulJacVec_island(m, d, ctx.Jaref, d->qacc, island, flg_resunc, flg_vecunc);
  if (island < 0) {
    mju_subFrom(ctx.Jaref, d->efc_aref, nefc);
  } else {
    for (int c=0; c < nefc; c++) {
      ctx.Jaref[c] -= d->efc_aref[efcind[c]];
    }
  }

  // first update
  CGupdateConstraint(m, d, &ctx);
  if (flg_Newton) {
    // compute and factorize Hessian
    MakeHessian(m, d, &ctx);
    FactorizeHessian(m, d, &ctx, /*flg_recompute=*/0);
  }
  CGupdateGradient(m, d, &ctx);

  // start both with preconditioned gradient
  mju_scl(ctx.search, ctx.Mgrad, -1, nv);

  // compute and save scaling factor
  mjtNum scale;
  if (island < 0) {
    scale = 1 / (m->stat.meaninertia * mjMAX(1, m->nv));
  } else {
    mjtNum island_inertia = 0;
    for (int c=0; c < nv; c++) {
      island_inertia += d->qM[m->dof_Madr[dofind[c]]];
    }
    scale = 1 / island_inertia;
  }
  ctx.scale = scale;

  // main loop
  while (iter < maxiter) {
    // perform linesearch
    alpha = CGsearch(m, d, &ctx);

    // no improvement: done
    if (alpha == 0) {
      break;
    }

    // move to new solution
    if (island < 0) {
      mju_addToScl(d->qacc, ctx.search, alpha, nv);
    } else {
      for (int c=0; c < nv; c++) {
        d->qacc[dofind[c]] += alpha * ctx.search[c];
      }
    }
    mju_addToScl(ctx.Ma, ctx.Mv, alpha, nv);
    mju_addToScl(ctx.Jaref, ctx.Jv, alpha, nefc);

    // save old
    if (!flg_Newton) {
      mju_copy(gradold, ctx.grad, nv);
      mju_copy(Mgradold, ctx.Mgrad, nv);
    }
    if (island < 0) {
      mju_copyInt(oldstate, d->efc_state, nefc);
    } else {
      for (int c=0; c < nefc; c++) {
        oldstate[c] = d->efc_state[efcind[c]];
      }
    }
    mjtNum oldcost = ctx.cost;

    // update
    CGupdateConstraint(m, d, &ctx);
    if (flg_Newton) {
      HessianIncremental(m, d, &ctx, oldstate);
    }
    CGupdateGradient(m, d, &ctx);

    // count state changes
    int nchange = 0;
    for (int c=0; c < nefc; c++) {
      int i = efcind ? efcind[c] : c;
      nchange += (d->efc_state[i] != oldstate[c]);
    }

    // scale improvement, gradient, save stats
    mjtNum improvement = scale * (oldcost - ctx.cost);
    mjtNum gradient = scale * mju_norm(ctx.grad, nv);
    saveStats(m, d, island, iter, improvement, gradient, ctx.LSslope,
              ctx.nactive, nchange, ctx.LSiter, ctx.nupdate);

    // increment iteration count
    iter++;

    // termination
    if (improvement < m->opt.tolerance || gradient < m->opt.tolerance) {
      break;
    }

    // update direction
    if (flg_Newton) {
      mju_scl(ctx.search, ctx.Mgrad, -1, nv);
    } else {
      // Polak-Ribiere
      mju_sub(Mgraddif, ctx.Mgrad, Mgradold, nv);
      beta = mju_dot(ctx.grad, Mgraddif, nv) /
             mju_max(mjMINVAL, mju_dot(gradold, Mgradold, nv));

      // reset if negative
      if (beta < 0) {
        beta = 0;
      }

      // update
      for (int c=0; c < nv; c++) {
        ctx.search[c] = -ctx.Mgrad[c] + beta*ctx.search[c];
      }
    }
  }

  // finalize statistics
  if (island < mjNISLAND) {
    // if island is -1 (monolithic), clamp to 0
    int island_stat = island < 0 ? 0 : island;

    // update solver iterations
    d->solver_niter[island_stat] += iter;

    // set solver_nnz
    if (flg_Newton) {
      if (mj_isSparse(m)) {
        // two L factors if Lcone is present
        int num_factors = 1 + (ctx.Lcone != NULL);
        d->solver_nnz[island_stat] = num_factors * ctx.nL + ctx.nH;
      } else {
        d->solver_nnz[island_stat] = nv*nv;
      }
    } else {
      d->solver_nnz[island_stat] = 0;
    }
  }

  mj_freeStack(d);
}



// CG entry point
void mj_solCG(const mjModel* m, mjData* d, int maxiter) {
  mj_solCGNewton(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/0);
}



// CG entry point (one island)
void mj_solCG_island(const mjModel* m, mjData* d, int island, int maxiter) {
  mj_solCGNewton(m, d, island, maxiter, /*flg_Newton=*/0);
}



// Newton entry point
void mj_solNewton(const mjModel* m, mjData* d, int maxiter) {
  mj_solCGNewton(m, d, /*island=*/-1, maxiter, /*flg_Newton=*/1);
}
