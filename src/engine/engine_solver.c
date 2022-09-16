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
#include <mujoco/mjmodel.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_sparse.h"

//---------------------------------- utility functions ---------------------------------------------

// rescale cost and gradient
static mjtNum rescale(const mjModel* m, mjtNum x) {
  return x / (m->stat.meaninertia * mjMAX(1, m->nv));
}



// save solver statistics, count
static void saveStats(const mjModel* m, mjData* d, int* piter,
                      mjtNum improvement, mjtNum gradient, mjtNum lineslope,
                      int nactive, int nchange, int neval, int nupdate) {
  // compute position, increase iter
  int i = d->solver_iter + (*piter);
  (*piter)++;

  // save if within range
  if (i<mjNSOLVER) {
    d->solver[i].improvement = improvement;
    d->solver[i].gradient = gradient;
    d->solver[i].lineslope = lineslope;
    d->solver[i].nactive = nactive;
    d->solver[i].nchange = nchange;
    d->solver[i].neval = neval;
    d->solver[i].nupdate = nupdate;
  }
}



// finalize dual solver: map to joint space
static void dualFinish(const mjModel* m, mjData* d) {
  // map constraint force to joint space
  mj_mulJacTVec(m, d, d->qfrc_constraint, d->efc_force);

  // compute constrained acceleration in joint space
  mj_solveM(m, d, d->qacc, d->qfrc_constraint, 1);
  mju_addTo(d->qacc, d->qacc_smooth, m->nv);
}



// compute 1/diag(AR)
static void ARdiaginv(const mjModel* m, mjData* d, mjtNum* res, int flg_subR) {
  int nefc = d->nefc;
  const int *rowadr = d->efc_AR_rowadr;

  // sparse
  if (mj_isSparse(m)) {
    for (int i=0; i<nefc; i++) {
      for (int j=0; j<d->efc_AR_rownnz[i]; j++) {
        if (i==d->efc_AR_colind[rowadr[i]+j]) {
          res[i] = 1/(flg_subR ? mju_max(mjMINVAL, d->efc_AR[rowadr[i]+j]-d->efc_R[i])
                      : d->efc_AR[rowadr[i]+j]);
          break;
        }
      }
    }
  }

  // dense
  else {
    for (int i=0; i<nefc; i++) {
      res[i] = 1/(flg_subR ? mju_max(mjMINVAL, d->efc_AR[i*(nefc+1)]-d->efc_R[i])
                  : d->efc_AR[i*(nefc+1)]);
    }
  }
}



// extract diagonal block from AR, clamp diag to 1e-10 if flg_subR
static void extractBlock(const mjModel* m, mjData* d, mjtNum* Ac,
                         int start, int n, int flg_subR) {
  int k, nefc = d->nefc;
  const mjtNum *AR = d->efc_AR;
  const int *rownnz = d->efc_AR_rownnz, *rowadr = d->efc_AR_rowadr, *colind = d->efc_AR_colind;

  // sparse
  if (mj_isSparse(m)) {
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
    for (k=0; k<rownnz[start]; k++) {
      if (colind[rowadr[start]+k]==start) {
        break;
      }
    }

    // sanity check; SHOULD NOT OCCUR
    if (k>=rownnz[start]) {
      mju_error("Internal error in extractComponent");
    }

    // copy rows
    for (int j=0; j<n; j++) {
      mju_copy(Ac+j*n, AR+rowadr[start+j]+k, n);
    }
  }

  // dense
  else {
    for (int j=0; j<n; j++) {
      mju_copy(Ac+j*n, AR+start+(start+j)*nefc, n);
    }
  }

  // subtract R from diagonal, clamp to 1e-10 from below
  if (flg_subR) {
    for (int j=0; j<n; j++) {
      Ac[j*(n+1)] -= d->efc_R[start+j];
      Ac[j*(n+1)] = mjMAX(1e-10, Ac[j*(n+1)]);
    }
  }
}



// compute residual for one block
static void residual(const mjModel* m, mjData* d, mjtNum* res, int i, int dim, int flg_subR) {
  int nefc = d->nefc;

  // sparse
  if (mj_isSparse(m)) {
    for (int j=0; j<dim; j++) {
      res[j] = d->efc_b[i+j] + mju_dotSparse(d->efc_AR + d->efc_AR_rowadr[i+j],
                                             d->efc_force, d->efc_AR_rownnz[i+j],
                                             d->efc_AR_colind + d->efc_AR_rowadr[i+j]);
    }
  }

  // dense
  else {
    for (int j=0; j<dim; j++) {
      res[j] = d->efc_b[i+j] + mju_dot(d->efc_AR+(i+j)*nefc, d->efc_force, nefc);
    }
  }

  if (flg_subR) {
    for (int j=0; j<dim; j++) {
      res[j] -= d->efc_R[i+j]*d->efc_force[i+j];
    }
  }
}



// compute cost change
static mjtNum costChange(const mjtNum* A, mjtNum* force, const mjtNum* oldforce,
                         const mjtNum* res, int dim) {
  mjtNum delta[6], change;

  // compute change
  if (dim==1) {
    delta[0] = force[0] - oldforce[0];
    change = 0.5*delta[0]*delta[0]*A[0] + delta[0]*res[0];
  } else {
    mju_sub(delta, force, oldforce, dim);
    change = 0.5*mju_mulVecMatVec(delta, A, delta, dim) + mju_dot(delta, res, dim);
  }

  // positive change: restore
  if (change>1e-10) {
    mju_copy(force, oldforce, dim);
    change = 0;
  }

  return change;
}



// set efc_state to dual constraint state; return nactive
static int dualState(const mjModel* m, mjData* d) {
  int nactive, ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum *force = d->efc_force, *floss = d->efc_frictionloss;
  int* state = d->efc_state;

  // equality and friction always active
  nactive = ne + nf;

  // equality
  for (int i=0; i<ne; i++) {
    state[i] = mjCNSTRSTATE_QUADRATIC;
  }

  // friction
  for (int i=ne; i<ne+nf; i++) {
    if (force[i]<=-floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARPOS;  // opposite of primal
    } else if (force[i]>=floss[i]) {
      state[i] = mjCNSTRSTATE_LINEARNEG;
    } else {
      state[i] = mjCNSTRSTATE_QUADRATIC;
    }
  }

  // limit and contact
  for (int i=ne+nf; i<nefc; i++) {
    // non-negative
    if (d->efc_type[i]!=mjCNSTR_CONTACT_ELLIPTIC) {
      if (force[i]<=0) {
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
      for (int j=1; j<dim; j++) {
        f[j] = force[i+j]/con->friction[j-1];
      }

      // N = normal, T = norm of tangent vector
      mjtNum N = f[0];
      mjtNum T = mju_norm(f+1, dim-1);

      // top zone
      if (mu*N>=T) {
        result = mjCNSTRSTATE_SATISFIED;
      }

      // bottom zone
      else if (N+mu*T<=0) {
        result = mjCNSTRSTATE_QUADRATIC;
        nactive += dim;
      }

      // middle zone
      else {
        result = mjCNSTRSTATE_CONE;
        nactive += dim;
      }

      // replicate state in all cone dimensions
      for (int j=0; j<dim; j++) {
        state[i+j] = result;
      }

      // advance
      i += (dim-1);
    }
  }

  return nactive;
}



//---------------------------- PGS solver ----------------------------------------------------------

void mj_solPGS(const mjModel* m, mjData* d, int maxiter) {
  int dim, iter = 0, ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mjtNum *mu, x, denom, improvement;
  mjtNum v[6], v1[6], Athis[36], Ac[25], bc[5], res[6], oldforce[6];
  mjContact* con;
  mjMARKSTACK;
  mjtNum* ARinv = mj_stackAlloc(d, nefc);
  int* oldstate = (int*)mj_stackAlloc(d, nefc);

  // precompute inverse diagonal of AR
  ARdiaginv(m, d, ARinv, 0);

  // initial constraint state
  dualState(m, d);

  // main iteration
  while (iter<maxiter) {
    // clear improvement
    improvement = 0;

    // perform one sweep
    for (int i=0; i<nefc; i++) {
      // get constraint dimensionality
      if (d->efc_type[i]==mjCNSTR_CONTACT_ELLIPTIC) {
        dim = d->contact[d->efc_id[i]].dim;
      } else {
        dim = 1;
      }

      // compute residuals for this constraint, save force
      residual(m, d, res, i, dim, 0);
      mju_copy(oldforce, force+i, dim);

      // simple constraint
      if (d->efc_type[i]!=mjCNSTR_CONTACT_ELLIPTIC) {
        // unconstrained minimum
        force[i] -= res[0]*ARinv[i];

        // impose interval and inequality constraints
        if (i>=ne && i<ne+nf) {
          if (force[i]<-floss[i]) {
            force[i] = -floss[i];
          } else if (force[i]>floss[i]) {
            force[i] = floss[i];
          }
        } else if (i>=ne+nf) {
          if (force[i]<0) {
            force[i] = 0;
          }
        }
      }

      // elliptic cone constraint
      else {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        //-------------------- perform normal or ray update

        // Athis = AR(this,this)
        extractBlock(m, d, Athis, i, dim, 0);

        // normal force too small: normal update
        if (force[i]<mjMINVAL) {
          // unconstrained minimum
          force[i] -= res[0]*ARinv[i];

          // clamp
          if (force[i]<0) {
            force[i] = 0;
          }

          // clear friction (just in case)
          mju_zero(force+i+1, dim-1);
        }

        // ray update
        else {
          // v = ray
          mju_copy(v, force+i, dim);

          // denom = v' * AR(this,this) * v
          mju_mulMatVec(v1, Athis, v, dim, dim);
          denom = mju_dot(v, v1, dim);

          // avoid division by 0
          if (denom>=mjMINVAL) {
            // x = v' * res / denom
            x = -mju_dot(v, res, dim) / denom;

            // make sure normal is non-negative
            if (force[i]+x*v[0]<0) {
              x = -v[0]/force[i];
            }

            //  add x*v to f
            for (int j=0; j<dim; j++) {
              force[i+j] += x*v[j];
            }
          }
        }

        //-------------------- perform friction update, keep normal fixed

        // Ac = AR-submatrix; bc = b-subvector + Ac,rest * f_rest
        mju_copy(bc, res+1, dim-1);
        for (int j=0; j<dim-1; j++) {
          mju_copy(Ac+j*(dim-1), Athis+(j+1)*dim+1, dim-1);
          bc[j] -= mju_dot(Ac+j*(dim-1), oldforce+1, dim-1);
          bc[j] += Athis[(j+1)*dim]*(force[i]-oldforce[0]);
        }

        // guard for f_normal==0
        if (force[i]<mjMINVAL) {
          mju_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active;

          // solve
          if (dim==3) {
            flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim==4) {
            flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            mjtNum s = 0;
            for (int j=0; j<dim-1; j++) {
              s += v[j]*v[j] / (mu[j]*mu[j]);
            }
            s = mju_sqrt(force[i]*force[i] / mju_max(mjMINVAL, s));
            for (int j=0; j<dim-1; j++) {
              v[j] *= s;
            }
          }

          // assign
          mju_copy(force+i+1, v, dim-1);
        }
      }

      // accumulate improvement
      if (dim==1) {
        Athis[0] = 1/ARinv[i];
      }
      improvement -= costChange(Athis, force+i, oldforce, res, dim);

      // skip the rest of this constraint
      i += (dim-1);
    }

    // process state
    memcpy(oldstate, d->efc_state, nefc*sizeof(int));
    int nactive = dualState(m, d);
    int nchange = 0;
    for (int i=0; i<nefc; i++) {
      nchange += (oldstate[i]!=d->efc_state[i]);
    }

    // scale improvement, save stats, count
    improvement = rescale(m, improvement);
    saveStats(m, d, &iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // terminate
    if (improvement<m->opt.tolerance) {
      break;
    }
  }

  // update solver iterations
  d->solver_iter += iter;

  // set nnz
  if (mj_isSparse(m)) {
    d->solver_nnz = 0;
    for (int i=0; i<nefc; i++) {
      d->solver_nnz += d->efc_AR_rownnz[i];
    }
  } else {
    d->solver_nnz = nefc*nefc;
  }

  // map to joint space
  dualFinish(m, d);

  mjFREESTACK;
}



//---------------------------- NoSlip solver -------------------------------------------------------

void mj_solNoSlip(const mjModel* m, mjData* d, int maxiter) {
  int dim, iter = 0, ne = d->ne, nf = d->nf, nefc = d->nefc;
  const mjtNum *floss = d->efc_frictionloss;
  mjtNum *force = d->efc_force;
  mjtNum *mu, improvement;
  mjtNum v[5], Ac[25], bc[5], res[5], oldforce[5], delta[5], mid, y, K0, K1;
  mjContact* con;
  mjMARKSTACK;
  mjtNum* ARinv = mj_stackAlloc(d, nefc);
  int* oldstate = (int*)mj_stackAlloc(d, nefc);

  // precompute inverse diagonal of A
  ARdiaginv(m, d, ARinv, 1);

  // initial constraint state
  dualState(m, d);

  // main iteration
  while (iter<maxiter) {
    // clear improvement
    improvement = 0;

    // correct for cost change at iter 0
    if (iter==0) {
      for (int i=0; i<nefc; i++) {
        improvement += 0.5*force[i]*force[i]*d->efc_R[i];
      }
    }

    // perform one sweep: dry friction
    for (int i=ne; i<ne+nf; i++) {
      // compute residual, save old
      residual(m, d, res, i, 1, 1);
      oldforce[0] = force[i];

      // unconstrained minimum
      force[i] -= res[0]*ARinv[i];

      // impose interval constraints
      if (force[i]<-floss[i]) {
        force[i] = -floss[i];
      } else if (force[i]>floss[i]) {
        force[i] = floss[i];
      }

      // add to improvement
      delta[0] = force[i] - oldforce[0];
      improvement -= 0.5*delta[0]*delta[0]/ARinv[i] + delta[0]*res[0];
    }

    // perform one sweep: contact friction
    for (int i=ne+nf; i<nefc; i++) {
      // pyramidal contact
      if (d->efc_type[i]==mjCNSTR_CONTACT_PYRAMIDAL) {
        // get contact info
        con = d->contact + d->efc_id[i];
        dim = con->dim;
        mu = con->friction;

        // loop over pairs of opposing pyramid edges
        for (int j=i; j<i+2*(dim-1); j+=2) {
          // compute residual, save old
          residual(m, d, res, j, 2, 1);
          mju_copy(oldforce, force+j, 2);

          // Ac = AR-submatirx
          extractBlock(m, d, Ac, j, 2, 1);

          // bc = b-subvector + Ac,rest * f_rest
          mju_copy(bc, res, 2);
          for (int k=0; k<2; k++) {
            bc[k] -= mju_dot(Ac+k*2, oldforce, 2);
          }

          // f0 = mid+y, f1 = mid-y
          mid = 0.5*(force[j]+force[j+1]);
          y = 0.5*(force[j]-force[j+1]);

          // K1 = A00 + A11 - 2*A01,  K0 = mid*A00 - mid*A11 + b0 - b1
          K1 = Ac[0] + Ac[3] - Ac[1] - Ac[2];
          K0 = mid*(Ac[0] - Ac[3]) + bc[0] - bc[1];

          // guard against Ac==0
          if (K1<mjMINVAL) {
            force[j] = force[j+1] = mid;
          }

          // otherwise minimize over y \in [-mid, mid]
          else {
            // unconstrained minimum
            y = -K0/K1;

            // clamp and assign
            if (y<-mid) {
              force[j] = 0;
              force[j+1] = 2*mid;
            } else if (y>mid) {
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
      else if (d->efc_type[i]==mjCNSTR_CONTACT_ELLIPTIC) {
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
        for (int j=0; j<dim-1; j++) {
          bc[j] -= mju_dot(Ac+j*(dim-1), oldforce, dim-1);
        }

        // guard for f_normal==0
        if (force[i]<mjMINVAL) {
          mju_zero(force+i+1, dim-1);
        }

        // QCQP
        else {
          int flg_active = 0;

          // solve
          if (dim==3) {
            flg_active = mju_QCQP2(v, Ac, bc, mu, force[i]);
          } else if (dim==4) {
            flg_active = mju_QCQP3(v, Ac, bc, mu, force[i]);
          } else {
            flg_active = mju_QCQP(v, Ac, bc, mu, force[i], dim-1);
          }

          // on constraint: put v on ellipsoid, in case QCQP is approximate
          if (flg_active) {
            mjtNum s = 0;
            for (int j=0; j<dim-1; j++) {
              s += v[j]*v[j]/(mu[j]*mu[j]);
            }
            s = mju_sqrt(force[i]*force[i] / mju_max(mjMINVAL, s));
            for (int j=0; j<dim-1; j++) {
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
    memcpy(oldstate, d->efc_state, nefc*sizeof(int));
    int nactive = dualState(m, d);
    int nchange = 0;
    for (int i=0; i<nefc; i++) {
      nchange += (oldstate[i]!=d->efc_state[i]);
    }

    // scale improvement, save stats, count
    improvement = rescale(m, improvement);
    saveStats(m, d, &iter, improvement, 0, 0, nactive, nchange, 0, 0);

    // terminate
    if (improvement<m->opt.noslip_tolerance) {
      break;
    }
  }

  // update solver iterations
  d->solver_iter += iter;

  // map to joint space
  dualFinish(m, d);

  mjFREESTACK;
}



//------------------------- CG and Newton solver  --------------------------------------------------

// CG context
struct _mjCGContext {
  // arrays
  mjtNum* Jaref;          // Jac*qacc - aref                              (nefc x 1)
  mjtNum* Jv;             // Jac*search                                   (nefc x 1)
  mjtNum* Ma;             // M*qacc                                       (nv x 1)
  mjtNum* Mv;             // M*search                                     (nv x 1)
  mjtNum* grad;           // gradient of master cost                      (nv x 1)
  mjtNum* Mgrad;          // M\grad or H\grad                             (nv x 1)
  mjtNum* search;         // linesearch vector                            (nv x 1)
  mjtNum* quad;           // quadratic polynomials for constraint costs   (nefc x 3)

  // Hessian (Newton only)
  int     flg_Newton;     // 1: Newton, 0: CG (const)
  int     nnz;            // total number of non-zeros
  mjtNum* H;              // Cholesky factorization of Hessian            (nv x nv)
  mjtNum* Hcone;          // with cone contributions if present           (nv x nv)
  int*    rownnz;         // non-zeros in row                             (nv X 1)
  int*    rowadr;         // row address                                  (nv x 1)
  int*    colind;         // column indices                               (nv x nv)

  // globals
  mjtNum  cost;           // constraint + Gauss cost
  mjtNum  quadGauss[3];   // quadratic polynomial for Gauss cost
  int     nactive;        // number of active constraints
  int     ncone;          // number of contacts in cone state
  int     nupdate;        // number of Cholesky updates

  // linesearch diagnostics
  int     LSiter;         // number of linesearch iterations
  int     LSresult;       // linesearch result
  mjtNum  LSslope;        // linesearch slope at solution
};
typedef struct _mjCGContext mjCGContext;



// allocate mjCGContext: mjMARK/FREE in caller function!
static void CGallocate(const mjModel* m, mjData* d,
                       mjCGContext* ctx, int flg_Newton) {
  int nv = m->nv, nefc = d->nefc;

  // clear everything
  memset(ctx, 0, sizeof(mjCGContext));

  // common arrays
  ctx->Jaref  = mj_stackAlloc(d, nefc);
  ctx->Jv     = mj_stackAlloc(d, nefc);
  ctx->Ma     = mj_stackAlloc(d, nv);
  ctx->Mv     = mj_stackAlloc(d, nv);
  ctx->grad   = mj_stackAlloc(d, nv);
  ctx->Mgrad  = mj_stackAlloc(d, nv);
  ctx->search = mj_stackAlloc(d, nv);
  ctx->quad   = mj_stackAlloc(d, nefc*3);

  // Hessian (Newton only)
  ctx->flg_Newton = flg_Newton;
  if (flg_Newton) {
    ctx->H      = mj_stackAlloc(d, nv*nv);
    ctx->Hcone  = mj_stackAlloc(d, nv*nv);
    ctx->rownnz = (int*)mj_stackAlloc(d, nv);
    ctx->rowadr = (int*)mj_stackAlloc(d, nv);
    ctx->colind = (int*)mj_stackAlloc(d, nv*nv);
  }
}



// update efc_force, qfrc_constraint, cost-related
static void CGupdateConstraint(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nefc = d->nefc, nv = m->nv;

  // update constraints
  mj_constraintUpdate(m, d, ctx->Jaref, &(ctx->cost), ctx->flg_Newton);

  // count active and cone
  ctx->nactive = 0;
  ctx->ncone = 0;
  for (int i=0; i<nefc; i++) {
    ctx->nactive += (d->efc_state[i]!=mjCNSTRSTATE_SATISFIED);
    ctx->ncone += (d->efc_state[i]==mjCNSTRSTATE_CONE);
  }

  // add Gauss cost, set in quadratic[0]
  mjtNum Gauss = 0;
  for (int i=0; i<nv; i++) {
    Gauss += 0.5*(ctx->Ma[i]-d->qfrc_smooth[i])*(d->qacc[i]-d->qacc_smooth[i]);
  }
  ctx->quadGauss[0] = Gauss;
  ctx->cost += Gauss;
}



// update grad, Mgrad
static void CGupdateGradient(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nv = m->nv;

  // grad = M*qacc - qfrc_smooth - qfrc_constraint
  for (int i=0; i<nv; i++) {
    ctx->grad[i] = ctx->Ma[i] - d->qfrc_smooth[i] - d->qfrc_constraint[i];
  }

  // Newton: Mgrad = H \ grad
  if (ctx->flg_Newton) {
    if (mj_isSparse(m)) {
      mju_cholSolveSparse(ctx->Mgrad, (ctx->ncone ? ctx->Hcone : ctx->H),
                          ctx->grad, nv, ctx->rownnz, ctx->rowadr, ctx->colind);
    } else {
      mju_cholSolve(ctx->Mgrad, (ctx->ncone ? ctx->Hcone : ctx->H), ctx->grad, nv);
    }
  }

  // CG: Mgrad = M \ grad
  else {
    mj_solveM(m, d, ctx->Mgrad, ctx->grad, 1);
  }
}



// prepare quadratic polynomials and contact cone quantities
static void CGprepare(const mjModel* m, const mjData* d, mjCGContext* ctx) {
  int nv = m->nv, nefc = d->nefc;
  const mjtNum* v = ctx->search;

  // Gauss: alpha^2*0.5*v'*M*v + alpha*v'*(Ma-qfrc_smooth) + 0.5*(a-qacc_smooth)'*(Ma-qfrc_smooth)
  //  quadGauss[0] already computed in CGupdateConstraint
  ctx->quadGauss[1] = mju_dot(v, ctx->Ma, nv) - mju_dot(v, d->qfrc_smooth, nv);
  ctx->quadGauss[2] = 0.5*mju_dot(v, ctx->Mv, nv);

  // process constraints
  for (int i=0; i<nefc; i++) {
    // pointers to numeric data
    mjtNum* Jv = ctx->Jv + i;
    mjtNum* Jaref = ctx->Jaref + i;
    mjtNum* D = d->efc_D + i;

    // pointer to this quadratic
    mjtNum* quad = ctx->quad + 3*i;

    // init with scalar quadratic
    mjtNum DJ0 = D[0]*Jaref[0];
    quad[0] = Jaref[0]*DJ0;
    quad[1] = Jv[0]*DJ0;
    quad[2] = Jv[0]*D[0]*Jv[0];

    // elliptic cone: extra processing
    if (d->efc_type[i]==mjCNSTR_CONTACT_ELLIPTIC) {
      // extract contact info
      mjContact* con = d->contact + d->efc_id[i];
      int dim = con->dim;
      mjtNum U[6], V[6], UU = 0, UV = 0, VV = 0, mu = con->mu;
      mjtNum* friction = con->friction;

      // complete vector quadratic (for bottom zone)
      for (int j=1; j<dim; j++) {
        mjtNum DJj = D[j]*Jaref[j];
        quad[0] += Jaref[j]*DJj;
        quad[1] += Jv[j]*DJj;
        quad[2] += Jv[j]*D[j]*Jv[j];
      }

      // rescale to make primal cone circular
      U[0] = Jaref[0]*mu;
      V[0] = Jv[0]*mu;
      for (int j=1; j<dim; j++) {
        U[j] = Jaref[j]*friction[j-1];
        V[j] = Jv[j]*friction[j-1];
      }

      // accumulate sums of squares
      for (int j=1; j<dim; j++) {
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
      quad[8] = D[0]/(mu*mu*(1+mu*mu));

      // advance to next constraint
      i += (dim-1);
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
static void CGeval(const mjModel* m, mjData* d, mjCGContext* ctx, mjCGPnt* p) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc;

  // clear result
  mjtNum cost = 0, alpha = p->alpha;
  mjtNum deriv[2] = {0, 0};

  // init quad with Gauss
  mjtNum quadTotal[3];
  mju_copy3(quadTotal, ctx->quadGauss);

  // equality
  for (int i=0; i<ne; i++) {
    mju_addTo3(quadTotal, ctx->quad+3*i);
  }

  // friction
  for (int i=ne; i<ne+nf; i++) {
    // search point, friction loss, bound (Rf)
    mjtNum start = ctx->Jaref[i], dir = ctx->Jv[i];
    mjtNum x = start + alpha*dir;
    mjtNum f = d->efc_frictionloss[i];
    mjtNum Rf = d->efc_R[i]*f;

    // -bound < x < bound : quadratic
    if (-Rf<x && x<Rf) {
      mju_addTo3(quadTotal, ctx->quad+3*i);
    }

    // x < -bound : linear negative
    else if (x<=-Rf) {
      mjtNum qf[3] = {f*(-0.5*Rf-start), -f*dir, 0};
      mju_addTo3(quadTotal, qf);
    }

    // bound < x : linear positive
    else {
      mjtNum qf[3] = {f*(-0.5*Rf+start), f*dir, 0};
      mju_addTo3(quadTotal, qf);
    }
  }

  // limit and contact
  for (int i=ne+nf; i<nefc; i++) {
    if (d->efc_type[i]==mjCNSTR_CONTACT_ELLIPTIC) {         // elliptic cone
      // extract contact info
      mjContact* con = d->contact + d->efc_id[i];
      mjtNum* quad = ctx->quad + 3*i;
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
      if (Tsqr<=0) {
        // bottom zone: quadratic cost
        if (N<0) {
          mju_addTo3(quadTotal, quad);
        }

        // top zone: nothing to do
      }

      // otherwise regular processing
      else {
        // tangential force
        mjtNum T = mju_sqrt(Tsqr);

        // N>=mu*T : top zone
        if (N>=mu*T) {
          // nothing to do
        }

        // mu*N+T<=0 : bottom zone
        else if (mu*N+T<=0) {
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
      i += (dim-1);
    } else {                                                  // inequality
      // search point
      mjtNum x = ctx->Jaref[i] + alpha*ctx->Jv[i];

      // active
      if (x<0) {
        mju_addTo3(quadTotal, ctx->quad+3*i);
      }
    }
  }

  // add total quadratic
  cost += alpha*alpha*quadTotal[2] + alpha*quadTotal[1] + quadTotal[0];
  deriv[0] += 2*alpha*quadTotal[2] + quadTotal[1];
  deriv[1] += 2*quadTotal[2];

  // check for convexity; SHOULD NOT OCCUR
  if (deriv[1]<=0) {
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
static int updateBracket(const mjModel* m, mjData* d, mjCGContext* ctx,
                         mjCGPnt* p, mjCGPnt candidates[3], mjCGPnt* pnext) {
  int flag = 0;
  for (int i=0; i<3; i++) {
    // negative deriv
    if (p->deriv[0]<0 && candidates[i].deriv[0]<0 && p->deriv[0]<candidates[i].deriv[0]) {
      *p = candidates[i];
      flag = 1;
    }

    // positive deriv
    else if (p->deriv[0]>0 && candidates[i].deriv[0]>0 && p->deriv[0]>candidates[i].deriv[0]) {
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
static mjtNum CGsearch(const mjModel* m, mjData* d, mjCGContext* ctx) {
  mjCGPnt p0, p1, p2, pmid, p1next, p2next;

  const int LSmaxiter = 50;
  const mjtNum LStolscl = 0.01;

  // clear results
  ctx->LSiter = 0;
  ctx->LSresult = 0;
  ctx->LSslope = 1;       // means not computed

  // save search vector length, check
  mjtNum snorm = mju_norm(ctx->search, m->nv);
  if (snorm<mjMINVAL) {
    ctx->LSresult = 1;                          // search vector too small
    return 0;
  }

  // compute scaled gradtol and slope scaling
  mjtNum gtol = m->opt.tolerance * LStolscl * snorm * m->stat.meaninertia * mjMAX(1, m->nv);
  mjtNum slopescl = 1 / (snorm * m->stat.meaninertia * mjMAX(1, m->nv));

  // compute Mv, Jv
  mj_mulM(m, d, ctx->Mv, ctx->search);
  mj_mulJacVec(m, d, ctx->Jv, ctx->search);

  // prepare quadratics and cones
  CGprepare(m, d, ctx);

  // init at alpha = 0, save
  p0.alpha = 0;
  CGeval(m, d, ctx, &p0);

  // always attempt one Newton step
  p1.alpha = p0.alpha - p0.deriv[0]/p0.deriv[1];
  CGeval(m, d, ctx, &p1);
  if (p0.cost<p1.cost) {
    p1 = p0;
  }

  // check for initial convergence
  if (mju_abs(p1.deriv[0])<gtol) {
    if (p1.alpha==0) {
      ctx->LSresult = 2;  // no improvement, initial convergence
    } else {
      ctx->LSresult = 0;  // SUCCESS
    }
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  }

  // save direction
  int dir = (p1.deriv[0]<0 ? +1 : -1);

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
  while (p1.deriv[0]*dir<=-gtol && ctx->LSiter<LSmaxiter) {
    // save current
    p2 = p1;
    p2update = 1;

    // move to Newton point w.r.t current
    p1.alpha -= p1.deriv[0]/p1.deriv[1];
    CGeval(m, d, ctx, &p1);

    // check for convergence
    if (mju_abs(p1.deriv[0])<gtol) {
      ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
      return p1.alpha;                          // SUCCESS
    }
  }

  // check for failure to bracket
  if (ctx->LSiter>=LSmaxiter) {
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
  while (ctx->LSiter<LSmaxiter) {
    // evaluate at midpoint
    pmid.alpha = 0.5*(p1.alpha + p2.alpha);
    CGeval(m, d, ctx, &pmid);

    // make list of candidates
    mjCGPnt candidates[3] = {p1next, p2next, pmid};

    // check candidates for convergence
    mjtNum bestcost = 0;
    int bestind = -1;
    for (int i=0; i<3; i++) {
      if (mju_abs(candidates[i].deriv[0])<gtol &&
          (bestind==-1 || candidates[i].cost<bestcost)) {
        bestcost = candidates[i].cost;
        bestind = i;
      }
    }
    if (bestind>=0) {
      ctx->LSslope = mju_abs(candidates[bestind].deriv[0])*slopescl;
      return candidates[bestind].alpha;       // SUCCESS
    }

    // update brackets
    int b1 = updateBracket(m, d, ctx, &p1, candidates, &p1next);
    int b2 = updateBracket(m, d, ctx, &p2, candidates, &p2next);

    // no update possible: numerical accuracy reached, use midpoint
    if (!b1 && !b2) {
      if (pmid.cost<p0.cost) {
        ctx->LSresult = 0;  // SUCCESS
      } else {
        ctx->LSresult = 7;  // no improvement, could not bracket
      }

      ctx->LSslope = mju_abs(pmid.deriv[0])*slopescl;
      return pmid.alpha;
    }
  }

  // choose bracket with best cost
  if (p1.cost<=p2.cost && p1.cost<p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p1.deriv[0])*slopescl;
    return p1.alpha;
  } else if (p2.cost<=p1.cost && p2.cost<p0.cost) {
    ctx->LSresult = 4;                          // improvement but no convergence
    ctx->LSslope = mju_abs(p2.deriv[0])*slopescl;
    return p2.alpha;
  } else {
    ctx->LSresult = 5;                          // no improvement
    return 0;
  }
}



// elliptic case: Hcone = H + cone_contributions
static void HessianCone(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nv = m->nv, nefc = d->nefc;
  mjtNum local[36];
  mjMARKSTACK;

  // storage for L'*J
  mjtNum* LTJ = mj_stackAlloc(d, 6*nv);
  mjtNum* LTJ_row = mj_stackAlloc(d, nv);
  int* LTJ_ind = (int*) mj_stackAlloc(d, nv);

  // start with Hcone = H
  mju_copy(ctx->Hcone, ctx->H, ctx->nnz);

  // add contributions
  for (int i=0; i<nefc; i++) {
    if (d->efc_state[i]==mjCNSTRSTATE_CONE) {
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
        for (int r=0; r<dim; r++) {
          for (int c=0; c<=r; c++) {
            mju_addToScl(LTJ+c*nnz, d->efc_J+d->efc_J_rowadr[i+r], local[r*dim+c], nnz);
          }
        }

        // update
        for (int r=0; r<dim; r++) {
          // copy data for this row
          mju_copy(LTJ_row, LTJ+r*nnz, nnz);
          memcpy(LTJ_ind, d->efc_J_colind+d->efc_J_rowadr[i+r], nnz*sizeof(int));

          // update
          mju_cholUpdateSparse(ctx->Hcone, LTJ_row, nv, 1,
                               ctx->rownnz, ctx->rowadr, ctx->colind, nnz, LTJ_ind,
                               d);
        }
      }

      // dense
      else {
        // compute LTJ = L'*J for this contact row
        mju_zero(LTJ, dim*nv);
        for (int r=0; r<dim; r++) {
          for (int c=0; c<=r; c++) {
            mju_addToScl(LTJ+c*nv, d->efc_J+(i+r)*nv, local[r*dim+c], nv);
          }
        }

        // update
        for (int r=0; r<dim; r++) {
          mju_cholUpdate(ctx->Hcone, LTJ+r*nv, nv, 1);
        }
      }

      // count updates
      ctx->nupdate += dim;

      // advance to next constraint
      i += (dim-1);
    }
  }

  mjFREESTACK;
}



// compute and factorize Hessian: direct method
static void HessianDirect(const mjModel* m, mjData* d, mjCGContext* ctx) {
  int nv = m->nv, nefc = d->nefc;
  mjMARKSTACK;

  // compute D corresponding to quad states
  mjtNum* D = mj_stackAlloc(d, nefc);
  for (int i=0; i<nefc; i++) {
    if (d->efc_state[i]==mjCNSTRSTATE_QUADRATIC) {
      D[i] = d->efc_D[i];
    } else {
      D[i] = 0;
    }
  }

  // sparse
  if (mj_isSparse(m)) {
    // compute H = J'*D*J, uncompressed layout
    mju_sqrMatTDSparse(ctx->H, d->efc_J, d->efc_JT, D, nefc, nv,
                       ctx->rownnz, ctx->rowadr, ctx->colind,
                       d->efc_J_rownnz, d->efc_J_rowadr,
                       d->efc_J_colind, d->efc_J_rowsuper,
                       d->efc_JT_rownnz, d->efc_JT_rowadr,
                       d->efc_JT_colind, d->efc_JT_rowsuper, d);

    // compute H = M + J'*D*J
    mj_addM(m, d, ctx->H, ctx->rownnz, ctx->rowadr, ctx->colind);

    // factorize H, uncompressed layout
    int rank = mju_cholFactorSparse(ctx->H, nv, mjMINVAL,
                                    ctx->rownnz, ctx->rowadr, ctx->colind,
                                    d);

    // rank-defficient, SHOULD NOT OCCUR
    if (rank!=nv) {
      mju_error("Rank-defficient Hessian in HessianDirect");
    }

    // compress layout of H
    mju_compressSparse(ctx->H, nv, nv, ctx->rownnz, ctx->rowadr, ctx->colind);

    // count nnz
    ctx->nnz = 0;
    for (int i=0; i<nv; i++) {
      ctx->nnz += ctx->rownnz[i];
    }
    if (ctx->nnz > nv*nv) {  // SHOULD NOT OCCUR
      mju_error("More nonzero values than elements in sparse direct-solver Hessian");
    }
  }

  // dense
  else {
    // compute H = M + J'*D*J
    mju_sqrMatTD(ctx->H, d->efc_J, D, nefc, nv);
    mj_addM(m, d, ctx->H, NULL, NULL, NULL);

    // factorize H
    mju_cholFactor(ctx->H, nv, mjMINVAL);

    // set nnz
    ctx->nnz = nv*nv;
  }

  mjFREESTACK;

  // add cones if present
  if (ctx->ncone) {
    HessianCone(m, d, ctx);
  }

  // mark full update
  ctx->nupdate = nefc;
}



// incremental update to Hessian
static void HessianIncremental(const mjModel* m, mjData* d,
                               mjCGContext* ctx, const int* oldstate) {
  int rank, nv = m->nv, nefc = d->nefc;
  mjMARKSTACK;

  // local space
  mjtNum* vec = mj_stackAlloc(d, nv);
  int* vec_ind = (int*) mj_stackAlloc(d, nv);

  // clear update counter
  ctx->nupdate = 0;

  // update H factorization
  for (int i=0; i<nefc; i++) {
    int flag_update = -1;

    // add quad
    if (oldstate[i]!=mjCNSTRSTATE_QUADRATIC && d->efc_state[i]==mjCNSTRSTATE_QUADRATIC) {
      flag_update = 1;
    }

    // subtract quad
    else if (oldstate[i]==mjCNSTRSTATE_QUADRATIC && d->efc_state[i]!=mjCNSTRSTATE_QUADRATIC) {
      flag_update = 0;
    }

    // perform update if flagged
    if (flag_update!=-1) {
      // update with vec = J(i,:)*sqrt(D[i]))
      if (mj_isSparse(m)) {
        // get nnz and adr of row i
        const int nnz = d->efc_J_rownnz[i], adr = d->efc_J_rowadr[i];

        // scale vec, copy colind
        mju_scl(vec, d->efc_J+adr, mju_sqrt(d->efc_D[i]), nnz);
        memcpy(vec_ind, d->efc_J_colind+adr, nnz*sizeof(int));

        // sparse update
        rank = mju_cholUpdateSparse(ctx->H, vec, nv, flag_update,
                                    ctx->rownnz, ctx->rowadr, ctx->colind, nnz, vec_ind,
                                    d);
      } else {
        mju_scl(vec, d->efc_J+i*nv, mju_sqrt(d->efc_D[i]), nv);
        rank = mju_cholUpdate(ctx->H, vec, nv, flag_update);
      }
      ctx->nupdate++;

      // recompute H directly if accuracy lost
      if (rank<nv) {
        mjFREESTACK;
        HessianDirect(m, d, ctx);

        // nothing else to do
        return;
      }
    }
  }

  // add cones if present
  if (ctx->ncone) {
    HessianCone(m, d, ctx);
  }

  mjFREESTACK;
}



// driver
static void mj_solCGNewton(const mjModel* m, mjData* d, int maxiter, int flg_Newton) {
  int iter = 0, nv = m->nv, nefc = d->nefc;
  mjtNum alpha, beta;
  mjtNum *gradold = NULL, *Mgradold = NULL, *Mgraddif = NULL;
  mjCGContext ctx;
  mjMARKSTACK;

  // allocate context
  CGallocate(m, d, &ctx, flg_Newton);

  // allocate local storage
  if (!flg_Newton) {
    gradold     = mj_stackAlloc(d, nv);
    Mgradold    = mj_stackAlloc(d, nv);
    Mgraddif    = mj_stackAlloc(d, nv);
  }
  int* oldstate       = (int*)mj_stackAlloc(d, nefc);

  // initialize matrix-vector products
  mj_mulM(m, d, ctx.Ma, d->qacc);
  mj_mulJacVec(m, d, ctx.Jaref, d->qacc);
  mju_subFrom(ctx.Jaref, d->efc_aref, nefc);

  // first update
  CGupdateConstraint(m, d, &ctx);
  if (flg_Newton) {
    HessianDirect(m, d, &ctx);
  }
  CGupdateGradient(m, d, &ctx);

  // start both with preconditioned gradient
  mju_scl(ctx.search, ctx.Mgrad, -1, nv);

  // main loop
  while (iter<maxiter) {
    // perform linesearch
    alpha = CGsearch(m, d, &ctx);

    // no improvement: done
    if (alpha==0) {
      break;
    }

    // move to new solution
    mju_addToScl(d->qacc, ctx.search, alpha, nv);
    mju_addToScl(ctx.Ma, ctx.Mv, alpha, nv);
    mju_addToScl(ctx.Jaref, ctx.Jv, alpha, nefc);

    // save old
    if (!flg_Newton) {
      mju_copy(gradold, ctx.grad, nv);
      mju_copy(Mgradold, ctx.Mgrad, nv);
    }
    memcpy(oldstate, d->efc_state, nefc*sizeof(int));
    mjtNum oldcost = ctx.cost;

    // update
    CGupdateConstraint(m, d, &ctx);
    if (flg_Newton) {
      HessianIncremental(m, d, &ctx, oldstate);
    }
    CGupdateGradient(m, d, &ctx);

    // count state changes
    int nchange = 0;
    for (int i=0; i<nefc; i++) {
      nchange += (d->efc_state[i]!=oldstate[i]);
    }

    // scale improvement, save stats, count
    mjtNum improvement = rescale(m, oldcost-ctx.cost);
    mjtNum gradient = rescale(m, mju_norm(ctx.grad, nv));
    saveStats(m, d, &iter, improvement, gradient, ctx.LSslope,
              ctx.nactive, nchange, ctx.LSiter, ctx.nupdate);

    // termination
    if (improvement<m->opt.tolerance || gradient<m->opt.tolerance) {
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
      if (beta<0) {
        beta = 0;
      }

      // update
      for (int i=0; i<nv; i++) {
        ctx.search[i] = -ctx.Mgrad[i] + beta*ctx.search[i];
      }
    }
  }

  // update solver iterations
  d->solver_iter += iter;

  // set solver_nnz
  if (flg_Newton) {
    if (mj_isSparse(m)) {
      d->solver_nnz = 2*ctx.nnz - nv;
    } else {
      d->solver_nnz = nv*nv;
    }
  } else {
    d->solver_nnz = 0;
  }

  mjFREESTACK;
}



// CG entry point
void mj_solCG(const mjModel* m, mjData* d, int maxiter) {
  mj_solCGNewton(m, d, maxiter, 0);
}



// Newton entry point
void mj_solNewton(const mjModel* m, mjData* d, int maxiter) {
  mj_solCGNewton(m, d, maxiter, 1);
}
