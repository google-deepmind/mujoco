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

#include "engine/engine_derivative.h"

#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_core_smooth.h"
#include "engine/engine_forward.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_inverse.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//------------------------- derivatives of spatial algebra -----------------------------------------



// derivatives of cross product, Da and Db are 3x3
static void mjd_cross(const mjtNum a[3], const mjtNum b[3],
                      mjtNum* restrict Da, mjtNum* restrict Db) {
  // derivative w.r.t a
  if (Da) {
    mju_zero(Da, 9);
    Da[1] =  b[2];
    Da[2] = -b[1];
    Da[3] = -b[2];
    Da[5] =  b[0];
    Da[6] =  b[1];
    Da[7] = -b[0];
  }

  // derivative w.r.t b
  if (Db) {
    mju_zero(Db, 9);
    Db[1] = -a[2];
    Db[2] =  a[1];
    Db[3] =  a[2];
    Db[5] = -a[0];
    Db[6] = -a[1];
    Db[7] =  a[0];
  }
}



// derivative of mju_crossMotion w.r.t velocity
static void mjd_crossMotion_vel(mjtNum D[36], const mjtNum v[6])
{
  mju_zero(D, 36);

  // res[0] = -vel[2]*v[1] + vel[1]*v[2]
  D[0 + 2] = -v[1];
  D[0 + 1] = v[2];

  // res[1] =  vel[2]*v[0] - vel[0]*v[2]
  D[6 + 2] = v[0];
  D[6 + 0] = -v[2];

  // res[2] = -vel[1]*v[0] + vel[0]*v[1]
  D[12 + 1] = -v[0];
  D[12 + 0] = v[1];

  // res[3] = -vel[2]*v[4] + vel[1]*v[5] - vel[5]*v[1] + vel[4]*v[2]
  D[18 + 2] = -v[4];
  D[18 + 1] = v[5];
  D[18 + 5] = -v[1];
  D[18 + 4] = v[2];

  // res[4] =  vel[2]*v[3] - vel[0]*v[5] + vel[5]*v[0] - vel[3]*v[2]
  D[24 + 2] = v[3];
  D[24 + 0] = -v[5];
  D[24 + 5] = v[0];
  D[24 + 3] = -v[2];

  // res[5] = -vel[1]*v[3] + vel[0]*v[4] - vel[4]*v[0] + vel[3]*v[1]
  D[30 + 1] = -v[3];
  D[30 + 0] = v[4];
  D[30 + 4] = -v[0];
  D[30 + 3] = v[1];
}



// derivative of mju_crossForce w.r.t. velocity
static void mjd_crossForce_vel(mjtNum D[36], const mjtNum f[6])
{
  mju_zero(D, 36);

  // res[0] = -vel[2]*f[1] + vel[1]*f[2] - vel[5]*f[4] + vel[4]*f[5]
  D[0 + 2] = -f[1];
  D[0 + 1] = f[2];
  D[0 + 5] = -f[4];
  D[0 + 4] = f[5];

  // res[1] =  vel[2]*f[0] - vel[0]*f[2] + vel[5]*f[3] - vel[3]*f[5]
  D[6 + 2] = f[0];
  D[6 + 0] = -f[2];
  D[6 + 5] = f[3];
  D[6 + 3] = -f[5];

  // res[2] = -vel[1]*f[0] + vel[0]*f[1] - vel[4]*f[3] + vel[3]*f[4]
  D[12 + 1] = -f[0];
  D[12 + 0] = f[1];
  D[12 + 4] = -f[3];
  D[12 + 3] = f[4];

  // res[3] = -vel[2]*f[4] + vel[1]*f[5]
  D[18 + 2] = -f[4];
  D[18 + 1] = f[5];

  // res[4] =  vel[2]*f[3] - vel[0]*f[5]
  D[24 + 2] = f[3];
  D[24 + 0] = -f[5];

  // res[5] = -vel[1]*f[3] + vel[0]*f[4]
  D[30 + 1] = -f[3];
  D[30 + 0] = f[4];
}



// derivative of mju_crossForce w.r.t. force
static void mjd_crossForce_frc(mjtNum D[36], const mjtNum vel[6])
{
  mju_zero(D, 36);

  // res[0] = -vel[2]*f[1] + vel[1]*f[2] - vel[5]*f[4] + vel[4]*f[5]
  D[0 + 1] = -vel[2];
  D[0 + 2] = vel[1];
  D[0 + 4] = -vel[5];
  D[0 + 5] = vel[4];

  // res[1] =  vel[2]*f[0] - vel[0]*f[2] + vel[5]*f[3] - vel[3]*f[5]
  D[6 + 0] = vel[2];
  D[6 + 2] = -vel[0];
  D[6 + 3] = vel[5];
  D[6 + 5] = -vel[3];

  // res[2] = -vel[1]*f[0] + vel[0]*f[1] - vel[4]*f[3] + vel[3]*f[4]
  D[12 + 0] = -vel[1];
  D[12 + 1] = vel[0];
  D[12 + 3] = -vel[4];
  D[12 + 4] = vel[3];

  // res[3] = -vel[2]*f[4] + vel[1]*f[5]
  D[18 + 4] = -vel[2];
  D[18 + 5] = vel[1];

  // res[4] =  vel[2]*f[3] - vel[0]*f[5]
  D[24 + 3] = vel[2];
  D[24 + 5] = -vel[0];

  // res[5] = -vel[1]*f[3] + vel[0]*f[4]
  D[30 + 3] = -vel[1];
  D[30 + 4] = vel[0];
}



// derivative of mju_mulInertVec w.r.t vel
static void mjd_mulInertVec_vel(mjtNum D[36], const mjtNum i[10])
{
  mju_zero(D, 36);

  // res[0] = i[0]*v[0] + i[3]*v[1] + i[4]*v[2] - i[8]*v[4] + i[7]*v[5]
  D[0 + 0] = i[0];
  D[0 + 1] = i[3];
  D[0 + 2] = i[4];
  D[0 + 4] = -i[8];
  D[0 + 5] = i[7];

  // res[1] = i[3]*v[0] + i[1]*v[1] + i[5]*v[2] + i[8]*v[3] - i[6]*v[5]
  D[6 + 0] = i[3];
  D[6 + 1] = i[1];
  D[6 + 2] = i[5];
  D[6 + 3] = i[8];
  D[6 + 5] = -i[6];

  // res[2] = i[4]*v[0] + i[5]*v[1] + i[2]*v[2] - i[7]*v[3] + i[6]*v[4]
  D[12 + 0] = i[4];
  D[12 + 1] = i[5];
  D[12 + 2] = i[2];
  D[12 + 3] = -i[7];
  D[12 + 4] = i[6];

  // res[3] = i[8]*v[1] - i[7]*v[2] + i[9]*v[3]
  D[18 + 1] = i[8];
  D[18 + 2] = -i[7];
  D[18 + 3] = i[9];

  // res[4] = i[6]*v[2] - i[8]*v[0] + i[9]*v[4]
  D[24 + 2] = i[6];
  D[24 + 0] = -i[8];
  D[24 + 4] = i[9];

  // res[5] = i[7]*v[0] - i[6]*v[1] + i[9]*v[5]
  D[30 + 0] = i[7];
  D[30 + 1] = -i[6];
  D[30 + 5] = i[9];
}



//--------------------------- utility functions for mjd_stepFD -------------------------------------

// get state=[qpos; qvel; act] and optionally sensordata
static void getState(const mjModel* m, const mjData* d, mjtNum* state, mjtNum* sensordata) {
  int nq = m->nq, nv = m->nv, na = m->na;

  mju_copy(state,       d->qpos, nq);
  mju_copy(state+nq,    d->qvel, nv);
  mju_copy(state+nq+nv, d->act,  na);
  if (sensordata) {
    mju_copy(sensordata, d->sensordata, m->nsensordata);
  }
}



// set state=[qpos; qvel; act] and optionally warmstart accelerations
static void setState(const mjModel* m, mjData* d, const mjtNum* state, const mjtNum* ctrl,
                     const mjtNum* warmstart) {
  int nq = m->nq, nv = m->nv, na = m->na;

  mju_copy(d->qpos, state,       nq);
  mju_copy(d->qvel, state+nq,    nv);
  mju_copy(d->act,  state+nq+nv, na);
  if (ctrl) {
    mju_copy(d->ctrl, ctrl, m->nu);
  }
  if (warmstart) {
    mju_copy(d->qacc_warmstart, warmstart, nv);
  }
}



// dx = (x2 - x1) / h
static void diff(mjtNum* restrict dx, const mjtNum* x1, const mjtNum* x2, mjtNum h, int n) {
  mjtNum inv_h = 1/h;
  for (int i=0; i<n; i++) {
    dx[i] = inv_h * (x2[i] - x1[i]);
  }
}



// finite-difference two state vectors ds = (s2 - s1) / h
static void stateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s1, const mjtNum* s2, mjtNum h) {
  int nq = m->nq, nv = m->nv, na = m->na;

  if (nq == nv) {
    diff(ds, s1, s2, h, nq+nv+na);
  } else {
    mj_differentiatePos(m, ds, h, s1, s2);
    diff(ds+nv, s1+nq, s2+nq, h, nv+na);
  }
}



// finite-difference two vectors, forward, backward or centered
static void clampedDiff(mjtNum* dx, const mjtNum* x, const mjtNum* x_plus, const mjtNum* x_minus,
                        mjtNum h, int nx) {
  if (x_plus && !x_minus) {
    // forward differencing
    diff(dx, x, x_plus, h, nx);
  } else if (!x_plus && x_minus) {
    // backward differencing
    diff(dx, x_minus, x, h, nx);
  } else if (x_plus && x_minus) {
    // centered differencing
    diff(dx, x_plus, x_minus, 2*h, nx);
  } else {
    // differencing failed, write zeros
    mju_zero(dx, nx);
  }
}



// finite-difference two state vectors, forward, backward or centered
static void clampedStateDiff(const mjModel* m, mjtNum* ds, const mjtNum* s, const mjtNum* s_plus,
                             const mjtNum* s_minus, mjtNum h) {
  if (s_plus && !s_minus) {
    // forward differencing
    stateDiff(m, ds, s, s_plus, h);
  } else if (!s_plus && s_minus) {
    // backward differencing
    stateDiff(m, ds, s_minus, s, h);
  } else if (s_plus && s_minus) {
    // centered differencing
    stateDiff(m, ds, s_minus, s_plus, 2*h);
  } else {
    // differencing failed, write zeros
    mju_zero(ds, m->nq + m->nv + m->na);
  }
}



// check if two numbers are inside a given range
static int inRange(const mjtNum x1, const mjtNum x2, const mjtNum* range) {
  return x1 >= range[0] && x1 <= range[1] &&
         x2 >= range[0] && x2 <= range[1];
}



// advance simulation using control callback, skipstage is mjtStage
void mj_stepSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor) {
  TM_START;

  // common to all integrators
  mj_checkPos(m, d);
  mj_checkVel(m, d);
  mj_forwardSkip(m, d, skipstage, skipsensor);
  mj_checkAcc(m, d);

  // compare forward and inverse solutions if enabled
  if (mjENABLED(mjENBL_FWDINV)) {
    mj_compareFwdInv(m, d);
  }

  // use selected integrator
  switch (m->opt.integrator) {
    case mjINT_EULER:
      mj_EulerSkip(m, d, skipstage >= mjSTAGE_POS);
      break;

    case mjINT_RK4:
      // ignore skipstage
      mj_RungeKutta(m, d, 4);
      break;

    case mjINT_IMPLICIT:
      mj_implicitSkip(m, d, skipstage >= mjSTAGE_VEL);
      break;

    default:
      mju_error("Invalid integrator");
  }

  TM_END(mjTIMER_STEP);
}



//------------------------- derivatives of component functions -------------------------------------

// derivative of cvel, cdof_dot w.r.t qvel
static void mjd_comVel_vel(const mjModel* m, mjData* d, mjtNum* Dcvel, mjtNum* Dcdofdot)
{
  int nv = m->nv, nbody = m->nbody;
  mjtNum mat[36];

  // clear Dcvel
  mju_zero(Dcvel, nbody*6*nv);

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int i=1; i<m->nbody; i++) {
    // Dcvel = Dcvel_parent
    mju_copy(Dcvel+i*6*nv, Dcvel+m->body_parentid[i]*6*nv, 6*nv);

    // Dcvel += D(cdof * qvel),  Dcdofdot = D(cvel x cdof)
    for (int j=m->body_dofadr[i]; j<m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      switch (m->jnt_type[m->dof_jntid[j]])
      {
      case mjJNT_FREE:
        // Dcdofdot = 0
        mju_zero(Dcdofdot+j*6*nv, 18*nv);

        // Dcvel += cdof * (D qvel)
        for (int k=0; k<6; k++) {
          Dcvel[i*6*nv + k*nv + j+0] += d->cdof[(j+0)*6 + k];
          Dcvel[i*6*nv + k*nv + j+1] += d->cdof[(j+1)*6 + k];
          Dcvel[i*6*nv + k*nv + j+2] += d->cdof[(j+2)*6 + k];
        }

        // continue with rotations
        j += 3;

      case mjJNT_BALL:
        // Dcdofdot = D crossMotion(cvel, cdof)
        for (int k=0; k<3; k++) {
          mjd_crossMotion_vel(mat, d->cdof+6*(j+k));
          mju_mulMatMat(Dcdofdot+(j+k)*6*nv, mat, Dcvel+i*6*nv, 6, 6, nv);
        }

        // Dcvel += cdof * (D qvel)
        for (int k=0; k<6; k++) {
          Dcvel[i*6*nv + k*nv + j+0] += d->cdof[(j+0)*6 + k];
          Dcvel[i*6*nv + k*nv + j+1] += d->cdof[(j+1)*6 + k];
          Dcvel[i*6*nv + k*nv + j+2] += d->cdof[(j+2)*6 + k];
        }

        // adjust for 3-dof joint
        j += 2;
        break;

      default:
        // Dcdofdot = D crossMotion(cvel, cdof) * Dcvel
        mjd_crossMotion_vel(mat, d->cdof+6*j);
        mju_mulMatMat(Dcdofdot+j*6*nv, mat, Dcvel+i*6*nv, 6, 6, nv);

        // Dcvel += cdof * (D qvel)
        for (int k=0; k<6; k++) {
          Dcvel[i*6*nv + k*nv + j] += d->cdof[j*6 + k];
        }
      }
    }
  }
}



// subtract (d qfrc_bias / d qvel) from DfDv
static void mjd_rne_vel(const mjModel* m, mjData* d, mjtNum* DfDv) {
  int nv = m->nv, nbody = m->nbody;
  mjtNum mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  mjMARKSTACK;
  mjtNum* Dcvel = mj_stackAlloc(d, nbody*6*nv);
  mjtNum* Dcdofdot = mj_stackAlloc(d, nv*6*nv);
  mjtNum* Dcacc = mj_stackAlloc(d, nbody*6*nv);
  mjtNum* Dcfrcbody = mj_stackAlloc(d, nbody*6*nv);

  // compute Dcdofdot and Dcvel
  mjd_comVel_vel(m, d, Dcvel, Dcdofdot);

  // clear Dcacc
  mju_zero(Dcacc, nbody*6*nv);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int i=1; i<nbody; i++) {
    // Dcacc = Dcacc_parent
    mju_copy(Dcacc + i*6*nv, Dcacc + m->body_parentid[i]*6*nv, 6*nv);

    // Dcacc += D(cdofdot * qvel)
    for (int j=m->body_dofadr[i]; j<m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      // Dcacc += cdofdot * (D qvel)
      for (int k=0; k<6; k++) {
        Dcacc[i*6*nv + k*nv + j] += d->cdof_dot[j*6 + k];
      }

      // Dcacc += (D cdofdot) * qvel
      mju_addToScl(Dcacc+i*6*nv, Dcdofdot+j*6*nv, d->qvel[j], 6*nv);
    }

    //---------- Dcfrcbody = D(cinert * cacc + cvel x (cinert * cvel))

    // Dcfrcbody = (D mul / D cacc) * Dcacc
    mjd_mulInertVec_vel(dmul, d->cinert+10*i);
    mju_mulMatMat(Dcfrcbody+i*6*nv, dmul, Dcacc+i*6*nv, 6, 6, nv);

    // mat = (D cross / D cvel) + (D cross / D mul) * (D mul / D cvel)
    mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+i*6);
    mjd_crossForce_vel(mat, tmp);
    mjd_crossForce_frc(mat1, d->cvel+i*6);
    mju_mulMatMat(mat2, mat1, dmul, 6, 6, 6);
    mju_addTo(mat, mat2, 36);

    // Dcfrcbody += mat * Dcvel  (use body 0 as temp)
    mju_mulMatMat(Dcfrcbody, mat, Dcvel+i*6*nv, 6, 6, nv);
    mju_addTo(Dcfrcbody+i*6*nv, Dcfrcbody, 6*nv);
  }

  // clear world Dcfrcbody, for style
  mju_zero(Dcfrcbody, 6*nv);

  // backward pass over bodies: accumulate Dcfrcbody
  for (int i=m->nbody-1; i>0; i--) {
    if (m->body_parentid[i]) {
      mju_addTo(Dcfrcbody+m->body_parentid[i]*6*nv, Dcfrcbody+i*6*nv, 6*nv);
    }
  }

  // DfDv -= D(cdof * cfrc_body)
  for (int i=0; i<nv; i++) {
    for (int k=0; k<6; k++) {
      mju_addToScl(DfDv+i*nv, Dcfrcbody+(m->dof_bodyid[i]*6+k)*nv, -d->cdof[i*6+k], nv);
    }
  }

  mjFREESTACK;
}



//--------------------- utility functions for (d force / d vel) Jacobians --------------------------

// construct sparse Jacobian structure of body; return nnz
static int bodyJacSparse(const mjModel* m, int body, int* ind) {
  // skip fixed bodies
  while (body>0 && m->body_dofnum[body]==0) {
    body = m->body_parentid[body];
  }

  // body is not movable: empty chain
  if (body==0) {
    return 0;
  }

  // count dofs
  int nnz = 0;
  int dof = m->body_dofadr[body] + m->body_dofnum[body] - 1;
  while (dof>=0) {
    nnz++;
    dof = m->dof_parentid[dof];
  }

  // fill array in reverse (increasing dof)
  int cnt = 0;
  dof = m->body_dofadr[body] + m->body_dofnum[body] - 1;
  while (dof>=0) {
    ind[nnz-cnt-1] = dof;
    cnt++;
    dof = m->dof_parentid[dof];
  }

  return nnz;
}



// add J'*B*J to DfDv
static void addJTBJ(mjtNum* DfDv, const mjtNum* J, const mjtNum* B, int n, int nv) {
  // process non-zero elements of B
  for (int i=0; i<n; i++) {
    for (int j=0; j<n; j++) {
      if (B[i*n+j]) {
        // process non-zero elements of J(i,:)
        for (int k=0; k<nv; k++) {
          if (J[i*nv+k]) {
            // add J(i,k)*B(i,j)*J(j,:) to DfDv(k,:)
            mju_addToScl(DfDv+k*nv, J+j*nv, J[i*nv+k]*B[i*n+j], nv);
          }
        }
      }
    }
  }
}



// add J'*B*J to DfDv, sparse version
static void addJTBJSparse(mjtNum* DfDv, const mjtNum* J, const mjtNum* B,
                          int n, int nv, int offset,
                          const int* rownnz, const int* rowadr, const int* colind) {
  // process non-zero elements of B
  for (int i=0; i<n; i++) {
    for (int j=0; j<n; j++) {
      if (B[i*n+j]) {
        // process non-zero elements of J(i,k)
        for (int k=0; k<rownnz[offset+i]; k++) {
          int ik = rowadr[offset+i] + k;
          int col_ik = colind[ik]*nv;
          mjtNum scl = J[ik]*B[i*n+j];

          // process non-zero elements of J(j,p)
          for (int p=0; p<rownnz[offset+j]; p++) {
            int jp = rowadr[offset+j] + p;

            // add J(i,k)*B(i,j)*J(j,p) to DfDv(k,p)
            DfDv[col_ik + colind[jp]] += scl * J[jp];
          }
        }
      }
    }
  }
}



//----------------------------- derivatives of actuator forces -------------------------------------

// derivative of mju_muscleGain w.r.t velocity
static mjtNum mjd_muscleGain_vel(mjtNum len, mjtNum vel, const mjtNum lengthrange[2], mjtNum acc0,
                                 const mjtNum prm[9]) {
  // unpack parameters
  mjtNum range[2] = {prm[0], prm[1]};
  mjtNum force    = prm[2];
  mjtNum scale    = prm[3];
  mjtNum lmin     = prm[4];
  mjtNum lmax     = prm[5];
  mjtNum vmax     = prm[6];
  mjtNum fvmax    = prm[8];

  // scale force if negative
  if (force<0) {
    force = scale / mjMAX(mjMINVAL, acc0);
  }

  // mid-ranges
  mjtNum a = 0.5*(lmin+1);
  mjtNum b = 0.5*(1+lmax);
  mjtNum x;

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mjMAX(mjMINVAL, range[1]-range[0]);

  // normalized length and velocity
  mjtNum L = range[0] + (len-lengthrange[0]) / mjMAX(mjMINVAL, L0);
  mjtNum V = vel / mjMAX(mjMINVAL, L0*vmax);

  // length curve
  mjtNum FL = 0;
  if (L>=lmin && L<=a) {
    x = (L-lmin) / mjMAX(mjMINVAL, a-lmin);
    FL = 0.5*x*x;
  } else if (L<=1) {
    x = (1-L) / mjMAX(mjMINVAL, 1-a);
    FL = 1 - 0.5*x*x;
  } else if (L<=b) {
    x = (L-1) / mjMAX(mjMINVAL, b-1);
    FL = 1 - 0.5*x*x;
  } else if (L<=lmax) {
    x = (lmax-L) / mjMAX(mjMINVAL, lmax-b);
    FL = 0.5*x*x;
  }

  // velocity curve
  mjtNum dFV;
  mjtNum y = fvmax-1;
  if (V<=-1) {
    // FV = 0
    dFV = 0;
  } else if (V<=0) {
    // FV = (V+1)*(V+1)
    dFV = 2*V + 2;
  } else if (V<=y) {
    // FV = fvmax - (y-V)*(y-V) / mjMAX(mjMINVAL, y)
    dFV = (-2*V + 2*y) / mjMAX(mjMINVAL, y);
  } else {
    // FV = fvmax
    dFV = 0;
  }

  // compute FVL and scale, make it negative
  return -force*FL*dFV/mjMAX(mjMINVAL, L0*vmax);
}



// add (d qfrc_actuator / d qvel) to DfDv
static void mjd_actuator_vel(const mjModel* m, mjData* d, mjtNum* DfDv) {
  int nv = m->nv;

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_ACTUATION)) {
    return;
  }

  // process actuators
  for (int i=0; i<m->nu; i++) {
    mjtNum bias_vel = 0, gain_vel = 0;

    // affine bias
    if (m->actuator_biastype[i]==mjBIAS_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      bias_vel = (m->actuator_biasprm + mjNBIAS*i)[2];
    }

    // affine gain
    if (m->actuator_gaintype[i]==mjGAIN_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      gain_vel = (m->actuator_gainprm + mjNGAIN*i)[2];
    }

    // muscle gain
    else if (m->actuator_gaintype[i]==mjGAIN_MUSCLE) {
      gain_vel = mjd_muscleGain_vel(d->actuator_length[i],
                                    d->actuator_velocity[i],
                                    m->actuator_lengthrange+2*i,
                                    m->actuator_acc0[i],
                                    m->actuator_gainprm + mjNGAIN*i);
    }

    // force = gain .* [ctrl/act]
    if (gain_vel!=0) {
      if (m->actuator_dyntype[i]==mjDYN_NONE) {
        bias_vel += gain_vel * d->ctrl[i];
      } else {
        bias_vel += gain_vel * d->act[i-(m->nu - m->na)];
      }
    }

    // add
    if (bias_vel!=0) {
      addJTBJ(DfDv, d->actuator_moment+i*nv, &bias_vel, 1, nv);
    }
  }
}



//----------------- utilities for ellipsoid-based fluid force derivatives --------------------------

static inline mjtNum pow2(const mjtNum val) {
  return val*val;
}



static inline mjtNum ellipsoid_max_moment(const mjtNum size[3], const int dir) {
  const mjtNum d0 = size[dir];
  const mjtNum d1 = size[(dir+1) % 3];
  const mjtNum d2 = size[(dir+2) % 3];
  return 8.0/15.0 * mjPI * d0 * pow2(pow2(mju_max(d1, d2)));
}



// add 3x3 matrix D to one of the four quadrants of the 6x6 matrix B
//   row_quad and col_quad should be either 0 or 1 (not checked)
static void addToQuadrant(mjtNum* restrict B, const mjtNum D[9], int col_quad, int row_quad) {
  int r = 3*row_quad, c = 3*col_quad;
  B[6*(c+0) + r+0] += D[0];
  B[6*(c+0) + r+1] += D[1];
  B[6*(c+0) + r+2] += D[2];
  B[6*(c+1) + r+0] += D[3];
  B[6*(c+1) + r+1] += D[4];
  B[6*(c+1) + r+2] += D[5];
  B[6*(c+2) + r+0] += D[6];
  B[6*(c+2) + r+1] += D[7];
  B[6*(c+2) + r+2] += D[8];
}



//----------------- components of ellipsoid-based fluid force derivatives --------------------------

// forces due to fluid mass moving with the body, B is 6x6
static void mjd_addedMassForces(
    mjtNum* restrict B, const mjtNum local_vels[6], const mjtNum fluid_density,
    const mjtNum virtual_mass[3], const mjtNum virtual_inertia[3]) {
  const mjtNum lin_vel[3] = {local_vels[3], local_vels[4], local_vels[5]};
  const mjtNum ang_vel[3] = {local_vels[0], local_vels[1], local_vels[2]};
  const mjtNum virtual_lin_mom[3] = {
    fluid_density * virtual_mass[0] * lin_vel[0],
    fluid_density * virtual_mass[1] * lin_vel[1],
    fluid_density * virtual_mass[2] * lin_vel[2]
  };
  const mjtNum virtual_ang_mom[3] = {
    fluid_density * virtual_inertia[0] * ang_vel[0],
    fluid_density * virtual_inertia[1] * ang_vel[1],
    fluid_density * virtual_inertia[2] * ang_vel[2]
  };
  mjtNum Da[9];
  mjtNum Db[9];

  // force[:3] += cross(virtual_ang_mom, ang_vel)
  mjd_cross(virtual_ang_mom, ang_vel, Da, Db);
  addToQuadrant(B, Db, 0, 0);
  for (int i=0; i<9; ++i) {
    Da[i] *= fluid_density * virtual_inertia[i % 3];
  }
  addToQuadrant(B, Da, 0, 0);

  // force[:3] += cross(virtual_lin_mom, lin_vel)
  mjd_cross(virtual_lin_mom, lin_vel, Da, Db);
  addToQuadrant(B, Db, 0, 1);
  for (int i=0; i<9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 0, 1);

  // force[3:] += cross(virtual_lin_mom, ang_vel)
  mjd_cross(virtual_lin_mom, ang_vel, Da, Db);
  addToQuadrant(B, Db, 1, 0);
  for (int i=0; i<9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 1, 1);
}



// torque due to motion in the fluid, D is 3x3
static inline void mjd_viscous_torque(
    mjtNum* restrict D, const mjtNum lvel[6], const mjtNum fluid_density,
    const mjtNum fluid_viscosity, const mjtNum size[3],
    const mjtNum slender_drag_coef, const mjtNum ang_drag_coef)
{
  const mjtNum d_max = mju_max(mju_max(size[0], size[1]), size[2]);
  const mjtNum d_min = mju_min(mju_min(size[0], size[1]), size[2]);
  const mjtNum d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const mjtNum eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const mjtNum lin_visc_torq_coef = mjPI * eq_sphere_D*eq_sphere_D*eq_sphere_D;

  // moments of inertia used to compute angular quadratic drag
  const mjtNum I_max = 8.0/15.0 * mjPI * d_mid * (d_max*d_max)*(d_max*d_max);
  const mjtNum II[3] = {
    ellipsoid_max_moment(size, 0),
    ellipsoid_max_moment(size, 1),
    ellipsoid_max_moment(size, 2)
  };
  const mjtNum x = lvel[0], y = lvel[1], z = lvel[2];
  const mjtNum mom_coef[3] = {
    ang_drag_coef*II[0] + slender_drag_coef*(I_max - II[0]),
    ang_drag_coef*II[1] + slender_drag_coef*(I_max - II[1]),
    ang_drag_coef*II[2] + slender_drag_coef*(I_max - II[2])
  };
  const mjtNum mom_visc[3] = {
    x * mom_coef[0],
    y * mom_coef[1],
    z * mom_coef[2]
  };
  const mjtNum density = fluid_density / mju_max(mjMINVAL, mju_norm3(mom_visc));

  // -density * [x, y, z] * mom_coef^2
  const mjtNum mom_sq[3] = {
    -density * x * mom_coef[0] * mom_coef[0],
    -density * y * mom_coef[1] * mom_coef[1],
    -density * z * mom_coef[2] * mom_coef[2]
  };
  const mjtNum lin_coef = fluid_viscosity * lin_visc_torq_coef;

  // initialize
  mju_zero(D, 9);

  // set diagonal
  D[0] = D[4] = D[8] = x*mom_sq[0] + y*mom_sq[1] + z*mom_sq[2] - lin_coef;

  // add outer product
  mju_addToScl3(D, mom_sq, x);
  mju_addToScl3(D+3, mom_sq, y);
  mju_addToScl3(D+6, mom_sq, z);
}



// drag due to motion in the fluid, D is 3x3
static inline void mjd_viscous_drag(
    mjtNum* restrict D, const mjtNum lvel[6], const mjtNum fluid_density,
    const mjtNum fluid_viscosity, const mjtNum size[3],
    const mjtNum blunt_drag_coef, const mjtNum slender_drag_coef) {
  const mjtNum d_max = mju_max(mju_max(size[0], size[1]), size[2]);
  const mjtNum d_min = mju_min(mju_min(size[0], size[1]), size[2]);
  const mjtNum d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const mjtNum eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const mjtNum A_max = mjPI * d_max * d_mid;

  const mjtNum a = pow2(size[1] * size[2]);
  const mjtNum b = pow2(size[2] * size[0]);
  const mjtNum c = pow2(size[0] * size[1]);
  const mjtNum aa = a*a, bb = b*b, cc = c*c;

  const mjtNum x = lvel[3], y = lvel[4], z = lvel[5];
  const mjtNum xx = x*x, yy = y*y, zz = z*z, xy=x*y, yz=y*z, xz=x*z;

  const mjtNum proj_denom = aa*xx + bb*yy + cc*zz;
  const mjtNum proj_num = a*xx + b*yy + c*zz;
  const mjtNum dA_coef = mjPI / mju_max(mjMINVAL,
                                        mju_sqrt(proj_num*proj_num*proj_num * proj_denom));

  const mjtNum A_proj = mjPI * mju_sqrt(proj_denom/mju_max(mjMINVAL, proj_num));

  const mjtNum norm = mju_sqrt(xx + yy + zz);
  const mjtNum inv_norm = 1.0 / mju_max(mjMINVAL, norm);

  const mjtNum lin_coef = fluid_viscosity * 3.0 * mjPI * eq_sphere_D;
  const mjtNum quad_coef = fluid_density * (
      A_proj*blunt_drag_coef + slender_drag_coef*(A_max - A_proj));
  const mjtNum Aproj_coef = fluid_density * norm * (blunt_drag_coef - slender_drag_coef);

  const mjtNum dAproj_dv[3] = {
    Aproj_coef * dA_coef * a * x * (b * yy * (a - b) + c * zz * (a - c)),
    Aproj_coef * dA_coef * b * y * (a * xx * (b - a) + c * zz * (b - c)),
    Aproj_coef * dA_coef * c * z * (a * xx * (c - a) + b * yy * (c - b))
  };

  // outer product
  D[0] = xx;  D[1] = xy;  D[2] = xz;
  D[3] = xy;  D[4] = yy;  D[5] = yz;
  D[6] = xz;  D[7] = yz;  D[8] = zz;

  // diag(D) += dot([x y z], [x y z])
  mjtNum inner = xx + yy + zz;
  D[0] += inner;
  D[4] += inner;
  D[8] += inner;

  // scale by -quad_coef*inv_norm
  mju_scl(D, D, -quad_coef*inv_norm, 9);

  // D += outer_product(-[x y z], dAproj_dv)
  mju_addToScl3(D+0, dAproj_dv, -x);
  mju_addToScl3(D+3, dAproj_dv, -y);
  mju_addToScl3(D+6, dAproj_dv, -z);

  // diag(D) -= lin_coef
  D[0] -= lin_coef;
  D[4] -= lin_coef;
  D[8] -= lin_coef;
}



// Kutta lift due to motion in the fluid, D is 3x3
static inline void mjd_kutta_lift(
    mjtNum* restrict D, const mjtNum lvel[6], const mjtNum fluid_density,
    const mjtNum size[3], const mjtNum kutta_lift_coef) {
  const mjtNum a = pow2(size[1] * size[2]);
  const mjtNum b = pow2(size[2] * size[0]);
  const mjtNum c = pow2(size[0] * size[1]);
  const mjtNum aa = a*a, bb = b*b, cc = c*c;
  const mjtNum x = lvel[3], y = lvel[4], z = lvel[5];
  const mjtNum xx = x*x, yy = y*y, zz = z*z, xy=x*y, yz=y*z, xz=x*z;

  const mjtNum proj_denom = aa * xx + bb * yy + cc * zz;
  const mjtNum proj_num = a * xx + b * yy + c * zz;
  const mjtNum norm2 = xx + yy + zz;
  const mjtNum df_denom = mjPI * kutta_lift_coef * fluid_density / mju_max(
      mjMINVAL, mju_sqrt(proj_denom * proj_num * norm2));

  const mjtNum dfx_coef = yy * (a - b) + zz * (a - c);
  const mjtNum dfy_coef = xx * (b - a) + zz * (b - c);
  const mjtNum dfz_coef = xx * (c - a) + yy * (c - b);
  const mjtNum proj_term = proj_num / mju_max(mjMINVAL, proj_denom);
  const mjtNum cos_term = proj_num / mju_max(mjMINVAL, norm2);

  // cosA = proj_num/(norm*proj_denom), A_proj = pi*sqrt(proj_denom/proj_num)
  // F = cosA * A_proj * (([a,b,c] * vel) \times vel) \times vel
  // derivative obtained with SymPy

  D[0] = a-a;  D[1] = b-a;  D[2] = c-a;
  D[3] = a-b;  D[4] = b-b;  D[5] = c-b;
  D[6] = a-c;  D[7] = b-c;  D[8] = c-c;
  mju_scl(D, D, 2 * proj_num, 9);

  const mjtNum inner_term[3] = {
    aa * proj_term - a + cos_term,
    bb * proj_term - b + cos_term,
    cc * proj_term - c + cos_term
  };
  mju_addToScl3(D + 0, inner_term, dfx_coef);
  mju_addToScl3(D + 3, inner_term, dfy_coef);
  mju_addToScl3(D + 6, inner_term, dfz_coef);

  D[0] *= xx;  D[1] *= xy;  D[2] *= xz;
  D[3] *= xy;  D[4] *= yy;  D[5] *= yz;
  D[6] *= xz;  D[7] *= yz;  D[8] *= zz;

  D[0] -= dfx_coef * proj_num;
  D[4] -= dfy_coef * proj_num;
  D[8] -= dfz_coef * proj_num;

  mju_scl(D, D, df_denom, 9);
}



// Magnus force due to motion in the fluid, B is 6x6
static inline void mjd_magnus_force(
    mjtNum* restrict B, const mjtNum lvel[6], const mjtNum fluid_density,
    const mjtNum size[3], const mjtNum magnus_lift_coef) {
  const mjtNum volume = 4.0/3.0 * mjPI * size[0] * size[1] * size[2];

  // magnus_coef = magnus_lift_coef * fluid_density * volume
  const mjtNum magnus_coef = magnus_lift_coef * fluid_density * volume;

  mjtNum D_lin[9], D_ang[9];

  // premultiply by magnus_coef
  const mjtNum lin_vel[3] = {
    magnus_coef * lvel[3], magnus_coef * lvel[4], magnus_coef * lvel[5]};
  const mjtNum ang_vel[3] = {
    magnus_coef * lvel[0], magnus_coef * lvel[1], magnus_coef * lvel[2]};

  // force[3:] += magnus_coef * cross(ang_vel, lin_vel)
  mjd_cross(ang_vel, lin_vel, D_ang, D_lin);

  addToQuadrant(B, D_ang, 1, 0);
  addToQuadrant(B, D_lin, 1, 1);
}



//----------------- fluid force derivatives, ellipsoid and inertia-box models ----------------------

// fluid forces based on ellipsoid approximation
void mjd_ellipsoidFluid(const mjModel* m, mjData* d, mjtNum* DfDv, int bodyid) {
  mjMARKSTACK;

  int nv = m->nv;
  int nnz = nv;
  int rownnz[6], rowadr[6];
  mjtNum* J = mj_stackAlloc(d, 6*nv);
  mjtNum* tmp = mj_stackAlloc(d, 3*nv);
  int* colind = (int*) mj_stackAlloc(d, 6*nv);
  int* colind_compressed = (int*) mj_stackAlloc(d, 6*nv);

  mjtNum lvel[6], wind[6], lwind[6];
  mjtNum geom_interaction_coef, magnus_lift_coef, kutta_lift_coef;
  mjtNum semiaxes[3], virtual_mass[3], virtual_inertia[3];
  mjtNum blunt_drag_coef, slender_drag_coef, ang_drag_coef;

  if (mj_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = bodyJacSparse(m, bodyid, colind);

    // prepare rownnz, rowadr, colind for all 6 rows
    for (int i=0; i<6; i++) {
      rownnz[i] = nnz;
      rowadr[i] = i == 0 ? 0 : rowadr[i-1] + nnz;
      for (int k=0; k<nnz; k++) {
        colind_compressed[i*nnz+k] = colind[k];
      }
    }
  }

  for (int j=0; j<m->body_geomnum[bodyid]; j++) {
    const int geomid = m->body_geomadr[bodyid] + j;

    mju_geomSemiAxes(m, geomid, semiaxes);

    readFluidGeomInteraction(
        m->geom_fluid + mjNFLUID*geomid, &geom_interaction_coef,
        &blunt_drag_coef, &slender_drag_coef, &ang_drag_coef,
        &kutta_lift_coef, &magnus_lift_coef,
        virtual_mass, virtual_inertia);

    // scales all forces, read from MJCF as boolean (0.0 or 1.0)
    if (geom_interaction_coef == 0.0) {
      continue;
    }

    // map from CoM-centered to local body-centered 6D velocity
    mj_objectVelocity(m, d, mjOBJ_GEOM, geomid, lvel, 1);
    // compute wind in local coordinates
    mju_zero(wind, 6);
    mju_copy3(wind+3, m->opt.wind);
    mju_transformSpatial(lwind, wind, 0,
                         d->geom_xpos + 3*geomid,  // Frame of ref's origin.
                         d->subtree_com + 3*m->body_rootid[bodyid],
                         d->geom_xmat + 9*geomid);  // Frame of ref's orientation.
    // subtract translational component from grom velocity
    mju_subFrom3(lvel+3, lwind+3);

    // get body global Jacobian: rotation then translation
    mj_jacGeom(m, d, J+3*nv, J, geomid);

    // compress geom Jacobian in-place
    if (mj_isSparse(m)) {
      for (int i=0; i<6; i++) {
        for (int k=0; k<nnz; k++) {
          J[i*nnz+k] = J[i*nv+colind[k]];
        }
      }
    }

    // rotate (compressed) Jacobian to local frame
    mju_mulMatTMat(tmp, d->geom_xmat+9*geomid, J, 3, 3, nnz);
    mju_copy(J, tmp, 3*nnz);
    mju_mulMatTMat(tmp, d->geom_xmat+9*geomid, J+3*nnz, 3, 3, nnz);
    mju_copy(J+3*nnz, tmp, 3*nnz);

    mjtNum B[36], D[9];
    mju_zero(B, 36);
    mjd_magnus_force(B, lvel, m->opt.density, semiaxes, magnus_lift_coef);

    mjd_kutta_lift(D, lvel, m->opt.density, semiaxes, kutta_lift_coef);
    addToQuadrant(B, D, 1, 1);

    mjd_viscous_drag(D, lvel, m->opt.density, m->opt.viscosity, semiaxes,
                     blunt_drag_coef, slender_drag_coef);
    addToQuadrant(B, D, 1, 1);

    mjd_viscous_torque(D, lvel, m->opt.density, m->opt.viscosity, semiaxes,
                       slender_drag_coef, ang_drag_coef);
    addToQuadrant(B, D, 0, 0);

    mjd_addedMassForces(B, lvel, m->opt.density, virtual_mass, virtual_inertia);

    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, B, 6, nv, 0, rownnz, rowadr, colind_compressed);
    } else {
      addJTBJ(DfDv, J, B, 6, nv);
    }
  }

  mjFREESTACK;
}


// fluid forces based on inertia-box approximation
void mjd_inertiaBoxFluid(const mjModel* m, mjData* d, mjtNum* DfDv, int i)
{
  mjMARKSTACK;

  int nv = m->nv;
  int rownnz[6], rowadr[6];
  mjtNum* J = mj_stackAlloc(d, 6*nv);
  mjtNum* tmp = mj_stackAlloc(d, 3*nv);
  int* colind = (int*) mj_stackAlloc(d, 6*nv);

  mjtNum lvel[6], wind[6], lwind[6], box[3], B;
  mjtNum* inertia = m->body_inertia + 3*i;

  // equivalent inertia box
  box[0] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[1] + inertia[2] - inertia[0])) / m->body_mass[i] * 6.0);
  box[1] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[0] + inertia[2] - inertia[1])) / m->body_mass[i] * 6.0);
  box[2] = mju_sqrt(mju_max(mjMINVAL,
                            (inertia[0] + inertia[1] - inertia[2])) / m->body_mass[i] * 6.0);

  // map from CoM-centered to local body-centered 6D velocity
  mj_objectVelocity(m, d, mjOBJ_BODY, i, lvel, 1);

  // compute wind in local coordinates
  mju_zero(wind, 6);
  mju_copy3(wind+3, m->opt.wind);
  mju_transformSpatial(lwind, wind, 0, d->xipos+3*i,
                       d->subtree_com+3*m->body_rootid[i], d->ximat+9*i);

  // subtract translational component from body velocity
  mju_subFrom3(lvel+3, lwind+3);

  // get body global Jacobian: rotation then translation
  mj_jacBodyCom(m, d, J+3*nv, J, i);

  // init with dense
  int nnz = nv;

  // prepare for sparse
  if (mj_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = bodyJacSparse(m, i, colind);

    // compress body Jacobian in-place
    for (int j=0; j<6; j++) {
      for (int k=0; k<nnz; k++) {
        J[j*nnz+k] = J[j*nv+colind[k]];
      }
    }

    // prepare rownnz, rowadr, colind for all 6 rows
    rownnz[0] = nnz;
    rowadr[0] = 0;
    for (int j=1; j<6; j++) {
      rownnz[j] = nnz;
      rowadr[j] = rowadr[j-1] + nnz;
      for (int k=0; k<nnz; k++) {
        colind[j*nnz+k] = colind[k];
      }
    }
  }

  // rotate (compressed) Jacobian to local frame
  mju_mulMatTMat(tmp, d->ximat+9*i, J, 3, 3, nnz);
  mju_copy(J, tmp, 3*nnz);
  mju_mulMatTMat(tmp, d->ximat+9*i, J+3*nnz, 3, 3, nnz);
  mju_copy(J+3*nnz, tmp, 3*nnz);

  // add viscous force and torque
  if (m->opt.viscosity>0) {
    // diameter of sphere approximation
    mjtNum diam = (box[0] + box[1] + box[2])/3.0;

    // mju_scl3(lfrc, lvel, -mjPI*diam*diam*diam*m->opt.viscosity)
    B = -mjPI*diam*diam*diam*m->opt.viscosity;
    for (int j=0; j<3; j++) {
      if (mj_isSparse(m)) {
        addJTBJSparse(DfDv, J, &B, 1, nv, j, rownnz, rowadr, colind);
      } else {
        addJTBJ(DfDv, J+j*nv, &B, 1, nv);
      }
    }

    // mju_scl3(lfrc+3, lvel+3, -3.0*mjPI*diam*m->opt.viscosity);
    B = -3.0*mjPI*diam*m->opt.viscosity;
    for (int j=0; j<3; j++) {
      if (mj_isSparse(m)) {
        addJTBJSparse(DfDv, J, &B, 1, nv, 3+j, rownnz, rowadr, colind);
      } else {
        addJTBJ(DfDv, J+3*nv+j*nv, &B, 1, nv);
      }
    }
  }

  // add lift and drag force and torque
  if (m->opt.density>0) {
    // lfrc[0] -= m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
    //            mju_abs(lvel[0])*lvel[0]/64.0;
    B = -m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
        2*mju_abs(lvel[0])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 0, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J, &B, 1, nv);
    }

    // lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
    //            mju_abs(lvel[1])*lvel[1]/64.0;
    B = -m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
        2*mju_abs(lvel[1])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 1, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J+nv, &B, 1, nv);
    }

    // lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
    //            mju_abs(lvel[2])*lvel[2]/64.0;
    B = -m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
        2*mju_abs(lvel[2])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 2, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J+2*nv, &B, 1, nv);
    }

    // lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*mju_abs(lvel[3])*lvel[3];
    B = -0.5*m->opt.density*box[1]*box[2]*2*mju_abs(lvel[3]);
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 3, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J+3*nv, &B, 1, nv);
    }

    // lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*mju_abs(lvel[4])*lvel[4];
    B = -0.5*m->opt.density*box[0]*box[2]*2*mju_abs(lvel[4]);
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 4, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J+4*nv, &B, 1, nv);
    }

    // lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*mju_abs(lvel[5])*lvel[5];
    B = -0.5*m->opt.density*box[0]*box[1]*2*mju_abs(lvel[5]);
    if (mj_isSparse(m)) {
      addJTBJSparse(DfDv, J, &B, 1, nv, 5, rownnz, rowadr, colind);
    } else {
      addJTBJ(DfDv, J+5*nv, &B, 1, nv);
    }
  }

  mjFREESTACK;
}



//------------------------- derivatives of passive forces ------------------------------------------

// add (d qfrc_passive / d qvel) to DfDv
void mjd_passive_vel(const mjModel* m, mjData* d, mjtNum* DfDv) {
  int nv = m->nv;

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_PASSIVE)) {
    return;
  }

  // dof damping
  for (int i=0; i<nv; i++) {
    DfDv[i*(nv+1)] -= m->dof_damping[i];
  }

  // tendon damping
  for (int i=0; i<m->ntendon; i++) {
    if (m->tendon_damping[i]>0) {
      mjtNum B = -m->tendon_damping[i];

      // add sparse or dense
      if (mj_isSparse(m)) {
        addJTBJSparse(DfDv, d->ten_J, &B, 1, nv, i,
                      d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind);
      } else {
        addJTBJ(DfDv, d->ten_J+i*nv, &B, 1, nv);
      }
    }
  }

  // fluid drag model, either body-level (inertia box) or geom-level (ellipsoid)
  if (m->opt.viscosity>0 || m->opt.density>0) {
    for (int i=1; i<m->nbody; i++) {
      if (m->body_mass[i]<mjMINVAL) {
        continue;
      }

      int use_ellipsoid_model = 0;
      // if any child geom uses the ellipsoid model, inertia-box model is disabled for parent body
      for (int j=0; j<m->body_geomnum[i] && use_ellipsoid_model==0; j++) {
        const int geomid = m->body_geomadr[i] + j;
        use_ellipsoid_model += (m->geom_fluid[mjNFLUID*geomid] > 0);
      }
      if (use_ellipsoid_model) {
        mjd_ellipsoidFluid(m, d, DfDv, i);
      } else {
        mjd_inertiaBoxFluid(m, d, DfDv, i);
      }
    }
  }
}



// add forward fin-diff approximation of (d qfrc_passive / d qvel) to DfDv
void mjd_passive_velFD(const mjModel* m, mjData* d, mjtNum eps, mjtNum* DfDv) {
  int nv = m->nv;

  mjMARKSTACK;
  mjtNum* qfrc_passive = mj_stackAlloc(d, nv);
  mjtNum* fd = mj_stackAlloc(d, nv);

  // save qfrc_passive, assume mj_fwdVelocity was called
  mju_copy(qfrc_passive, d->qfrc_passive, nv);

  // loop over dofs
  for (int i=0; i<nv; i++) {
    // save qvel[i]
    mjtNum saveqvel = d->qvel[i];

    // eval at qvel[i]+eps
    d->qvel[i] = saveqvel + eps;
    mj_fwdVelocity(m, d);

    // restore qvel[i]
    d->qvel[i] = saveqvel;

    // finite difference result in fd
    mju_sub(fd, d->qfrc_passive, qfrc_passive, nv);
    mju_scl(fd, fd, 1/eps, nv);

    // copy to i-th column of DfDv
    for (int j=0; j<nv; j++) {
      DfDv[j*nv+i] += fd[j];
    }
  }

  // restore
  mj_fwdVelocity(m, d);

  mjFREESTACK;
}




//-------------------- derivatives of all smooth (unconstrained) forces ----------------------------


// centered finite difference approximation to mjd_smooth_vel
void mjd_smooth_velFD(const mjModel* m, mjData* d, mjtNum eps) {
  int nv = m->nv;

  mjMARKSTACK;
  mjtNum* plus = mj_stackAlloc(d, nv);
  mjtNum* minus = mj_stackAlloc(d, nv);
  mjtNum* fd = mj_stackAlloc(d, nv);
  int* cnt = (int*) mj_stackAlloc(d, nv);

  // clear row counters
  memset(cnt, 0, nv*sizeof(int));

  // loop over dofs
  for (int i=0; i<nv; i++) {
    // save qvel[i]
    mjtNum saveqvel = d->qvel[i];

    // eval at qvel[i]+eps
    d->qvel[i] = saveqvel + eps;
    mj_fwdVelocity(m, d);
    mj_fwdActuation(m, d);
    mju_add(plus, d->qfrc_actuator, d->qfrc_passive, nv);
    mju_subFrom(plus, d->qfrc_bias, nv);

    // eval at qvel[i]-eps
    d->qvel[i] = saveqvel - eps;
    mj_fwdVelocity(m, d);
    mj_fwdActuation(m, d);
    mju_add(minus, d->qfrc_actuator, d->qfrc_passive, nv);
    mju_subFrom(minus, d->qfrc_bias, nv);

    // restore qvel[i]
    d->qvel[i] = saveqvel;

    // finite difference result in fd
    mju_sub(fd, plus, minus, nv);
    mju_scl(fd, fd, 0.5/eps, nv);

    // copy to sparse qDeriv
    for (int j=0; j<nv; j++) {
      if (cnt[j]<d->D_rownnz[j] && d->D_colind[d->D_rowadr[j]+cnt[j]]==i) {
        d->qDeriv[d->D_rowadr[j]+cnt[j]] = fd[j];
        cnt[j]++;
      }
    }
  }

  // make sure final row counters equal rownnz
  for (int i=0; i<nv; i++) {
    if (cnt[i]!=d->D_rownnz[i]) {
      mju_error("error in constructing FD sparse derivative");
    }
  }

  // restore
  mj_fwdVelocity(m, d);
  mj_fwdActuation(m, d);

  mjFREESTACK;
}



//------------------------- main entry points ------------------------------------------------------

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - qfrc_bias) / d qvel
void mjd_smooth_vel(const mjModel *m, mjData *d) {
  int nv = m->nv;

  // allocate space
  mjMARKSTACK;
  mjtNum *DfDv = mj_stackAlloc(d, nv*nv);

  // clear DfDv
  mju_zero(DfDv, nv*nv);

  // DfDv = d (qfrc_actuator + qfrc_passive - qfrc_bias) / d qvel
  mjd_actuator_vel(m, d, DfDv);
  mjd_passive_vel(m, d, DfDv);
  mjd_rne_vel(m, d, DfDv);

  // copy dense DfDv to sparse qDeriv
  for (int i=0; i<nv; i++) {
    for (int j=0; j<d->D_rownnz[i]; j++) {
      int adr = d->D_rowadr[i] + j;
      d->qDeriv[adr] = DfDv[i*nv + d->D_colind[adr]];
    }
  }

  mjFREESTACK;
}


// finite differenced Jacobian of  (next_state, sensors) = mj_step(state, control)
//   all outputs are optional
//   output dimensions (transposed w.r.t Control Theory convention):
//     DyDq: (nv x 2*nv+na)
//     DyDv: (nv x 2*nv+na)
//     DyDa: (na x 2*nv+na)
//     DyDu: (nu x 2*nv+na)
//     DsDq: (nv x nsensordata)
//     DsDv: (nv x nsensordata)
//     DsDa: (na x nsensordata)
//     DsDu: (nu x nsensordata)
//   single-letter shortcuts:
//     inputs: q=qpos, v=qvel, a=act, u=ctrl
//     outputs: y=next_state (concatenated next qpos, qvel, act), s=sensordata
void mjd_stepFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte centered,
                mjtNum* DyDq, mjtNum* DyDv, mjtNum* DyDa, mjtNum* DyDu,
                mjtNum* DsDq, mjtNum* DsDv, mjtNum* DsDa, mjtNum* DsDu) {
  int nq = m->nq, nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
  int ndx = 2*nv+na;  // row length of Dy Jacobians
  mjMARKSTACK;

  // states
  mjtNum *state      = mj_stackAlloc(d, nq+nv+na);  // current state
  mjtNum *next       = mj_stackAlloc(d, nq+nv+na);  // next state
  mjtNum *next_plus  = mj_stackAlloc(d, nq+nv+na);  // forward-nudged next state
  mjtNum *next_minus = mj_stackAlloc(d, nq+nv+na);  // backward-nudged next state

  // warmstart accelerations
  mjtNum *warmstart = mjDISABLED(mjDSBL_WARMSTART) ? NULL : mj_stackAlloc(d, nv);

  // sensors
  int skipsensor = !DsDq && !DsDv && !DsDa && !DsDu;
  mjtNum *sensor       = skipsensor ? NULL : mj_stackAlloc(d, ns);  // sensor values
  mjtNum *sensor_plus  = skipsensor ? NULL : mj_stackAlloc(d, ns);  // forward-nudged sensors
  mjtNum *sensor_minus = skipsensor ? NULL : mj_stackAlloc(d, ns);  // backward-nudged sensors

  // controls
  mjtNum *ctrl = mj_stackAlloc(d, nu);

  // save current inputs
  mju_copy(ctrl, d->ctrl, nu);
  getState(m, d, state, NULL);
  if (warmstart) {
    mju_copy(warmstart, d->qacc_warmstart, nv);
  }

  // step input
  mj_step(m, d);

  // save output
  getState(m, d, next, sensor);

  // restore input
  setState(m, d, state, ctrl, warmstart);

  // finite-difference controls: skip=mjSTAGE_VEL, handle ctrl at range limits
  if (DyDu || DsDu) {
    for (int i=0; i<nu; i++) {
      int limited = m->actuator_ctrllimited[i];
      // nudge forward, if possible given ctrlrange
      int nudge_fwd = !limited || inRange(ctrl[i], ctrl[i]+eps, m->actuator_ctrlrange+2*i);
      if (nudge_fwd) {
        // nudge forward
        d->ctrl[i] += eps;

        // step, get nudged output
        mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
        getState(m, d, next_plus, sensor_plus);

        // reset
        setState(m, d, state, ctrl, warmstart);
      }

      // nudge backward, if possible given ctrlrange
      int nudge_back = (centered || !nudge_fwd) &&
                       (!limited || inRange(ctrl[i]-eps, ctrl[i], m->actuator_ctrlrange+2*i));
      if (nudge_back) {
        // nudge backward
        d->ctrl[i] -= eps;

        // step, get nudged output
        mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        setState(m, d, state, ctrl, warmstart);
      }

      // difference states
      if (DyDu) {
        clampedStateDiff(m, DyDu+i*ndx, next, nudge_fwd ? next_plus : NULL,
                         nudge_back ? next_minus : NULL, eps);
      }

      // difference sensors
      if (DsDu) {
        clampedDiff(DsDu+i*ns, sensor, nudge_fwd ? sensor_plus : NULL,
                    nudge_back ? sensor_minus : NULL, eps, ns);
      }
    }
  }

  // finite-difference activations: skip=mjSTAGE_VEL
  if (DyDa || DsDa) {
    for (int i=0; i<na; i++) {
      // nudge forward
      d->act[i] += eps;

      // step, get nudged output
      mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      setState(m, d, state, NULL, warmstart);

      // nudge backward
      if (centered) {
        // nudge backward
        d->act[i] -= eps;

        // step, get nudged output
        mj_stepSkip(m, d, mjSTAGE_VEL, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        setState(m, d, state, NULL, warmstart);
      }

      // difference states
      if (DyDa) {
        if (!centered) {
          stateDiff(m, DyDa+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDa+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDa) {
        if (!centered) {
          diff(DsDa+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDa+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }


  // finite-difference velocities: skip=mjSTAGE_POS
  if (DyDv || DsDv) {
    for (int i=0; i<nv; i++) {
      // nudge forward
      d->qvel[i] += eps;

      // step, get nudged output
      mj_stepSkip(m, d, mjSTAGE_POS, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      setState(m, d, state, NULL, warmstart);

      // nudge backward
      if (centered) {
        // nudge
        d->qvel[i] -= eps;

        // step, get nudged output
        mj_stepSkip(m, d, mjSTAGE_POS, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        setState(m, d, state, NULL, warmstart);
      }

      // difference states
      if (DyDv) {
        if (!centered) {
          stateDiff(m, DyDv+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDv+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDv) {
        if (!centered) {
          diff(DsDv+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDv+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }

  // finite-difference positions: skip=mjSTAGE_NONE
  if (DyDq || DsDq) {
    mjtNum *dpos  = mj_stackAlloc(d, nv);  // allocate position perturbation
    for (int i=0; i<nv; i++) {
      // nudge forward
      mju_zero(dpos, nv);
      dpos[i] = 1;
      mj_integratePos(m, d->qpos, dpos, eps);

      // step, get nudged output
      mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
      getState(m, d, next_plus, sensor_plus);

      // reset
      setState(m, d, state, NULL, warmstart);

      // nudge backward
      if (centered) {
        // nudge backward
        mju_zero(dpos, nv);
        dpos[i] = 1;
        mj_integratePos(m, d->qpos, dpos, -eps);

        // step, get nudged output
        mj_stepSkip(m, d, mjSTAGE_NONE, skipsensor);
        getState(m, d, next_minus, sensor_minus);

        // reset
        setState(m, d, state, NULL, warmstart);
      }

      // difference states
      if (DyDq) {
        if (!centered) {
          stateDiff(m, DyDq+i*ndx, next, next_plus, eps);
        } else {
          stateDiff(m, DyDq+i*ndx, next_minus, next_plus, 2*eps);
        }
      }

      // difference sensors
      if (DsDq) {
        if (!centered) {
          diff(DsDq+i*ns, sensor, sensor_plus, eps, ns);
        } else {
          diff(DsDq+i*ns, sensor_minus, sensor_plus, 2*eps, ns);
        }
      }
    }
  }

  mjFREESTACK;
}



// finite differenced transition matrices (control theory notation)
//   d(x_next) = A*dx + B*du
//   d(sensor) = C*dx + D*du
//   required output matrix dimensions:
//      A: (2*nv+na x 2*nv+na)
//      B: (2*nv+na x nu)
//      D: (nsensordata x 2*nv+na)
//      C: (nsensordata x nu)
void mjd_transitionFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte centered,
                      mjtNum* A, mjtNum* B, mjtNum* C, mjtNum* D) {
  int nv = m->nv, na = m->na, nu = m->nu, ns = m->nsensordata;
  int ndx = 2*nv+na;  // row length of state Jacobians

  // stepFD() offset pointers, initialised to NULL
  mjtNum *DyDq, *DyDv, *DyDa, *DsDq, *DsDv, *DsDa;
  DyDq = DyDv = DyDa = DsDq = DsDv = DsDa = NULL;

  mjMARKSTACK;

  // allocate transposed matrices
  mjtNum *AT = A ? mj_stackAlloc(d, ndx*ndx) : NULL; // state-transition matrix   (transposed)
  mjtNum *BT = B ? mj_stackAlloc(d, nu*ndx) : NULL;  // control-transition matrix (transposed)
  mjtNum *CT = C ? mj_stackAlloc(d, ndx*ns) : NULL;  // state-observation matrix   (transposed)
  mjtNum *DT = D ? mj_stackAlloc(d, nu*ns) : NULL;   // control-observation matrix (transposed)

  // set offset pointers
  if (A) {
    DyDq = AT;
    DyDv = AT+ndx*nv;
    DyDa = AT+ndx*2*nv;
  }

  if (C) {
    DsDq = CT;
    DsDv = CT + ns*nv;
    DsDa = CT + ns*2*nv;
  }

  // get Jacobians
  mjd_stepFD(m, d, eps, centered, DyDq, DyDv, DyDa, BT, DsDq, DsDv, DsDa, DT);


  // transpose
  if (A) mju_transpose(A, AT, ndx, ndx);
  if (B) mju_transpose(B, BT, nu, ndx);
  if (C) mju_transpose(C, CT, ndx, ns);
  if (D) mju_transpose(D, DT, nu, ns);

  mjFREESTACK;
}
