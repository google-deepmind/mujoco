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

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_inline.h"
#include "engine/engine_memory.h"
#include "engine/engine_passive.h"
#include "engine/engine_sleep.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_solve.h"
#include "engine/engine_util_spatial.h"
#include "engine/engine_util_sparse.h"



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
static void mjd_crossMotion_vel(mjtNum D[36], const mjtNum v[6]) {
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
static void mjd_crossForce_vel(mjtNum D[36], const mjtNum f[6]) {
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
static void mjd_crossForce_frc(mjtNum D[36], const mjtNum vel[6]) {
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
static void mjd_mulInertVec_vel(mjtNum D[36], const mjtNum i[10]) {
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


// derivative of mju_subQuat w.r.t inputs
void mjd_subQuat(const mjtNum qa[4], const mjtNum qb[4], mjtNum Da[9], mjtNum Db[9]) {
  // no outputs, quick return
  if (!Da && !Db) {
    return;
  }

  // compute axis-angle quaternion difference
  mjtNum axis[3];
  mju_subQuat(axis, qa, qb);

  // normalize axis, get half-angle
  mjtNum half_angle = 0.5 * mju_normalize3(axis);

  // identity
  mjtNum Da_tmp[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };

  // add term linear in cross product matrix K
  mjtNum K[9] = {
    0, -axis[2], axis[1],
    axis[2], 0, -axis[0],
    -axis[1], axis[0], 0
  };
  mju_addToScl(Da_tmp, K, half_angle, 9);

  // add term linear in K * K
  mjtNum KK[9];
  mju_mulMatMat3(KK, K, K);
  mjtNum coef = 1.0 - (half_angle < 6e-8 ? 1.0 : half_angle / mju_tan(half_angle));
  mju_addToScl(Da_tmp, KK, coef, 9);

  if (Da) {
    mju_copy9(Da, Da_tmp);
  }

  if (Db) {  // Db = -Da^T
    mju_transpose(Db, Da_tmp, 3, 3);
    mju_scl(Db, Db, -1.0, 9);
  }
}


// derivative of mju_quatIntegrate w.r.t scaled velocity
//  reference: https://arxiv.org/abs/1711.02508, Eq. 183
void mjd_quatIntegrate(const mjtNum vel[3], mjtNum scale,
                       mjtNum Dquat[9], mjtNum Dvel[9], mjtNum Dscale[3]) {
  // scaled velocity
  mjtNum s[3] = {scale*vel[0], scale*vel[1], scale*vel[2]};

  // 3 basis matrices
  mjtNum eye[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  mjtNum cross[9] = {
    0,     s[2], -s[1],
   -s[2],  0,     s[0],
    s[1], -s[0],  0
  };
  mjtNum outer[9] = {
    s[0]*s[0], s[0]*s[1], s[0]*s[2],
    s[1]*s[0], s[1]*s[1], s[1]*s[2],
    s[2]*s[0], s[2]*s[1], s[2]*s[2]
  };

  // squared norm, norm of s
  mjtNum xx = mju_dot3(s, s);
  mjtNum x = mju_sqrt(xx);

  // 4 coefficients: a=cos(x), b=sin(x)/x, c=(1-cos(x))/x^2, d=(x-sin(x))/x^3
  mjtNum a = mju_cos(x);
  mjtNum b, c, d;

  // x is not small: use full expressions
  if (mju_abs(x) > 1.0/32) {
    b = mju_sin(x) / x;
    c = (1.0 - a) / xx;
    d = (1.0 - b) / xx;
  }

  // |x| <= 1/32: use 6th order Taylor expansion (Horner form)
  else {
    b =  1 + xx/6  * (xx/20 * (1 - xx/42) - 1);
    c = (1 + xx/12 * (xx/30 * (1 - xx/56) - 1)) / 2;
    d = (1 + xx/20 * (xx/42 * (1 - xx/72) - 1)) / 6;
  }

  // derivatives
  mjtNum Dvel_[9];
  for (int i=0; i < 9; i++) {
    if (Dquat)          Dquat[i] = a*eye[i] + b*cross[i] + c*outer[i];
    if (Dvel || Dscale) Dvel_[i] = b*eye[i] + c*cross[i] + d*outer[i];
  }
  if (Dvel) mju_copy9(Dvel, Dvel_);
  if (Dscale) mju_mulMatVec3(Dscale, Dvel_, vel);
}


//------------------------- dense derivatives of component functions -------------------------------
// no longer used, except in tests

// derivative of cvel, cdof_dot w.r.t qvel (dense version)
static void mjd_comVel_vel_dense(const mjModel* m, mjData* d, mjtNum* Dcvel, mjtNum* Dcdofdot) {
  int nv = m->nv, nbody = m->nbody;
  mjtNum mat[36];

  // clear Dcvel
  mju_zero(Dcvel, nbody*6*nv);

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int i=1; i < nbody; i++) {
    // Dcvel = Dcvel_parent
    mju_copy(Dcvel+i*6*nv, Dcvel+m->body_parentid[i]*6*nv, 6*nv);

    // Dcvel += D(cdof * qvel),  Dcdofdot = D(cvel x cdof)
    for (int j=m->body_dofadr[i]; j < m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      switch ((mjtJoint) m->jnt_type[m->dof_jntid[j]]) {
      case mjJNT_FREE:
        // Dcdofdot = 0
        mju_zero(Dcdofdot+j*6*nv, 18*nv);

        // Dcvel += cdof * (D qvel)
        for (int k=0; k < 6; k++) {
          Dcvel[i*6*nv + k*nv + j+0] += d->cdof[(j+0)*6 + k];
          Dcvel[i*6*nv + k*nv + j+1] += d->cdof[(j+1)*6 + k];
          Dcvel[i*6*nv + k*nv + j+2] += d->cdof[(j+2)*6 + k];
        }

        // continue with rotations
        j += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // Dcdofdot = D crossMotion(cvel, cdof)
        for (int k=0; k < 3; k++) {
          mjd_crossMotion_vel(mat, d->cdof+6*(j+k));
          mju_mulMatMat(Dcdofdot+(j+k)*6*nv, mat, Dcvel+i*6*nv, 6, 6, nv);
        }

        // Dcvel += cdof * (D qvel)
        for (int k=0; k < 6; k++) {
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
        for (int k=0; k < 6; k++) {
          Dcvel[i*6*nv + k*nv + j] += d->cdof[j*6 + k];
        }
      }
    }
  }
}


// subtract (d qfrc_bias / d qvel) from qDeriv (dense version)
void mjd_rne_vel_dense(const mjModel* m, mjData* d) {
  int nv = m->nv, nbody = m->nbody;
  mjtNum mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  mj_markStack(d);
  mjtNum* Dcvel = mjSTACKALLOC(d, nbody*6*nv, mjtNum);
  mjtNum* Dcdofdot = mjSTACKALLOC(d, nv*6*nv, mjtNum);
  mjtNum* Dcacc = mjSTACKALLOC(d, nbody*6*nv, mjtNum);
  mjtNum* Dcfrcbody = mjSTACKALLOC(d, nbody*6*nv, mjtNum);
  mjtNum* row = mjSTACKALLOC(d, nv, mjtNum);

  // compute Dcvel and Dcdofdot
  mjd_comVel_vel_dense(m, d, Dcvel, Dcdofdot);

  // clear Dcacc
  mju_zero(Dcacc, nbody*6*nv);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int i=1; i < nbody; i++) {
    // Dcacc = Dcacc_parent
    mju_copy(Dcacc + i*6*nv, Dcacc + m->body_parentid[i]*6*nv, 6*nv);

    // Dcacc += D(cdofdot * qvel)
    for (int j=m->body_dofadr[i]; j < m->body_dofadr[i]+m->body_dofnum[i]; j++) {
      // Dcacc += cdofdot * (D qvel)
      for (int k=0; k < 6; k++) {
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
  for (int i=nbody-1; i > 0; i--) {
    if (m->body_parentid[i]) {
      mju_addTo(Dcfrcbody+m->body_parentid[i]*6*nv, Dcfrcbody+i*6*nv, 6*nv);
    }
  }

  // qDeriv -= D(cdof * cfrc_body)
  for (int i=0; i < nv; i++) {
    for (int k=0; k < 6; k++) {
      // compute D(cdof * cfrc_body), store in row
      mju_scl(row, Dcfrcbody + (m->dof_bodyid[i]*6+k)*nv, d->cdof[i*6+k], nv);

      // dense to sparse: qDeriv -= row
      int end = m->D_rowadr[i] + m->D_rownnz[i];
      for (int adr=m->D_rowadr[i]; adr < end; adr++) {
        d->qDeriv[adr] -= row[m->D_colind[adr]];
      }
    }
  }

  mj_freeStack(d);
}


//------------------------- sparse derivatives of component functions ------------------------------
// internal sparse format: dense body/dof x sparse dof x 6   (inner size is 6)

// copy sparse B-row from parent, shared ancestors only
static void copyFromParent(const mjModel* m, mjData* d, mjtNum* mat, int n) {
  // return if this is world or parent is world
  if (n == 0 || m->body_weldid[m->body_parentid[n]] == 0) {
    return;
  }

  // count dofs in ancestors
  int ndof = 0;
  int np = m->body_weldid[m->body_parentid[n]];
  while (np > 0) {
    // add self dofs
    ndof += m->body_dofnum[np];

    // advance to parent
    np = m->body_weldid[m->body_parentid[np]];
  }

  // copy: guaranteed to be at beginning of sparse array, due to sorting
  mju_copy(mat + 6*m->B_rowadr[n], mat + 6*m->B_rowadr[m->body_parentid[n]], 6*ndof);
}


// add sparse B-row to parent, all overlapping nonzeros
static void addToParent(const mjModel* m, mjData* d, mjtNum* mat, int n) {
  // return if this is world or parent is world
  if (n == 0 || m->body_weldid[m->body_parentid[n]] == 0) {
    return;
  }

  // find matching nonzeros
  int np = m->body_parentid[n];
  int i = 0, ip = 0;
  while (i < m->B_rownnz[n] && ip < m->B_rownnz[np]) {
    // columns match
    if (m->B_colind[m->B_rowadr[n] + i] == m->B_colind[m->B_rowadr[np] + ip]) {
      mju_addTo(mat + 6*(m->B_rowadr[np] + ip), mat + 6*(m->B_rowadr[n] + i), 6);

      // advance both
      i++;
      ip++;
    }

    // mismatch columns: advance parent
    else if (m->B_colind[m->B_rowadr[n] + i] > m->B_colind[m->B_rowadr[np] + ip]) {
      ip++;
    }

    // child nonzeroes must be subset of parent; SHOULD NOT OCCUR
    else {
      mjERROR("child nonzeroes must be subset of parent");
    }
  }
}


// derivative of cvel, cdof_dot w.r.t qvel
static void mjd_comVel_vel(const mjModel* m, mjData* d, mjtNum* Dcvel, mjtNum* Dcdofdot) {
  int nv = m->nv, nM = m->nM;
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int* Badr = m->B_rowadr, * Dadr = m->D_rowadr;
  mjtNum mat[36], matT[36];   // 6x6 matrices

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // Dcvel = Dcvel_parent
    copyFromParent(m, d, Dcvel, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j = m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < nv - 1 ? m->dof_Madr[j + 1] : nM) - (m->dof_Madr[j] + 1);

      // Dcvel += D(cdof * qvel),  Dcdofdot = D(cvel x cdof)
      switch ((mjtJoint) m->jnt_type[m->dof_jntid[j]]) {
      case mjJNT_FREE:
        // Dcdofdot = 0 (already cleared)

        // Dcvel += cdof * D(qvel)
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 0), d->cdof + 6*(j + 0), 6);
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 1), d->cdof + 6*(j + 1), 6);
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 2), d->cdof + 6*(j + 2), 6);

        // continue with rotations
        j += 3;
        Jadr += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // Dcdofdot = Dcvel * D crossMotion(cvel, cdof)
        for (int dj=0; dj < 3; dj++) {
          mjd_crossMotion_vel(mat, d->cdof + 6 * (j + dj));
          mju_transpose(matT, mat, 6, 6);
          mju_mulMatMat(Dcdofdot + 6*Dadr[j + dj], Dcvel + 6*Badr[i], matT, Jadr + dj, 6, 6);
        }

        // Dcvel += cdof * (D qvel)
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 0), d->cdof + 6*(j + 0), 6);
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 1), d->cdof + 6*(j + 1), 6);
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr + 2), d->cdof + 6*(j + 2), 6);

        // adjust for 3-dof joint
        j += 2;
        break;

      case mjJNT_HINGE:
      case mjJNT_SLIDE:
        // Dcdofdot = D crossMotion(cvel, cdof) * Dcvel
        mjd_crossMotion_vel(mat, d->cdof + 6 * j);
        mju_transpose(matT, mat, 6, 6);
        mju_mulMatMat(Dcdofdot + 6*Dadr[j], Dcvel + 6*Badr[i], matT, Jadr, 6, 6);

        // Dcvel += cdof * (D qvel)
        mju_addTo(Dcvel + 6*(Badr[i] + Jadr), d->cdof + 6*j, 6);
        break;

      default:
        mjERROR("unknown joint type");
      }
    }
  }
}


// subtract d qfrc_bias / d qvel from qDeriv
static void mjd_rne_vel(const mjModel* m, mjData* d) {
  int nM = m->nM;
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;
  int nparent = sleep_filter ? d->nparent_awake : m->nbody;
  int mnv = m->nv;
  int nv = sleep_filter ? d->nv_awake : mnv;

  const int* Badr = m->B_rowadr;
  const int* Dadr = m->D_rowadr;
  const int* Bnnz = m->B_rownnz;

  mjtNum mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  mj_markStack(d);
  mjtNum* Dcdofdot = mjSTACKALLOC(d, 6*m->nD, mjtNum);
  mjtNum* Dcvel = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* Dcacc = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* Dcfrcbody = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* row = mjSTACKALLOC(d, m->nv, mjtNum);

  // clear
  if (!sleep_filter) {
    mju_zero(Dcdofdot,  6*m->nD);
    mju_zero(Dcvel,     6*m->nB);
    mju_zero(Dcacc,     6*m->nB);
    mju_zero(Dcfrcbody, 6*m->nB);
  } else {
    for (int i = 0; i < nv; i++) {
      int dof = d->dof_awake_ind[i];
      mju_zero(Dcdofdot + 6*m->D_rowadr[dof], 6*m->D_rownnz[dof]);
    }

    for (int i = 0; i < nbody; i++) {
      int body = d->body_awake_ind[i];
      int adr = 6*m->B_rowadr[body];
      int nnz = 6*m->B_rownnz[body];
      mju_zero(Dcvel     + adr, nnz);
      mju_zero(Dcacc     + adr, nnz);
      mju_zero(Dcfrcbody + adr, nnz);
    }
  }

  // compute Dcvel and Dcdofdot
  mjd_comVel_vel(m, d, Dcvel, Dcdofdot);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int b=1; b < nbody; b++) {
    int i = sleep_filter ? d->body_awake_ind[b] : b;

    // Dcacc = Dcacc_parent
    copyFromParent(m, d, Dcacc, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j=m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < mnv - 1 ? m->dof_Madr[j + 1] : nM) - (m->dof_Madr[j] + 1);

      // Dcacc += cdofdot * (D qvel)
      mju_addTo(Dcacc + 6*(Badr[i] + Jadr), d->cdof_dot + 6*j, 6);

      // Dcacc += (D cdofdot) * qvel
      // Dcacc[row i] and Dcdofdot[row j] have identical sparsity
      mju_addToScl(Dcacc + 6*Badr[i], Dcdofdot + 6*Dadr[j], d->qvel[j], 6*Bnnz[i]);
    }

    //---------- Dcfrcbody = D(cinert * cacc + cvel x (cinert * cvel))

    // Dcfrcbody = (D mul / D cacc) * Dcacc
    mjd_mulInertVec_vel(dmul, d->cinert + 10*i);
    mju_transpose(mat1, dmul, 6, 6);
    mju_mulMatMat(Dcfrcbody + 6*Badr[i], Dcacc + 6*Badr[i], mat1, Bnnz[i], 6, 6);

    // mat = (D cross / D cvel) + (D cross / D mul) * (D mul / D cvel)
    mju_mulInertVec(tmp, d->cinert + 10*i, d->cvel + i*6);
    mjd_crossForce_vel(mat, tmp);
    mjd_crossForce_frc(mat1, d->cvel + i*6);
    mju_mulMatMat(mat2, mat1, dmul, 6, 6, 6);
    mju_addTo(mat, mat2, 36);

    // Dcfrcbody += mat * Dcvel  (use worldbody as temp)
    mju_transpose(mat1, mat, 6, 6);
    mju_mulMatMat(Dcfrcbody, Dcvel + 6*Badr[i], mat1, Bnnz[i], 6, 6);
    mju_addTo(Dcfrcbody + 6*Badr[i], Dcfrcbody, 6*Bnnz[i]);
  }

  // clear worldbody Dcfrcbody
  mju_zero(Dcfrcbody, 6*Bnnz[0]);

  // backward pass over bodies: accumulate Dcfrcbody
  for (int b=nparent-1; b > 0; b--) {
    int i = sleep_filter ? d->parent_awake_ind[b] : b;
    addToParent(m, d, Dcfrcbody, i);
  }

  // process all dofs, update qDeriv
  for (int v=0; v < nv; v++) {
    int j = sleep_filter ? d->dof_awake_ind[v] : v;

    // get body index
    int i = m->dof_bodyid[j];

    // qDeriv -= D(cdof * cfrc_body)
    mju_mulMatVec(row, Dcfrcbody + 6*Badr[i], d->cdof + 6*j, Bnnz[i], 6);
    mju_subFrom(d->qDeriv + Dadr[j], row, Bnnz[i]);
  }

  mj_freeStack(d);
}


// 3x3 sub-blocks of (d qfrc_bias / d qvel) for a standalone free body
//   outputs the two 3x3 blocks lin and rot such that the rotational columns
//   of the full 6x6 bias Jacobian B are  [-mass*lin; rot]  (linear columns are zero)
//
// derivation: let R = xmat, s = xipos - xpos, w = R*qvel[rot] (world angular velocity),
// Iw = ximat * diag(body_inertia) * ximat' (world inertia about the CoM). with qacc = 0,
// the CoM acceleration is w x (w x s) and the world bias force/torque at the CoM are
//   f = mass * w x (w x s),   tau = w x Iw*w
// projected onto the joint coordinates: bias = [f;  R'*(s x f + tau)]. differentiating
// w.r.t. the rotational dofs (through w = R*qvel[rot]), with K = [w x s]_x + [w]_x [s]_x:
//   d f / d w   = -mass * K            =>  lin = K * R
//   d tau / d w = [w]_x Iw - [Iw*w]_x  =>  rot = R' * (-mass*[s]_x K + d tau/d w) * R
static void freeBias_vel_blocks(mjtNum mass, const mjtNum R[9], const mjtNum Xi[9],
                                const mjtNum inertia[3], const mjtNum s[3],
                                const mjtNum qvel_rot[3], mjtNum lin[9], mjtNum rot[9]) {
  // world-frame angular velocity
  mjtNum w[3];
  mji_mulMatVec3(w, R, qvel_rot);

  // world-frame inertia about CoM: Iw = Xi * diag(inertia) * Xi^T
  mjtNum Xi_I[9];
  for (int i=0; i < 3; i++) {
    Xi_I[3*i+0] = Xi[3*i+0] * inertia[0];
    Xi_I[3*i+1] = Xi[3*i+1] * inertia[1];
    Xi_I[3*i+2] = Xi[3*i+2] * inertia[2];
  }
  mjtNum Iw[9];
  Iw[0] = Xi_I[0]*Xi[0] + Xi_I[1]*Xi[1] + Xi_I[2]*Xi[2];
  Iw[4] = Xi_I[3]*Xi[3] + Xi_I[4]*Xi[4] + Xi_I[5]*Xi[5];
  Iw[8] = Xi_I[6]*Xi[6] + Xi_I[7]*Xi[7] + Xi_I[8]*Xi[8];
  Iw[1] = Iw[3] = Xi_I[0]*Xi[3] + Xi_I[1]*Xi[4] + Xi_I[2]*Xi[5];
  Iw[2] = Iw[6] = Xi_I[0]*Xi[6] + Xi_I[1]*Xi[7] + Xi_I[2]*Xi[8];
  Iw[5] = Iw[7] = Xi_I[3]*Xi[6] + Xi_I[4]*Xi[7] + Xi_I[5]*Xi[8];

  // intermediate vectors: ws = w x s  (CoM offset velocity),  Iww = Iw * w  (angular momentum)
  mjtNum ws[3], Iww[3];
  mji_cross(ws, w, s);
  mji_mulMatVec3(Iww, Iw, w);

  // K = [w x s]_x + [w]_x [s]_x = s w^T - (w . s) I + [ws]_x
  mjtNum w_dot_s = w[0]*s[0] + w[1]*s[1] + w[2]*s[2];
  mjtNum K[9];
  K[0] = s[0]*w[0] - w_dot_s;
  K[1] = s[0]*w[1] - ws[2];
  K[2] = s[0]*w[2] + ws[1];

  K[3] = s[1]*w[0] + ws[2];
  K[4] = s[1]*w[1] - w_dot_s;
  K[5] = s[1]*w[2] - ws[0];

  K[6] = s[2]*w[0] - ws[1];
  K[7] = s[2]*w[1] + ws[0];
  K[8] = s[2]*w[2] - w_dot_s;

  // lin = K * R
  mji_mulMatMat3(lin, K, R);

  // C = -mass * [s]_x K + [w]_x Iw - [Iww]_x,  column by column
  // the last term (-[Iww]_x) is the negated cross-product matrix, added via ternaries
  mjtNum C[9];
  for (int c=0; c < 3; c++) {
    mjtNum s_x_K_row0 = s[1]*K[6+c] - s[2]*K[3+c];
    mjtNum s_x_K_row1 = s[2]*K[c] - s[0]*K[6+c];
    mjtNum s_x_K_row2 = s[0]*K[3+c] - s[1]*K[c];

    mjtNum w_x_Iw_row0 = w[1]*Iw[6+c] - w[2]*Iw[3+c];
    mjtNum w_x_Iw_row1 = w[2]*Iw[c] - w[0]*Iw[6+c];
    mjtNum w_x_Iw_row2 = w[0]*Iw[3+c] - w[1]*Iw[c];

    C[c]     = -mass * s_x_K_row0 + w_x_Iw_row0 + (c == 1 ? Iww[2] : (c == 2 ? -Iww[1] : 0));
    C[3 + c] = -mass * s_x_K_row1 + w_x_Iw_row1 + (c == 0 ? -Iww[2] : (c == 2 ? Iww[0] : 0));
    C[6 + c] = -mass * s_x_K_row2 + w_x_Iw_row2 + (c == 0 ? Iww[1] : (c == 1 ? -Iww[0] : 0));
  }

  // rot = R^T * C * R
  mjtNum tmp[9];
  mji_mulMatTMat3(tmp, R, C);
  mji_mulMatMat3(rot, tmp, R);
}


// 6x6 block B = d qfrc_bias / d qvel for a standalone free body
//   assembles the full 6x6 from the 3x3 sub-blocks computed by freeBias_vel_blocks
//   rows/cols ordered like the free joint dofs: [linear(3); rotational(3)]
//   linear columns are zero: the bias force does not depend on linear velocity
void mjd_freeBias_vel(const mjModel* m, const mjData* d, int jnt, mjtNum B[36]) {
  int body = m->jnt_bodyid[jnt];
  int adr = m->jnt_dofadr[jnt];
  mjtNum mass = m->body_mass[body];
  const mjtNum* R = d->xmat + 9*body;    // body  -> world
  const mjtNum* Xi = d->ximat + 9*body;  // inertia -> world
  const mjtNum* inertia = m->body_inertia + 3*body;

  // CoM offset from joint origin, world frame
  mjtNum s[3];
  mji_sub3(s, d->xipos + 3*body, d->xpos + 3*body);

  mjtNum lin[9], rot[9];
  freeBias_vel_blocks(mass, R, Xi, inertia, s, d->qvel + adr + 3, lin, rot);

  mju_zero(B, 36);
  for (int r=0; r < 3; r++) {
    for (int c=0; c < 3; c++) {
      B[6*r + 3+c] = -mass * lin[3*r+c];
      B[6*(3+r) + 3+c] = rot[3*r+c];
    }
  }
}


// return 1 if body is a standalone free body (single free joint, no children)
mjtBool mj_isFreeBody(const mjModel* m, int body) {
  // must have exactly one joint, of free type
  if (m->body_jntnum[body] != 1 || m->jnt_type[m->body_jntadr[body]] != mjJNT_FREE) {
    return false;
  }

  int adr = m->jnt_dofadr[m->body_jntadr[body]];

  // must be a standalone 6-DOF tree with no children
  if (m->tree_dofnum[m->dof_treeid[adr]] != 6 ||
      m->body_subtreemass[body] != m->body_mass[body]) {
    return false;
  }

  return true;
}


// 6x6 block A = M - h * (d qfrc_smooth / d qvel) for the free joint of a standalone body
//   returns 1 and writes A if jnt is the free joint of a standalone awake body, 0 otherwise
//   requires valid d->qDeriv rows for the block, computed with flg_bias = 0; the bias
//   derivative excluded from qDeriv is added here via mjd_freeBias_vel
int mjd_freeMhat(const mjModel* m, const mjData* d, int jnt, mjtNum h, mjtNum A[36]) {
  int body = m->jnt_bodyid[jnt];
  int adr = m->jnt_dofadr[jnt];

  // must be a standalone free body, awake
  if (!mj_isFreeBody(m, body) || !d->tree_awake[m->dof_treeid[adr]]) {
    return 0;
  }

  // A = M block (gather from sparse lower triangle)
  mju_zero(A, 36);
  for (int r=0; r < 6; r++) {
    int rowadr = m->M_rowadr[adr+r];
    int rownnz = m->M_rownnz[adr+r];
    for (int k=0; k < rownnz; k++) {
      int c = m->M_colind[rowadr+k] - adr;
      A[6*r+c] = A[6*c+r] = d->M[rowadr+k];
    }
  }

  // A -= h * qDeriv block (actuator and passive derivatives)
  for (int r=0; r < 6; r++) {
    int rowadr = m->D_rowadr[adr+r];
    int rownnz = m->D_rownnz[adr+r];
    for (int k=0; k < rownnz; k++) {
      int c = m->D_colind[rowadr+k] - adr;
      A[6*r+c] -= h * d->qDeriv[rowadr+k];
    }
  }

  // A -= h * d(qfrc_smooth)/d(qvel) for the bias term missing from qDeriv;
  // qfrc_smooth includes -qfrc_bias, so subtracting its derivative adds +h*B
  mjtNum s[3];
  mji_sub3(s, d->xipos + 3*body, d->xpos + 3*body);

  mjtNum mass = m->body_mass[body];
  mjtNum lin[9], rot[9];
  freeBias_vel_blocks(mass, d->xmat + 9*body, d->ximat + 9*body,
                      m->body_inertia + 3*body, s, d->qvel + adr + 3, lin, rot);

  mjtNum h_mass = -h * mass;
  for (int r=0; r < 3; r++) {
    for (int c=0; c < 3; c++) {
      A[6*r + 3+c] += h_mass * lin[3*r+c];
      A[6*(3+r) + 3+c] += h * rot[3*r+c];
    }
  }

  return 1;
}


//--------------------- utility functions for (d force / d vel) Jacobians --------------------------

// add J'*B*J to qDeriv
static void addJTBJ(const mjModel* m, mjData* d, const mjtNum* J, const mjtNum* B, int n) {
  int nv = m->nv;

  // allocate dense row
  mj_markStack(d);
  mjtNum* row = mjSTACKALLOC(d, nv, mjtNum);

  // process non-zero elements of B
  for (int i=0; i < n; i++) {
    for (int j=0; j < n; j++) {
      if (!B[i*n+j]) {
        continue;
      }
      // process non-zero elements of J(i,:)
      for (int k=0; k < nv; k++) {
        if (J[i*nv+k]) {
          // row = J(i,k)*B(i,j)*J(j,:)
          mju_scl(row, J+j*nv, J[i*nv+k] * B[i*n+j], nv);

          // add row to qDeriv(k,:)
          int rownnz_k = m->D_rownnz[k];
          for (int s=0; s < rownnz_k; s++) {
            int adr = m->D_rowadr[k] + s;
            d->qDeriv[adr] += row[m->D_colind[adr]];
          }
        }
      }
    }
  }

  mj_freeStack(d);
}


// add J'*B*J to qDeriv, sparse version
static void addJTBJSparse(
  const mjModel* m, mjData* d, const mjtNum* J,
  const mjtNum* B, int n, int offset,
  const int* J_rownnz, const int* J_rowadr, const int* J_colind) {

  // compute qDeriv(k,p) += sum_{i,j} ( J(i,k)*B(i,j)*J(j,p) )
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (!B[i*n+j]) {
        continue;
      }

      // loop over non-zero elements of J(i,:)
      int nnz_i = J_rownnz[offset+i];
      int adr_i = J_rowadr[offset+i];
      int nnz_j = J_rownnz[offset+j];
      int adr_j = J_rowadr[offset+j];
      for (int k = 0; k < nnz_i; k++) {
        int ik = adr_i + k;
        int colik = J_colind[ik];

        // qDeriv(k,:) += J(j,:) * J(i,k)*B(i,j)
        mju_addToSclSparseInc(d->qDeriv + m->D_rowadr[colik], J + adr_j,
                              m->D_rownnz[colik], m->D_colind + m->D_rowadr[colik],
                              nnz_j, J_colind + adr_j,
                              J[ik]*B[i*n+j]);
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
  if (force < 0) {
    force = scale / mju_max(mjMINVAL, acc0);
  }

  // optimum length
  mjtNum L0 = (lengthrange[1]-lengthrange[0]) / mju_max(mjMINVAL, range[1]-range[0]);

  // normalized length and velocity
  mjtNum L = range[0] + (len-lengthrange[0]) / mju_max(mjMINVAL, L0);
  mjtNum V = vel / mju_max(mjMINVAL, L0*vmax);

  // length curve
  mjtNum FL = mju_muscleGainLength(L, lmin, lmax);

  // velocity curve
  mjtNum dFV;
  mjtNum y = fvmax-1;
  if (V <= -1) {
    // FV = 0
    dFV = 0;
  } else if (V <= 0) {
    // FV = (V+1)*(V+1)
    dFV = 2*V + 2;
  } else if (V <= y) {
    // FV = fvmax - (y-V)*(y-V) / mju_max(mjMINVAL, y)
    dFV = (-2*V + 2*y) / mju_max(mjMINVAL, y);
  } else {
    // FV = fvmax
    dFV = 0;
  }

  // compute FVL and scale, make it negative
  return -force*FL*dFV/mju_max(mjMINVAL, L0*vmax);
}


//--------------------- utility functions for (d force / d pos) * vec Jacobians --------------------

// add J'*B*J*vec to res, sparse version
static void addJTBJ_mulSparse(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                              const int* J_rownnz, const int* J_rowadr, const int* J_colind,
                              const mjtNum* J, const mjtNum* B, int n) {
  // allocate temp vectors
  mj_markStack(d);
  mjtNum* Jv = mjSTACKALLOC(d, n, mjtNum);
  mjtNum* BJv = mjSTACKALLOC(d, n, mjtNum);

  // Jv = J*vec (Sparse Matrix-Vector Multiplication)
  mju_zero(Jv, n);
  for (int i=0; i < n; i++) {
    int nnz = J_rownnz[i];
    int adr = J_rowadr[i];
    for (int k=0; k < nnz; k++) {
      Jv[i] += J[adr + k] * vec[J_colind[adr + k]];
    }
  }

  // BJv = B*Jv (Dense Matrix-Vector Multiplication)
  mju_mulMatVec(BJv, B, Jv, n, n);

  // res += J'*BJv (Sparse Transpose Matrix-Vector Multiplication)
  for (int i=0; i < n; i++) {
    int nnz = J_rownnz[i];
    int adr = J_rowadr[i];
    mjtNum val = BJv[i];
    for (int k=0; k < nnz; k++) {
      res[J_colind[adr + k]] += J[adr + k] * val;
    }
  }

  mj_freeStack(d);
}


// shared kernel for flex interpolation derivatives, scale = s1 + s2*damping
//  res: output vector (res += J'*K*J*vec), or NULL for cache-only mode
//  vec: input vector, or NULL for cache-only mode
//  K_rot_cache: if non-NULL, use pre-cached K_rot values instead of computing them
//  K_rot_out: if non-NULL and K_rot_cache is NULL, store computed K_rot values here
static void mjd_flexInterp_kernel(const mjModel* m, mjData* d,
                                  mjtNum* res, const mjtNum* vec, mjtNum s1, mjtNum s2,
                                  const mjtNum* K_rot_cache, mjtNum* K_rot_out) {
  int nv = m->nv;

  // compute upper bounds across all interpolated flexes
  int max_nodenum = 0;
  int max_npe = 0;  // max nodes per element (3D cell or 2D face)
  for (int f = 0; f < m->nflex; f++) {
    if (!m->flex_interp[f]) continue;
    if (m->flex_rigid[f]) continue;
    int order = m->flex_interp[f];
    int shell_mode = order < 0;
    order = order < 0 ? -order : order;
    int npe;
    if (shell_mode) {
      npe = (order+1)*(order+1);
    } else {
      npe = (order+1)*(order+1)*(order+1);
    }
    if (npe > max_npe) max_npe = npe;
    if (m->flex_nodenum[f] > max_nodenum) max_nodenum = m->flex_nodenum[f];
  }

  // nothing to do
  if (max_npe == 0) {
    return;
  }

  int max_dim_c = 3 * max_npe;

  // single unconditional markStack
  mj_markStack(d);

  // per-flex node positions (upper bound)
  mjtNum* xpos = mjSTACKALLOC(d, 3*max_nodenum, mjtNum);

  // per-element arrays (upper bound)
  mjtNum* xpos_c = mjSTACKALLOC(d, 3*max_npe, mjtNum);
  mjtNum* K_rot_cell = mjSTACKALLOC(d, max_dim_c*max_dim_c, mjtNum);

  // sparse Jacobian for one cell (upper bound)
  int* J_rownnz = mjSTACKALLOC(d, max_dim_c, int);
  int* J_rowadr = mjSTACKALLOC(d, max_dim_c, int);
  mjtNum* J_val = mjSTACKALLOC(d, max_dim_c*nv, mjtNum);
  int* J_colind = mjSTACKALLOC(d, max_dim_c*nv, int);

  // temp allocations for chain
  int* chain_colind = mjSTACKALLOC(d, nv, int);
  mjtNum* blk_jac = mjSTACKALLOC(d, 3*nv, mjtNum);

  // temp buffer for fast path element result
  mjtNum* fast_tmp = mjSTACKALLOC(d, max_dim_c, mjtNum);

  // loop over flexes
  for (int f=0; f < m->nflex; f++) {
    // only process flex_interp
    if (!m->flex_interp[f]) {
      continue;
    }

    // get stiffness and damping
    int stiffnessadr = m->flex_stiffnessadr[f];
    if (stiffnessadr < 0) {
      continue;
    }
    mjtNum* K = m->flex_stiffness + stiffnessadr;

    // skip if rigid or no stiffness
    if (m->flex_rigid[f] || K[0] == 0) {
      continue;
    }

    // skip if strain constraints present (stiffness handled by constraint solver)
    if (m->flex_edgeequality[f] == 3) {
      continue;
    }

    // compute scale
    mjtNum damping = m->flex_damping[f];
    mjtNum scale = s1 + s2 * damping;

    // skip if scale is zero
    if (scale == 0) {
      continue;
    }

    int order = m->flex_interp[f];
    int shell_mode = order < 0;
    order = order < 0 ? -order : order;

    int cx = m->flex_cellnum[3*f+0];
    int cy = m->flex_cellnum[3*f+1];
    int cz = m->flex_cellnum[3*f+2];

    int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];

    // determine element type: 2D boundary quads (shell) or 3D cells (volume)
    int npe;
    int nelem_fe;
    if (shell_mode) {
      npe = (order+1)*(order+1);
      nelem_fe = 2*(cy*cz + cx*cz + cx*cy);
    } else {
      npe = (order+1)*(order+1)*(order+1);
      nelem_fe = cx * cy * cz;
    }
    int dim_e = 3 * npe;

    // gather raw node positions (unrotated)
    mju_flexGatherState(m, d, f, xpos, NULL);

    // check if centered fast path applies: centered, all nodes on simple slider
    // bodies (body_simple == 2 means diag M with sliders only, J = R_body per node)
    int use_fast_path = m->flex_centered[f];
    if (use_fast_path) {
      int nodenum_f = m->flex_nodenum[f];
      for (int n = 0; n < nodenum_f; n++) {
        if (m->body_simple[bodyid[n]] != 2) {
          use_fast_path = 0;
          break;
        }
      }
    }

    // loop over finite elements
    for (int fe = 0; fe < nelem_fe; fe++) {
      // get element stiffness
      mjtNum* k_elem = K + fe * 3*npe * 3*npe;

      // skip empty elements: stiffness buffer is zero-initialized at compile time
      // (user_model.cc), and non-empty elements have strictly positive diagonal
      if (k_elem[0] == 0) {
        continue;
      }

      // use cached K_rot or compute from scratch
      int gindices[125];  // element node indices (max npe = 125 for quadratic 3D)
      int krot_adr = stiffnessadr + fe * dim_e * dim_e;
      if (K_rot_cache) {
        // read K_rot from cache and apply scale
        for (int i = 0; i < dim_e*dim_e; i++) {
          K_rot_cell[i] = scale * K_rot_cache[krot_adr + i];
        }

        // recompute gindices (cheap index-only call, no rotation)
        if (shell_mode) {
          mju_flexGatherFaceState(order, cx, cy, cz, fe, NULL, NULL, NULL,
                                  NULL, NULL, NULL, gindices, NULL);
        } else {
          int ci = fe / (cy * cz);
          int cj = (fe / cz) % cy;
          int ck = fe % cz;
          mju_flexGatherCellState(order, cy, cz, ci, cj, ck, NULL, NULL, NULL,
                                  NULL, NULL, NULL, gindices, NULL);
        }
      } else {
        // gather element-local node positions and rotation
        mjtNum quat[4];
        if (shell_mode) {
          mju_flexGatherFaceState(order, cx, cy, cz, fe, xpos, NULL, NULL,
                                  xpos_c, NULL, NULL, gindices, quat);
        } else {
          int ci = fe / (cy * cz);
          int cj = (fe / cz) % cy;
          int ck = fe % cz;
          mju_flexGatherCellState(order, cy, cz, ci, cj, ck, xpos, NULL, NULL,
                                  xpos_c, NULL, NULL, gindices, quat);
        }

        // R = R_global2local, RT = R_local2global
        mjtNum R[9], RT[9];
        mju_quat2Mat(R, quat);
        mju_transpose(RT, R, 3, 3);

        // compute K_rot = RT * K_elem * R (block-wise)
        mju_zero(K_rot_cell, dim_e*dim_e);
        for (int a = 0; a < npe; a++) {
          for (int b = 0; b < npe; b++) {
            mjtNum blk[9], tmp[9];

            // get K_elem(a,b) 3x3 block
            int adr_cell = (3*a)*(3*npe) + 3*b;
            for (int r = 0; r < 3; r++) {
              for (int c = 0; c < 3; c++) {
                blk[3*r+c] = k_elem[adr_cell + r*(3*npe) + c];
              }
            }

            // tmp = K * R
            mju_mulMatMat3(tmp, blk, R);
            // blk = RT * tmp = RT * K * R
            mju_mulMatMat3(blk, RT, tmp);

            // store in K_rot_cell at (a, b)
            int adr_out = (3*a)*dim_e + 3*b;
            for (int r = 0; r < 3; r++) {
              for (int c = 0; c < 3; c++) {
                K_rot_cell[adr_out + r*dim_e + c] = scale * blk[3*r+c];
              }
            }
          }
        }

        // optionally store unscaled K_rot to output cache
        if (K_rot_out) {
          for (int i = 0; i < dim_e*dim_e; i++) {
            K_rot_out[krot_adr + i] = K_rot_cell[i] / scale;
          }
        }
      }

      // skip Jacobian construction and op when only caching (res == NULL)
      if (!res) {
        continue;
      }

      // fast path: centered flex with 3 translational DOFs per body
      // J = R_body (not I when body is rotated), so J'*K*J*vec = R^T*K*(R*vec)
      if (use_fast_path) {
        // apply R_body to vec for each node
        for (int n = 0; n < npe; n++) {
          int dof = m->body_dofadr[bodyid[gindices[n]]];
          mji_mulMatVec3(fast_tmp + 3*n, d->xmat + 9*bodyid[gindices[n]], vec + dof);
        }

        // compute R^T * K_rot * (R*vec) and add to res
        for (int a = 0; a < npe; a++) {
          int dof_a = m->body_dofadr[bodyid[gindices[a]]];
          mjtNum row[3] = {0, 0, 0};
          for (int b = 0; b < npe; b++) {
            int adr = (3*a)*dim_e + 3*b;
            for (int r = 0; r < 3; r++) {
              for (int c = 0; c < 3; c++) {
                row[r] += K_rot_cell[adr + r*dim_e + c] * fast_tmp[3*b + c];
              }
            }
          }
          // apply R^T and add to res
          mjtNum qfrc_loc[3];
          mji_mulMatTVec3(qfrc_loc, d->xmat + 9*bodyid[gindices[a]], row);
          for (int r = 0; r < 3; r++) {
            res[dof_a + r] += qfrc_loc[r];
          }
        }
      } else {
        // general path: construct sparse Jacobian for this element's nodes
        int current_adr = 0;
        for (int n = 0; n < npe; n++) {
          int bid = bodyid[gindices[n]];
          int chain_nnz = mj_bodyChain(m, bid, chain_colind);
          mj_jacSparse(m, d, blk_jac, NULL, xpos+3*gindices[n], bid,
                       chain_nnz, chain_colind, /*flg_skipcommon=*/0);

          for (int r = 0; r < 3; r++) {
            int row_idx = 3*n + r;
            J_rownnz[row_idx] = chain_nnz;
            J_rowadr[row_idx] = current_adr;

            for (int idx = 0; idx < chain_nnz; idx++) {
              J_colind[current_adr] = chain_colind[idx];
              J_val[current_adr] = blk_jac[r*chain_nnz + idx];
              current_adr++;
            }
          }
        }

        // res += J'*K_rot*J*vec
        addJTBJ_mulSparse(m, d, res, vec, J_rownnz, J_rowadr, J_colind,
                          J_val, K_rot_cell, dim_e);
      }

    }
  }

  mj_freeStack(d);
}



// compute res += (s1 + s2*damping) * J'*K*J * vec, for all interpolated flexes
//   K_rot_cache: if non-NULL, use pre-cached K_rot (same layout as m->flex_stiffness)
void mjd_flexInterp_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                        mjtNum s1, mjtNum s2, const mjtNum* K_rot_cache) {
  mjd_flexInterp_kernel(m, d, res, vec, s1, s2, K_rot_cache, NULL);
}


// precompute unscaled K_rot for all elements into cache (same layout as m->flex_stiffness)
void mjd_flexInterp_cacheKrot(const mjModel* m, mjData* d, mjtNum* K_rot_out) {
  // use s1=1, s2=0 so scale=1 and K_rot_out gets unscaled values
  mjd_flexInterp_kernel(m, d, NULL, NULL, 1, 0, NULL, K_rot_out);
}



// compute res += scale * K_bend * vec for standard (non-interp) flex bending
//   scale = s1 + s2 * flex_damping[f]  per flex
//   for stiffness+damping: s1=h^2, s2=h  =>  scale = h^2 + h*damping
//   for stiffness only:    s1=h,   s2=0  =>  scale = h
void mjd_flexBend_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                      mjtNum s1, mjtNum s2) {
  for (int f = 0; f < m->nflex; f++) {
    // skip interp, rigid, or non-2D
    if (m->flex_interp[f] || m->flex_rigid[f] || m->flex_dim[f] != 2) {
      continue;
    }

    int bendingadr = m->flex_bendingadr[f];
    if (bendingadr < 0) {
      continue;
    }

    mjtNum scale = s1 + s2 * m->flex_damping[f];
    if (!scale) {
      continue;
    }

    const mjtNum* b = m->flex_bending + bendingadr;
    const int* bodyid = m->flex_vertbodyid + m->flex_vertadr[f];
    int edgenum = m->flex_edgenum[f];
    int edgeadr = m->flex_edgeadr[f];

    for (int e = 0; e < edgenum; e++) {
      const int* edge = m->flex_edge + 2*(e + edgeadr);
      const int* flap = m->flex_edgeflap + 2*(e + edgeadr);
      int v[4] = {edge[0], edge[1], flap[0], flap[1]};

      // skip boundary edges (no second flap vertex)
      if (v[3] == -1) {
        continue;
      }

      // apply 4x4 bending stencil, coordinate-wise
      for (int i = 0; i < 4; i++) {
        int dof_i = m->body_dofadr[bodyid[v[i]]];
        for (int x = 0; x < 3; x++) {
          mjtNum val = 0;
          for (int j = 0; j < 4; j++) {
            int dof_j = m->body_dofadr[bodyid[v[j]]];
            val += b[17*e + 4*i + j] * vec[dof_j + x];
          }
          res[dof_i + x] += scale * val;
        }
      }
    }
  }
}


// local edge-based vertex indexing for 2D and 3D elements (mirrors engine_passive.c: the
// flex_stiffness metric ordering is tied to this edge order)
static const int stretch_edges[2][6][2] = {
  {{1, 2}, {2, 0}, {0, 1}, {0, 0}, {0, 0}, {0, 0}},
  {{0, 1}, {1, 2}, {2, 0}, {2, 3}, {0, 3}, {1, 3}}};

// compute res += (s1 + s2*flex_damping) * K_stretch * vec for standard (non-interp) flex
// stretch, where K_stretch is the Gauss-Newton Hessian of the passive stretch force in
// mj_flexPassiveStretch: with elongation e_a = L_a^2 - L0_a^2 and force
// f = -sum_ab M_ab e_a grad(e_b)/2, the GN Hessian is K = 2 sum_ab M_ab (s_a d_a)(s_b d_b)^T,
// d_a the current edge vector. Pinned vertices (zero-dof bodies) contribute nothing.
void mjd_flexStretch_mul(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec,
                         mjtNum s1, mjtNum s2) {
  for (int f = 0; f < m->nflex; f++) {
    // skip interp, rigid, or 1D
    if (m->flex_interp[f] || m->flex_rigid[f] || m->flex_dim[f] < 2) {
      continue;
    }

    int stiffnessadr = m->flex_stiffnessadr[f];
    if (stiffnessadr < 0 || m->flex_stiffness[stiffnessadr] == 0) {
      continue;
    }

    mjtNum scale = s1 + s2 * m->flex_damping[f];
    if (!scale) {
      continue;
    }

    int dim = m->flex_dim[f];
    int nedge = (dim == 2) ? 3 : 6;
    const int (*edge)[2] = stretch_edges[dim-2];
    const int* elem = m->flex_elem + m->flex_elemdataadr[f];
    const mjtNum* xpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
    const int* bodyid = m->flex_vertbodyid + m->flex_vertadr[f];
    const mjtNum* k = m->flex_stiffness + stiffnessadr;
    int elemnum = m->flex_elemnum[f];

    for (int t = 0; t < elemnum; t++) {
      const int* vert = elem + (dim+1)*t;

      // current edge vectors and g_a = d_a . (vec_{a0} - vec_{a1}), zero on pinned vertices
      mjtNum dvec[6][3];
      mjtNum g[6];
      for (int e = 0; e < nedge; e++) {
        int v0 = vert[edge[e][0]], v1 = vert[edge[e][1]];
        int b0 = bodyid[v0],       b1 = bodyid[v1];
        g[e] = 0;
        for (int x = 0; x < 3; x++) {
          dvec[e][x] = xpos[3*v0+x] - xpos[3*v1+x];
          mjtNum dv = 0;
          if (m->body_dofnum[b0]) dv += vec[m->body_dofadr[b0]+x];
          if (m->body_dofnum[b1]) dv -= vec[m->body_dofadr[b1]+x];
          g[e] += dvec[e][x]*dv;
        }
      }

      // unpack upper triangular metric (21 elements, matching mj_flexPassiveStretch)
      mjtNum metric[36];
      int id = 0;

      for (int e1 = 0; e1 < nedge; e1++) {
        for (int e2 = e1; e2 < nedge; e2++) {
          metric[nedge*e1 + e2] = k[21*t + id];
          metric[nedge*e2 + e1] = k[21*t + id++];
        }
      }

      // scatter: res_{b0/b1} +/-= 2*scale*(sum_a M_ba g_a) * d_b
      for (int e = 0; e < nedge; e++) {
        mjtNum coef = 0;
        for (int a = 0; a < nedge; a++) {
          coef += metric[nedge*e + a]*g[a];
        }
        coef *= 2*scale;
        int b0 = bodyid[vert[edge[e][0]]], b1 = bodyid[vert[edge[e][1]]];
        for (int x = 0; x < 3; x++) {
          if (m->body_dofnum[b0]) res[m->body_dofadr[b0]+x] += coef*dvec[e][x];
          if (m->body_dofnum[b1]) res[m->body_dofadr[b1]+x] -= coef*dvec[e][x];
        }
      }
    }
  }
}


// returns true if the interpolated flex is processed by mjd_flexInterp_mul
static mjtBool flexInterp_processed(const mjModel* m, int f) {
  if (!m->flex_interp[f]) {
    return 0;
  }
  int sa = m->flex_stiffnessadr[f];
  if (sa < 0 || m->flex_rigid[f] || m->flex_stiffness[sa] == 0) {
    return 0;
  }
  if (m->flex_edgeequality[f] == 3) {
    return 0;
  }
  return 1;
}


// can ALL operator-processed interp flexes be assembled to dof-level CSR? Requires every node
// to sit on a simple slider body (body_simple == 2: sliders only, so the point Jacobian is I3
// regardless of the node offset -- flex_centered is NOT required) or to be fixed (0 dofs, row
// dropped like a pinned vertex). All-or-nothing: mjd_flexInterp_mul applies every interp flex,
// so the CSR can replace the operator only if it covers them all.
mjtBool mjd_flexInterpAssemblable(const mjModel* m) {
  for (int f = 0; f < m->nflex; f++) {
    if (!flexInterp_processed(m, f)) {
      continue;
    }
    const int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];
    for (int n = 0; n < m->flex_nodenum[f]; n++) {
      int dofnum = m->body_dofnum[bodyid[n]];
      if (dofnum == 0) {
        continue;
      }
      if (m->body_simple[bodyid[n]] != 2 || dofnum != 3) {
        return 0;
      }
    }
  }
  return 1;
}


// element walk shared by the interp assembly passes: geometry of interp finite elements
// (mirrors mjd_flexInterp_kernel; index-only gather calls)
#define FLEXINTERP_WALK(f, body)                                             \
  {                                                                          \
    int order_ = m->flex_interp[f];                                          \
    int shell_ = order_ < 0;                                                 \
    order_ = order_ < 0 ? -order_ : order_;                                  \
    int cx_ = m->flex_cellnum[3 * (f) + 0];                                  \
    int cy_ = m->flex_cellnum[3 * (f) + 1];                                  \
    int cz_ = m->flex_cellnum[3 * (f) + 2];                                  \
    int npe = shell_ ? (order_ + 1) * (order_ + 1)                           \
                     : (order_ + 1) * (order_ + 1) * (order_ + 1);           \
    int nelem_fe_ =                                                          \
        shell_ ? 2 * (cy_ * cz_ + cx_ * cz_ + cx_ * cy_) : cx_ * cy_ * cz_;  \
    const mjtNum* K_ = m->flex_stiffness + m->flex_stiffnessadr[f];          \
    int gindices[125];                                                       \
    for (int fe = 0; fe < nelem_fe_; fe++) {                                 \
      if (K_[(size_t)fe * 3 * npe * 3 * npe] == 0) continue;                 \
      if (shell_) {                                                          \
        mju_flexGatherFaceState(order_, cx_, cy_, cz_, fe, NULL, NULL, NULL, \
                                NULL, NULL, NULL, gindices, NULL);           \
      } else {                                                               \
        int ci_ = fe / (cy_ * cz_), cj_ = (fe / cz_) % cy_, ck_ = fe % cz_;  \
        mju_flexGatherCellState(order_, cy_, cz_, ci_, cj_, ck_, NULL, NULL, \
                                NULL, NULL, NULL, NULL, gindices, NULL);     \
      }                                                                      \
      body                                                                   \
    }                                                                        \
  }

// does ANY flex contribute assemblable implicit stiffness? (cheap existence check for the
// solver gate: stretch stiffness on a standard flex, or -- when Krot will be supplied -- an
// operator-processed interp flex)
mjtBool mjd_flexStiff_any(const mjModel* m, int flg_interp) {
  for (int f = 0; f < m->nflex; f++) {
    if (flg_interp && flexInterp_processed(m, f)) {
      return 1;
    }
    if (!m->flex_interp[f] && !m->flex_rigid[f] && m->flex_dim[f] >= 2 &&
        m->flex_stiffnessadr[f] >= 0 && m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      return 1;
    }
  }
  return 0;
}


// does this standard flex contribute implicit stiffness under the given term flags?
static mjtBool flexStiff_active(const mjModel* m, int f, int flg_bend, int flg_stretch) {
  if (m->flex_interp[f] || m->flex_rigid[f] || m->flex_dim[f] < 2) {
    return 0;
  }
  int bend = flg_bend && m->flex_bendingadr[f] >= 0;
  int stretch = flg_stretch && m->flex_stiffnessadr[f] >= 0 &&
                m->flex_stiffness[m->flex_stiffnessadr[f]] != 0;
  return bend || stretch;
}


// assemble the standard-flex implicit stiffness K = (s1 + s2*damping) * (K_bend + K_stretch)
// into dof-level CSR (same terms mjd_flexBend_mul / mjd_flexStretch_mul apply matrix-free; the
// matrix is constant during a solve, so assembling once and applying as a sparse matvec avoids
// re-walking stencils and re-unpacking metrics on every apply). Rows/columns exist only on the
// dofs of unpinned vertices of standard dim>=2 flexes with bending or stretch stiffness.
// Phase 1 (colind == NULL): fill rownnz/rowadr over nv, return total nnz.
// Phase 2: fill colind and val (rownnz/rowadr must come from phase 1).
// Interp flexes are assembled iff Krot (the mjd_flexInterp_cacheKrot cache) is non-NULL and
// they qualify for the centered fast path (caller checks mjd_flexInterpAssemblable): rows on
// node body dofs, corotated 3x3 blocks, with the operator's NEGATED sign convention folded in
// so one CSR replaces all three matrix-free operators uniformly.
int mjd_flexStiff_assemble(const mjModel* m, mjData* d, int* rownnz, int* rowadr,
                           int* colind, mjtNum* val, mjtNum s1, mjtNum s2,
                           int flg_bend, int flg_stretch, const mjtNum* Krot) {
  int nv = m->nv;
  mj_markStack(d);

  // collect participating vertices: global flex vertex id -> local slot, dofadr
  int nvert = 0;
  int* vslot = mjSTACKALLOC(d, m->nflexvert > 0 ? m->nflexvert : 1, int);
  for (int i = 0; i < m->nflexvert; i++) {
    vslot[i] = -1;
  }
  for (int f = 0; f < m->nflex; f++) {
    if (!flexStiff_active(m, f, flg_bend, flg_stretch)) {
      continue;
    }
    for (int lv = 0; lv < m->flex_vertnum[f]; lv++) {
      int gv = m->flex_vertadr[f] + lv;
      if (m->body_dofnum[m->flex_vertbodyid[gv]] == 3) {
        vslot[gv] = nvert++;
      }
    }
  }

  // interp nodes participate when the caller supplies the K_rot cache (centered fast path)
  int* nslot = mjSTACKALLOC(d, m->nflexnode > 0 ? (int)m->nflexnode : 1, int);
  for (int i = 0; i < m->nflexnode; i++) {
    nslot[i] = -1;
  }
  if (Krot) {
    for (int f = 0; f < m->nflex; f++) {
      if (!flexInterp_processed(m, f)) {
        continue;
      }
      const int* bodyid = m->flex_nodebodyid + m->flex_nodeadr[f];
      for (int ln = 0; ln < m->flex_nodenum[f]; ln++) {
        if (m->body_dofnum[bodyid[ln]] == 3) {
          nslot[m->flex_nodeadr[f] + ln] = nvert++;
        }
      }
    }
  }

  if (!nvert) {
    mju_zeroInt(rownnz, nv);
    if (rowadr) mju_zeroInt(rowadr, nv);
    mj_freeStack(d);
    return 0;
  }
  int* vdof = mjSTACKALLOC(d, nvert, int);
  for (int gv = 0; gv < m->nflexvert; gv++) {
    if (vslot[gv] >= 0) {
      vdof[vslot[gv]] = m->body_dofadr[m->flex_vertbodyid[gv]];
    }
  }
  for (int gn = 0; gn < m->nflexnode; gn++) {
    if (nslot[gn] >= 0) {
      vdof[nslot[gn]] = m->body_dofadr[m->flex_nodebodyid[gn]];
    }
  }

  // count stencil incidence per vertex slot (with duplicates)
  int* ncand = mjSTACKALLOC(d, nvert, int);
  mju_zeroInt(ncand, nvert);
  for (int f = 0; f < m->nflex; f++) {
    if (!flexStiff_active(m, f, flg_bend, flg_stretch)) {
      continue;
    }
    int dim = m->flex_dim[f], nvrt = dim + 1;
    // bending stencils: 4-vertex flaps per edge
    if (flg_bend && m->flex_bendingadr[f] >= 0) {
      for (int e = 0; e < m->flex_edgenum[f]; e++) {
        const int* edge = m->flex_edge + 2*(e + m->flex_edgeadr[f]);
        const int* flap = m->flex_edgeflap + 2*(e + m->flex_edgeadr[f]);
        if (flap[1] == -1) continue;
        int v[4] = {edge[0], edge[1], flap[0], flap[1]};
        for (int i = 0; i < 4; i++) {
          int si = vslot[m->flex_vertadr[f] + v[i]];
          if (si >= 0) ncand[si] += 4;
        }
      }
    }
    // stretch elements: (dim+1)-vertex cliques
    if (flg_stretch && m->flex_stiffnessadr[f] >= 0 &&
        m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      const int* elem = m->flex_elem + m->flex_elemdataadr[f];
      for (int t = 0; t < m->flex_elemnum[f]; t++) {
        for (int i = 0; i < nvrt; i++) {
          int si = vslot[m->flex_vertadr[f] + elem[(dim+1)*t + i]];
          if (si >= 0) ncand[si] += nvrt;
        }
      }
    }
  }

  // interp stencils: npe-node element cliques (counting)
  if (Krot) {
    for (int f = 0; f < m->nflex; f++) {
      if (!flexInterp_processed(m, f)) {
        continue;
      }
      FLEXINTERP_WALK(f, {
        for (int i = 0; i < npe; i++) {
          int si = nslot[m->flex_nodeadr[f] + gindices[i]];
          if (si >= 0) ncand[si] += npe;
        }
      })
    }
  }

  // gather candidate neighbor lists (vertex slots, with duplicates)
  int* cadr = mjSTACKALLOC(d, nvert + 1, int);
  cadr[0] = 0;
  for (int s = 0; s < nvert; s++) {
    cadr[s+1] = cadr[s] + ncand[s];
  }
  int* cand = mjSTACKALLOC(d, cadr[nvert] > 0 ? cadr[nvert] : 1, int);
  mju_zeroInt(ncand, nvert);
  for (int f = 0; f < m->nflex; f++) {
    if (!flexStiff_active(m, f, flg_bend, flg_stretch)) {
      continue;
    }
    int dim = m->flex_dim[f], nvrt = dim + 1;
    if (flg_bend && m->flex_bendingadr[f] >= 0) {
      for (int e = 0; e < m->flex_edgenum[f]; e++) {
        const int* edge = m->flex_edge + 2*(e + m->flex_edgeadr[f]);
        const int* flap = m->flex_edgeflap + 2*(e + m->flex_edgeadr[f]);
        if (flap[1] == -1) continue;
        int v[4] = {edge[0], edge[1], flap[0], flap[1]};
        for (int i = 0; i < 4; i++) {
          int si = vslot[m->flex_vertadr[f] + v[i]];
          if (si < 0) continue;
          for (int j = 0; j < 4; j++) {
            int sj = vslot[m->flex_vertadr[f] + v[j]];
            if (sj >= 0) cand[cadr[si] + ncand[si]++] = sj;
          }
        }
      }
    }
    if (flg_stretch && m->flex_stiffnessadr[f] >= 0 &&
        m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      const int* elem = m->flex_elem + m->flex_elemdataadr[f];
      for (int t = 0; t < m->flex_elemnum[f]; t++) {
        const int* vert = elem + (dim+1)*t;
        for (int i = 0; i < nvrt; i++) {
          int si = vslot[m->flex_vertadr[f] + vert[i]];
          if (si < 0) continue;
          for (int j = 0; j < nvrt; j++) {
            int sj = vslot[m->flex_vertadr[f] + vert[j]];
            if (sj >= 0) cand[cadr[si] + ncand[si]++] = sj;
          }
        }
      }
    }
  }

  // interp stencils: npe-node element cliques (filling)
  if (Krot) {
    for (int f = 0; f < m->nflex; f++) {
      if (!flexInterp_processed(m, f)) {
        continue;
      }
      FLEXINTERP_WALK(f, {
        for (int i = 0; i < npe; i++) {
          int si = nslot[m->flex_nodeadr[f] + gindices[i]];
          if (si < 0) continue;
          for (int j = 0; j < npe; j++) {
            int sj = nslot[m->flex_nodeadr[f] + gindices[j]];
            if (sj >= 0) cand[cadr[si] + ncand[si]++] = sj;
          }
        }
      })
    }
  }

  // per vertex: sort by neighbor dofadr, unique -> neighbor lists
  int* nadr = mjSTACKALLOC(d, nvert + 1, int);
  int* neigh = mjSTACKALLOC(d, cadr[nvert] > 0 ? cadr[nvert] : 1, int);
  nadr[0] = 0;
  for (int s = 0; s < nvert; s++) {
    int* c = cand + cadr[s];
    int n = ncand[s];
    // insertion sort by dof address (stencil-local lists are short)
    for (int i = 1; i < n; i++) {
      int key = c[i], kd = vdof[key], j = i - 1;
      while (j >= 0 && vdof[c[j]] > kd) {
        c[j+1] = c[j];
        j--;
      }
      c[j+1] = key;
    }
    int nn = 0;
    for (int i = 0; i < n; i++) {
      if (nn == 0 || neigh[nadr[s] + nn - 1] != c[i]) {
        neigh[nadr[s] + nn++] = c[i];
      }
    }
    nadr[s+1] = nadr[s] + nn;
  }

  // dof-level CSR structure: each vertex contributes 3 rows of 3*nneigh entries
  mju_zeroInt(rownnz, nv);
  int nnz = 0;
  for (int s = 0; s < nvert; s++) {
    int nn = nadr[s+1] - nadr[s];
    for (int k = 0; k < 3; k++) {
      rownnz[vdof[s] + k] = 3*nn;
    }
    nnz += 9*nn;
  }
  if (rowadr) {
    rowadr[0] = 0;
    for (int i = 1; i < nv; i++) {
      rowadr[i] = rowadr[i-1] + rownnz[i-1];
    }
  }

  // phase 1: structure only
  if (!colind) {
    mj_freeStack(d);
    return nnz;
  }

  // column indices (per vertex row: the 3 dofs of each neighbor, neighbor-dof-sorted)
  for (int s = 0; s < nvert; s++) {
    for (int k = 0; k < 3; k++) {
      int adr = rowadr[vdof[s] + k];
      for (int j = nadr[s]; j < nadr[s+1]; j++) {
        for (int c = 0; c < 3; c++) {
          colind[adr++] = vdof[neigh[j]] + c;
        }
      }
    }
  }
  mju_zero(val, nnz);

// block accumulation helper data: find neighbor position by binary search on dofadr
#define FLEXSTIFF_BLOCK(si, sj, pos)          \
  {                                           \
    int lo = nadr[si], hi = nadr[si + 1] - 1; \
    pos = -1;                                 \
    while (lo <= hi) {                        \
      int mid = (lo + hi) / 2;                \
      if (vdof[neigh[mid]] == vdof[sj]) {     \
        pos = mid - nadr[si];                 \
        break;                                \
      }                                       \
      if (vdof[neigh[mid]] < vdof[sj])        \
        lo = mid + 1;                         \
      else                                    \
        hi = mid - 1;                         \
    }                                         \
  }

  // values: bending (Q_ij * I3 per 4-vertex stencil) and stretch (GN blocks per element)
  for (int f = 0; f < m->nflex; f++) {
    if (!flexStiff_active(m, f, flg_bend, flg_stretch)) {
      continue;
    }
    mjtNum scale = s1 + s2*m->flex_damping[f];
    if (!scale) {
      continue;
    }
    int dim = m->flex_dim[f], nvrt = dim + 1;
    int nedge = (dim == 2) ? 3 : 6;
    const int (*edget)[2] = stretch_edges[dim-2];

    if (flg_bend && m->flex_bendingadr[f] >= 0) {
      const mjtNum* b = m->flex_bending + m->flex_bendingadr[f];
      for (int e = 0; e < m->flex_edgenum[f]; e++) {
        const int* edge = m->flex_edge + 2*(e + m->flex_edgeadr[f]);
        const int* flap = m->flex_edgeflap + 2*(e + m->flex_edgeadr[f]);
        if (flap[1] == -1) continue;
        int v[4] = {edge[0], edge[1], flap[0], flap[1]};
        for (int i = 0; i < 4; i++) {
          int si = vslot[m->flex_vertadr[f] + v[i]];
          if (si < 0) continue;
          for (int j = 0; j < 4; j++) {
            int sj = vslot[m->flex_vertadr[f] + v[j]];
            if (sj < 0) continue;
            mjtNum q = scale*b[17*e + 4*i + j];
            if (!q) continue;
            int pos;
            FLEXSTIFF_BLOCK(si, sj, pos);
            for (int k = 0; k < 3; k++) {
              val[rowadr[vdof[si] + k] + 3*pos + k] += q;
            }
          }
        }
      }
    }

    if (flg_stretch && m->flex_stiffnessadr[f] >= 0 &&
        m->flex_stiffness[m->flex_stiffnessadr[f]] != 0) {
      const int* elem = m->flex_elem + m->flex_elemdataadr[f];
      const mjtNum* xpos = d->flexvert_xpos + 3*m->flex_vertadr[f];
      const mjtNum* kk = m->flex_stiffness + m->flex_stiffnessadr[f];
      for (int t = 0; t < m->flex_elemnum[f]; t++) {
        const int* vert = elem + (dim+1)*t;

        // current edge vectors
        mjtNum dvec[6][3];
        for (int e = 0; e < nedge; e++) {
          int v0 = vert[edget[e][0]], v1 = vert[edget[e][1]];
          for (int x = 0; x < 3; x++) {
            dvec[e][x] = xpos[3*v0+x] - xpos[3*v1+x];
          }
        }

        // unpack triangular metric
        mjtNum metric[36];
        int id = 0;
        for (int e1 = 0; e1 < nedge; e1++) {
          for (int e2 = e1; e2 < nedge; e2++) {
            metric[nedge*e1 + e2] = kk[21*t + id];
            metric[nedge*e2 + e1] = kk[21*t + id++];
          }
        }

        // per vertex pair: block += 2*scale * sum_ab M_ab s_a,vi s_b,vj d_a d_b^T
        for (int i = 0; i < nvrt; i++) {
          int si = vslot[m->flex_vertadr[f] + vert[i]];
          if (si < 0) continue;
          for (int j = 0; j < nvrt; j++) {
            int sj = vslot[m->flex_vertadr[f] + vert[j]];
            if (sj < 0) continue;
            mjtNum blk[9] = {0};
            for (int a = 0; a < nedge; a++) {
              mjtNum sa = (i == edget[a][0]) ? 1 : ((i == edget[a][1]) ? -1 : 0);
              if (!sa) continue;
              for (int bb = 0; bb < nedge; bb++) {
                mjtNum sb = (j == edget[bb][0]) ? 1 : ((j == edget[bb][1]) ? -1 : 0);
                if (!sb) continue;
                mjtNum w = 2*scale*metric[nedge*a + bb]*sa*sb;
                for (int r = 0; r < 3; r++) {
                  for (int c = 0; c < 3; c++) {
                    blk[3*r+c] += w*dvec[a][r]*dvec[bb][c];
                  }
                }
              }
            }
            int pos;
            FLEXSTIFF_BLOCK(si, sj, pos);
            for (int k = 0; k < 3; k++) {
              for (int c = 0; c < 3; c++) {
                val[rowadr[vdof[si] + k] + 3*pos + c] += blk[3*k+c];
              }
            }
          }
        }
      }
    }
  }

  // values: interp (corotated K_rot 3x3 node blocks per element). The interp operator's sign
  // convention is opposite to bend/stretch -- FlexBmulAdd calls it with negated scales -- so
  // the negation is folded in here and one CSR replaces all three operators uniformly.
  if (Krot) {
    for (int f = 0; f < m->nflex; f++) {
      if (!flexInterp_processed(m, f)) {
        continue;
      }
      mjtNum iscale = -(s1 + s2*m->flex_damping[f]);
      if (!iscale) {
        continue;
      }
      FLEXINTERP_WALK(f, {
        int dim_e = 3*npe;
        const mjtNum* kb = Krot + m->flex_stiffnessadr[f] + (size_t)fe*dim_e*dim_e;
        for (int i = 0; i < npe; i++) {
          int si = nslot[m->flex_nodeadr[f] + gindices[i]];
          if (si < 0) continue;
          int bi = m->flex_nodebodyid[m->flex_nodeadr[f] + gindices[i]];
          for (int j = 0; j < npe; j++) {
            int sj = nslot[m->flex_nodeadr[f] + gindices[j]];
            if (sj < 0) continue;
            int bj = m->flex_nodebodyid[m->flex_nodeadr[f] + gindices[j]];
            // extract K_rot 3x3 block for (i,j)
            mjtNum kb_ij[9];
            for (int r = 0; r < 3; r++) {
              for (int c = 0; c < 3; c++) {
                kb_ij[3*r+c] = iscale*kb[(3*i + r)*dim_e + 3*j + c];
              }
            }
            // apply R_bi^T * kb_ij * R_bj to transform to DOF space
            mjtNum tmp[9];
            mjtNum blk[9];
            mji_mulMatMat3(tmp, kb_ij, d->xmat + 9*bj);     // tmp = kb * R_bj
            mji_mulMatTMat3(blk, d->xmat + 9*bi, tmp);      // blk = R_bi^T * tmp
            int pos;
            FLEXSTIFF_BLOCK(si, sj, pos);
            for (int r = 0; r < 3; r++) {
              for (int c = 0; c < 3; c++) {
                val[rowadr[vdof[si] + r] + 3*pos + c] += blk[3*r+c];
              }
            }
          }
        }
      })
    }
  }
  #undef FLEXSTIFF_BLOCK
  #undef FLEXINTERP_WALK

  mj_freeStack(d);
  return nnz;
}





// add (d qfrc_actuator / d qvel) to qDeriv
void mjd_actuator_vel(const mjModel* m, mjData* d) {
  int nactuator = m->nactuator;
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_ACTUATION)) {
    return;
  }

  // process actuators
  for (int i=0; i < nactuator; i++) {
    int uadr = m->actuator_ctrladr[i];
    int oadr = m->actuator_outadr[i];

    // skip if disabled
    if (mj_actuatorDisabled(m, i)) {
      continue;
    }

    // skip if sleeping
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_ACTUATOR, i) == mjS_ASLEEP) {
      continue;
    }

    // skip if force is clamped by forcerange
    if (m->actuator_forcelimited[i]) {
      const mjtNum* range = m->actuator_forcerange + 2*i;

      // SO3: force is norm-clamped (approximation: saturated force still varies tangentially)
      if (m->actuator_gaintype[i] == mjGAIN_SO3) {
        if (mju_norm3(d->actuator_force + oadr) >= range[1]) {
          continue;
        }
      } else {
        mjtNum force = d->actuator_force[oadr];
        if (force <= range[0] || force >= range[1]) {
          continue;
        }
      }
    }

    mjtNum bias_vel = 0, gain_vel = 0;

    // affine bias
    if (m->actuator_biastype[i] == mjBIAS_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      bias_vel = (m->actuator_biasprm + mjNBIAS*i)[2];
    }

    // SO3 geodesic servo: kv term, applied to each output row below
    else if (m->actuator_biastype[i] == mjBIAS_SO3) {
      bias_vel = (m->actuator_biasprm + mjNBIAS*i)[2];
    }

    // DC motor bias (back-EMF)
    else if (m->actuator_biastype[i] == mjBIAS_DCMOTOR) {
      const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
      const mjtNum* gainprm = m->actuator_gainprm + mjNGAIN*i;
      if (dynprm[0] <= 0) {
        mjtNum R = mju_max(mjMINVAL, gainprm[0]);
        mjtNum K = gainprm[1];
        bias_vel -= K * K / R;
      }
    }

    // affine gain
    if (m->actuator_gaintype[i] == mjGAIN_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      gain_vel = (m->actuator_gainprm + mjNGAIN*i)[2];
    }

    // muscle gain
    else if (m->actuator_gaintype[i] == mjGAIN_MUSCLE) {
      gain_vel = mjd_muscleGain_vel(d->actuator_length[oadr],
                                    d->actuator_velocity[oadr],
                                    m->actuator_lengthrange+2*oadr,
                                    m->actuator_acc0[oadr],
                                    m->actuator_gainprm + mjNGAIN*i);
    }

    // DC motor controller damping and LuGre micro-damping
    else if (m->actuator_gaintype[i] == mjGAIN_DCMOTOR) {
      const mjtNum* dynprm = m->actuator_dynprm + mjNDYN*i;
      const mjtNum* gainprm = m->actuator_gainprm + mjNGAIN*i;
      mjtNum te = dynprm[0];

      // controller velocity derivative: dV/dω
      int input_mode = (int)gainprm[8];
      mjtNum dVdw = 0;
      if (input_mode == 1) dVdw = -gainprm[6];       // position: -kd
      else if (input_mode == 2) dVdw = -gainprm[4];   // velocity: -kp

      if (te > 0) {
        // stateful current with actearly: d(K*next_act)/dω
        // includes both back-EMF (-K) and controller (dVdw) through act_dot
        mjtNum R = mju_max(mjMINVAL, gainprm[0]);
        mjtNum K = gainprm[1];
        mjtNum s = 1 - mju_exp(-m->opt.timestep / te);
        bias_vel += K * (dVdw - K) * s / R;
      } else if (dVdw != 0) {
        // stateless: controller terms only (back-EMF handled in bias block)
        mjtNum R = mju_max(mjMINVAL, gainprm[0]);
        mjtNum K = gainprm[1];
        bias_vel += K * dVdw / R;
      }

      // LuGre: force includes -sigma1*z_dot, z_dot = a*z + v
      // d(sigma1*z_dot)/dv = sigma1*(da/dv*z + 1), ignoring higher-order da/dv*z
      mjtNum sigma1 = dynprm[6];
      if (sigma1 > 0) {
        bias_vel -= sigma1;
      }
    }

    // force = gain .* [ctrl/act]
    if (gain_vel != 0) {
      if (m->actuator_dyntype[i] == mjDYN_NONE) {
        bias_vel += gain_vel * d->ctrl[uadr];
      } else {
        int act_adr = m->actuator_actadr[i] + m->actuator_actnum[i] - 1;
        mjtNum act = d->act[act_adr];

        // use next activation if actearly is set (matching forward pass)
        if (m->actuator_actearly[i]) {
          act = mj_nextActivation(m, d, i, act_adr, d->act_dot[act_adr]);
        }

        bias_vel += gain_vel * act;
      }
    }

    // add, once per output row
    if (bias_vel != 0) {
      for (int k=0; k < m->actuator_outnum[i]; k++) {
        addJTBJSparse(m, d, d->actuator_moment, &bias_vel, 1, oadr+k,
                      d->moment_rownnz, d->moment_rowadr, d->moment_colind);
      }
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
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_inertia[i % 3];
  }
  addToQuadrant(B, Da, 0, 0);

  // force[:3] += cross(virtual_lin_mom, lin_vel)
  mjd_cross(virtual_lin_mom, lin_vel, Da, Db);
  addToQuadrant(B, Db, 0, 1);
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 0, 1);

  // force[3:] += cross(virtual_lin_mom, ang_vel)
  mjd_cross(virtual_lin_mom, ang_vel, Da, Db);
  addToQuadrant(B, Db, 1, 0);
  for (int i=0; i < 9; ++i) {
    Da[i] *= fluid_density * virtual_mass[i % 3];
  }
  addToQuadrant(B, Da, 1, 1);
}


// torque due to motion in the fluid, D is 3x3
static inline void mjd_viscous_torque(
  mjtNum* restrict D, const mjtNum lvel[6], const mjtNum fluid_density,
  const mjtNum fluid_viscosity, const mjtNum size[3],
  const mjtNum slender_drag_coef, const mjtNum ang_drag_coef) {
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
    magnus_coef * lvel[3], magnus_coef * lvel[4], magnus_coef * lvel[5]
  };
  const mjtNum ang_vel[3] = {
    magnus_coef * lvel[0], magnus_coef * lvel[1], magnus_coef * lvel[2]
  };

  // force[3:] += magnus_coef * cross(ang_vel, lin_vel)
  mjd_cross(ang_vel, lin_vel, D_ang, D_lin);

  addToQuadrant(B, D_ang, 1, 0);
  addToQuadrant(B, D_lin, 1, 1);
}


//----------------- fluid force derivatives, ellipsoid and inertia-box models ----------------------

// fluid forces based on ellipsoid approximation
void mjd_ellipsoidFluid(const mjModel* m, mjData* d, int bodyid) {
  mj_markStack(d);

  int nv = m->nv;
  int nnz = nv;
  int rownnz[6], rowadr[6];
  mjtNum* J = mjSTACKALLOC(d, 6*nv, mjtNum);
  mjtNum* tmp = mjSTACKALLOC(d, 3*nv, mjtNum);
  int* colind = mjSTACKALLOC(d, 6*nv, int);
  int* colind_compressed = mjSTACKALLOC(d, 6*nv, int);

  mjtNum lvel[6], wind[6], lwind[6];
  mjtNum geom_interaction_coef, magnus_lift_coef, kutta_lift_coef;
  mjtNum semiaxes[3], virtual_mass[3], virtual_inertia[3];
  mjtNum blunt_drag_coef, slender_drag_coef, ang_drag_coef;

  if (mj_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = mj_bodyChain(m, bodyid, colind);

    // prepare rownnz, rowadr, colind for all 6 rows
    for (int i=0; i < 6; i++) {
      rownnz[i] = nnz;
      rowadr[i] = i == 0 ? 0 : rowadr[i-1] + nnz;
      for (int k=0; k < nnz; k++) {
        colind_compressed[i*nnz+k] = colind[k];
      }
    }
  }

  for (int j=0; j < m->body_geomnum[bodyid]; j++) {
    const int geomid = m->body_geomadr[bodyid] + j;

    mju_geomSemiAxes(semiaxes, m->geom_size + 3*geomid, m->geom_type[geomid]);

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

    // get geom global Jacobian: rotation then translation
    if (mj_isSparse(m)) {
      mj_jacSparse(m, d, J+3*nnz, J, d->geom_xpos+3*geomid, m->geom_bodyid[geomid], nnz, colind,
                   /*flg_skipcommon=*/0);
    } else {
      mj_jacGeom(m, d, J+3*nv, J, geomid);
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

    // make B symmetric if integrator is IMPLICITFAST, except for standalone free bodies
    if (m->opt.integrator == mjINT_IMPLICITFAST && !mj_isFreeBody(m, bodyid)) {
      mju_symmetrize(B, B, 6);
    }

    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, B, 6, 0, rownnz, rowadr, colind_compressed);
    } else {
      addJTBJ(m, d, J, B, 6);
    }
  }

  mj_freeStack(d);
}


// fluid forces based on inertia-box approximation
void mjd_inertiaBoxFluid(const mjModel* m, mjData* d, int i) {
  mj_markStack(d);

  int nv = m->nv;
  int rownnz[6], rowadr[6];
  mjtNum* J = mjSTACKALLOC(d, 6*nv, mjtNum);
  mjtNum* tmp = mjSTACKALLOC(d, 3*nv, mjtNum);
  int* colind = mjSTACKALLOC(d, 6*nv, int);

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

  // init with dense
  int nnz = nv;

  // sparse Jacobian
  if (mj_isSparse(m)) {
    // get sparse body Jacobian structure
    nnz = mj_bodyChain(m, i, colind);

    // get sparse jacBodyCom
    mj_jacSparse(m, d, J+3*nnz, J, d->xipos+3*i, i, nnz, colind, /*flg_skipcommon=*/0);

    // prepare rownnz, rowadr, colind for all 6 rows
    rownnz[0] = nnz;
    rowadr[0] = 0;
    for (int j=1; j < 6; j++) {
      rownnz[j] = nnz;
      rowadr[j] = rowadr[j-1] + nnz;
      for (int k=0; k < nnz; k++) {
        colind[j*nnz+k] = colind[k];
      }
    }
  }

  // dense Jacobian
  else {
    mj_jacBodyCom(m, d, J+3*nv, J, i);
  }

  // rotate (compressed) Jacobian to local frame
  mju_mulMatTMat(tmp, d->ximat+9*i, J, 3, 3, nnz);
  mju_copy(J, tmp, 3*nnz);
  mju_mulMatTMat(tmp, d->ximat+9*i, J+3*nnz, 3, 3, nnz);
  mju_copy(J+3*nnz, tmp, 3*nnz);

  // add viscous force and torque
  if (m->opt.viscosity > 0) {
    // diameter of sphere approximation
    mjtNum diam = (box[0] + box[1] + box[2])/3.0;

    // mju_scl3(lfrc, lvel, -mjPI*diam*diam*diam*m->opt.viscosity)
    B = -mjPI*diam*diam*diam*m->opt.viscosity;
    for (int j=0; j < 3; j++) {
      if (mj_isSparse(m)) {
        addJTBJSparse(m, d, J, &B, 1, j, rownnz, rowadr, colind);
      } else {
        addJTBJ(m, d, J+j*nv, &B, 1);
      }
    }

    // mju_scl3(lfrc+3, lvel+3, -3.0*mjPI*diam*m->opt.viscosity);
    B = -3.0*mjPI*diam*m->opt.viscosity;
    for (int j=0; j < 3; j++) {
      if (mj_isSparse(m)) {
        addJTBJSparse(m, d, J, &B, 1, 3+j, rownnz, rowadr, colind);
      } else {
        addJTBJ(m, d, J+3*nv+j*nv, &B, 1);
      }
    }
  }

  // add lift and drag force and torque
  if (m->opt.density > 0) {
    // lfrc[0] -= m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
    //            mju_abs(lvel[0])*lvel[0]/64.0;
    B = -m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
        2*mju_abs(lvel[0])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 0, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J, &B, 1);
    }

    // lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
    //            mju_abs(lvel[1])*lvel[1]/64.0;
    B = -m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
        2*mju_abs(lvel[1])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 1, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+nv, &B, 1);
    }

    // lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
    //            mju_abs(lvel[2])*lvel[2]/64.0;
    B = -m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
        2*mju_abs(lvel[2])/64.0;
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 2, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+2*nv, &B, 1);
    }

    // lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*mju_abs(lvel[3])*lvel[3];
    B = -0.5*m->opt.density*box[1]*box[2]*2*mju_abs(lvel[3]);
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 3, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+3*nv, &B, 1);
    }

    // lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*mju_abs(lvel[4])*lvel[4];
    B = -0.5*m->opt.density*box[0]*box[2]*2*mju_abs(lvel[4]);
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 4, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+4*nv, &B, 1);
    }

    // lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*mju_abs(lvel[5])*lvel[5];
    B = -0.5*m->opt.density*box[0]*box[1]*2*mju_abs(lvel[5]);
    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, &B, 1, 5, rownnz, rowadr, colind);
    } else {
      addJTBJ(m, d, J+5*nv, &B, 1);
    }
  }

  mj_freeStack(d);
}


//------------------------- derivatives of passive forces ------------------------------------------

// add (d qfrc_passive / d qvel) to qDeriv
void mjd_passive_vel(const mjModel* m, mjData* d) {
  // all disabled: nothing to add
  if (mjDISABLED(mjDSBL_SPRING) && mjDISABLED(mjDSBL_DAMPER)) {
    return;
  }

  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->ntree_awake < m->ntree;
  int nbody = sleep_filter ? d->nbody_awake : m->nbody;

  // fluid drag model, either body-level (inertia box) or geom-level (ellipsoid)
  if (m->opt.viscosity > 0 || m->opt.density > 0) {
    for (int b=0; b < nbody; b++) {
      int i = sleep_filter ? d->body_awake_ind[b] : b;

      if (m->body_mass[i] < mjMINVAL) {
        continue;
      }

      int use_ellipsoid_model = 0;
      // if any child geom uses the ellipsoid model, inertia-box model is disabled for parent body
      for (int j=0; j < m->body_geomnum[i] && use_ellipsoid_model == 0; j++) {
        const int geomid = m->body_geomadr[i] + j;
        use_ellipsoid_model += (m->geom_fluid[mjNFLUID*geomid] > 0);
      }
      if (use_ellipsoid_model) {
        mjd_ellipsoidFluid(m, d, i);
      } else {
        mjd_inertiaBoxFluid(m, d, i);
      }
    }
  }

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_DAMPER)) {
    return;
  }

  // dof damping
  int nv = m->nv;
  int nv_awake = sleep_filter ? d->nv_awake : nv;
  for (int j = 0; j < nv_awake; j++) {
    int i = sleep_filter ? d->dof_awake_ind[j] : j;
    mjtNum v = d->qvel[i];
    mjtNum poly[mjNPOLY];
    mju_copy(poly, m->dof_dampingpoly + mjNPOLY*i, mjNPOLY);
    mjtNum damping = m->dof_damping[i] + mj_actuatorDamping(m, mjOBJ_JOINT, m->dof_jntid[i], poly);
    int adr = m->D_rowadr[i] + m->D_diag[i];
    d->qDeriv[adr] -= mjd_xPolyForce(damping, poly, v, mjNPOLY, 1);
  }

  // flex edge damping
  for (int f=0; f < m->nflex; f++) {
    mjtNum B = -m->flex_edgedamping[f];
    if (m->flex_rigid[f] || !B) {
      continue;
    }

    int flex_edgeadr = m->flex_edgeadr[f];
    int flex_edgenum = m->flex_edgenum[f];

    // process non-rigid edges of this flex
    for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
      // skip rigid
      if (m->flexedge_rigid[e]) {
        continue;
      }

      // always sparse
      addJTBJSparse(m, d, d->flexedge_J, &B, 1, e,
                    m->flexedge_J_rownnz, m->flexedge_J_rowadr, m->flexedge_J_colind);
    }
  }

  // tendon damping
  int ntendon = m->ntendon;
  for (int i=0; i < ntendon; i++) {
    // skip tendon in one or two sleeping trees
    if (sleep_filter) {
      int treenum = m->tendon_treenum[i];
      int id1 = m->tendon_treeid[2*i];
      if (treenum == 1 && !d->tree_awake[id1]) continue;
      int id2 = m->tendon_treeid[2*i+1];
      if (treenum == 2 && !d->tree_awake[id1] && !d->tree_awake[id2]) continue;
    }

    mjtNum v = d->ten_velocity[i];
    mjtNum poly[mjNPOLY];
    mju_copy(poly, m->tendon_dampingpoly+mjNPOLY*i, mjNPOLY);
    mjtNum damping = m->tendon_damping[i] + mj_actuatorDamping(m, mjOBJ_TENDON, i, poly);
    mjtNum B = -mjd_xPolyForce(damping, poly, v, mjNPOLY, 1);

    if (!B) {
      continue;
    }

    // add sparse
    addJTBJSparse(m, d, d->ten_J, &B, 1, i, m->ten_J_rownnz, m->ten_J_rowadr, m->ten_J_colind);
  }
}


//------------------------- main entry points ------------------------------------------------------

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - [qfrc_bias]) / d qvel
void mjd_smooth_vel(const mjModel* m, mjData* d, int flg_bias) {
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nv_awake < m->nv;

  // clear qDeriv
  if (!sleep_filter) {
    mju_zero(d->qDeriv, m->nD);
  } else {
    mju_zeroSparse(d->qDeriv, m->D_rownnz, m->D_rowadr, d->dof_awake_ind, d->nv_awake);
  }

  // qDeriv += d qfrc_actuator / d qvel
  mjd_actuator_vel(m, d);

  // qDeriv += d qfrc_passive / d qvel
  mjd_passive_vel(m, d);

  // qDeriv -= d qfrc_bias / d qvel; optional
  if (flg_bias) {
    mjd_rne_vel(m, d);
  }
}


//------------------------- implicit effective metric Mtilde = M + K -------------------------------
// K = (h^2 + h*damping) * (K_bend + K_stretch), the PSD implicit flex stiffness. Built once per step on the arena by
// mjd_effBuild (called from mj_fwdAcceleration under the mj_flexCG gate), then consumed uniformly:
// the smooth acceleration, the constraint solver and inverse dynamics all see the same metric.

// arena allocation with hard failure (mirrors stack overflow semantics)
static void* effAlloc(mjData* d, size_t bytes, size_t align) {
  void* p = mj_arenaAllocByte(d, bytes, align);
  if (!p) {
    mjERROR("arena overflow in implicit effective metric");
  }
  return p;
}
#define EFMALLOC(type, n) (type*) effAlloc(d, sizeof(type)*(size_t)(n), _Alignof(type))

// res += B*vec. The STRETCH (and, when assemblable, bending and interp) part is applied from the
// per-step assembled CSR; terms not in the CSR fall back to the matrix-free stencil operators.
void mjd_effMulAdd(const mjModel* m, mjData* d, mjtNum* res, const mjtNum* vec) {
  mjtNum h = m->opt.timestep;
  if (d->nefmK) {
    int nv = m->nv;
    for (int i=0; i < nv; i++) {
      int nnz = d->efm_K_rownnz[i];
      if (!nnz) {
        continue;
      }
      res[i] += mju_dotSparse(d->efm_K_val + d->efm_K_rowadr[i], vec, nnz,
                              d->efm_K_colind + d->efm_K_rowadr[i]);
    }
  }

  // terms not folded into the CSR fall back to the matrix-free operators; which terms those
  // are is derivable, not state: the CSR is only ever built with bending included, and with
  // interp included iff the model is assemblable
  if (!d->nefmK) {
    mjd_flexBend_mul(m, d, res, vec, h*h, h);
  }
  if (!d->nefmK || !mjd_flexInterpAssemblable(m)) {
    mjd_flexInterp_mul(m, d, res, vec, -(h*h), -h, d->flexelem_krot);
  }
}


// z = P \ r with P = M everywhere except the flex block: per-step factor where present, else
// block-Jacobi, plus the constant mj_setConst bending factor on its covered dofs
static void effPrecond(const mjModel* m, mjData* d, mjtNum* z, const mjtNum* r,
                       mjtNum* psr, mjtNum* psz, mjtNum* bfr, mjtNum* bfz) {
  int nv = m->nv;
  mju_copy(z, r, nv);
  mj_solveLD(z, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);

  // precomputed bending factor (mj_setConst): exact (M + K_bend)^-1 on covered dofs.
  // Skipped when the per-step factor exists: it covers these rows and is applied last,
  // so this solve would be overwritten
  int nbd = m->nefm0dof;
  if (nbd && !d->nefmdof) {
    for (int i=0; i < nbd; i++) {
      bfr[i] = r[m->efm0_dofid[i]];
    }
    mju_cholSolveSparse(bfz, m->efm0_L, bfr, nbd,
                        m->efm0_L_rownnz, m->efm0_L_rowadr, m->efm0_L_colind);
    for (int i=0; i < nbd; i++) {
      z[m->efm0_dofid[i]] = bfz[i];
    }
  }

  // per-step factor: exact (diag(M) + K)^-1 on its covered dofs, applied last
  if (d->nefmdof) {
    int n = d->nefmdof;
    for (int i=0; i < n; i++) {
      psr[i] = r[d->efm_dofid[i]];
    }
    mju_cholSolveSparse(psz, d->efm_L, psr, n,
                        d->efm_L_rownnz, d->efm_L_rowadr, d->efm_L_colind);
    for (int i=0; i < n; i++) {
      z[d->efm_dofid[i]] = psz[i];
    }
  }
}


// solve x = Mtilde \ b, where Mtilde is this step's effective metric:
//   efm_active == 0:  Mtilde = M      one sparse LD solve, no elasticity anywhere
//   efm_active == 2:  Mtilde = M + K  exact direct solve, blockdiag(qLD, flex factor);
//                                     exactness conditions in mjd_effBuild
//   efm_active == 1:  Mtilde = M + K  iterative: x0 = M \ b ignores the elasticity, then
//                                     matrix-free PCG on the residual, preconditioned by
//                                     effPrecond (tolerance/cap match the old post-hoc)
void mjd_effSolve(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* b) {
  int nv = m->nv;

  // inactive metric: x = M \ b
  if (!d->efm_active) {
    if (x != b) {
      mju_copy(x, b, nv);
    }
    mj_solveLD(x, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);
    return;
  }

  // exact preconditioner: blockdiag(qLD, flex factor) is (M+K)^-1, solve directly
  if (d->efm_active == 2) {
    mj_markStack(d);
    mjtNum* psr = mjSTACKALLOC(d, d->nefmdof > 0 ? d->nefmdof : 1, mjtNum);
    mjtNum* psz = mjSTACKALLOC(d, d->nefmdof > 0 ? d->nefmdof : 1, mjtNum);
    int nbd0 = m->nefm0dof;
    mjtNum* bfr = mjSTACKALLOC(d, nbd0 > 0 ? nbd0 : 1, mjtNum);
    mjtNum* bfz = mjSTACKALLOC(d, nbd0 > 0 ? nbd0 : 1, mjtNum);
    effPrecond(m, d, x, b, psr, psz, bfr, bfz);
    mj_freeStack(d);
    return;
  }

  // general path: warm start from M \ b, refine below
  if (x != b) {
    mju_copy(x, b, nv);
  }
  mj_solveLD(x, d->qLD, d->qLDiagInv, nv, 1, m->M_rownnz, m->M_rowadr, m->M_colind, NULL);

  mj_markStack(d);
  mjtNum* r  = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* z  = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* p  = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* Ap = mjSTACKALLOC(d, nv, mjtNum);
  mjtNum* psr = mjSTACKALLOC(d, d->nefmdof > 0 ? d->nefmdof : 1, mjtNum);
  mjtNum* psz = mjSTACKALLOC(d, d->nefmdof > 0 ? d->nefmdof : 1, mjtNum);
  int nbd = m->nefm0dof;
  mjtNum* bfr = mjSTACKALLOC(d, nbd > 0 ? nbd : 1, mjtNum);
  mjtNum* bfz = mjSTACKALLOC(d, nbd > 0 ? nbd : 1, mjtNum);

  // r = b - (M+K)*x
  mju_mulSymVecSparse(Ap, d->M, x, nv, m->M_rownnz, m->M_rowadr, m->M_colind);
  mjd_effMulAdd(m, d, Ap, x);
  mju_sub(r, b, Ap, nv);

  // relative tolerance on the residual
  mjtNum tol = 1e-10 * mju_dot(b, b, nv);
  if (mju_dot(r, r, nv) < tol) {
    mj_freeStack(d);
    return;
  }

  effPrecond(m, d, z, r, psr, psz, bfr, bfz);
  mju_copy(p, z, nv);
  mjtNum rz = mju_dot(r, z, nv);

  for (int k=0; k < 50; k++) {
    mju_mulSymVecSparse(Ap, d->M, p, nv, m->M_rownnz, m->M_rowadr, m->M_colind);
    mjd_effMulAdd(m, d, Ap, p);
    mjtNum pAp = mju_dot(p, Ap, nv);
    if (pAp < mjMINVAL) {
      break;
    }
    mjtNum alpha = rz/pAp;
    mju_addToScl(x, p, alpha, nv);
    mju_addToScl(r, Ap, -alpha, nv);
    if (mju_dot(r, r, nv) < tol) {
      break;
    }
    effPrecond(m, d, z, r, psr, psz, bfr, bfz);
    mjtNum rznew = mju_dot(r, z, nv);
    mju_addScl(p, z, p, rznew/rz, nv);
    rz = rznew;
  }
  mj_freeStack(d);
}


// geometric nested-dissection ordering for the per-step factor: recursive coordinate bisection
// with adjacency-detected separators, emitted ancestors-first (the reverse-Cholesky convention)
typedef struct {
  const mjtNum* pos;     // block positions                              (3 x nblk)
  const int* B_rownnz;   // dof-level B pattern, for block adjacency
  const int* B_rowadr;
  const int* B_colind;
  const int* dofid;      // block -> first dof address (3 dofs per block)
  const int* dof2c;      // dof -> compact index (pre-permutation)
  int* work;             // block id work array                          (nblk x 1)
  int* stamp;            // current-range stamp per block                (nblk x 1)
  int* side;             // bisection side per block (valid when stamped)(nblk x 1)
  int stampctr;          // running range id
  int* scratch;          // side-1 gather scratch                        (nblk x 1)
  int* perm;             // output: block emission order                 (nblk x 1)
  int nperm;             // emitted count
} mjEffND;

static void effNDOrder(mjEffND* nd, int lo, int hi) {
  int nblk = hi - lo;
  if (nblk <= 16) {
    for (int i=lo; i < hi; i++) {
      nd->perm[nd->nperm++] = nd->work[i];
    }
    return;
  }

  // widest axis of the range's bounding box, split at the mean coordinate
  mjtNum bmin[3] = {mjMAXVAL, mjMAXVAL, mjMAXVAL}, bmax[3] = {-mjMAXVAL, -mjMAXVAL, -mjMAXVAL};
  mjtNum mean[3] = {0, 0, 0};
  for (int i=lo; i < hi; i++) {
    const mjtNum* p = nd->pos + 3*nd->work[i];
    for (int x=0; x < 3; x++) {
      bmin[x] = p[x] < bmin[x] ? p[x] : bmin[x];
      bmax[x] = p[x] > bmax[x] ? p[x] : bmax[x];
      mean[x] += p[x];
    }
  }
  int axis = 0;
  for (int x=1; x < 3; x++) {
    if (bmax[x] - bmin[x] > bmax[axis] - bmin[axis]) {
      axis = x;
    }
  }
  mjtNum split = mean[axis] / nblk;

  // stamp the range, assign sides
  int id = ++nd->stampctr, n0 = 0;
  for (int i=lo; i < hi; i++) {
    int b = nd->work[i];
    nd->stamp[b] = id;
    nd->side[b] = nd->pos[3*b + axis] > split;
    n0 += !nd->side[b];
  }

  // degenerate split (coincident positions): fall back to an arbitrary halving
  if (n0 == 0 || n0 == nblk) {
    for (int i=lo; i < hi; i++) {
      nd->side[nd->work[i]] = (i - lo) >= nblk/2;
    }
  }

  // emit the separator (side-0 blocks adjacent to side 1) first; compact A in place and
  // side-1 blocks via the scratch list (in-place would clobber unread entries)
  int na = 0, nb = 0;
  for (int i=lo; i < hi; i++) {
    int b = nd->work[i];
    if (nd->side[b]) {
      nd->scratch[nb++] = b;
      continue;
    }

    // side 0: separator iff adjacent to side 1 (block adjacency via the first dof's B row)
    int dof = nd->dofid[3*b];
    int adr = nd->B_rowadr[dof], nnz = nd->B_rownnz[dof], sep = 0;
    for (int k=0; k < nnz; k++) {
      int cc = nd->dof2c[nd->B_colind[adr + k]];
      if (cc >= 0) {
        int nbr = cc/3;
        if (nd->stamp[nbr] == id && nd->side[nbr]) {
          sep = 1;
          break;
        }
      }
    }
    if (sep) {
      nd->perm[nd->nperm++] = b;
    } else {
      nd->work[lo + na++] = b;
    }
  }
  for (int i=0; i < nb; i++) {
    nd->work[lo + na + i] = nd->scratch[i];
  }
  effNDOrder(nd, lo, lo + na);
  effNDOrder(nd, lo + na, lo + na + nb);
}


// per-step sparse factor of the flex block of (M + K): reverse-Cholesky over the covered dofs,
// nested-dissection ordered. M enters as its diagonal there -- exact for free vertices;
// parent-coupled vertices make this a preconditioner, refined to tolerance by mjd_effSolve.
// Exact zeros are dropped from the off-diagonal pattern (bending couples same-coordinate dofs
// only). The matrix is SPD by construction, so rank deficiency can only mean a degenerate
// model (near-zero mass and stiffness on a covered dof) and is a hard error.
static void effFactor(const mjModel* m, mjData* d) {
  int nv = m->nv;
  const int* B_rownnz = d->efm_K_rownnz;
  const int* B_rowadr = d->efm_K_rowadr;
  const int* B_colind = d->efm_K_colind;
  const mjtNum* B_val = d->efm_K_val;

  mj_markStack(d);

  // compact dof map over covered rows (ascending, so compact indices stay sorted)
  int* dof2c = mjSTACKALLOC(d, nv, int);
  int n = 0;
  for (int i=0; i < nv; i++) {
    dof2c[i] = B_rownnz[i] ? n++ : -1;
  }
  int* dofid = mjSTACKALLOC(d, n, int);
  for (int i=0; i < nv; i++) {
    if (dof2c[i] >= 0) {
      dofid[dof2c[i]] = i;
    }
  }

  // nested-dissection reordering of the covered blocks (one block = 3 dofs of one point)
  int nblk = n/3;
  int* nd_perm = mjSTACKALLOC(d, nblk, int);
  {
    int* nd_work  = mjSTACKALLOC(d, nblk, int);
    int* nd_stamp = mjSTACKALLOC(d, nblk, int);
    int* nd_side  = mjSTACKALLOC(d, nblk, int);
    int* nd_scr   = mjSTACKALLOC(d, nblk, int);
    mjtNum* bpos  = mjSTACKALLOC(d, 3*nblk, mjtNum);
    for (int b=0; b < nblk; b++) {
      nd_work[b] = b;
      nd_stamp[b] = 0;
      mju_copy3(bpos + 3*b, d->xpos + 3*m->dof_bodyid[dofid[3*b]]);
    }
    mjEffND nd;
    nd.pos = bpos;
    nd.B_rownnz = B_rownnz;
    nd.B_rowadr = B_rowadr;
    nd.B_colind = B_colind;
    nd.dofid = dofid;
    nd.dof2c = dof2c;
    nd.work = nd_work;
    nd.stamp = nd_stamp;
    nd.side = nd_side;
    nd.stampctr = 0;
    nd.scratch = nd_scr;
    nd.perm = nd_perm;
    nd.nperm = 0;
    effNDOrder(&nd, 0, nblk);
  }

  // apply the permutation to the compact indexing; the permuted dofid persists on the arena
  int* psdofid = EFMALLOC(int, n);
  for (int r=0; r < nblk; r++) {
    psdofid[3*r]   = dofid[3*nd_perm[r]];
    psdofid[3*r+1] = dofid[3*nd_perm[r] + 1];
    psdofid[3*r+2] = dofid[3*nd_perm[r] + 2];
  }
  for (int i=0; i < n; i++) {
    dof2c[psdofid[i]] = i;
  }

  // H = diag(M) + K in compact indices: lower CSR (values, diagonal last) + upper CSR (pattern)
  int nHl = 0, nHu = 0;
  for (int c=0; c < n; c++) {
    int adr = B_rowadr[psdofid[c]], nnzB = B_rownnz[psdofid[c]];
    for (int k=0; k < nnzB; k++) {
      int cc = dof2c[B_colind[adr + k]];
      if (B_val[adr + k] == 0 && cc != c) {
        continue;
      }
      if (cc <= c) {
        nHl++;
      } else {
        nHu++;
      }
    }
  }
  int* Hl_rownnz = mjSTACKALLOC(d, n, int);
  int* Hl_rowadr = mjSTACKALLOC(d, n, int);
  int* Hl_colind = mjSTACKALLOC(d, nHl, int);
  mjtNum* Hl_val  = mjSTACKALLOC(d, nHl, mjtNum);
  int* Hu_rownnz = mjSTACKALLOC(d, n, int);
  int* Hu_rowadr = mjSTACKALLOC(d, n, int);
  int* Hu_colind = mjSTACKALLOC(d, nHu > 0 ? nHu : 1, int);
  int maxrow = 0;
  for (int c=0; c < n; c++) {
    maxrow = B_rownnz[psdofid[c]] > maxrow ? B_rownnz[psdofid[c]] : maxrow;
  }
  int* rind = mjSTACKALLOC(d, maxrow, int);
  mjtNum* rval = mjSTACKALLOC(d, maxrow, mjtNum);
  int ladr = 0, uadr = 0;
  for (int c=0; c < n; c++) {
    int i = psdofid[c];
    Hl_rowadr[c] = ladr;
    Hu_rowadr[c] = uadr;

    // gather the row in permuted compact indices, then sort (columns are no longer monotone)
    int adr = B_rowadr[i], nnzB = B_rownnz[i], nr = 0;
    for (int k=0; k < nnzB; k++) {
      int cc = dof2c[B_colind[adr + k]];
      if (B_val[adr + k] == 0 && cc != c) {
        continue;
      }
      rind[nr] = cc;
      rval[nr++] = B_val[adr + k];
    }
    for (int k=1; k < nr; k++) {
      int ci = rind[k];
      mjtNum vi = rval[k];
      int j = k - 1;
      while (j >= 0 && rind[j] > ci) {
        rind[j+1] = rind[j];
        rval[j+1] = rval[j];
        j--;
      }
      rind[j+1] = ci;
      rval[j+1] = vi;
    }
    for (int k=0; k < nr; k++) {
      if (rind[k] < c) {
        Hl_colind[ladr] = rind[k];
        Hl_val[ladr++] = rval[k];
      } else if (rind[k] == c) {
        Hl_colind[ladr] = c;
        Hl_val[ladr++] = rval[k] + d->M[m->M_rowadr[i] + m->M_rownnz[i] - 1];
      } else {
        Hu_colind[uadr++] = rind[k];
      }
    }
    Hl_rownnz[c] = ladr - Hl_rowadr[c];
    Hu_rownnz[c] = uadr - Hu_rowadr[c];
  }

  // symbolic factorization: counting phase, then filling phase
  int* L_rownnz  = EFMALLOC(int, n);
  int* L_rowadr  = EFMALLOC(int, n);
  int* LT_rownnz = mjSTACKALLOC(d, n, int);
  int* LT_rowadr = mjSTACKALLOC(d, n, int);
  int nnz = mju_cholFactorSymbolic(NULL, L_rownnz, L_rowadr, NULL, LT_rownnz, LT_rowadr, NULL,
                                   Hu_rownnz, Hu_rowadr, Hu_colind, n, d);
  int* L_colind  = EFMALLOC(int, nnz);
  int* LT_colind = mjSTACKALLOC(d, nnz, int);
  int* LT_map    = mjSTACKALLOC(d, nnz, int);
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz, LT_rowadr, LT_map,
                         Hu_rownnz, Hu_rowadr, Hu_colind, n, d);

  // numeric factorization
  mjtNum* L = EFMALLOC(mjtNum, nnz);
  int rank = mju_cholFactorNumeric(L, n, mjMINVAL, L_rownnz, L_rowadr, L_colind,
                                   LT_rownnz, LT_rowadr, LT_colind, LT_map,
                                   Hl_val, Hl_rownnz, Hl_rowadr, Hl_colind, d);
  mj_freeStack(d);
  if (rank != n) {
    mjERROR("effective metric factorization is rank-deficient (%d of %d): "
            "degenerate mass or stiffness in the flex block", rank, n);
  }

  d->nefmdof      = n;
  d->nefmL        = nnz;
  d->efm_dofid    = psdofid;
  d->efm_L_rownnz = L_rownnz;
  d->efm_L_rowadr = L_rowadr;
  d->efm_L_colind = L_colind;
  d->efm_L        = L;
}


// refresh the smooth-force shift c = h*K*qvel of the active metric (values only, no
// allocation: called from the velocity stage, mirroring the efc value refresh pattern)
void mjd_effShift(const mjModel* m, mjData* d) {
  if (!d->efm_active) {
    return;
  }
  mjtNum h = m->opt.timestep;
  mju_zero(d->efm_c, m->nv);
  mjd_flexInterp_mul(m, d, d->efm_c, d->qvel, h, 0, d->flexelem_krot);
  mjd_flexBend_mul(m, d, d->efm_c, d->qvel, -h, 0);
  mjd_flexStretch_mul(m, d, d->efm_c, d->qvel, -h, 0);
}


// build the per-step implicit effective metric on the arena, or deactivate it. The gate
// decision (mj_flexCG) is the caller's: the metric module has no dependency on the solver
// configuration beyond what it is told here.
void mjd_effBuild(const mjModel* m, mjData* d, int active, int flg_factor) {
  int nv = m->nv;
  d->efm_active = 0;
  d->nefmK = 0;
  d->nefmdof = 0;
  d->nefmL = 0;
  if (!active) {
    return;
  }
  mjtNum h = m->opt.timestep;

  // corotated element stiffness cache (used by the shift and the matrix-free fallback)
  mju_zero(d->flexelem_krot, m->nflexstiffness);
  mjd_flexInterp_cacheKrot(m, d, d->flexelem_krot);

  // smooth-force shift c = h*K*qvel (values refreshed by mjd_effShift in the velocity stage)
  d->efm_c = EFMALLOC(mjtNum, nv);

  // assemble the standard-flex part of B into CSR (constant during the step). With stretch or
  // assemblable interp present, assemble the FULL matrix (bending included): one CSR then
  // serves both the matvec and the per-step factor. Bending-only models keep the stencil
  // operator + the constant mj_setConst factor.
  const mjtNum* krot = mjd_flexInterpAssemblable(m) ? d->flexelem_krot : NULL;
  d->efm_K_rownnz = EFMALLOC(int, nv);
  d->efm_K_rowadr = EFMALLOC(int, nv);
  if (mjd_flexStiff_any(m, krot != NULL)) {
    d->nefmK = mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                                      NULL, NULL, h*h, h, /*bend*/ 1, /*stretch*/ 1, krot);
  }
  if (d->nefmK) {
    d->efm_K_colind = EFMALLOC(int, d->nefmK);
    d->efm_K_val    = EFMALLOC(mjtNum, d->nefmK);
    mjd_flexStiff_assemble(m, d, d->efm_K_rownnz, d->efm_K_rowadr,
                           d->efm_K_colind, d->efm_K_val, h*h, h,
                           /*bend*/ 1, /*stretch*/ 1, krot);
    // per-step factor of the flex block of (M + K): the stiffness is constant during the
    // step, so one factorization here turns every preconditioner application into a direct
    // solve (the stiff flex block stops being iterated on). Consumers that only multiply
    // (inverse dynamics) skip it.
    if (flg_factor) {
      effFactor(m, d);
    }

  } else {
    mju_zeroInt(d->efm_K_rownnz, nv);
    mju_zeroInt(d->efm_K_rowadr, nv);
  }
  d->efm_active = 1;

  // preconditioner exactness (efm_active == 2): when every dof the stiffness touches sits on
  // a simple slider body (diagonal M row, no kinematic children), M has no coupling across
  // the covered block, so blockdiag(qLD, flex factor) is exactly (M+K)^-1 and mjd_effSolve
  // skips the refinement. Interp flexes outside the assembled CSR act only through the
  // matrix-free operator (no factor rows), which breaks exactness.
  int exact = d->nefmK ? (d->nefmdof > 0) : 1;
  for (int f=0; exact && f < m->nflex; f++) {
    if (!krot && flexInterp_processed(m, f)) {
      exact = 0;
    }
  }
  if (exact && d->nefmK) {
    for (int i=0; i < nv; i++) {
      if (d->efm_K_rownnz[i] && m->body_simple[m->dof_bodyid[i]] != 2) {
        exact = 0;
        break;
      }
    }
  } else if (exact) {
    for (int i=0; i < m->nefm0dof; i++) {
      if (m->body_simple[m->dof_bodyid[m->efm0_dofid[i]]] != 2) {
        exact = 0;
        break;
      }
    }
  }
  if (exact) {
    d->efm_active = 2;
  }

  // fill the shift with the current velocity (refreshed again in the velocity stage)
  mjd_effShift(m, d);
}
