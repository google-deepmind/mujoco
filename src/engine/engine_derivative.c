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
#include "engine/engine_core_constraint.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_passive.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
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
    mju_copy(Da, Da_tmp, 9);
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

  // 4 coefficients: a=cos(x), b=sin(x)/x, c=(1-cos(x))/x^2, d=(x-sin(x))/x^3}
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
  if (Dvel) mju_copy(Dvel, Dvel_, 9);
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
      int end = d->D_rowadr[i] + d->D_rownnz[i];
      for (int adr=d->D_rowadr[i]; adr < end; adr++) {
        d->qDeriv[adr] -= row[d->D_colind[adr]];
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
  mju_copy(mat + 6*d->B_rowadr[n], mat + 6*d->B_rowadr[m->body_parentid[n]], 6*ndof);
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
  while (i < d->B_rownnz[n] && ip < d->B_rownnz[np]) {
    // columns match
    if (d->B_colind[d->B_rowadr[n] + i] == d->B_colind[d->B_rowadr[np] + ip]) {
      mju_addTo(mat + 6*(d->B_rowadr[np] + ip), mat + 6*(d->B_rowadr[n] + i), 6);

      // advance both
      i++;
      ip++;
    }

    // mismatch columns: advance parent
    else if (d->B_colind[d->B_rowadr[n] + i] > d->B_colind[d->B_rowadr[np] + ip]) {
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
  int nv = m->nv, nbody = m->nbody;
  int* Badr = d->B_rowadr, * Dadr = d->D_rowadr;
  mjtNum mat[36], matT[36];   // 6x6 matrices

  // forward pass over bodies: accumulate Dcvel, set Dcdofdot
  for (int i = 1; i < nbody; i++) {
    // Dcvel = Dcvel_parent
    copyFromParent(m, d, Dcvel, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j = m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < nv - 1 ? m->dof_Madr[j + 1] : m->nM) - (m->dof_Madr[j] + 1);

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
  int nv = m->nv, nbody = m->nbody;
  const int* Badr = d->B_rowadr;
  const int* Dadr = d->D_rowadr;
  const int* Bnnz = d->B_rownnz;

  mjtNum mat[36], mat1[36], mat2[36], dmul[36], tmp[6];

  mj_markStack(d);
  mjtNum* Dcdofdot = mjSTACKALLOC(d, 6*m->nD, mjtNum);
  mjtNum* Dcvel = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* Dcacc = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* Dcfrcbody = mjSTACKALLOC(d, 6*m->nB, mjtNum);
  mjtNum* row = mjSTACKALLOC(d, nv, mjtNum);

  // clear
  mju_zero(Dcdofdot, 6*m->nD);
  mju_zero(Dcvel, 6*m->nB);
  mju_zero(Dcacc, 6*m->nB);
  mju_zero(Dcfrcbody, 6*m->nB);

  // compute Dcvel and Dcdofdot
  mjd_comVel_vel(m, d, Dcvel, Dcdofdot);

  // forward pass over bodies: accumulate Dcacc, set Dcfrcbody
  for (int i=1; i < nbody; i++) {
    // Dcacc = Dcacc_parent
    copyFromParent(m, d, Dcacc, i);

    // process all dofs of this body
    int doflast = m->body_dofadr[i] + m->body_dofnum[i];
    for (int j=m->body_dofadr[i]; j < doflast; j++) {
      // number of dof ancestors of dof j
      int Jadr = (j < nv - 1 ? m->dof_Madr[j + 1] : m->nM) - (m->dof_Madr[j] + 1);

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
  for (int i=m->nbody-1; i > 0; i--) {
    addToParent(m, d, Dcfrcbody, i);
  }

  // process all dofs, update qDeriv
  for (int j=0; j < nv; j++) {
    // get body index
    int i = m->dof_bodyid[j];

    // qDeriv -= D(cdof * cfrc_body)
    mju_mulMatVec(row, Dcfrcbody + 6*Badr[i], d->cdof + 6*j, Bnnz[i], 6);
    mju_subFrom(d->qDeriv + Dadr[j], row, Bnnz[i]);
  }

  mj_freeStack(d);
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
          int rownnz_k = d->D_rownnz[k];
          for (int s=0; s < rownnz_k; s++) {
            int adr = d->D_rowadr[k] + s;
            d->qDeriv[adr] += row[d->D_colind[adr]];
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
      int offset_i = offset+i, offset_j = offset+j;
      if (!B[i*n+j]) {
        continue;
      }

      // loop over non-zero elements of J(i,:)
      for (int k = 0; k < J_rownnz[offset_i]; k++) {
        int ik = J_rowadr[offset_i] + k;
        int colik = J_colind[ik];

        // qDeriv(k,:) += J(j,:) * J(i,k)*B(i,j)
        mju_addToSclSparseInc(d->qDeriv + d->D_rowadr[colik], J + J_rowadr[offset_j],
                              d->D_rownnz[colik], d->D_colind + d->D_rowadr[colik],
                              J_rownnz[offset_j], J_colind + J_rowadr[offset_j],
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



// add (d qfrc_actuator / d qvel) to qDeriv
void mjd_actuator_vel(const mjModel* m, mjData* d) {
  int nv = m->nv, nu = m->nu;

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_ACTUATION)) {
    return;
  }

  // allocate dense actuator_moment row
  mj_markStack(d);
  mjtNum* moment = mjSTACKALLOC(d, nv, mjtNum);

  // process actuators
  for (int i=0; i < nu; i++) {
    // skip if disabled
    if (mj_actuatorDisabled(m, i)) {
      continue;
    }

    mjtNum bias_vel = 0, gain_vel = 0;

    // affine bias
    if (m->actuator_biastype[i] == mjBIAS_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      bias_vel = (m->actuator_biasprm + mjNBIAS*i)[2];
    }

    // affine gain
    if (m->actuator_gaintype[i] == mjGAIN_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      gain_vel = (m->actuator_gainprm + mjNGAIN*i)[2];
    }

    // muscle gain
    else if (m->actuator_gaintype[i] == mjGAIN_MUSCLE) {
      gain_vel = mjd_muscleGain_vel(d->actuator_length[i],
                                    d->actuator_velocity[i],
                                    m->actuator_lengthrange+2*i,
                                    m->actuator_acc0[i],
                                    m->actuator_gainprm + mjNGAIN*i);
    }

    // force = gain .* [ctrl/act]
    if (gain_vel != 0) {
      if (m->actuator_dyntype[i] == mjDYN_NONE) {
        bias_vel += gain_vel * d->ctrl[i];
      } else {
        int act_first = m->actuator_actadr[i];
        int act_last = act_first + m->actuator_actnum[i] - 1;
        bias_vel += gain_vel * d->act[act_last];
      }
    }

    // add
    if (bias_vel != 0) {
      mju_sparse2dense(moment, d->actuator_moment, 1, nv, d->moment_rownnz + i,
                       d->moment_rowadr + i, d->moment_colind);
      addJTBJ(m, d, moment, &bias_vel, 1);
    }
  }

  // free space
  mj_freeStack(d);
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

    // get geom global Jacobian: rotation then translation
    if (mj_isSparse(m)) {
      mj_jacSparse(m, d, J+3*nnz, J, d->geom_xpos+3*geomid, m->geom_bodyid[geomid], nnz, colind);
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

    // make B symmetric if integrator is IMPLICITFAST
    if (m->opt.integrator == mjINT_IMPLICITFAST) {
      mju_symmetrize(B, B, 6);
    }

    if (mj_isSparse(m)) {
      addJTBJSparse(m, d, J, B, 6, 0, rownnz, rowadr, colind_compressed);
    }
    else {
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
    mj_jacSparse(m, d, J+3*nnz, J, d->xipos+3*i, i, nnz, colind);

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
  int nv = m->nv, nbody = m->nbody;

  // disabled: nothing to add
  if (mjDISABLED(mjDSBL_PASSIVE)) {
    return;
  }

  // dof damping
  for (int i=0; i < nv; i++) {
    int nnz_i = d->D_rownnz[i];
    for (int j=0; j < nnz_i; j++) {
      int ij = d->D_rowadr[i] + j;

      // identify diagonal element
      if (d->D_colind[ij] == i) {
        d->qDeriv[ij] -= m->dof_damping[i];
        break;
      }
    }
  }

  // flex edge damping
  for (int f=0; f < m->nflex; f++) {
    if (!m->flex_rigid[f] && m->flex_edgedamping[f]) {
      mjtNum B = -m->flex_edgedamping[f];
      int flex_edgeadr = m->flex_edgeadr[f];
      int flex_edgenum = m->flex_edgenum[f];

      // process non-rigid edges of this flex
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        // skip rigid
        if (m->flexedge_rigid[e]) {
          continue;
        }

        // add sparse or dense
        if (mj_isSparse(m)) {
          addJTBJSparse(m, d, d->flexedge_J, &B, 1, e,
                        d->flexedge_J_rownnz, d->flexedge_J_rowadr, d->flexedge_J_colind);
        } else {
          addJTBJ(m, d, d->flexedge_J+e*nv, &B, 1);
        }
      }
    }
  }

  // tendon damping
  for (int i=0; i < m->ntendon; i++) {
    if (m->tendon_damping[i] > 0) {
      mjtNum B = -m->tendon_damping[i];

      // add sparse or dense
      if (mj_isSparse(m)) {
        addJTBJSparse(m, d, d->ten_J, &B, 1, i,
                      d->ten_J_rownnz, d->ten_J_rowadr, d->ten_J_colind);
      } else {
        addJTBJ(m, d, d->ten_J+i*nv, &B, 1);
      }
    }
  }

  // fluid drag model, either body-level (inertia box) or geom-level (ellipsoid)
  if (m->opt.viscosity > 0 || m->opt.density > 0) {
    for (int i=1; i < nbody; i++) {
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
}



//------------------------- main entry points ------------------------------------------------------

// analytical derivative of smooth forces w.r.t velocities:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - [qfrc_bias]) / d qvel
void mjd_smooth_vel(const mjModel* m, mjData* d, int flg_bias) {
  // clear qDeriv
  mju_zero(d->qDeriv, m->nD);

  // qDeriv += d qfrc_actuator / d qvel
  mjd_actuator_vel(m, d);

  // qDeriv += d qfrc_passive / d qvel
  mjd_passive_vel(m, d);

  // qDeriv -= d qfrc_bias / d qvel; optional
  if (flg_bias) {
    mjd_rne_vel(m, d);
  }
}
