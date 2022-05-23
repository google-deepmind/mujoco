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
#include "engine/engine_callback.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_sparse.h"
#include "engine/engine_util_spatial.h"



//------------------------- derivatives of spatial algebra -----------------------------------------

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



// add (d qfrc_passive / d qvel) to DfDv
void mjd_passive_vel(const mjModel* m, mjData* d, mjtNum* DfDv) {
  mjMARKSTACK;
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

  // body viscosity, lift and drag
  if (m->opt.viscosity>0 || m->opt.density>0) {
    int rownnz[6], rowadr[6];
    mjtNum* J = mj_stackAlloc(d, 6*nv);
    mjtNum* tmp = mj_stackAlloc(d, 3*nv);
    int* colind = (int*) mj_stackAlloc(d, 6*nv);

    for (int i=1; i<m->nbody; i++) {
      if (m->body_mass[i]>mjMINVAL) {
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
              addJTBJSparse(DfDv, J, &B, 1, nv, j,
                            rownnz, rowadr, colind);
            } else {
              addJTBJ(DfDv, J+j*nv, &B, 1, nv);
            }
          }

          // mju_scl3(lfrc+3, lvel+3, -3.0*mjPI*diam*m->opt.viscosity);
          B = -3.0*mjPI*diam*m->opt.viscosity;
          for (int j=0; j<3; j++) {
            if (mj_isSparse(m)) {
              addJTBJSparse(DfDv, J, &B, 1, nv, 3+j,
                            rownnz, rowadr, colind);
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
            addJTBJSparse(DfDv, J, &B, 1, nv, 0,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J, &B, 1, nv);
          }

          // lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
          //            mju_abs(lvel[1])*lvel[1]/64.0;
          B = -m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
              2*mju_abs(lvel[1])/64.0;
          if (mj_isSparse(m)) {
            addJTBJSparse(DfDv, J, &B, 1, nv, 1,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J+nv, &B, 1, nv);
          }

          // lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
          //            mju_abs(lvel[2])*lvel[2]/64.0;
          B = -m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
              2*mju_abs(lvel[2])/64.0;
          if (mj_isSparse(m)) {
            addJTBJSparse(DfDv, J, &B, 1, nv, 2,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J+2*nv, &B, 1, nv);
          }

          // lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*mju_abs(lvel[3])*lvel[3];
          B = -0.5*m->opt.density*box[1]*box[2]*2*mju_abs(lvel[3]);
          if (mj_isSparse(m)) {
            addJTBJSparse(DfDv, J, &B, 1, nv, 3,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J+3*nv, &B, 1, nv);
          }

          // lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*mju_abs(lvel[4])*lvel[4];
          B = -0.5*m->opt.density*box[0]*box[2]*2*mju_abs(lvel[4]);
          if (mj_isSparse(m)) {
            addJTBJSparse(DfDv, J, &B, 1, nv, 4,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J+4*nv, &B, 1, nv);
          }

          // lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*mju_abs(lvel[5])*lvel[5];
          B = -0.5*m->opt.density*box[0]*box[1]*2*mju_abs(lvel[5]);
          if (mj_isSparse(m)) {
            addJTBJSparse(DfDv, J, &B, 1, nv, 5,
                          rownnz, rowadr, colind);
          } else {
            addJTBJ(DfDv, J+5*nv, &B, 1, nv);
          }
        }
      }
    }
  }
  mjFREESTACK;
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
  return -force*FL*dFV/mjMAX(mjMINVAL,L0*vmax);
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
    // affine bias
    if (m->actuator_biastype[i]==mjBIAS_AFFINE) {
      // extract bias info: prm = [const, kp, kv]
      mjtNum* prm = m->actuator_biasprm + mjNBIAS*i;

      // add
      mjtNum B = prm[2];
      addJTBJ(DfDv, d->actuator_moment+i*nv, &B, 1, nv);
    }

    // muscle gain
    else if (m->actuator_gaintype[i]==mjGAIN_MUSCLE) {
      mjtNum B = mjd_muscleGain_vel(d->actuator_length[i],
                                    d->actuator_velocity[i],
                                    m->actuator_lengthrange+2*i,
                                    m->actuator_acc0[i],
                                    m->actuator_gainprm + mjNGAIN*i);

      // force = gain .* [ctrl/act]
      if (m->actuator_dyntype[i]==mjDYN_NONE) {
        B *= d->ctrl[i];
      } else {
        B *= d->act[i-(m->nu - m->na)];
      }

      // add
      addJTBJ(DfDv, d->actuator_moment+i*nv, &B, 1, nv);
    }
  }
}



//------------------------- main entry points ------------------------------------------------------

// Analytical derivative:
//   d->qDeriv = d (qfrc_actuator + qfrc_passive - qfrc_bias) / d qvel.
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



// Centered finite difference approximation to mj_derivative.
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

