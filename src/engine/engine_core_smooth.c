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

#include "engine/engine_core_smooth.h"

#include <stddef.h>
#include <string.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
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

//--------------------------- position -------------------------------------------------------------

// forward kinematics
void mj_kinematics(const mjModel* m, mjData* d) {
  mjtNum pos[3], quat[4], *bodypos, *bodyquat;
  mjtNum qloc[4], qtmp[4], vec[3], vec1[3], xanchor[3], xaxis[3];

  // set world position and orientation
  mju_zero3(d->xpos);
  mju_unit4(d->xquat);
  mju_zero3(d->xipos);
  mju_zero(d->xmat, 9);
  mju_zero(d->ximat, 9);
  d->xmat[0] = d->xmat[4] = d->xmat[8] = 1;
  d->ximat[0] = d->ximat[4] = d->ximat[8] = 1;

  // normalize all quaternions in qpos
  mj_normalizeQuat(m, d->qpos);

  // normalize mocap quaternions
  for (int i=0; i<m->nmocap; i++) {
    mju_normalize4(d->mocap_quat+4*i);
  }

  // compute global cartesian positions and orientations of all bodies
  for (int i=1; i<m->nbody; i++) {
    // free joint
    if (m->body_jntnum[i]==1 && m->jnt_type[m->body_jntadr[i]]==mjJNT_FREE) {
      // get addresses
      int jid = m->body_jntadr[i];
      int qadr = m->jnt_qposadr[jid];

      // copy pos and quat from qpos
      mju_copy3(pos, d->qpos+qadr);
      mju_copy4(quat, d->qpos+qadr+3);

      // set xanchor = 0, xaxis = (0,0,1)
      mju_copy3(xanchor, pos);
      xaxis[0] = xaxis[1] = 0;
      xaxis[2] = 1;

      // assign xanchor and xaxis
      mju_copy3(d->xanchor+3*jid, xanchor);
      mju_copy3(d->xaxis+3*jid, xaxis);
    }

    // regular or no joint
    else {
      int pid = m->body_parentid[i];

      // get body pos and quat: from model or mocap
      if (m->body_mocapid[i]>=0) {
        bodypos = d->mocap_pos + 3*m->body_mocapid[i];
        bodyquat = d->mocap_quat + 4*m->body_mocapid[i];
      } else {
        bodypos = m->body_pos+3*i;
        bodyquat = m->body_quat+4*i;
      }

      // apply fixed translation and rotation relative to parent
      mju_rotVecMat(vec, bodypos, d->xmat+9*pid);
      mju_add3(pos, d->xpos+3*pid, vec);
      mju_mulQuat(quat, d->xquat+4*pid, bodyquat);

      // accumulate joints, compute pos and quat for this body
      for (int j=0; j<m->body_jntnum[i]; j++) {
        // get joint id, qpos address, joint type
        int jid = m->body_jntadr[i] + j;
        int qadr = m->jnt_qposadr[jid];
        int jtype = m->jnt_type[jid];

        // compute axis in global frame; ball jnt_axis is (0,0,1), set by compiler
        mju_rotVecQuat(xaxis, m->jnt_axis+3*jid, quat);

        // compute anchor in global frame
        mju_rotVecQuat(xanchor, m->jnt_pos+3*jid, quat);
        mju_addTo3(xanchor, pos);

        // apply joint transformation
        switch (jtype) {
        case mjJNT_SLIDE:
          mju_addToScl3(pos, xaxis, d->qpos[qadr] - m->qpos0[qadr]);
          break;

        case mjJNT_BALL:
        case mjJNT_HINGE:
          // compute local quaternion rotation (qloc)
          if (jtype==mjJNT_BALL) {
            mju_copy4(qloc, d->qpos+qadr);
          } else {
            mju_axisAngle2Quat(qloc, m->jnt_axis+3*jid, d->qpos[qadr] - m->qpos0[qadr]);
          }

          // apply rotation
          mju_mulQuat(qtmp, quat, qloc);
          mju_copy4(quat, qtmp);

          // correct for off-center rotation
          mju_sub3(vec, xanchor, pos);
          mju_rotVecQuat(vec1, m->jnt_pos+3*jid, quat);
          pos[0] += (vec[0] - vec1[0]);
          pos[1] += (vec[1] - vec1[1]);
          pos[2] += (vec[2] - vec1[2]);
          break;

        default:
          mju_error_i("Unknown joint type %d", jtype);    // SHOULD NOT OCCUR
        }

        // assign xanchor and xaxis
        mju_copy3(d->xanchor+3*jid, xanchor);
        mju_copy3(d->xaxis+3*jid, xaxis);
      }
    }

    // assign xquat and xpos, construct xmat
    mju_normalize4(quat);
    mju_copy4(d->xquat+4*i, quat);
    mju_copy3(d->xpos+3*i, pos);
    mju_quat2Mat(d->xmat+9*i, quat);
  }

  // compute/copy Cartesian positions and orientations of body inertial frames
  for (int i=1; i<m->nbody; i++) {
    mj_local2Global(d, d->xipos+3*i, d->ximat+9*i,
                    m->body_ipos+3*i, m->body_iquat+4*i,
                    i, m->body_sameframe[i]);
  }

  // compute/copy Cartesian positions and orientations of geoms
  for (int i=0; i<m->ngeom; i++) {
    mj_local2Global(d, d->geom_xpos+3*i, d->geom_xmat+9*i,
                    m->geom_pos+3*i, m->geom_quat+4*i,
                    m->geom_bodyid[i], m->geom_sameframe[i]);
  }

  // compute/copy Cartesian positions and orientations of sites
  for (int i=0; i<m->nsite; i++) {
    mj_local2Global(d, d->site_xpos+3*i, d->site_xmat+9*i,
                    m->site_pos+3*i, m->site_quat+4*i,
                    m->site_bodyid[i], m->site_sameframe[i]);
  }
}



// map inertias and motion dofs to global frame centered at subtree-CoM
void mj_comPos(const mjModel* m, mjData* d) {
  mjtNum offset[3], axis[3];
  mjMARKSTACK;
  mjtNum* mass_subtree = mj_stackAlloc(d, m->nbody);

  // clear subtree
  mju_zero(mass_subtree, m->nbody);
  mju_zero(d->subtree_com, m->nbody*3);

  // backwards pass over bodies: compute subtree_com and mass_subtree
  for (int i=m->nbody-1; i>=0; i--) {
    // add local info
    mju_addToScl3(d->subtree_com+3*i, d->xipos+3*i, m->body_mass[i]);
    mass_subtree[i] += m->body_mass[i];

    // add to parent, except for world
    if (i) {
      int j = m->body_parentid[i];
      mju_addTo3(d->subtree_com+3*j, d->subtree_com+3*i);
      mass_subtree[j] += mass_subtree[i];
    }

    // compute local com
    if (mass_subtree[i]<mjMINVAL) {
      mju_copy3(d->subtree_com+3*i, d->xipos+3*i);
    } else {
      mju_scl3(d->subtree_com+3*i, d->subtree_com+3*i,
               1.0/mjMAX(mjMINVAL, mass_subtree[i]));
    }
  }

  // map inertias to frame centered at subtree_com
  for (int i=1; i<m->nbody; i++) {
    mju_sub3(offset, d->xipos+3*i, d->subtree_com+3*m->body_rootid[i]);
    mju_inertCom(d->cinert+10*i, m->body_inertia+3*i, d->ximat+9*i,
                 offset, m->body_mass[i]);
  }

  // map motion dofs to global frame centered at subtree_com
  for (int j=0; j<m->njnt; j++) {
    // get dof address, body index
    int da = 6*m->jnt_dofadr[j];
    int bi = m->jnt_bodyid[j];

    // compute com-anchor vector
    mju_sub3(offset, d->subtree_com+3*m->body_rootid[bi], d->xanchor+3*j);

    // create motion dof
    int skip = 0;
    switch (m->jnt_type[j]) {
    case mjJNT_FREE:
      // translation components: x, y, z in global frame
      mju_zero(d->cdof+da, 18);
      for (int i=0; i<3; i++) {
        d->cdof[da+3+7*i] = 1;
      }

      // rotation components: same as ball
      skip = 18;

    case mjJNT_BALL:
      for (int i=0; i<3; i++) {
        // I_3 rotation in child frame (assume no subsequent rotations)
        axis[0] = d->xmat[9*bi+i+0];
        axis[1] = d->xmat[9*bi+i+3];
        axis[2] = d->xmat[9*bi+i+6];

        mju_dofCom(d->cdof+da+skip+6*i, axis, offset);
      }
      break;

    case mjJNT_SLIDE:
      mju_dofCom(d->cdof+da, d->xaxis+3*j, 0);
      break;

    case mjJNT_HINGE:
      mju_dofCom(d->cdof+da, d->xaxis+3*j, offset);
      break;
    }
  }

  mjFREESTACK;
}



// compute camera and light positions and orientations
void mj_camlight(const mjModel* m, mjData* d) {
  mjtNum pos[3], matT[9];

  // compute Cartesian positions and orientations of cameras
  for (int i=0; i<m->ncam; i++) {
    // default processing for fixed mode
    mj_local2Global(d, d->cam_xpos+3*i, d->cam_xmat+9*i,
                    m->cam_pos+3*i, m->cam_quat+4*i, m->cam_bodyid[i], 0);

    // get camera body id and target body id
    int id = m->cam_bodyid[i];
    int id1 = m->cam_targetbodyid[i];

    // adjust for mode
    switch (m->cam_mode[i]) {
    case mjCAMLIGHT_TRACK:
    case mjCAMLIGHT_TRACKCOM:
      // fixed global orientation
      mju_copy(d->cam_xmat+9*i, m->cam_mat0+9*i, 9);

      // position: track camera body
      if (m->cam_mode[i]==mjCAMLIGHT_TRACK) {
        mju_add3(d->cam_xpos+3*i, d->xpos+3*id, m->cam_pos0+3*i);
      }

      // position: track subtree com
      else {
        mju_add3(d->cam_xpos+3*i, d->subtree_com+3*id, m->cam_poscom0+3*i);
      }
      break;

    case mjCAMLIGHT_TARGETBODY:
    case mjCAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1>=0) {
        // get position to look at
        if (m->cam_mode[i]==mjCAMLIGHT_TARGETBODY) {
          mju_copy3(pos, d->xpos+3*id1);
        } else {
          mju_copy3(pos, d->subtree_com+3*id1);
        }

        // zaxis = -desired camera direction, in global frame
        mju_sub3(matT+6, d->cam_xpos+3*i, pos);
        mju_normalize3(matT+6);

        // xaxis: orthogonal to zaxis and to (0,0,1)
        matT[3] = 0;
        matT[4] = 0;
        matT[5] = 1;
        mju_cross(matT, matT+3, matT+6);
        mju_normalize3(matT);

        // yaxis: orthogonal to xaxis and zaxis
        mju_cross(matT+3, matT+6, matT);
        mju_normalize3(matT+3);

        // set camera frame
        mju_transpose(d->cam_xmat+9*i, matT, 3, 3);
      }
    }
  }

  // compute Cartesian positions and directions of lights
  for (int i=0; i<m->nlight; i++) {
    // default processing for fixed mode
    mj_local2Global(d, d->light_xpos+3*i, 0, m->light_pos+3*i, 0, m->light_bodyid[i], 0);
    mju_rotVecQuat(d->light_xdir+3*i, m->light_dir+3*i, d->xquat+4*m->light_bodyid[i]);

    // get light body id and target body id
    int id = m->light_bodyid[i];
    int id1 = m->light_targetbodyid[i];

    // adjust for mode
    switch (m->light_mode[i]) {
    case mjCAMLIGHT_TRACK:
    case mjCAMLIGHT_TRACKCOM:
      // fixed global orientation
      mju_copy3(d->light_xdir+3*i, m->light_dir0+3*i);

      // position: track light body
      if (m->light_mode[i]==mjCAMLIGHT_TRACK) {
        mju_add3(d->light_xpos+3*i, d->xpos+3*id, m->light_pos0+3*i);
      }

      // position: track subtree com
      else {
        mju_add3(d->light_xpos+3*i, d->subtree_com+3*id, m->light_poscom0+3*i);
      }
      break;

    case mjCAMLIGHT_TARGETBODY:
    case mjCAMLIGHT_TARGETBODYCOM:
      // only if target body is specified
      if (id1>=0) {
        // get position to look at
        if (m->light_mode[i]==mjCAMLIGHT_TARGETBODY) {
          mju_copy3(pos, d->xpos+3*id1);
        } else {
          mju_copy3(pos, d->subtree_com+3*id1);
        }

        // set dir
        mju_sub3(d->light_xdir+3*i, pos, d->light_xpos+3*i);
      }
    }

    // normalize dir
    mju_normalize3(d->light_xdir+3*i);
  }
}



// compute tendon lengths and moments
void mj_tendon(const mjModel* m, mjData* d) {
  int issparse = mj_isSparse(m), nv = m->nv, nten = m->ntendon;
  int id0, id1, idw, adr, wcnt, wbody[4], sideid;
  int tp0, tp1, tpw, NV, *chain = NULL, *buf_ind = NULL;
  int *rownnz = d->ten_J_rownnz, *rowadr = d->ten_J_rowadr, *colind = d->ten_J_colind;
  mjtNum dif[3], divisor, wpnt[12], wlen;
  mjtNum *L = d->ten_length, *J = d->ten_J;
  mjtNum *jac1, *jac2, *jacdif, *tmp, *sparse_buf = NULL;
  mjMARKSTACK;

  if (!nten) {
    return;
  }

  // allocate space
  jac1 = mj_stackAlloc(d, 3*nv);
  jac2 = mj_stackAlloc(d, 3*nv);
  jacdif = mj_stackAlloc(d, 3*nv);
  tmp = mj_stackAlloc(d, nv);
  if (issparse) {
    chain = (int*)mj_stackAlloc(d, nv);
    buf_ind = (int*)mj_stackAlloc(d, nv);
    sparse_buf = mj_stackAlloc(d, nv);
  }

  // clear results
  mju_zero(L, nten);
  wcnt = 0;

  // clear Jacobian: sparse or dense
  if (issparse) {
    memset(rownnz, 0, nten*sizeof(int));
  } else {
    mju_zero(J, nten*nv);
  }

  // loop over tendons
  for (int i=0; i<nten; i++) {
    // initialize tendon path
    adr = m->tendon_adr[i];
    d->ten_wrapadr[i] = wcnt;
    d->ten_wrapnum[i] = 0;

    // sparse Jacobian row init
    if (issparse) {
      rowadr[i] = (i>0 ? rowadr[i-1] + rownnz[i-1] : 0);
    }

    // process joint tendon
    if (m->wrap_type[adr]==mjWRAP_JOINT) {
      // process all defined joints
      for (int j=0; j<m->tendon_num[i]; j++) {
        // get joint id
        int k = m->wrap_objid[adr+j];

        // add to length
        L[i] += m->wrap_prm[adr+j] * d->qpos[m->jnt_qposadr[k]];

        // add to moment
        if (issparse) {
          J[rowadr[i] + rownnz[i]] = m->wrap_prm[adr+j];
          colind[rowadr[i] + rownnz[i]] = m->jnt_dofadr[k];
          rownnz[i]++;
        }

        // add to moment: dense
        else {
          J[i*nv + m->jnt_dofadr[k]] = m->wrap_prm[adr+j];
        }
      }

      // sort on colind if sparse: custom insertion sort
      if (issparse) {
        int x, *list = colind+rowadr[i];
        mjtNum y, *listy = J+rowadr[i];

        for (int k=1; k<rownnz[i]; k++) {
          x = list[k];
          y = listy[k];
          int j = k-1;
          while (j>=0 && list[j]>x) {
            list[j+1] = list[j];
            listy[j+1] = listy[j];
            j--;
          }
          list[j+1] = x;
          listy[j+1] = y;
        }
      }

      continue;
    }

    // process spatial tendon
    divisor = 1;
    int j = 0;
    while (j<m->tendon_num[i]-1) {
      // get 1st and 2nd object
      tp0 = m->wrap_type[adr+j];
      id0 = m->wrap_objid[adr+j];
      tp1 = m->wrap_type[adr+j+1];
      id1 = m->wrap_objid[adr+j+1];

      // pulley
      if (tp0==mjWRAP_PULLEY || tp1==mjWRAP_PULLEY) {
        // get divisor, insert obj=-2
        if (tp0==mjWRAP_PULLEY) {
          divisor = m->wrap_prm[adr+j];
          mju_zero3(d->wrap_xpos+wcnt*3);
          d->wrap_obj[wcnt] = -2;
          d->ten_wrapnum[i]++;
          wcnt++;
        }

        // move to next
        j++;
        continue;
      }

      // init sequence; assume it starts with site
      wlen = -1;
      mju_copy3(wpnt, d->site_xpos+3*id0);
      wbody[0] = m->site_bodyid[id0];

      // second object is geom: process site-geom-site
      if (tp1==mjWRAP_SPHERE || tp1==mjWRAP_CYLINDER) {
        // reassign, get 2nd site info
        tpw = tp1;
        idw = id1;
        tp1 = m->wrap_type[adr+j+2];
        id1 = m->wrap_objid[adr+j+2];

        // do wrapping, possibly get 2 extra points (wlen>=0)
        sideid = mju_round(m->wrap_prm[adr+j+1]);
        if (sideid<-1 || sideid>=m->nsite) {
          mju_error_i("Invalid sideid %d in wrap_prm", sideid);  // SHOULD NOT OCCUR
        }

        wlen = mju_wrap(wpnt+3, d->site_xpos+3*id0, d->site_xpos+3*id1,
                        d->geom_xpos+3*idw, d->geom_xmat+9*idw, m->geom_size+3*idw, tpw,
                        (sideid>=0 ? d->site_xpos+3*sideid : 0));
      } else {
        tpw = mjWRAP_NONE;
      }

      // complete sequence, accumulate lengths
      if (wlen<0) {
        mju_copy3(wpnt+3, d->site_xpos+3*id1);
        wbody[1] = m->site_bodyid[id1];
        L[i] += mju_dist3(wpnt, wpnt+3)/divisor;
      } else {
        mju_copy3(wpnt+9, d->site_xpos+3*id1);
        wbody[1] = wbody[2] = m->geom_bodyid[idw];
        wbody[3] = m->site_bodyid[id1];
        L[i] += (mju_dist3(wpnt, wpnt+3) + wlen + mju_dist3(wpnt+6, wpnt+9))/divisor;
      }

      // accumulate moments if consequtive points are in different bodies
      for (int k=0; k<(wlen<0 ? 1 : 3); k++) {
        if (wbody[k]!=wbody[k+1]) {
          // get 3D position difference, normalize
          mju_sub3(dif, wpnt+3*k+3, wpnt+3*k);
          mju_normalize3(dif);

          // sparse
          if (issparse) {
            // get endpoint Jacobians, subtract
            NV = mj_jacDifPair(m, d, chain,
                               wbody[k], wbody[k+1], wpnt+3*k, wpnt+3*k+3,
                               jac1, jac2, jacdif, NULL, NULL, NULL);

            // no dofs: skip
            if (!NV) {
              continue;
            }

            // apply chain rule to compute tendon Jacobian
            mju_mulMatTVec(tmp, jacdif, dif, 3, NV);

            // add to existing
            rownnz[i] = mju_combineSparse(J+rowadr[i], tmp, nv, 1, 1/divisor,
                                          rownnz[i], NV, colind+rowadr[i], chain,
                                          sparse_buf, buf_ind);
          }

          // dense
          else {
            // get endpoint Jacobians, subtract
            mj_jac(m, d, jac1, 0, wpnt+3*k, wbody[k]);
            mj_jac(m, d, jac2, 0, wpnt+3*k+3, wbody[k+1]);
            mju_sub(jacdif, jac2, jac1, 3*nv);

            // apply chain rule to compute tendon Jacobian
            mju_mulMatTVec(tmp, jacdif, dif, 3, nv);

            // add to existing
            mju_addToScl(J + i*nv, tmp, 1/divisor, nv);
          }
        }
      }

      // assign to wrap
      mju_copy(d->wrap_xpos+wcnt*3, wpnt, (wlen<0 ? 3:9));
      d->wrap_obj[wcnt] = -1;
      if (wlen>=0) {
        d->wrap_obj[wcnt+1] = d->wrap_obj[wcnt+2] = idw;
      }
      d->ten_wrapnum[i] += (wlen<0 ? 1:3);
      wcnt += (wlen<0 ? 1:3);

      // advance
      j += (tpw!=mjWRAP_NONE ? 2 : 1);

      // assign last site before pulley or tendon end
      if (j==m->tendon_num[i]-1 || m->wrap_type[adr+j+1]==mjWRAP_PULLEY) {
        mju_copy3(d->wrap_xpos+wcnt*3, d->site_xpos+3*id1);
        d->wrap_obj[wcnt] = -1;
        d->ten_wrapnum[i]++;
        wcnt++;
      }
    }
  }

  mjFREESTACK;
}



// compute actuator/transmission lengths and moments
void mj_transmission(const mjModel* m, mjData* d) {
  int id, idslider, ok, nv = m->nv, nu = m->nu;
  mjtNum det, sdet, av, rod, axis[3], vec[3], dlda[3], dldv[3], quat[4];
  mjtNum wrench[6], gearAxis[3];
  mjtNum *jac, *jacA, *jacS;
  mjtNum *length = d->actuator_length, *moment = d->actuator_moment, *gear;
  mjtNum *jacref = NULL, *moment_tmp = NULL;  // required for site actuators
  mjMARKSTACK;

  if (!nu) {
    return;
  }

  // allocate space, clear moments
  jac  = mj_stackAlloc(d, 3*nv);
  jacA = mj_stackAlloc(d, 3*nv);
  jacS = mj_stackAlloc(d, 3*nv);
  mju_zero(moment, nu*nv);

  // define variables required for body transmission, don't allocate
  int issparse = mj_isSparse(m);
  mjtNum* efc_force = NULL;  // used as marker for allocation requirement
  mjtNum *moment_exclude, *jacdifp, *jac1p, *jac2p;
  int *chain;

  // compute lengths and moments
  for (int i=0; i<nu; i++) {
    // extract info
    id = m->actuator_trnid[2*i];
    idslider = m->actuator_trnid[2*i+1];    // for slider-crank only
    gear = m->actuator_gear+6*i;

    // process according to transmission type
    switch (m->actuator_trntype[i]) {
    case mjTRN_JOINT:                   // joint
    case mjTRN_JOINTINPARENT:           // joint, force in parent frame
      // slide and hinge joint: scalar gear
      if (m->jnt_type[id]==mjJNT_SLIDE || m->jnt_type[id]==mjJNT_HINGE) {
        length[i] = d->qpos[m->jnt_qposadr[id]]*gear[0];
        moment[i*nv + m->jnt_dofadr[id]] = gear[0];
      }

      // ball joint: 3D wrench gear
      else if (m->jnt_type[id]==mjJNT_BALL) {
        // j: qpos start address
        int j = m->jnt_qposadr[id];

        // axis: expmap representation of quaternion
        mju_quat2Vel(axis, d->qpos+j, 1);

        // gearAxis: rotate to parent frame if necessary
        if (m->actuator_trntype[i]==mjTRN_JOINT) {
          mju_copy3(gearAxis, gear);
        } else {
          mju_negQuat(quat, d->qpos+j);
          mju_rotVecQuat(gearAxis, gear, quat);
        }

        // length: axis*gearAxis
        length[i] = mju_dot3(axis, gearAxis);

        // j: dof start address
        j = m->jnt_dofadr[id];

        // moment: gearAxis
        mju_copy3(moment+i*nv+j, gearAxis);
      }

      // free joint: 6D wrench gear
      else {
        // cannot compute meaningful length, set to 0
        length[i] = 0;

        // j: qpos start address
        int j = m->jnt_qposadr[id];

        // vec: translational components
        mju_copy3(vec, d->qpos+j);

        // axis: expmap representation of quaternion
        mju_quat2Vel(axis, d->qpos+j+3, 1);

        // gearAxis: rotate to world frame if necessary
        if (m->actuator_trntype[i]==mjTRN_JOINT) {
          mju_copy3(gearAxis, gear+3);
        } else {
          mju_negQuat(quat, d->qpos+j+3);
          mju_rotVecQuat(gearAxis, gear+3, quat);
        }

        // j: dof start address
        j = m->jnt_dofadr[id];

        // moment: gear(tran), gearAxis
        mju_copy3(moment+i*nv+j, gear);
        mju_copy3(moment+i*nv+j+3, gearAxis);
      }
      break;

    case mjTRN_SLIDERCRANK:             // slider-crank
      // get data
      rod = m->actuator_cranklength[i];
      axis[0] = d->site_xmat[9*idslider+2];
      axis[1] = d->site_xmat[9*idslider+5];
      axis[2] = d->site_xmat[9*idslider+8];
      mju_sub3(vec, d->site_xpos+3*id, d->site_xpos+3*idslider);

      // compute length and determinant
      //  length = a'*v - sqrt(det);  det = (a'*v)^2 + r^2 - v'*v)
      av = mju_dot3(vec, axis);
      det = av*av + rod*rod - mju_dot3(vec, vec);
      ok = 1;
      if (det<=0) {
        ok = 0;
        sdet = 0;
        length[i] = av;
      } else {
        sdet = mju_sqrt(det);
        length[i] = av - sdet;
      }

      // compute derivatives of length w.r.t. vec and axis
      if (ok) {
        mju_scl3(dldv, axis, 1-av/sdet);
        mju_scl3(dlda, vec, 1/sdet);        // use dlda as temp
        mju_addTo3(dldv, dlda);

        mju_scl3(dlda, vec, 1-av/sdet);
      } else {
        mju_copy3(dlda, vec);
        mju_copy3(dldv, axis);
      }

      // get Jacobians of axis(jacA) and vec(jac)
      mj_jacPointAxis(m, d, jacS, jacA, d->site_xpos+3*idslider,
                      axis, m->site_bodyid[idslider]);
      mj_jacSite(m, d, jac, 0, id);
      mju_subFrom(jac, jacS, 3*nv);

      // apply chain rule
      for (int j=0; j<nv; j++) {
        for (int k=0; k<3; k++) {
          moment[i*nv+j] += dlda[k]*jacA[k*nv+j] + dldv[k]*jac[k*nv+j];
        }
      }

      // scale by gear ratio
      length[i] *= gear[0];
      for (int j = 0; j<nv; j++) {
        moment[i*nv + j] *= gear[0];
      }
      break;

    case mjTRN_TENDON:                  // tendon
      length[i] = d->ten_length[id]*gear[0];

      // moment: sparse or dense
      if (mj_isSparse(m)) {
        int end = d->ten_J_rowadr[id] + d->ten_J_rownnz[id];
        for (int j=d->ten_J_rowadr[id]; j<end; j++) {
          moment[i*nv + d->ten_J_colind[j]] = d->ten_J[j] * gear[0];
        }
      } else {
        mju_scl(moment + i*nv, d->ten_J + id*nv, gear[0], nv);
      }
      break;

    case mjTRN_SITE:                    // site
      // get site translation (jac) and rotation (jacS) Jacobians in global frame
      mj_jacSite(m, d, jac, jacS, id);

      // reference site undefined
      if (m->actuator_trnid[2*i+1] == -1) {
        // cannot compute meaningful length, set to 0
        length[i] = 0;

        // wrench: gear expressed in global frame
        mju_rotVecMat(wrench, gear, d->site_xmat+9*id);      // translation
        mju_rotVecMat(wrench+3, gear+3, d->site_xmat+9*id);  // rotation

        // moment: global Jacobian projected on wrench
        mju_mulMatTVec(moment+i*nv, jac, wrench, 3, nv);     // translation
        mju_mulMatTVec(jac, jacS, wrench+3, 3, nv);          // rotation
        mju_addTo(moment+i*nv, jac, nv);                     // add the two
      }

      // reference site defined
      else {
        int refid = m->actuator_trnid[2*i+1];
        if (!jacref) jacref = mj_stackAlloc(d, 3*nv);

        // clear length
        length[i] = 0;

        // translational transmission
        if (!mju_isZero(gear, 3)) {
          // vec: site position in reference site frame
          mju_sub3(vec, d->site_xpos+3*id, d->site_xpos+3*refid);
          mju_rotVecMatT(vec, vec, d->site_xmat+9*refid);

          // length: dot product with gear
          length[i] += mju_dot3(vec, gear);

          // jacref: global Jacobian of reference site
          mj_jacSite(m, d, jacref, NULL, refid);

          // subtract jacref from jac
          mju_subFrom(jac, jacref, 3*nv);

          // wrench: translational gear expressed in global frame
          mju_rotVecMat(wrench, gear, d->site_xmat+9*refid);

          // moment: global Jacobian projected on wrench
          mju_mulMatTVec(moment+i*nv, jac, wrench, 3, nv);
        }

        // rotational transmission
        if (!mju_isZero(gear+3, 3)) {
          mjtNum refquat[4];

          // get site and refsite quats from parent bodies (avoiding mju_mat2Quat)
          mju_mulQuat(quat, m->site_quat+4*id, d->xquat+4*m->site_bodyid[id]);
          mju_mulQuat(refquat, m->site_quat+4*refid, d->xquat+4*m->site_bodyid[refid]);

          // convert difference to expmap (axis-angle)
          mju_subQuat(vec, quat, refquat);

          // add length: dot product with gear
          length[i] += mju_dot3(vec, gear+3);

          // jacref: global rotational Jacobian of reference site
          mj_jacSite(m, d, NULL, jacref, refid);

          // subtract jacref from jacS
          mju_subFrom(jacS, jacref, 3*nv);

          // wrench: rotational gear expressed in global frame
          mju_rotVecMat(wrench, gear+3, d->site_xmat+9*refid);

          // moment_tmp: global Jacobian projected on wrench, add to moment
          if (!moment_tmp) moment_tmp = mj_stackAlloc(d, nv);
          mju_mulMatTVec(moment_tmp, jacS, wrench, 3, nv);
          mju_addTo(moment+i*nv, moment_tmp, nv);
        }
      }

      break;

    case mjTRN_BODY:                  // body (adhesive contacts)
      // cannot compute meaningful length, set to 0
      length[i] = 0;

      // moment is average of all contact normal Jacobians
      {
        // allocate stack variables for the first mjTRN_BODY
        if (!efc_force) {
          efc_force = mj_stackAlloc(d, d->nefc);
          moment_exclude = mj_stackAlloc(d, nv);
          jacdifp = mj_stackAlloc(d, 3*nv);
          jac1p = mj_stackAlloc(d, 3*nv);
          jac2p = mj_stackAlloc(d, 3*nv);
          chain = issparse ? (int*)mj_stackAlloc(d, nv) : NULL;
        }

        // clear efc_force and moment_exclude
        mju_zero(efc_force, d->nefc);
        mju_zero(moment_exclude, nv);

        // count all relevant contacts, accumulate Jacobians
        int counter = 0;
        for (int j=0; j<d->ncon; j++) {
          const mjContact* con = d->contact+j;
          int b1 = m->geom_bodyid[con->geom1];
          int b2 = m->geom_bodyid[con->geom2];

          // irrelevant contact, continue
          if (b1 != id && b2 != id) {
            continue;
          }

          // mark contact normals in efc_force
          if (!con->exclude) {
            counter++;

            // condim 1 or elliptic cones: normal is in the first row
            if (con->dim == 1 || m->opt.cone==mjCONE_ELLIPTIC) {
              efc_force[con->efc_address] = 1;
            }

            // pyramidal cones: average all pyramid directions
            else {
              int npyramid = con->dim-1;  // number of frictional directions
              for (int k=0; k<2*npyramid; k++) {
                efc_force[con->efc_address+k] = 0.5/npyramid;
              }
            }
          }

          // excluded contact, get sparse or dense Jacobian, accumulate
          else if (con->exclude == 1) {
            counter++;

            // get Jacobian difference
            int NV = mj_jacDifPair(m, d, chain, b1, b2, con->pos, con->pos,
                                   jac1p, jac2p, jacdifp, NULL, NULL, NULL);

            // project Jacobian along the normal of the contact frame
            mju_mulMatMat(jac, con->frame, jacdifp, 1, 3, NV);

            // accumulate in moment_exclude
            if (issparse) {
              for (int k=0; k<NV; k++) {
                moment_exclude[chain[k]] += jac[k];
              }
            } else {
              mju_addTo(moment_exclude, jac, nv);
            }
          }
        }

        // moment is average over contact normal Jacobians, make negative for adhesion
        if (counter) {
          // accumulate active contact Jacobians into moment
          mj_mulJacTVec(m, d, moment+i*nv, efc_force);

          // add Jacobians from excluded contacts
          mju_addTo(moment+i*nv, moment_exclude, nv);

          // normalize by total contacts, flip sign
          mju_scl(moment+i*nv, moment+i*nv, -1.0/counter, nv);
        }
      }
      break;

    default:
      mju_error_i("Unknown transmission type %d", m->actuator_trntype[i]);  // SHOULD NOT OCCUR
    }
  }

  mjFREESTACK;
}



//-------------------------- inertia ---------------------------------------------------------------

// composite rigid body inertia algorithm, with skipsimple
void mj_crbSkip(const mjModel* m, mjData* d, int skipsimple) {
  mjtNum tmp[6];
  mjtNum* crb = d->crb;

  // crb = cinert
  mju_copy(crb, d->cinert, 10*m->nbody);

  // backward pass over bodies, accumulate composite inertias
  for (int i=m->nbody-1; i>0; i--) {
    if (m->body_parentid[i]>0) {
      mju_addTo(crb+10*m->body_parentid[i], crb+10*i, 10);
    }
  }

  // clear qM
  mju_zero(d->qM, m->nM);

  // dense backward pass over dofs
  for (int i=m->nv-1; i>=0; i--) {
    // copy
    if (skipsimple && m->dof_simplenum[i]) {
      d->qM[m->dof_Madr[i]] = m->dof_M0[i];
    }

    // compute
    else {
      // init M(i,i) with armature inertia
      int Madr_ij = m->dof_Madr[i];
      d->qM[Madr_ij] = m->dof_armature[i];

      // precompute tmp = crb * cdof
      mju_mulInertVec(tmp, crb+10*m->dof_bodyid[i], d->cdof+6*i);

      // sparse backward pass over ancestors
      int j = i;
      while (j>=0) {
        // M(i,j) += cdof_j * crb_body(i) * cdof_i = cdof_j * tmp
        d->qM[Madr_ij] += mju_dot(d->cdof+6*j, tmp, 6);

        // advance to parent
        j = m->dof_parentid[j];
        Madr_ij++;
      }
    }
  }
}



// composite rigid body inertia algorithm
void mj_crb(const mjModel* m, mjData* d) {
  mj_crbSkip(m, d, 1);
}



// sparse L'*D*L factorizaton of inertia-like matrix M, assumed spd
void mj_factorI(const mjModel* m, mjData* d, const mjtNum* M, mjtNum* qLD, mjtNum* qLDiagInv,
                mjtNum* qLDiagSqrtInv) {
  int cnt;
  int Madr_kk, Madr_ki;
  mjtNum tmp;

  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // copy M into LD
  mju_copy(qLD, M, m->nM);

  // dense backward loop over dofs (regular only, simple diagonal already copied)
  for (int k=nv-1; k>=0; k--) {
    // get address of M(k,k)
    Madr_kk = dof_Madr[k];

    // check for small/negative numbers on diagonal
    if (qLD[Madr_kk]<mjMINVAL) {
      mj_warning(d, mjWARN_INERTIA, k);
      qLD[Madr_kk] = mjMINVAL;
    }

    // skip the rest if simple
    if (m->dof_simplenum[k]) {
      continue;
    }

    // sparse backward loop over ancestors of k (excluding k)
    Madr_ki = Madr_kk + 1;
    int i = dof_parentid[k];
    while (i>=0) {
      tmp = qLD[Madr_ki] / qLD[Madr_kk];          // tmp = M(k,i) / M(k,k)

      // get number of ancestors of i (including i)
      if (i<nv-1) {
        cnt = dof_Madr[i+1] - dof_Madr[i];
      } else {
        cnt = m->nM - dof_Madr[i+1];
      }

      // M(i,j) -= M(k,j) * tmp
      mju_addToScl(qLD+dof_Madr[i], qLD+Madr_ki, -tmp, cnt);

      qLD[Madr_ki] = tmp;                         // M(k,i) = tmp

      // advance to i's parent
      i = dof_parentid[i];
      Madr_ki++;
    }
  }

  // compute 1/diag(D), 1/sqrt(diag(D))
  for (int i=0; i<nv; i++) {
    mjtNum qLDi = qLD[dof_Madr[i]];
    qLDiagInv[i] = 1.0/qLDi;
    if (qLDiagSqrtInv) {
      qLDiagSqrtInv[i] = 1.0/mju_sqrt(qLDi);
    }
  }
}



// sparse L'*D*L factorizaton of the inertia matrix M, assumed spd
void mj_factorM(const mjModel* m, mjData* d) {
  mj_factorI(m, d, d->qM, d->qLD, d->qLDiagInv, d->qLDiagSqrtInv);
}



// sparse backsubstitution:  x = inv(L'*D*L)*y
//  L is in lower triangle of qLD; D is on diagonal of qLD
//  handle n vectors at once
void mj_solveLD(const mjModel* m, mjtNum* x, const mjtNum* y, int n,
                const mjtNum* qLD, const mjtNum* qLDiagInv) {
  mjtNum tmp;

  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // x = y
  if (x != y) {
    mju_copy(x, y, n*nv);
  }

  // single vector
  if (n==1) {
    // x <- inv(L') * x; skip simple, exploit sparsity of input vector
    for (int i=nv-1; i>=0; i--) {
      if (!m->dof_simplenum[i] && (tmp = x[i])) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        while (j>=0) {
          x[j] -= qLD[Madr_ij++]*tmp;         // x(j) -= L(i,j) * x(i)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i<nv; i++) {
      x[i] *= qLDiagInv[i];  // x(i) /= L(i,i)
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i<nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        tmp = x[i];
        while (j>=0) {
          tmp -= qLD[Madr_ij++]*x[j];             // x(i) -= L(i,j) * x(j)

          // advance to parent
          j = dof_parentid[j];
        }
        x[i] = tmp;
      }
    }
  }

  // multiple vectors
  else {
    int offset;

    // x <- inv(L') * x; skip simple
    for (int i=nv-1; i>=0; i--) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        while (j>=0) {
          // process all vectors, exploit sparsity
          for (offset=0; offset<n*nv; offset+=nv)
            if ((tmp = x[i+offset])) {
              x[j+offset] -= qLD[Madr_ij]*tmp;  // x(j) -= L(i,j) * x(i)
            }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }

    // x <- inv(D) * x
    for (int i=0; i<nv; i++) {
      for (offset=0; offset<n*nv; offset+=nv) {
        x[i+offset] *= qLDiagInv[i];  // x(i) /= L(i,i)
      }
    }

    // x <- inv(L) * x; skip simple
    for (int i=0; i<nv; i++) {
      if (!m->dof_simplenum[i]) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        tmp = x[i+offset];
        while (j>=0) {
          // process all vectors
          for (offset=0; offset<n*nv; offset+=nv) {
            x[i+offset] -= qLD[Madr_ij]*x[j+offset];  // x(i) -= L(i,j) * x(j)
          }

          // advance to parent
          Madr_ij++;
          j = dof_parentid[j];
        }
      }
    }
  }
}



// sparse backsubstitution:  x = inv(L'*D*L)*y
//  use factorization in d
void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n) {
  mj_solveLD(m, x, y, n, d->qLD, d->qLDiagInv);
}



// half of sparse backsubstitution:  x = sqrt(inv(D))*inv(L')*y
void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n) {
  // local copies of key variables
  mjtNum* qLD = d->qLD;
  mjtNum* qLDiagSqrtInv = d->qLDiagSqrtInv;
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // x = y
  mju_copy(x, y, n * nv);

  // loop over the n input vectors
  for (int ivec=0; ivec<n; ivec++) {
    int offset = ivec*nv;

    // x <- inv(L') * x; skip simple, exploit sparsity of input vector
    for (int i=nv-1; i>=0; i--) {
      mjtNum tmp;
      if (!m->dof_simplenum[i] && (tmp = x[i+offset])) {
        // init
        int Madr_ij = dof_Madr[i]+1;
        int j = dof_parentid[i];

        // traverse ancestors backwards
        while (j>=0) {
          x[j+offset] -= qLD[Madr_ij++] * tmp;        // x(j) -= L(i,j) * x(i)

          // advance to parent
          j = dof_parentid[j];
        }
      }
    }

    // x <- sqrt(inv(D)) * x
    for (int i=0; i<nv; i++) {
      x[i+offset] *= qLDiagSqrtInv[i];  // x(i) /= sqrt(L(i,i))
    }
  }
}



//---------------------------------- velocity ------------------------------------------------------

// compute cvel, cdof_dot
void mj_comVel(const mjModel* m, mjData* d) {
  mjtNum tmp[6], cvel[6], cdofdot[36];

  // set world vel to 0
  mju_zero(d->cvel, 6);

  // forward pass over bodies
  for (int i=1; i<m->nbody; i++) {
    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cvel = cvel_parent
    mju_copy(cvel, d->cvel+6*m->body_parentid[i], 6);

    // cvel = cvel_parent + cdof * qvel,  cdofdot = cvel x cdof
    for (int j=0; j<m->body_dofnum[i]; j++) {
      // compute cvel and cdofdot
      switch (m->jnt_type[m->dof_jntid[bda+j]]) {
      case mjJNT_FREE:
        // cdofdot = 0
        mju_zero(cdofdot, 18);

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*bda, d->qvel+bda, 3);
        mju_addTo(cvel, tmp, 6);

        // continue with rotations
        j += 3;

      case mjJNT_BALL:
        // compute all 3 cdofdots using parent velocity
        for (int k=0; k<3; k++) {
          mju_crossMotion(cdofdot+6*(j+k), cvel, d->cdof+6*(bda+j+k));
        }

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 3);
        mju_addTo(cvel, tmp, 6);

        // adjust for 3-dof joint
        j += 2;
        break;

      default:
        // in principle we should use the new velocity to compute cdofdot,
        // but it makes no difference becase crossMotion(cdof, cdof) = 0,
        // and using the old velocity may be more accurate numerically
        mju_crossMotion(cdofdot+6*j, cvel, d->cdof+6*(bda+j));

        // update velocity
        mju_mulDofVec(tmp, d->cdof+6*(bda+j), d->qvel+bda+j, 1);
        mju_addTo(cvel, tmp, 6);
      }
    }

    // assign cvel, cdofdot
    mju_copy(d->cvel+6*i, cvel, 6);
    mju_copy(d->cdof_dot+6*bda, cdofdot, 6*m->body_dofnum[i]);
  }
}



// passive forces
void mj_passive(const mjModel* m, mjData* d) {
  int issparse = mj_isSparse(m);
  int nv = m->nv;
  mjtNum dif[3], frc, stiffness, damping;

  // clear passive force
  mju_zero(d->qfrc_passive, m->nv);

  // disabled: return
  if (mjDISABLED(mjDSBL_PASSIVE)) {
    return;
  }

  // joint-level springs
  for (int i=0; i<m->njnt; i++) {
    stiffness = m->jnt_stiffness[i];

    int padr = m->jnt_qposadr[i];
    int dadr = m->jnt_dofadr[i];

    switch (m->jnt_type[i]) {
    case mjJNT_FREE:
      // apply force
      d->qfrc_passive[dadr+0] -= stiffness*(d->qpos[padr+0] - m->qpos_spring[padr+0]);
      d->qfrc_passive[dadr+1] -= stiffness*(d->qpos[padr+1] - m->qpos_spring[padr+1]);
      d->qfrc_passive[dadr+2] -= stiffness*(d->qpos[padr+2] - m->qpos_spring[padr+2]);

      // continue with rotations
      dadr += 3;
      padr += 3;

    case mjJNT_BALL:
      // covert quatertion difference into angular "velocity"
      mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);

      // apply torque
      d->qfrc_passive[dadr+0] -= stiffness*dif[0];
      d->qfrc_passive[dadr+1] -= stiffness*dif[1];
      d->qfrc_passive[dadr+2] -= stiffness*dif[2];
      break;

    case mjJNT_SLIDE:
    case mjJNT_HINGE:
      // apply force or torque
      d->qfrc_passive[dadr] -= stiffness*(d->qpos[padr] - m->qpos_spring[padr]);
      break;
    }
  }

  // dof-level dampers
  for (int i=0; i<m->nv; i++) {
    damping = m->dof_damping[i];
    d->qfrc_passive[i] -= damping*d->qvel[i];
  }

  // tendon-level spring-dampers
  for (int i=0; i<m->ntendon; i++) {
    stiffness = m->tendon_stiffness[i];
    damping = m->tendon_damping[i];

    // compute spring-damper linear force along tendon
    frc = -stiffness * (d->ten_length[i] - m->tendon_lengthspring[i])
          -damping * d->ten_velocity[i];

    // transform to joint torque, add to qfrc_passive: dense or sparse
    if (issparse) {
      int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
      for (int j=d->ten_J_rowadr[i]; j<end; j++) {
        d->qfrc_passive[d->ten_J_colind[j]] += d->ten_J[j] * frc;
      }
    } else {
      mju_addToScl(d->qfrc_passive, d->ten_J+i*nv, frc, nv);
    }
  }

  // body-level viscosity, lift and drag
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
        mj_ellipsoidFluidModel(m, d, i);
      } else {
        mj_inertiaBoxFluidModel(m, d, i);
      }
    }
  }

  // user callback: add custom passive forces
  if (mjcb_passive) {
    mjcb_passive(m, d);
  }
}



// subtree linear velocity and angular momentum
void mj_subtreeVel(const mjModel* m, mjData* d) {
  mjtNum dx[3], dv[3], dp[3], dL[3];
  mjMARKSTACK;
  mjtNum* body_vel = mj_stackAlloc(d, 6*m->nbody);

  // bodywise quantities
  for (int i=0; i<m->nbody; i++) {
    // compute and save body velocity
    mj_objectVelocity(m, d, mjOBJ_BODY, i, body_vel+6*i, 0);

    // body linear momentum
    mju_scl3(d->subtree_linvel+3*i, body_vel+6*i+3, m->body_mass[i]);

    // body angular momentum
    mju_rotVecMatT(dv, body_vel+6*i, d->ximat+9*i);
    dv[0] *= m->body_inertia[3*i];
    dv[1] *= m->body_inertia[3*i+1];
    dv[2] *= m->body_inertia[3*i+2];
    mju_rotVecMat(d->subtree_angmom+3*i, dv, d->ximat+9*i);
  }

  // subtree linvel
  for (int i=m->nbody-1; i>=0; i--) {
    // non-world: add linear momentum to parent
    if (i) {
      mju_addTo3(d->subtree_linvel+3*m->body_parentid[i], d->subtree_linvel+3*i);
    }

    // convert linear momentum to linear velocity
    mju_scl3(d->subtree_linvel+3*i, d->subtree_linvel+3*i,
             1/mjMAX(mjMINVAL, m->body_subtreemass[i]));
  }

  // subtree angmom
  for (int i=m->nbody-1; i>0; i--) {
    int parent = m->body_parentid[i];

    // momentum wrt body i
    mju_sub3(dx, d->xipos+3*i, d->subtree_com+3*i);
    mju_sub3(dv, body_vel+6*i+3, d->subtree_linvel+3*i);
    mju_scl3(dp, dv, m->body_mass[i]);
    mju_cross(dL, dx, dp);

    // add to subtree i
    mju_addTo3(d->subtree_angmom+3*i, dL);

    // add to parent
    mju_addTo3(d->subtree_angmom+3*parent, d->subtree_angmom+3*i);

    // momentum wrt parent
    mju_sub3(dx, d->subtree_com+3*i, d->subtree_com+3*parent);
    mju_sub3(dv, d->subtree_linvel+3*i, d->subtree_linvel+3*parent);
    mju_scl3(dv, dv, m->body_subtreemass[i]);
    mju_cross(dL, dx, dv);

    // add to parent
    mju_addTo3(d->subtree_angmom+3*parent, dL);
  }

  mjFREESTACK;
}



//---------------------------------- fluid models --------------------------------------------------

// fluid forces based on inertia-box approximation
void mj_inertiaBoxFluidModel(const mjModel* m, mjData* d, int i) {
  mjtNum lvel[6], wind[6], lwind[6], lfrc[6], bfrc[6], box[3], diam, *inertia;
  inertia = m->body_inertia + 3*i;
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
  mju_zero(lfrc, 6);

  // set viscous force and torque
  if (m->opt.viscosity>0) {
    // diameter of sphere approximation
    diam = (box[0] + box[1] + box[2])/3.0;

    // angular viscosity
    mju_scl3(lfrc, lvel, -mjPI*diam*diam*diam*m->opt.viscosity);

    // linear viscosity
    mju_scl3(lfrc+3, lvel+3, -3.0*mjPI*diam*m->opt.viscosity);
  }

  // add lift and drag force and torque
  if (m->opt.density>0) {
    // force
    lfrc[3] -= 0.5*m->opt.density*box[1]*box[2]*mju_abs(lvel[3])*lvel[3];
    lfrc[4] -= 0.5*m->opt.density*box[0]*box[2]*mju_abs(lvel[4])*lvel[4];
    lfrc[5] -= 0.5*m->opt.density*box[0]*box[1]*mju_abs(lvel[5])*lvel[5];

    // torque
    lfrc[0] -= m->opt.density*box[0]*(box[1]*box[1]*box[1]*box[1]+box[2]*box[2]*box[2]*box[2])*
               mju_abs(lvel[0])*lvel[0]/64.0;
    lfrc[1] -= m->opt.density*box[1]*(box[0]*box[0]*box[0]*box[0]+box[2]*box[2]*box[2]*box[2])*
               mju_abs(lvel[1])*lvel[1]/64.0;
    lfrc[2] -= m->opt.density*box[2]*(box[0]*box[0]*box[0]*box[0]+box[1]*box[1]*box[1]*box[1])*
               mju_abs(lvel[2])*lvel[2]/64.0;
  }
  // rotate to global orientation: lfrc -> bfrc
  mju_rotVecMat(bfrc, lfrc, d->ximat+9*i);
  mju_rotVecMat(bfrc+3, lfrc+3, d->ximat+9*i);

  // apply force and torque to body com
  mj_applyFT(m, d, bfrc+3, bfrc, d->xipos+3*i, i, d->qfrc_passive);
}



// fluid forces based on ellipsoid approximation
void mj_ellipsoidFluidModel(const mjModel* m, mjData* d, int bodyid) {
  mjtNum lvel[6], wind[6], lwind[6], lfrc[6], bfrc[6];
  mjtNum geom_interaction_coef, magnus_lift_coef, kutta_lift_coef;
  mjtNum semiaxes[3], virtual_mass[3], virtual_inertia[3];
  mjtNum blunt_drag_coef, slender_drag_coef, ang_drag_coef;

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

    // initialize viscous force and torque
    mju_zero(lfrc, 6);

    // added-mass forces and torques
    mj_addedMassForces(lvel, NULL, m->opt.density, virtual_mass, virtual_inertia, lfrc);

    // lift force orthogonal to lvel from Kutta-Joukowski theorem
    mj_viscousForces(lvel, m->opt.density, m->opt.viscosity, semiaxes, magnus_lift_coef,
                     kutta_lift_coef, blunt_drag_coef, slender_drag_coef, ang_drag_coef, lfrc);

    // scale by geom_interaction_coef (1.0 by default)
    mju_scl(lfrc, lfrc, geom_interaction_coef, 6);

    // rotate to global orientation: lfrc -> bfrc
    mju_rotVecMat(bfrc, lfrc, d->geom_xmat + 9*geomid);
    mju_rotVecMat(bfrc+3, lfrc+3, d->geom_xmat + 9*geomid);

    // apply force and torque to body com
    mj_applyFT(m, d, bfrc+3, bfrc,
               d->geom_xpos + 3*geomid,  // point where FT is generated
               bodyid, d->qfrc_passive);
  }
}


// compute forces due to fluid mass moving with the body
void mj_addedMassForces(const mjtNum local_vels[6], const mjtNum local_accels[6],
                        const mjtNum fluid_density, const mjtNum virtual_mass[3],
                        const mjtNum virtual_inertia[3], mjtNum local_force[6])
{
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

  // disabled due to dependency on qacc but included for completeness
  if (local_accels) {
    local_force[0] -= fluid_density * virtual_inertia[0] * local_accels[0];
    local_force[1] -= fluid_density * virtual_inertia[1] * local_accels[1];
    local_force[2] -= fluid_density * virtual_inertia[2] * local_accels[2];
    local_force[3] -= fluid_density * virtual_mass[0] * local_accels[3];
    local_force[4] -= fluid_density * virtual_mass[1] * local_accels[4];
    local_force[5] -= fluid_density * virtual_mass[2] * local_accels[5];
  }

  mjtNum added_mass_force[3], added_mass_torque1[3], added_mass_torque2[3];
  mju_cross(added_mass_force, virtual_lin_mom, ang_vel);
  mju_cross(added_mass_torque1, virtual_lin_mom, lin_vel);
  mju_cross(added_mass_torque2, virtual_ang_mom, ang_vel);

  mju_addTo3(local_force, added_mass_torque1);
  mju_addTo3(local_force, added_mass_torque2);
  mju_addTo3(local_force+3, added_mass_force);
}


// inlined helper functions
static inline mjtNum mji_pow4(const mjtNum val) {
  return (val*val)*(val*val);
}

static inline mjtNum mji_pow2(const mjtNum val) {
  return val*val;
}

static inline mjtNum mji_ellipsoid_max_moment(const mjtNum size[3], const int dir) {
  const mjtNum d0 = size[dir], d1 = size[(dir+1) % 3], d2 = size[(dir+2) % 3];
  return 8.0/15.0 * mjPI * d0 * mji_pow4(mju_max(d1, d2));
}



// lift and drag forces due to motion in the fluid
void mj_viscousForces(
    const mjtNum local_vels[6], const mjtNum fluid_density,
    const mjtNum fluid_viscosity, const mjtNum size[3],
    const mjtNum magnus_lift_coef, const mjtNum kutta_lift_coef,
    const mjtNum blunt_drag_coef, const mjtNum slender_drag_coef,
    const mjtNum ang_drag_coef, mjtNum local_force[6])
{
  const mjtNum lin_vel[3] = {local_vels[3], local_vels[4], local_vels[5]};
  const mjtNum ang_vel[3] = {local_vels[0], local_vels[1], local_vels[2]};
  const mjtNum volume = 4.0/3.0 * mjPI * size[0] * size[1] * size[2];
  const mjtNum d_max = mju_max(mju_max(size[0], size[1]), size[2]);
  const mjtNum d_min = mju_min(mju_min(size[0], size[1]), size[2]);
  const mjtNum d_mid = size[0] + size[1] + size[2] - d_max - d_min;
  const mjtNum A_max = mjPI * d_max * d_mid;

  mjtNum magnus_force[3];
  mju_cross(magnus_force, ang_vel, lin_vel);
  magnus_force[0] *= magnus_lift_coef * fluid_density * volume;
  magnus_force[1] *= magnus_lift_coef * fluid_density * volume;
  magnus_force[2] *= magnus_lift_coef * fluid_density * volume;

  // the dot product between velocity and the normal to the cross-section that
  // defines the body's projection along velocity is proj_num/sqrt(proj_denom)
  const mjtNum proj_denom = mji_pow4(size[1] * size[2]) * mji_pow2(lin_vel[0]) +
                            mji_pow4(size[2] * size[0]) * mji_pow2(lin_vel[1]) +
                            mji_pow4(size[0] * size[1]) * mji_pow2(lin_vel[2]);
  const mjtNum proj_num = mji_pow2(size[1] * size[2] * lin_vel[0]) +
                          mji_pow2(size[2] * size[0] * lin_vel[1]) +
                          mji_pow2(size[0] * size[1] * lin_vel[2]);

  // projected surface in the direction of the velocity
  const mjtNum A_proj = mjPI * mju_sqrt(proj_denom/mju_max(mjMINVAL, proj_num));

  // not-unit normal to ellipsoid's projected area in the direction of velocity
  const mjtNum norm[3] = {
    mji_pow2(size[1] * size[2]) * lin_vel[0],
    mji_pow2(size[2] * size[0]) * lin_vel[1],
    mji_pow2(size[0] * size[1]) * lin_vel[2]
  };

  // cosine between velocity and normal to the surface
  // divided by proj_denom instead of sqrt(proj_denom) to account for skipped normalization in norm
  const mjtNum cos_alpha = proj_num / mju_max(
      mjMINVAL, mju_norm3(lin_vel) * proj_denom);
  mjtNum kutta_circ[3];
  mju_cross(kutta_circ, norm, lin_vel);
  kutta_circ[0] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  kutta_circ[1] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  kutta_circ[2] *= kutta_lift_coef * fluid_density * cos_alpha * A_proj;
  mjtNum kutta_force[3];
  mju_cross(kutta_force, kutta_circ, lin_vel);

  // viscous force and torque in Stokes flow, analytical for spherical bodies
  const mjtNum eq_sphere_D = 2.0/3.0 * (size[0] + size[1] + size[2]);
  const mjtNum lin_visc_force_coef = 3.0 * mjPI * eq_sphere_D;
  const mjtNum lin_visc_torq_coef = mjPI * eq_sphere_D*eq_sphere_D*eq_sphere_D;

  // moments of inertia used to compute angular quadratic drag
  const mjtNum I_max = 8.0/15.0 * mjPI * d_mid * mji_pow4(d_max);
  const mjtNum II[3] = {
    mji_ellipsoid_max_moment(size, 0),
    mji_ellipsoid_max_moment(size, 1),
    mji_ellipsoid_max_moment(size, 2)
  };
  const mjtNum mom_visc[3] = {
    ang_vel[0] * (ang_drag_coef*II[0] + slender_drag_coef*(I_max - II[0])),
    ang_vel[1] * (ang_drag_coef*II[1] + slender_drag_coef*(I_max - II[1])),
    ang_vel[2] * (ang_drag_coef*II[2] + slender_drag_coef*(I_max - II[2]))
  };

  const mjtNum drag_lin_coef =  // linear plus quadratic
      fluid_viscosity*lin_visc_force_coef + fluid_density*mju_norm3(lin_vel)*(
          A_proj*blunt_drag_coef + slender_drag_coef*(A_max - A_proj));
  const mjtNum drag_ang_coef =  // linear plus quadratic
      fluid_viscosity * lin_visc_torq_coef +
      fluid_density * mju_norm3(mom_visc);

  local_force[0] -= drag_ang_coef * ang_vel[0];
  local_force[1] -= drag_ang_coef * ang_vel[1];
  local_force[2] -= drag_ang_coef * ang_vel[2];
  local_force[3] += magnus_force[0] + kutta_force[0] - drag_lin_coef*lin_vel[0];
  local_force[4] += magnus_force[1] + kutta_force[1] - drag_lin_coef*lin_vel[1];
  local_force[5] += magnus_force[2] + kutta_force[2] - drag_lin_coef*lin_vel[2];
}



// read the geom_fluid_coefs array into its constituent parts
void readFluidGeomInteraction(const mjtNum* geom_fluid_coefs,
                              mjtNum* geom_fluid_coef,
                              mjtNum* blunt_drag_coef,
                              mjtNum* slender_drag_coef,
                              mjtNum* ang_drag_coef,
                              mjtNum* kutta_lift_coef,
                              mjtNum* magnus_lift_coef,
                              mjtNum virtual_mass[3],
                              mjtNum virtual_inertia[3]) {
  int i = 0;
  geom_fluid_coef[0]   = geom_fluid_coefs[i++];
  blunt_drag_coef[0]   = geom_fluid_coefs[i++];
  slender_drag_coef[0] = geom_fluid_coefs[i++];
  ang_drag_coef[0]     = geom_fluid_coefs[i++];
  kutta_lift_coef[0]   = geom_fluid_coefs[i++];
  magnus_lift_coef[0]  = geom_fluid_coefs[i++];
  virtual_mass[0]      = geom_fluid_coefs[i++];
  virtual_mass[1]      = geom_fluid_coefs[i++];
  virtual_mass[2]      = geom_fluid_coefs[i++];
  virtual_inertia[0]   = geom_fluid_coefs[i++];
  virtual_inertia[1]   = geom_fluid_coefs[i++];
  virtual_inertia[2]   = geom_fluid_coefs[i++];
  if (i != mjNFLUID) {
    mju_error("Error in reading geom_fluid_coefs: wrong number of entries.");
  }
}



// write components into geom_fluid_coefs array
void writeFluidGeomInteraction (mjtNum* geom_fluid_coefs,
                                const mjtNum* geom_fluid_coef,
                                const mjtNum* blunt_drag_coef,
                                const mjtNum* slender_drag_coef,
                                const mjtNum* ang_drag_coef,
                                const mjtNum* kutta_lift_coef,
                                const mjtNum* magnus_lift_coef,
                                const mjtNum virtual_mass[3],
                                const mjtNum virtual_inertia[3]) {
  int i = 0;
  geom_fluid_coefs[i++] = geom_fluid_coef[0];
  geom_fluid_coefs[i++] = blunt_drag_coef[0];
  geom_fluid_coefs[i++] = slender_drag_coef[0];
  geom_fluid_coefs[i++] = ang_drag_coef[0];
  geom_fluid_coefs[i++] = kutta_lift_coef[0];
  geom_fluid_coefs[i++] = magnus_lift_coef[0];
  geom_fluid_coefs[i++] = virtual_mass[0];
  geom_fluid_coefs[i++] = virtual_mass[1];
  geom_fluid_coefs[i++] = virtual_mass[2];
  geom_fluid_coefs[i++] = virtual_inertia[0];
  geom_fluid_coefs[i++] = virtual_inertia[1];
  geom_fluid_coefs[i++] = virtual_inertia[2];
  if (i != mjNFLUID) {
    mju_error("Error in writing geom_fluid_coefs: wrong number of entries.");
  }
}



//---------------------------------- RNE -----------------------------------------------------------

// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term
void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result) {
  mjtNum tmp[6], tmp1[6];
  mjMARKSTACK;
  mjtNum* loc_cacc = mj_stackAlloc(d, m->nbody*6);
  mjtNum* loc_cfrc_body = mj_stackAlloc(d, m->nbody*6);

  // set world acceleration to -gravity
  mju_zero(loc_cacc, 6);
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    mju_scl3(loc_cacc+3, m->opt.gravity, -1);
  }

  // forward pass over bodies: accumulate cacc, set cfrc_body
  for (int i=1; i<m->nbody; i++) {
    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cacc = cacc_parent + cdofdot * qvel
    mju_mulDofVec(tmp, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[i]);
    mju_add(loc_cacc+6*i, loc_cacc+6*m->body_parentid[i], tmp, 6);

    // cacc += cdof * qacc
    if (flg_acc) {
      mju_mulDofVec(tmp, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[i]);
      mju_addTo(loc_cacc+6*i, tmp, 6);
    }

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    mju_mulInertVec(loc_cfrc_body+6*i, d->cinert+10*i, loc_cacc+6*i);
    mju_mulInertVec(tmp, d->cinert+10*i, d->cvel+6*i);
    mju_crossForce(tmp1, d->cvel+6*i, tmp);
    mju_addTo(loc_cfrc_body+6*i, tmp1, 6);
  }

  // clear world cfrc_body, for style
  mju_zero(loc_cfrc_body, 6);

  // backward pass over bodies: accumulate cfrc_body from children
  for (int i=m->nbody-1; i>0; i--)
    if (m->body_parentid[i]) {
      mju_addTo(loc_cfrc_body+6*m->body_parentid[i], loc_cfrc_body+6*i, 6);
    }

  // result = cdof * cfrc_body
  for (int i=0; i<m->nv; i++) {
    result[i] = mju_dot(d->cdof+6*i, loc_cfrc_body+6*m->dof_bodyid[i], 6);
  }

  mjFREESTACK;
}



// RNE with complete data: compute cacc, cfrc_ext, cfrc_int
void mj_rnePostConstraint(const mjModel* m, mjData* d) {
  int nbody=m->nbody;
  mjtNum cfrc_com[6], cfrc[6], lfrc[6];
  mjContact* con;

  // clear cacc, set world acceleration to -gravity
  mju_zero(d->cacc, 6);
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    mju_scl3(d->cacc+3, m->opt.gravity, -1);
  }

  // cfrc_ext = perturb
  mju_zero(d->cfrc_ext, 6*nbody);
  for (int i=1; i<nbody; i++)
    if (!mju_isZero(d->xfrc_applied+6*i, 6)) {
      // rearrange as torque:force
      mju_copy3(cfrc, d->xfrc_applied+6*i+3);
      mju_copy3(cfrc+3, d->xfrc_applied+6*i);

      // map force from application point to com; both world-oriented
      mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[i], d->xipos+3*i, 0);

      // accumulate
      mju_addTo(d->cfrc_ext+6*i, cfrc_com, 6);
    }

  // cfrc_ext += contacts
  for (int i=0; i<d->ncon; i++)
    if (d->contact[i].efc_address>=0) {
      // get contact pointer
      con = d->contact+i;

      // tmp = contact-local force:torque vector
      mj_contactForce(m, d, i, lfrc);

      // cfrc = world-oriented torque:force vector (swap in the process)
      mju_rotVecMatT(cfrc, lfrc+3, con->frame);
      mju_rotVecMatT(cfrc+3, lfrc, con->frame);

      // body 1
      int k;
      if ((k = m->geom_bodyid[con->geom1])) {
        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

        // apply (opposite for body 1)
        mju_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // body 2
      if ((k = m->geom_bodyid[con->geom2])) {
        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], con->pos, 0);

        // apply
        mju_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
      }
    }

  // cfrc_ext += connect and weld constraints
  int i = 0;
  while (i < d->ne) {
    if (d->efc_type[i]!=mjCNSTR_EQUALITY)
      mju_error_i("Row %d of efc is not an equality constraint", i);  // SHOULD NOT OCCUR

    int id = d->efc_id[i];
    mjtNum* eq_data = m->eq_data + mjNEQDATA*id;
    mjtNum pos[3];
    int k;
    switch (m->eq_type[id]) {
    case mjEQ_CONNECT:
    case mjEQ_WELD:
      // cfrc = world-oriented torque:force vector
      mju_copy3(cfrc + 3, d->efc_force + i);
      if (m->eq_type[id]==mjEQ_WELD) {
        mju_copy3(cfrc, d->efc_force + i + 3);
      } else {
        mju_zero3(cfrc);  // no torque from connect
      }

      // body 1
      if ((k = m->eq_obj1id[id])) {
        // transform point on body1: local -> global
        mj_local2Global(d, pos, 0, eq_data + 3*(m->eq_type[id]==mjEQ_WELD), 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply (opposite for body 1)
        mju_addTo(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // body 2
      if ((k = m->eq_obj2id[id])) {
        // transform point on body2: local -> global
        mj_local2Global(d, pos, 0, eq_data + 3*(m->eq_type[id]==mjEQ_CONNECT), 0, k, 0);

        // tmp = subtree CoM-based torque_force vector
        mju_transformSpatial(cfrc_com, cfrc, 1, d->subtree_com+3*m->body_rootid[k], pos, 0);

        // apply
        mju_subFrom(d->cfrc_ext+6*k, cfrc_com, 6);
      }

      // increment rows
      i += m->eq_type[id]==mjEQ_WELD ? 6 : 3;
      break;

    case mjEQ_JOINT:
    case mjEQ_TENDON:
      // increment 1 row
      i++;
      break;

    case mjEQ_DISTANCE:
      mju_error("distance equality constraints are no longer supported");

    default:
      mju_error_i("Unknown constraint type type %d", m->eq_type[id]);    // SHOULD NOT OCCUR
    }
  }

  // forward pass over bodies: compute cacc, cfrc_int
  mjtNum cacc[6], cfrc_body[6], cfrc_corr[6];
  mju_zero(d->cfrc_int, 6);
  for (int i=1; i<m->nbody; i++) {
    // get body's first dof address
    int bda = m->body_dofadr[i];

    // cacc = cacc_parent + cdofdot * qvel + cdof * qacc
    mju_mulDofVec(cacc, d->cdof_dot+6*bda, d->qvel+bda, m->body_dofnum[i]);
    mju_add(d->cacc+6*i, d->cacc+6*m->body_parentid[i], cacc, 6);
    mju_mulDofVec(cacc, d->cdof+6*bda, d->qacc+bda, m->body_dofnum[i]);
    mju_addTo(d->cacc+6*i, cacc, 6);

    // cfrc_body = cinert * cacc + cvel x (cinert * cvel)
    mju_mulInertVec(cfrc_body, d->cinert+10*i, d->cacc+6*i);
    mju_mulInertVec(cfrc_corr, d->cinert+10*i, d->cvel+6*i);
    mju_crossForce(cfrc, d->cvel+6*i, cfrc_corr);
    mju_addTo(cfrc_body, cfrc, 6);

    // set cfrc_int = cfrc_body - cfrc_ext
    mju_sub(d->cfrc_int+6*i, cfrc_body, d->cfrc_ext+6*i, 6);
  }

  // backward pass over bodies: accumulate cfrc_int from children
  for (int i=m->nbody-1; i>0; i--) {
    mju_addTo(d->cfrc_int+6*m->body_parentid[i], d->cfrc_int+6*i, 6);
  }
}
