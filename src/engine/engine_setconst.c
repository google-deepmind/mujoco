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

#include "engine/engine_setconst.h"

#include <stdio.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_forward.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"

// set quantities that depend on qpos0
static void set0(mjModel* m, mjData* d) {
  int id, id1, id2, dnum, nv = m->nv;
  mjtNum A[36] = {0}, pos[3], quat[4];
  mjMARKSTACK;
  mjtNum* jac = mj_stackAlloc(d, 6*nv);
  mjtNum* tmp = mj_stackAlloc(d, 6*nv);
  int* cammode = 0;
  int* lightmode = 0;

  // save camera and light mode, set to fixed
  if (m->ncam) {
    cammode = (int*) mj_stackAlloc(d, m->ncam);
    for (int i=0; i<m->ncam; i++) {
      cammode[i] = m->cam_mode[i];
      m->cam_mode[i] = mjCAMLIGHT_FIXED;
    }
  }
  if (m->nlight) {
    lightmode = (int*) mj_stackAlloc(d, m->nlight);
    for (int i=0; i<m->nlight; i++) {
      lightmode[i] = m->light_mode[i];
      m->light_mode[i] = mjCAMLIGHT_FIXED;
    }
  }

  // run computations in qpos0
  mju_copy(d->qpos, m->qpos0, m->nq);
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_camlight(m, d);
  mj_crbSkip(m, d, 0);

  // save dof_M0
  for (int i=0; i<nv; i++) {
    m->dof_M0[i] = d->qM[m->dof_Madr[i]];
  }

  // run remaining computations (factorM needs dof_M0)
  mj_factorM(m, d);
  mj_tendon(m, d);
  mj_transmission(m, d);

  // restore camera and light mode
  for (int i=0; i<m->ncam; i++) {
    m->cam_mode[i] = cammode[i];
  }
  for (int i=0; i<m->nlight; i++) {
    m->light_mode[i] = lightmode[i];
  }

  // set tendon_length0, actuator_length0
  mju_copy(m->tendon_length0, d->ten_length, m->ntendon);
  mju_copy(m->actuator_length0, d->actuator_length, m->nu);

  // compute body_invweight0
  m->body_invweight0[0] = m->body_invweight0[1] = 0.0;
  for (int i=1; i<m->nbody; i++) {
    if (nv) {
      // inverse spatial inertia:  A = J*inv(M)*J'
      mj_jacBodyCom(m, d, jac, jac+3*nv, i);
      mj_solveM(m, d, tmp, jac, 6);
      mju_mulMatMatT(A, jac, tmp, 6, nv, 6);
    }

    // average diagonal and assign
    m->body_invweight0[2*i] = (A[0] + A[7] + A[14])/3;
    m->body_invweight0[2*i+1] = (A[21] + A[28] + A[35])/3;
  }

  // compute dof_invweight0
  for (int i=0; i<m->njnt; i++) {
    id = m->jnt_dofadr[i];

    // get number of components
    if (m->jnt_type[i]==mjJNT_FREE) {
      dnum = 6;
    } else if (m->jnt_type[i]==mjJNT_BALL) {
      dnum = 3;
    } else {
      dnum = 1;
    }

    // inverse joint inertia:  A = J*inv(M)*J'
    if (nv) {
      mju_zero(jac, dnum*nv);
      for (int j=0; j<dnum; j++) {
        jac[j*(nv+1) + id] = 1;
      }
      mj_solveM(m, d, tmp, jac, dnum);
      mju_mulMatMatT(A, jac, tmp, dnum, nv, dnum);
    }

    // average diagonal and assign
    if (dnum==6) {
      m->dof_invweight0[id] = m->dof_invweight0[id+1] = m->dof_invweight0[id+2] =
                                                          (A[0] + A[7] + A[14])/3;
      m->dof_invweight0[id+3] = m->dof_invweight0[id+4] = m->dof_invweight0[id+5] =
                                                            (A[21] + A[28] + A[35])/3;
    } else if (dnum==3)
      m->dof_invweight0[id] = m->dof_invweight0[id+1] = m->dof_invweight0[id+2] =
                                                          (A[0] + A[4] + A[8])/3;
    else {
      m->dof_invweight0[id] = A[0];
    }
  }

  // compute tendon_invweight0
  if (nv) {
    for (int i=0; i<m->ntendon; i++) {
      // make dense vector into tmp
      if (mj_isSparse(m)) {
        mju_zero(tmp, nv);
        int end = d->ten_J_rowadr[i] + d->ten_J_rownnz[i];
        for (int j=d->ten_J_rowadr[i]; j<end; j++) {
          tmp[d->ten_J_colind[j]] = d->ten_J[j];
        }
      } else {
        mju_copy(tmp, d->ten_J+i*nv, nv);
      }

      // solve into tmp+nv
      mj_solveM(m, d, tmp+nv, tmp, 1);
      m->tendon_invweight0[i] = mju_dot(tmp, tmp+nv, nv);
    }

    // compute actuator_acc0
    for (int i=0; i<m->nu; i++) {
      mj_solveM(m, d, tmp, d->actuator_moment+i*nv, 1);
      m->actuator_acc0[i] = mju_norm(tmp, nv);
    }
  } else {
    for (int i=0; i<m->nu; i++) {
      m->actuator_acc0[i] = 0;
    }
  }

  // compute missing eq_data for body constraints
  for (int i=0; i<m->neq; i++) {
    // get ids
    id1 = m->eq_obj1id[i];
    id2 = m->eq_obj2id[i];

    // connect constraint
    if (m->eq_type[i]==mjEQ_CONNECT) {
      // pos = anchor position in global frame
      mj_local2Global(d, pos, 0, m->eq_data+mjNEQDATA*i, 0, id1, 0);

      // data[3-5] = anchor position in body2 local frame
      mju_subFrom3(pos, d->xpos+3*id2);
      mju_rotVecMatT(m->eq_data+mjNEQDATA*i+3, pos, d->xmat+9*id2);
    }

    // weld constraint
    else if (m->eq_type[i]==mjEQ_WELD) {
      // skip if user has set any quaternion data
      if (m->eq_data[mjNEQDATA*i+6] ||
          m->eq_data[mjNEQDATA*i+7] ||
          m->eq_data[mjNEQDATA*i+8] ||
          m->eq_data[mjNEQDATA*i+9]) {
        // normalize quaternion just in case
        mju_normalize4(m->eq_data+mjNEQDATA*i+6);
        continue;
      }

      // anchor position is in body2 local frame
      mj_local2Global(d, pos, 0, m->eq_data+mjNEQDATA*i, 0, id2, 0);

      // data[3-5] = anchor position in body1 local frame
      mju_subFrom3(pos, d->xpos+3*id1);
      mju_rotVecMatT(m->eq_data+mjNEQDATA*i+3, pos, d->xmat+9*id1);

      // data[6-9] = neg(xquat1)*xquat2 = "xquat2-xquat1" in body1 local frame
      mju_negQuat(quat, d->xquat+4*id1);
      mju_mulQuat(m->eq_data+mjNEQDATA*i+6, quat, d->xquat+4*id2);
    }
  }

  // camera compos0, pos0, mat0
  for (int i=0; i<m->ncam; i++) {
    // get body ids
    id = m->cam_bodyid[i];              // camera body
    id1 = m->cam_targetbodyid[i];       // target body

    // compute positional offsets
    mju_sub3(m->cam_pos0+3*i, d->cam_xpos+3*i, d->xpos+3*id);
    mju_sub3(m->cam_poscom0+3*i, d->cam_xpos+3*i, d->subtree_com+ (id1>=0 ? 3*id1 : 3*id));

    // copy mat
    mju_copy(m->cam_mat0+9*i, d->cam_xmat+9*i, 9);
  }

  // light compos0, pos0, dir0
  for (int i=0; i<m->nlight; i++) {
    // get body ids
    id = m->light_bodyid[i];            // light body
    id1 = m->light_targetbodyid[i];     // target body

    // compute positional offsets
    mju_sub3(m->light_pos0+3*i, d->light_xpos+3*i, d->xpos+3*id);
    mju_sub3(m->light_poscom0+3*i, d->light_xpos+3*i, d->subtree_com+ (id1>=0 ? 3*id1 : 3*id));

    // copy dir
    mju_copy3(m->light_dir0+3*i, d->light_xdir+3*i);
  }

  mjFREESTACK;
}



// accumulate bounding box
static void updateBox(mjtNum* xmin, mjtNum* xmax, mjtNum* pos, mjtNum radius) {
  for (int i=0; i<3; i++) {
    xmin[i] = mjMIN(xmin[i], pos[i] - radius);
    xmax[i] = mjMAX(xmax[i], pos[i] + radius);
  }
}


// compute stat; assume computations already executed in qpos0
static void setStat(mjModel* m, mjData* d) {
  mjtNum xmin[3] = {1E+10, 1E+10, 1E+10};
  mjtNum xmax[3] = {-1E+10, -1E+10, -1E+10};
  mjtNum rbound;
  mjMARKSTACK;
  mjtNum* body = mj_stackAlloc(d, m->nbody);

  // compute bounding box of bodies, joint centers, geoms and sites
  for (int i=1; i<m->nbody; i++) {
    updateBox(xmin, xmax, d->xpos+3*i, 0);
    updateBox(xmin, xmax, d->xipos+3*i, 0);
  }
  for (int i=0; i<m->njnt; i++) {
    updateBox(xmin, xmax, d->xanchor+3*i, 0);
  }
  for (int i=0; i<m->nsite; i++) {
    updateBox(xmin, xmax, d->site_xpos+3*i, 0);
  }
  for (int i=0; i<m->ngeom; i++) {
    // set rbound: regular geom rbound, or 0.1 of plane or hfield max size
    rbound = 0;
    if (m->geom_rbound[i] > 0) {
      rbound = m->geom_rbound[i];
    } else if (m->geom_type[i]==mjGEOM_PLANE) {
      // finite in at least one direction
      if (m->geom_size[3*i] || m->geom_size[3*i+1]) {
        rbound = mjMAX(m->geom_size[3*i], m->geom_size[3*i+1]) * 0.1;
      }

      // infinite in both directions
      else {
        rbound = 1;
      }
    } else if (m->geom_type[i]==mjGEOM_HFIELD) {
      int j = m->geom_dataid[i];
      rbound = mjMAX(m->hfield_size[4*j],
                     mjMAX(m->hfield_size[4*j+1],
                           mjMAX(m->hfield_size[4*j+2], m->hfield_size[4*j+3]))) * 0.1;
    }

    updateBox(xmin, xmax, d->geom_xpos+3*i, rbound);
  }

  // compute center
  mju_add3(m->stat.center, xmin, xmax);
  mju_scl3(m->stat.center, m->stat.center, 0.5);

  // compute bounding box size
  if (xmax[0]>xmin[0])
    m->stat.extent = mju_max(1E-5,
                             mju_max(xmax[0]-xmin[0], mju_max(xmax[1]-xmin[1], xmax[2]-xmin[2])));

  // set body size to max com-joint distance
  mju_zero(body, m->nbody);
  for (int i=0; i<m->njnt; i++) {
    // handle this body
    int id = m->jnt_bodyid[i];
    body[id] = mju_max(body[id], mju_dist3(d->xipos+3*id, d->xanchor+3*i));

    // handle parent body
    id = m->body_parentid[id];
    body[id] = mju_max(body[id], mju_dist3(d->xipos+3*id, d->xanchor+3*i));
  }
  body[0] = 0;

  // set body size to max of old value, and geom rbound + com-geom dist
  for (int i=1; i<m->nbody; i++) {
    for (int id=m->body_geomadr[i]; id<m->body_geomadr[i]+m->body_geomnum[i]; id++) {
      if (m->geom_rbound[id]>0) {
        body[i] = mju_max(body[i], m->geom_rbound[id] + mju_dist3(d->xipos+3*i, d->geom_xpos+3*id));
      }
    }
  }

  // compute meansize, make sure all sizes are above min
  if (m->nbody>1) {
    m->stat.meansize = 0;
    for (int i=1; i<m->nbody; i++) {
      body[i] = mju_max(body[i], 1E-5);
      m->stat.meansize += body[i]/(m->nbody-1);
    }
  }

  // fix extent if too small compared to meanbody
  m->stat.extent = mju_max(m->stat.extent, 2 * m->stat.meansize);

  // compute meanmass
  if (m->nbody>1) {
    m->stat.meanmass = 0;
    for (int i=1; i<m->nbody; i++) {
      m->stat.meanmass += m->body_mass[i];
    }
    m->stat.meanmass /= (m->nbody-1);
  }

  // compute meaninertia
  if (m->nv) {
    m->stat.meaninertia = 0;
    for (int i=0; i<m->nv; i++) {
      m->stat.meaninertia += d->qM[m->dof_Madr[i]];
    }
    m->stat.meaninertia /= m->nv;
  }

  mjFREESTACK;
}



// set quantities that depend on qpos_spring
static void setSpring(mjModel* m, mjData* d) {
  // run computations in qpos_spring
  mju_copy(d->qpos, m->qpos_spring, m->nq);
  mj_kinematics(m, d);
  mj_comPos(m, d);
  mj_tendon(m, d);
  mj_transmission(m, d);

  // copy if model spring length is negative
  for (int i=0; i<m->ntendon; i++) {
    if (m->tendon_lengthspring[i]<0) {
      m->tendon_lengthspring[i] = d->ten_length[i];
    }
  }
}



// entry point: set all constant fields of mjModel, except for lengthrange
void mj_setConst(mjModel* m, mjData* d) {
  // compute subtreemass
  for (int i=0; i<m->nbody; i++) {
    m->body_subtreemass[i] = m->body_mass[i];
  }
  for (int i=m->nbody-1; i>0; i--) {
    m->body_subtreemass[m->body_parentid[i]] += m->body_subtreemass[i];
  }

  // call functions
  set0(m, d);
  setStat(m, d);
  setSpring(m, d);
}



//----------------------------- actuator length range computation ----------------------------------

// evaluate actuator length, advance special dynamics
static mjtNum evalAct(const mjModel* m, mjData* d, int index, int side,
                      const mjLROpt* opt) {
  int nv = m->nv;

  // reduce velocity
  mju_scl(d->qvel, d->qvel, mju_exp(-m->opt.timestep/mjMAX(0.01, opt->timeconst)), nv);

  // step1: compute inertia and actuator moments
  mj_step1(m, d);

  // set force to generate desired acceleration
  mj_solveM(m, d, d->qfrc_applied, d->actuator_moment+index*nv, 1);
  mjtNum nrm = mju_norm(d->qfrc_applied, nv);
  mju_scl(d->qfrc_applied, d->actuator_moment+index*nv,
          (2*side-1)*opt->accel/mjMAX(mjMINVAL, nrm), nv);

  // impose maxforce
  nrm = mju_norm(d->qfrc_applied, nv);
  if (opt->maxforce>0 && nrm>opt->maxforce) {
    mju_scl(d->qfrc_applied, d->qfrc_applied, opt->maxforce/mjMAX(mjMINVAL, nrm), nv);
  }

  // step2: apply force
  mj_step2(m, d);

  // return actuator length
  return d->actuator_length[index];
}



// Set length range for specified actuator, return 1 if ok, 0 if error.
int mj_setLengthRange(mjModel* m, mjData* d, int index,
                      const mjLROpt* opt, char* error, int error_sz) {
  // check index
  if (index<0 || index>=m->nu) {
    mju_error("Invalid actuator index in mj_setLengthRange");
  }

  // skip depending on mode and type
  int ismuscle = (m->actuator_gaintype[index]==mjGAIN_MUSCLE ||
                  m->actuator_biastype[index]==mjBIAS_MUSCLE);
  int isuser = (m->actuator_gaintype[index]==mjGAIN_USER ||
                m->actuator_biastype[index]==mjBIAS_USER);
  if ((opt->mode==mjLRMODE_NONE) ||
      (opt->mode==mjLRMODE_MUSCLE && !ismuscle) ||
      (opt->mode==mjLRMODE_MUSCLEUSER && !ismuscle && !isuser)) {
    return 1;
  }

  // use existing length range if available
  if (opt->useexisting && (m->actuator_lengthrange[2*index] < m->actuator_lengthrange[2*index+1])) {
    return 1;
  }

  // get transmission id
  int threadid = m->actuator_trnid[index];

  // use joint and tendon limits if available
  if (opt->uselimit) {
    // joint or jointinparent
    if (m->actuator_trntype[index]==mjTRN_JOINT ||
        m->actuator_trntype[index]==mjTRN_JOINTINPARENT) {
      // make sure joint is limited
      if (m->jnt_limited[threadid]) {
        // copy range
        m->actuator_lengthrange[2*index] = m->jnt_range[2*threadid];
        m->actuator_lengthrange[2*index+1] = m->jnt_range[2*threadid+1];

        // skip optimization
        return 1;
      }
    }

    // tendon
    if (m->actuator_trntype[index]==mjTRN_TENDON) {
      // make sure tendon is limited
      if (m->tendon_limited[threadid]) {
        // copy range
        m->actuator_lengthrange[2*index] = m->tendon_range[2*threadid];
        m->actuator_lengthrange[2*index+1] = m->tendon_range[2*threadid+1];

        // skip optimization
        return 1;
      }
    }
  }

  // optimize in both directions
  mjtNum lmin[2] = {0, 0}, lmax[2] = {0, 0};
  int side;
  for (side=0; side<2; side++) {
    // init at qpos0
    mj_resetData(m, d);

    // simulate
    int updated = 0;
    while (d->time < opt->inttotal) {
      // advance and get length
      mjtNum len = evalAct(m, d, index, side, opt);

      // reset: cannot proceed
      if (d->time==0) {
        snprintf(error, error_sz, "Unstable lengthrange simulation in actuator %d", index);
        return 0;
      }

      // update limits
      if (d->time > opt->inttotal-opt->inteval) {
        if (len<lmin[side] || !updated) {
          lmin[side] = len;
        }
        if (len>lmax[side] || !updated) {
          lmax[side] = len;
        }

        updated = 1;
      }
    }

    // assign
    m->actuator_lengthrange[2*index+side] = (side==0 ? lmin[side] : lmax[side]);
  }

  // check range
  mjtNum dif = m->actuator_lengthrange[2*index+1] - m->actuator_lengthrange[2*index];
  if (dif<=0) {
    snprintf(error, error_sz,
             "Invalid lengthrange (%g, %g) in actuator %d",
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1], index);
    return 0;
  }

  // check convergence, side 0
  if (lmax[0]-lmin[0]>opt->tolrange*dif) {
    snprintf(error, error_sz,
             "Lengthrange computation did not converge in actuator %d:\n"
             "  eval (%g, %g)\n  range (%g, %g)",
             index, lmin[0], lmax[0],
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1]);
    return 0;
  }

  // check convergence, side 1
  if (lmax[1]-lmin[1]>opt->tolrange*dif) {
    snprintf(error, error_sz,
             "Lengthrange computation did not converge in actuator %d:\n"
             "  eval (%g, %g)\n range (%g, %g)",
             index, lmin[1], lmax[1],
             m->actuator_lengthrange[2*index],
             m->actuator_lengthrange[2*index+1]);
    return 0;
  }

  return 1;
}
