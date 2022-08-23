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

#include "engine/engine_sensor.h"

#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_callback.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_ray.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"


//-------------------------------- utility ---------------------------------------------------------

// add sensor noise after each stage
static void add_noise(const mjModel* m, mjData* d, mjtStage stage) {
  int adr, dim;
  mjtNum rnd[4], noise, quat[4], res[4];

  // process sensors matching stage and having positive noise
  for (int i=0; i<m->nsensor; i++) {
    if (m->sensor_needstage[i]==stage && m->sensor_noise[i]>0) {
      // get sensor info
      adr = m->sensor_adr[i];
      dim = m->sensor_dim[i];
      noise = m->sensor_noise[i];

      // real or positive: add noise directly, with clamp for positive
      if (m->sensor_datatype[i]==mjDATATYPE_REAL ||
          m->sensor_datatype[i]==mjDATATYPE_POSITIVE) {
        for (int j=0; j<dim; j++) {
          // get random numbers; use only the first one
          rnd[0] = mju_standardNormal(rnd+1);

          // positive
          if (m->sensor_datatype[i]==mjDATATYPE_POSITIVE) {
            // add noise only if positive, keep it positive
            if (d->sensordata[adr+j]>0) {
              d->sensordata[adr+j] = mjMAX(0, d->sensordata[adr+j]+rnd[0]*noise);
            }
          }

          // real
          else {
            d->sensordata[adr+j] += rnd[0]*noise;
          }
        }
      }

      // axis or quat: rotate around random axis by random angle
      else {
        // get four random numbers
        rnd[0] = mju_standardNormal(rnd+1);
        rnd[2] = mju_standardNormal(rnd+3);

        // scale angle, normalize axis, make quaternion
        rnd[0] *= noise;
        mju_normalize3(rnd+1);
        mju_axisAngle2Quat(quat, rnd+1, rnd[0]);

        // axis
        if (m->sensor_datatype[i]==mjDATATYPE_AXIS) {
          // apply quaternion rotation to axis, assign
          mju_rotVecQuat(res, d->sensordata+adr, quat);
          mju_copy3(d->sensordata+adr, res);
        }

        // quaternion
        else if (m->sensor_datatype[i]==mjDATATYPE_QUATERNION) {
          // apply quaternion rotation to quaternion, assign
          mju_mulQuat(res, d->sensordata+adr, quat);
          mju_copy4(d->sensordata+adr, res);
        }

        // unknown datatype
        else {
          mju_error_i("Unknown datatype in sensor %d", i);
        }
      }
    }
  }
}



// apply cutoff after each stage
static void apply_cutoff(const mjModel* m, mjData* d, mjtStage stage) {
  // process sensors matching stage and having positive cutoff
  for (int i=0; i<m->nsensor; i++) {
    if (m->sensor_needstage[i]==stage && m->sensor_cutoff[i]>0) {
      // get sensor info
      int adr = m->sensor_adr[i];
      int dim = m->sensor_dim[i];
      mjtNum cutoff = m->sensor_cutoff[i];

      // process all dimensions
      for (int j=0; j<dim; j++) {
        // real: apply on both sides
        if (m->sensor_datatype[i]==mjDATATYPE_REAL) {
          d->sensordata[adr+j] = mju_clip(d->sensordata[adr+j], -cutoff, cutoff);
        }

        // positive: apply on positive side only
        else if (m->sensor_datatype[i]==mjDATATYPE_POSITIVE) {
          d->sensordata[adr+j] = mju_min(cutoff, d->sensordata[adr+j]);
        }
      }
    }
  }
}



// get xpos and xmat pointers to an object in mjData
static void get_xpos_xmat(const mjData* d, int type, int id, int sensor_id,
                          mjtNum **xpos, mjtNum **xmat) {
  switch (type) {
  case mjOBJ_XBODY:
    *xpos = d->xpos + 3*id;
    *xmat = d->xmat + 9*id;
    break;
  case mjOBJ_BODY:
    *xpos = d->xipos + 3*id;
    *xmat = d->ximat + 9*id;
    break;
  case mjOBJ_GEOM:
    *xpos = d->geom_xpos + 3*id;
    *xmat = d->geom_xmat + 9*id;
    break;
  case mjOBJ_SITE:
    *xpos = d->site_xpos + 3*id;
    *xmat = d->site_xmat + 9*id;
    break;
  case mjOBJ_CAMERA:
    *xpos = d->cam_xpos + 3*id;
    *xmat = d->cam_xmat + 9*id;
    break;
  default:
    mju_error_i("Invalid object type in sensor %d", sensor_id);
  }
}

// get global quaternion of an object in mjData
static void get_xquat(const mjModel* m, const mjData* d, int type, int id, int sensor_id,
                      mjtNum *quat) {
  switch (type) {
  case mjOBJ_XBODY:
    mju_copy4(quat, d->xquat+4*id);
    break;
  case mjOBJ_BODY:
    mju_mulQuat(quat, d->xquat+4*id, m->body_iquat+4*id);
    break;
  case mjOBJ_GEOM:
    mju_mulQuat(quat, d->xquat+4*m->geom_bodyid[id], m->geom_quat+4*id);
    break;
  case mjOBJ_SITE:
    mju_mulQuat(quat, d->xquat+4*m->site_bodyid[id], m->site_quat+4*id);
    break;
  case mjOBJ_CAMERA:
    mju_mulQuat(quat, d->xquat+4*m->cam_bodyid[id], m->cam_quat+4*id);
    break;
  default:
    mju_error_i("Invalid object type in sensor %d", sensor_id);
  }
}


//-------------------------------- sensor ----------------------------------------------------------

// position-dependent sensors
void mj_sensorPos(const mjModel* m, mjData* d) {
  int rgeomid, objtype, objid, reftype, refid, adr, offset, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  mjtNum rvec[3], *xpos, *xmat, *xpos_ref, *xmat_ref;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  for (int i=0; i<m->nsensor; i++) {
    if (m->sensor_needstage[i]==mjSTAGE_POS) {
      // get sensor info
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      refid = m->sensor_refid[i];
      reftype = m->sensor_reftype[i];
      adr = m->sensor_adr[i];

      // process according to type
      switch (m->sensor_type[i]) {
      case mjSENS_MAGNETOMETER:                           // magnetometer
        mju_mulMatTVec(d->sensordata+adr, d->site_xmat+9*objid, m->opt.magnetic, 3, 3);
        break;

      case mjSENS_RANGEFINDER:                            // rangefinder
        rvec[0] = d->site_xmat[9*objid+2];
        rvec[1] = d->site_xmat[9*objid+5];
        rvec[2] = d->site_xmat[9*objid+8];
        d->sensordata[adr] = mj_ray(m, d, d->site_xpos+3*objid, rvec, NULL, 1,
                                    m->site_bodyid[objid], &rgeomid);
        break;

      case mjSENS_JOINTPOS:                               // jointpos
        d->sensordata[adr] = d->qpos[m->jnt_qposadr[objid]];
        break;

      case mjSENS_TENDONPOS:                              // tendonpos
        d->sensordata[adr] = d->ten_length[objid];
        break;

      case mjSENS_ACTUATORPOS:                            // actuatorpos
        d->sensordata[adr] = d->actuator_length[objid];
        break;

      case mjSENS_BALLQUAT:                               // ballquat
        mju_copy4(d->sensordata+adr, d->qpos+m->jnt_qposadr[objid]);
        break;

      case mjSENS_JOINTLIMITPOS:                          // jointlimitpos
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_JOINT && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_pos[j] - d->efc_margin[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITPOS:                         // tendonlimitpos
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_TENDON && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_pos[j] - d->efc_margin[j];
            break;
          }
        }
        break;

      case mjSENS_FRAMEPOS:                               // framepos
      case mjSENS_FRAMEXAXIS:                             // framexaxis
      case mjSENS_FRAMEYAXIS:                             // frameyaxis
      case mjSENS_FRAMEZAXIS:                             // framezaxis
        // get xpos and xmat pointers for object frame
        get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);

        // reference frame unspecified: global frame
        if (refid == -1) {
          if (m->sensor_type[i]==mjSENS_FRAMEPOS) {
            mju_copy3(d->sensordata+adr, xpos);
          } else {
            // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
            offset = m->sensor_type[i] - mjSENS_FRAMEXAXIS;
            d->sensordata[adr] = xmat[offset];
            d->sensordata[adr+1] = xmat[offset+3];
            d->sensordata[adr+2] = xmat[offset+6];
          }
        }

        // reference frame specified
        else {
          get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
          if (m->sensor_type[i]==mjSENS_FRAMEPOS) {
            mju_sub3(rvec, xpos, xpos_ref);
            mju_rotVecMatT(d->sensordata+adr, rvec, xmat_ref);
          } else {
            // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
            offset = m->sensor_type[i] - mjSENS_FRAMEXAXIS;
            mjtNum axis[3] = {xmat[offset], xmat[offset+3], xmat[offset+6]};
            mju_rotVecMatT(d->sensordata+adr, axis, xmat_ref);
          }
        }
        break;

      case mjSENS_FRAMEQUAT:                              // framequat
        {
          // get global object quaternion
          mjtNum objquat[4];
          get_xquat(m, d, objtype, objid, i, objquat);

          // reference frame unspecified: copy object quaternion
          if (refid == -1) {
            mju_copy4(d->sensordata+adr, objquat);
          } else {
            // reference frame specified, get global reference quaternion
            mjtNum refquat[4];
            get_xquat(m, d, reftype, refid, i, refquat);

            // relative quaternion
            mju_negQuat(refquat, refquat);
            mju_mulQuat(d->sensordata+adr, refquat, objquat);
          }
        }
        break;

      case mjSENS_SUBTREECOM:                             // subtreecom
        mju_copy3(d->sensordata+adr, d->subtree_com+3*objid);
        break;

      case mjSENS_CLOCK:                                  // clock
        d->sensordata[adr] = d->time;
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mju_error_i("Invalid sensor type in POS stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_POS);
  }

  // add noise if enabled
  if (mjENABLED(mjENBL_SENSORNOISE)) {
    add_noise(m, d, mjSTAGE_POS);
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_POS);
}



// velocity-dependent sensors
void mj_sensorVel(const mjModel* m, mjData* d) {
  int type, objtype, objid, reftype, refid, adr, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  mjtNum xvel[6];

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  int subtreeVel = 0;
  for (int i=0; i<m->nsensor; i++) {
    if (m->sensor_needstage[i]==mjSTAGE_VEL) {
      // get sensor info
      type = m->sensor_type[i];
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      refid = m->sensor_refid[i];
      reftype = m->sensor_reftype[i];
      adr = m->sensor_adr[i];

      // call mj_subtreeVel when first relevant sensor is encountered
      if (subtreeVel==0 &&
          (type==mjSENS_SUBTREELINVEL ||
           type==mjSENS_SUBTREEANGMOM ||
           type==mjSENS_USER)) {
        // compute subtree_linvel, subtree_angmom
        mj_subtreeVel(m, d);

        // mark computed
        subtreeVel = 1;
      }

      // process according to type
      switch (type) {
      case mjSENS_VELOCIMETER:                            // velocimeter
        // xvel = site velocity, in site frame
        mj_objectVelocity(m, d, mjOBJ_SITE, objid, xvel, 1);

        // assign linear velocity
        mju_copy3(d->sensordata+adr, xvel+3);
        break;

      case mjSENS_GYRO:                                   // gyro
        // xvel = site velocity, in site frame
        mj_objectVelocity(m, d, mjOBJ_SITE, objid, xvel, 1);

        // assign angular velocity
        mju_copy3(d->sensordata+adr, xvel);
        break;

      case mjSENS_JOINTVEL:                               // jointvel
        d->sensordata[adr] = d->qvel[m->jnt_dofadr[objid]];
        break;

      case mjSENS_TENDONVEL:                              // tendonvel
        d->sensordata[adr] = d->ten_velocity[objid];
        break;

      case mjSENS_ACTUATORVEL:                            // actuatorvel
        d->sensordata[adr] = d->actuator_velocity[objid];
        break;

      case mjSENS_BALLANGVEL:                             // ballangvel
        mju_copy3(d->sensordata+adr, d->qvel+m->jnt_dofadr[objid]);
        break;

      case mjSENS_JOINTLIMITVEL:                          // jointlimitvel
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_JOINT && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_vel[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITVEL:                         // tendonlimitvel
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_TENDON && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_vel[j];
            break;
          }
        }
        break;

      case mjSENS_FRAMELINVEL:                            // framelinvel
      case mjSENS_FRAMEANGVEL:                            // frameangvel
        // xvel = 6D object velocity, in global frame
        mj_objectVelocity(m, d, objtype, objid, xvel, 0);

        if (refid > -1) {  // reference frame specified
          mjtNum *xpos, *xmat, *xpos_ref, *xmat_ref, xvel_ref[6], rel_vel[6], cross[3], rvec[3];

          // in global frame: object and reference position, reference orientation and velocity
          get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);
          get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
          mj_objectVelocity(m, d, reftype, refid, xvel_ref, 0);

          // subtract velocities
          mju_sub(rel_vel, xvel, xvel_ref, 6);

          // linear velocity: add correction due to rotating reference frame
          mju_sub3(rvec, xpos, xpos_ref);
          mju_cross(cross, rvec, xvel_ref);
          mju_addTo3(rel_vel+3, cross);

          // project into reference frame
          mju_rotVecMatT(xvel, rel_vel, xmat_ref);
          mju_rotVecMatT(xvel+3, rel_vel+3, xmat_ref);
        }

        // copy linear or angular component
        if (m->sensor_type[i]==mjSENS_FRAMELINVEL) {
          mju_copy3(d->sensordata+adr, xvel+3);
        } else {
          mju_copy3(d->sensordata+adr, xvel);
        }
        break;

      case mjSENS_SUBTREELINVEL:                          // subtreelinvel
        mju_copy3(d->sensordata+adr, d->subtree_linvel+3*objid);
        break;

      case mjSENS_SUBTREEANGMOM:                          // subtreeangmom
        mju_copy3(d->sensordata+adr, d->subtree_angmom+3*objid);
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mju_error_i("Invalid type in VEL stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_VEL);
  }

  // add noise if enabled
  if (mjENABLED(mjENBL_SENSORNOISE)) {
    add_noise(m, d, mjSTAGE_VEL);
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_VEL);
}



// acceleration/force-dependent sensors
void mj_sensorAcc(const mjModel* m, mjData* d) {
  int rootid, bodyid, type, objtype, objid, body1, body2, adr, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  mjtNum tmp[6], conforce[6], conray[3];
  mjContact* con;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  int rnePost = 0;
  for (int i=0; i<m->nsensor; i++) {
    if (m->sensor_needstage[i]==mjSTAGE_ACC) {
      // get sensor info
      type =  m->sensor_type[i];
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      adr = m->sensor_adr[i];

      // call mj_rnePostConstraint when first relevant sensor is encountered
      if (rnePost==0                  &&
          type!=mjSENS_TOUCH          &&
          type!=mjSENS_ACTUATORFRC    &&
          type!=mjSENS_JOINTLIMITFRC  &&
          type!=mjSENS_TENDONLIMITFRC) {
        // compute cacc, cfrc_int, cfrc_ext
        mj_rnePostConstraint(m, d);

        // mark computed
        rnePost = 1;
      }

      // process according to type
      switch (type) {
      case mjSENS_TOUCH:                                  // touch
        // extract body data
        bodyid = m->site_bodyid[objid];
        rootid = m->body_rootid[bodyid];

        // clear result
        d->sensordata[adr] = 0;

        // find contacts in sensor zone, add normal forces
        for (int j=0; j<d->ncon; j++) {
          // contact pointer, contacting bodies
          con = d->contact + j;
          body1 = m->geom_bodyid[con->geom1];
          body2 = m->geom_bodyid[con->geom2];

          // select contacts involving sensorized body
          if (con->efc_address>=0 && (bodyid==body1 || bodyid==body2)) {
            // get contact force:torque in contact frame
            mj_contactForce(m, d, j, conforce);

            // nothing to do if normal is zero
            if (conforce[0]<=0) {
              continue;
            }

            // convert contact normal force to global frame, normalize
            mju_scl3(conray, con->frame, conforce[0]);
            mju_normalize3(conray);

            // flip ray direction if sensor is on body2
            if (bodyid==body2) {
              mju_scl3(conray, conray, -1);
            }

            // add if ray-zone intersection (always true when con->pos inside zone)
            if (mju_rayGeom(d->site_xpos+3*objid, d->site_xmat+9*objid,
                            m->site_size+3*objid, con->pos, conray,
                            m->site_type[objid]) >= 0) {
              d->sensordata[adr] += conforce[0];
            }
          }
        }
        break;

      case mjSENS_ACCELEROMETER:                          // accelerometer
        // tmp = site acceleration, in site frame
        mj_objectAcceleration(m, d, mjOBJ_SITE, objid, tmp, 1);

        // assign linear acceleration
        mju_copy3(d->sensordata+adr, tmp+3);
        break;

      case mjSENS_FORCE:                                  // force
        // extract body data
        bodyid = m->site_bodyid[objid];
        rootid = m->body_rootid[bodyid];

        // tmp = interaction force between body and parent, in site frame
        mju_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                             d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

        // assign force
        mju_copy3(d->sensordata+adr, tmp+3);
        break;

      case mjSENS_TORQUE:                                 // torque
        // extract body data
        bodyid = m->site_bodyid[objid];
        rootid = m->body_rootid[bodyid];

        // tmp = interaction force between body and parent, in site frame
        mju_transformSpatial(tmp, d->cfrc_int+6*bodyid, 1,
                             d->site_xpos+3*objid, d->subtree_com+3*rootid, d->site_xmat+9*objid);

        // assign torque
        mju_copy3(d->sensordata+adr, tmp);
        break;

      case mjSENS_ACTUATORFRC:                            // actuatorfrc
        d->sensordata[adr] = d->actuator_force[objid];
        break;

      case mjSENS_JOINTLIMITFRC:                          // jointlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_JOINT && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_force[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITFRC:                         // tendonlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j<nefc; j++) {
          if (d->efc_type[j]==mjCNSTR_LIMIT_TENDON && d->efc_id[j]==objid) {
            d->sensordata[adr] = d->efc_force[j];
            break;
          }
        }
        break;

      case mjSENS_FRAMELINACC:                            // framelinacc
      case mjSENS_FRAMEANGACC:                            // frameangacc
        // get 6D object acceleration, in global frame
        mj_objectAcceleration(m, d, objtype, objid, tmp, 0);

        // copy linear or angular component
        if (m->sensor_type[i]==mjSENS_FRAMELINACC) {
          mju_copy3(d->sensordata+adr, tmp+3);
        } else {
          mju_copy3(d->sensordata+adr, tmp);
        }
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mju_error_i("Invalid type in ACC stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_ACC);
  }

  // add noise if enabled
  if (mjENABLED(mjENBL_SENSORNOISE)) {
    add_noise(m, d, mjSTAGE_ACC);
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_ACC);
}



//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
void mj_energyPos(const mjModel* m, mjData* d) {
  int padr;
  mjtNum dif[3], stiffness;

  // disabled: clear and return
  if (!mjENABLED(mjENBL_ENERGY)) {
    d->energy[0] = d->energy[1] = 0;
    return;
  }

  // init potential energy:  -sum_i body(i).mass * mju_dot(body(i).pos, gravity)
  d->energy[0] = 0;
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    for (int i=1; i<m->nbody; i++) {
      d->energy[0] -= m->body_mass[i] * mju_dot3(m->opt.gravity, d->xipos+3*i);
    }
  }

  // add joint-level springs
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i<m->njnt; i++) {
      stiffness = m->jnt_stiffness[i];
      padr = m->jnt_qposadr[i];

      switch (m->jnt_type[i]) {
      case mjJNT_FREE:
        mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
        d->energy[0] += 0.5*stiffness*mju_dot3(dif, dif);

        // continue with rotations
        padr += 3;

      case mjJNT_BALL:
        // covert quatertion difference into angular "velocity"
        mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
        d->energy[0] += 0.5*stiffness*mju_dot3(dif, dif);
        break;

      case mjJNT_SLIDE:
      case mjJNT_HINGE:
        d->energy[0] += 0.5*stiffness*
                        (d->qpos[padr] - m->qpos_spring[padr])*
                        (d->qpos[padr] - m->qpos_spring[padr]);
        break;
      }
    }
  }

  // add tendon-level springs
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i<m->ntendon; i++) {
      stiffness = m->tendon_stiffness[i];

      d->energy[0] += 0.5*stiffness*(d->ten_length[i] - m->tendon_lengthspring[i])*
                      (d->ten_length[i] - m->tendon_lengthspring[i]);
    }
  }
}



// velocity-dependent energy (kinetic)
void mj_energyVel(const mjModel* m, mjData* d) {
  mjtNum *vec;
  mjMARKSTACK;

  // return if disabled (already cleared in potential)
  if (!mjENABLED(mjENBL_ENERGY)) {
    return;
  }

  vec = mj_stackAlloc(d, m->nv);

  // kinetic energy:  0.5 * qvel' * M * qvel
  mj_mulM(m, d, vec, d->qvel);
  d->energy[1] = 0.5*mju_dot(vec, d->qvel, m->nv);

  mjFREESTACK;
}
