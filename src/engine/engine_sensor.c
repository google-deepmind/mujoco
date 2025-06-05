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
#include <mujoco/mjplugin.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_callback.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_io.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//-------------------------------- utility ---------------------------------------------------------

// apply cutoff after each stage
static void apply_cutoff(const mjModel* m, mjData* d, mjtStage stage) {
  // process sensors matching stage and having positive cutoff
  for (int i=0; i < m->nsensor; i++) {
    if (m->sensor_needstage[i] == stage && m->sensor_cutoff[i] > 0) {
      // skip fromto sensors
      if (m->sensor_type[i] == mjSENS_GEOMFROMTO) {
        continue;
      }

      // get sensor info
      int adr = m->sensor_adr[i];
      int dim = m->sensor_dim[i];
      mjtNum cutoff = m->sensor_cutoff[i];

      // process all dimensions
      for (int j=0; j < dim; j++) {
        // real: apply on both sides
        if (m->sensor_datatype[i] == mjDATATYPE_REAL) {
          d->sensordata[adr+j] = mju_clip(d->sensordata[adr+j], -cutoff, cutoff);
        }

        // positive: apply on positive side only
        else if (m->sensor_datatype[i] == mjDATATYPE_POSITIVE) {
          d->sensordata[adr+j] = mju_min(cutoff, d->sensordata[adr+j]);
        }
      }
    }
  }
}



// get xpos and xmat pointers to an object in mjData
static void get_xpos_xmat(const mjData* d, mjtObj type, int id, int sensor_id,
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
    mjERROR("invalid object type in sensor %d", sensor_id);
  }
}

// get global quaternion of an object in mjData
static void get_xquat(const mjModel* m, const mjData* d, mjtObj type, int id, int sensor_id,
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
    mjERROR("invalid object type in sensor %d", sensor_id);
  }
}


static void cam_project(mjtNum sensordata[2], const mjtNum target_xpos[3],
                        const mjtNum cam_xpos[3], const mjtNum cam_xmat[9],
                        const int cam_res[2], mjtNum cam_fovy,
                        const float cam_intrinsic[4], const float cam_sensorsize[2]) {
  mjtNum fx, fy;

  // translation matrix (4x4)
  mjtNum translation[4][4] = {0};
  translation[0][0] = 1;
  translation[1][1] = 1;
  translation[2][2] = 1;
  translation[3][3] = 1;
  translation[0][3] = -cam_xpos[0];
  translation[1][3] = -cam_xpos[1];
  translation[2][3] = -cam_xpos[2];

  // rotation matrix (4x4)
  mjtNum rotation[4][4] = {0};
  rotation[0][0] = 1;
  rotation[1][1] = 1;
  rotation[2][2] = 1;
  rotation[3][3] = 1;
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      rotation[i][j] = cam_xmat[j*3+i];
    }
  }

  // focal transformation matrix (3x4)
  if (cam_sensorsize[0] && cam_sensorsize[1]) {
    fx = cam_intrinsic[0] / cam_sensorsize[0] * cam_res[0];
    fy = cam_intrinsic[1] / cam_sensorsize[1] * cam_res[1];
  } else {
    fx = fy = .5 / mju_tan(cam_fovy * mjPI / 360.) * cam_res[1];
  }

  mjtNum focal[3][4] = {0};
  focal[0][0] = -fx;
  focal[1][1] =  fy;
  focal[2][2] = 1.0;

  // image matrix (3x3)
  mjtNum image[3][3] = {0};
  image[0][0] = 1;
  image[1][1] = 1;
  image[2][2] = 1;
  image[0][2] = (mjtNum)cam_res[0] / 2.0;
  image[1][2] = (mjtNum)cam_res[1] / 2.0;

  // projection matrix (3x4): product of all 4 matrices
  mjtNum proj[3][4] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 3; j++) {
      for (int k=0; k < 4; k++) {
        for (int l=0; l < 4; l++) {
          for (int n=0; n < 4; n++) {
            proj[i][n] += image[i][j] * focal[j][k] * rotation[k][l] * translation[l][n];
          }
        }
      }
    }
  }

  // projection matrix multiplies homogenous [x, y, z, 1] vectors
  mjtNum pos_hom[4] = {0, 0, 0, 1};
  mju_copy3(pos_hom, target_xpos);

  // project world coordinates into pixel space, see:
  // https://en.wikipedia.org/wiki/3D_projection#Mathematical_formula
  mjtNum pixel_coord_hom[3] = {0};
  for (int i=0; i < 3; i++) {
    for (int j=0; j < 4; j++) {
      pixel_coord_hom[i] += proj[i][j] * pos_hom[j];
    }
  }

  // avoid dividing by tiny numbers
  mjtNum denom = pixel_coord_hom[2];
  if (mju_abs(denom) < mjMINVAL) {
    if (denom < 0) {
      denom = mju_min(denom, -mjMINVAL);
    } else {
      denom = mju_max(denom, mjMINVAL);
    }
  }

  // compute projection
  sensordata[0] = pixel_coord_hom[0] / denom;
  sensordata[1] = pixel_coord_hom[1] / denom;
}



//-------------------------------- sensor ----------------------------------------------------------

// position-dependent sensors
void mj_sensorPos(const mjModel* m, mjData* d) {
  int ne = d->ne, nf = d->nf, nefc = d->nefc, nsensor = m->nsensor;
  int nusersensor = 0;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  for (int i=0; i < nsensor; i++) {
    mjtSensor type = (mjtSensor) m->sensor_type[i];

    // skip sensor plugins -- these are handled after builtin sensor types
    if (type == mjSENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == mjSTAGE_POS) {
      // get sensor info
      int objtype = m->sensor_objtype[i];
      int objid = m->sensor_objid[i];
      int refid = m->sensor_refid[i];
      int reftype = m->sensor_reftype[i];
      int adr = m->sensor_adr[i];

      mjtNum rvec[3], *xpos, *xmat, *xpos_ref, *xmat_ref;

      // process according to type
      switch (type) {
      case mjSENS_MAGNETOMETER:                           // magnetometer
        mju_mulMatTVec(d->sensordata+adr, d->site_xmat+9*objid, m->opt.magnetic, 3, 3);
        break;

      case mjSENS_CAMPROJECTION:                          // camera projection
        cam_project(d->sensordata+adr, d->site_xpos+3*objid, d->cam_xpos+3*refid,
                    d->cam_xmat+9*refid, m->cam_resolution+2*refid, m->cam_fovy[refid],
                    m->cam_intrinsic+4*refid, m->cam_sensorsize+2*refid);
        break;

      case mjSENS_RANGEFINDER:                            // rangefinder
        rvec[0] = d->site_xmat[9*objid+2];
        rvec[1] = d->site_xmat[9*objid+5];
        rvec[2] = d->site_xmat[9*objid+8];
        d->sensordata[adr] = mj_ray(m, d, d->site_xpos+3*objid, rvec, NULL, 1,
                                    m->site_bodyid[objid], NULL);

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
        mju_normalize4(d->sensordata+adr);
        break;

      case mjSENS_JOINTLIMITPOS:                          // jointlimitpos
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
            d->sensordata[adr] = d->efc_pos[j] - d->efc_margin[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITPOS:                         // tendonlimitpos
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
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
          if (type == mjSENS_FRAMEPOS) {
            mju_copy3(d->sensordata+adr, xpos);
          } else {
            // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
            int offset = type - mjSENS_FRAMEXAXIS;
            d->sensordata[adr] = xmat[offset];
            d->sensordata[adr+1] = xmat[offset+3];
            d->sensordata[adr+2] = xmat[offset+6];
          }
        }

        // reference frame specified
        else {
          get_xpos_xmat(d, reftype, refid, i, &xpos_ref, &xmat_ref);
          if (type == mjSENS_FRAMEPOS) {
            mju_sub3(rvec, xpos, xpos_ref);
            mju_mulMatTVec3(d->sensordata+adr, xmat_ref, rvec);
          } else {
            // offset = (0 or 1 or 2) for (x or y or z)-axis sensors, respectively
            int offset = type - mjSENS_FRAMEXAXIS;
            mjtNum axis[3] = {xmat[offset], xmat[offset+3], xmat[offset+6]};
            mju_mulMatTVec3(d->sensordata+adr, xmat_ref, axis);
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

      case mjSENS_GEOMDIST:                               // signed distance between two geoms
      case mjSENS_GEOMNORMAL:                             // normal direction between two geoms
      case mjSENS_GEOMFROMTO:                             // segment between two geoms
        {
          // use cutoff for collision margin
          mjtNum margin = m->sensor_cutoff[i];

          // initialize outputs
          mjtNum dist = margin;    // collision distance
          mjtNum fromto[6] = {0};  // segment between geoms

          // get lists of geoms to collide
          int n1, id1;
          if (objtype == mjOBJ_BODY) {
            n1 = m->body_geomnum[objid];
            id1 = m->body_geomadr[objid];
          } else {
            n1 = 1;
            id1 = objid;
          }
          int n2, id2;
          if (reftype == mjOBJ_BODY) {
            n2 = m->body_geomnum[refid];
            id2 = m->body_geomadr[refid];
          } else {
            n2 = 1;
            id2 = refid;
          }

          // collide all pairs
          for (int geom1=id1; geom1 < id1+n1; geom1++) {
            for (int geom2=id2; geom2 < id2+n2; geom2++) {
              mjtNum fromto_new[6] = {0};
              mjtNum dist_new = mj_geomDistance(m, d, geom1, geom2, margin, fromto_new);
              if (dist_new < dist) {
                dist = dist_new;
                mju_copy(fromto, fromto_new, 6);
              }
            }
          }

          // write sensordata for this sensor and all subsequent sensors with identical signature
          int write_sensor = 1;
          while (write_sensor) {
            // write geom distance
            if (type == mjSENS_GEOMDIST) {
              d->sensordata[adr] = dist;
            }

            // write distance normal
            else if (type == mjSENS_GEOMNORMAL) {
              mjtNum normal[3] = {fromto[3]-fromto[0], fromto[4]-fromto[1], fromto[5]-fromto[2]};
              if (normal[0] || normal[1] || normal[2]) {
                mju_normalize3(normal);
              }
              mju_copy3(d->sensordata + adr, normal);
            }

            // write distance fromto
            else {
              mju_copy(d->sensordata + adr, fromto, 6);
            }

            // if this is the last sensor, break
            if (i+1 == nsensor) {
              break;
            }

            // type of the next sensor
            mjtSensor type_next = m->sensor_type[i+1];

            // check if signature of next sensor matches this sensor
            write_sensor = (type_next == mjSENS_GEOMDIST   ||
                            type_next == mjSENS_GEOMNORMAL ||
                            type_next == mjSENS_GEOMFROMTO)   &&
                           m->sensor_objtype[i+1] == objtype  &&
                           m->sensor_objid[i+1]   == objid    &&
                           m->sensor_reftype[i+1] == reftype  &&
                           m->sensor_refid[i+1]   == refid    &&
                           m->sensor_cutoff[i+1]  == margin;

            // if signature matches, increment external loop variable i
            if (write_sensor) {
              i++;

              // update adr and type, everything else is the same
              adr = m->sensor_adr[i];
              type = type_next;
            }
          }
        }
        break;

      case mjSENS_E_POTENTIAL:                            // potential energy
        mj_energyPos(m, d);
        d->sensordata[adr] = d->energy[0];
        break;

      case mjSENS_E_KINETIC:                              // kinetic energy
        mj_energyVel(m, d);
        d->sensordata[adr] = d->energy[1];
        break;

      case mjSENS_CLOCK:                                  // clock
        d->sensordata[adr] = d->time;
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mjERROR("invalid sensor type in POS stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_POS);
  }

  // compute plugin sensor values
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if ((plugin->capabilityflags & mjPLUGIN_SENSOR) &&
          (plugin->needstage == mjSTAGE_POS || plugin->needstage == mjSTAGE_NONE)) {
        if (!plugin->compute) {
          mjERROR("`compute` is a null function pointer for plugin at slot %d", slot);
        }
        plugin->compute(m, d, i, mjPLUGIN_SENSOR);
      }
    }
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_POS);
}



// velocity-dependent sensors
void mj_sensorVel(const mjModel* m, mjData* d) {
  int objtype, objid, reftype, refid, adr, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc;
  mjtNum xvel[6];

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  int subtreeVel = 0;
  for (int i=0; i < m->nsensor; i++) {
    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == mjSENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == mjSTAGE_VEL) {
      // get sensor info
      mjtSensor type = m->sensor_type[i];
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      refid = m->sensor_refid[i];
      reftype = m->sensor_reftype[i];
      adr = m->sensor_adr[i];

      // call mj_subtreeVel when first relevant sensor is encountered
      if (subtreeVel == 0 &&
          (type == mjSENS_SUBTREELINVEL ||
           type == mjSENS_SUBTREEANGMOM ||
           type == mjSENS_USER)) {
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
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
            d->sensordata[adr] = d->efc_vel[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITVEL:                         // tendonlimitvel
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
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
          mju_mulMatTVec3(xvel, xmat_ref, rel_vel);
          mju_mulMatTVec3(xvel+3, xmat_ref, rel_vel+3);
        }

        // copy linear or angular component
        if (m->sensor_type[i] == mjSENS_FRAMELINVEL) {
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
        mjERROR("invalid type in VEL stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_VEL);
  }

  // trigger computation of plugins
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if ((plugin->capabilityflags & mjPLUGIN_SENSOR) && plugin->needstage == mjSTAGE_VEL) {
        if (!plugin->compute) {
          mjERROR("`compute` is null for plugin at slot %d", slot);
        }
        if (subtreeVel == 0) {
          // compute subtree_linvel, subtree_angmom
          // TODO(b/247107630): add a flag to allow plugin to specify whether it actually needs this
          mj_subtreeVel(m, d);

          // mark computed
          subtreeVel = 1;
        }
        plugin->compute(m, d, i, mjPLUGIN_SENSOR);
      }
    }
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_VEL);
}



// acceleration/force-dependent sensors
void mj_sensorAcc(const mjModel* m, mjData* d) {
  int rootid, bodyid, objtype, objid, adr, nusersensor = 0;
  int ne = d->ne, nf = d->nf, nefc = d->nefc, nu = m->nu;
  mjtNum tmp[6], conforce[6], conray[3], frc;
  mjContact* con;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // process sensors matching stage
  int rnePost = 0;
  for (int i=0; i < m->nsensor; i++) {
    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == mjSENS_PLUGIN) {
      continue;
    }

    if (m->sensor_needstage[i] == mjSTAGE_ACC) {
      // get sensor info
      mjtSensor type =  m->sensor_type[i];
      objtype = m->sensor_objtype[i];
      objid = m->sensor_objid[i];
      adr = m->sensor_adr[i];

      // call mj_rnePostConstraint when first relevant sensor is encountered
      if (rnePost == 0  && (type == mjSENS_ACCELEROMETER ||
                            type == mjSENS_FORCE         ||
                            type == mjSENS_TORQUE        ||
                            type == mjSENS_FRAMELINACC   ||
                            type == mjSENS_FRAMEANGACC   ||
                            type == mjSENS_USER)) {
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

        // clear result
        d->sensordata[adr] = 0;

        // find contacts in sensor zone, add normal forces
        for (int j=0; j < d->ncon; j++) {
          // contact pointer, contacting bodies  (-1 for flex)
          con = d->contact + j;
          int conbody[2];
          for (int k=0; k < 2; k++) {
            conbody[k] = (con->geom[k] >= 0) ? m->geom_bodyid[con->geom[k]] : -1;
          }

          // select contacts involving sensorized body
          if (con->efc_address >= 0 && (bodyid == conbody[0] || bodyid == conbody[1])) {
            // get contact force:torque in contact frame
            mj_contactForce(m, d, j, conforce);

            // nothing to do if normal is zero
            if (conforce[0] <= 0) {
              continue;
            }

            // convert contact normal force to global frame, normalize
            mju_scl3(conray, con->frame, conforce[0]);
            mju_normalize3(conray);

            // flip ray direction if sensor is on body2
            if (bodyid == conbody[1]) {
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

      case mjSENS_JOINTACTFRC:                            // jointactfrc
        d->sensordata[adr] = d->qfrc_actuator[m->jnt_dofadr[objid]];
        break;

      case mjSENS_TENDONACTFRC:  // tendonactfrc
        frc = 0.0;
        for (int j=0; j < nu; j++) {
          if (m->actuator_trntype[j] == mjTRN_TENDON && m->actuator_trnid[2*j] == objid) {
            frc += d->actuator_force[j];
          }
        }
        d->sensordata[adr] = frc;
        break;

      case mjSENS_JOINTLIMITFRC:                          // jointlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_JOINT && d->efc_id[j] == objid) {
            d->sensordata[adr] = d->efc_force[j];
            break;
          }
        }
        break;

      case mjSENS_TENDONLIMITFRC:                         // tendonlimitfrc
        d->sensordata[adr] = 0;
        for (int j=ne+nf; j < nefc; j++) {
          if (d->efc_type[j] == mjCNSTR_LIMIT_TENDON && d->efc_id[j] == objid) {
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
        if (m->sensor_type[i] == mjSENS_FRAMELINACC) {
          mju_copy3(d->sensordata+adr, tmp+3);
        } else {
          mju_copy3(d->sensordata+adr, tmp);
        }
        break;

      case mjSENS_USER:                                   // user
        nusersensor++;
        break;

      default:
        mjERROR("invalid type in ACC stage, sensor %d", i);
      }
    }
  }

  // fill in user sensors if detected
  if (nusersensor && mjcb_sensor) {
    mjcb_sensor(m, d, mjSTAGE_ACC);
  }

  // trigger computation of plugins
  if (m->nplugin) {
    const int nslot = mjp_pluginCount();
    for (int i=0; i < m->nplugin; i++) {
      const int slot = m->plugin[i];
      const mjpPlugin* plugin = mjp_getPluginAtSlotUnsafe(slot, nslot);
      if (!plugin) {
        mjERROR("invalid plugin slot: %d", slot);
      }
      if ((plugin->capabilityflags & mjPLUGIN_SENSOR) && plugin->needstage == mjSTAGE_ACC) {
        if (!plugin->compute) {
          mjERROR("`compute` is null for plugin at slot %d", slot);
        }
        if (rnePost == 0) {
          // compute cacc, cfrc_int, cfrc_ext
          // TODO(b/247107630): add a flag to allow plugin to specify whether it actually needs this
          mj_rnePostConstraint(m, d);

          // mark computed
          rnePost = 1;
        }
        plugin->compute(m, d, i, mjPLUGIN_SENSOR);
      }
    }
  }

  // cutoff
  apply_cutoff(m, d, mjSTAGE_ACC);
}



//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
void mj_energyPos(const mjModel* m, mjData* d) {
  int padr;
  mjtNum dif[3], quat[4], stiffness;

  // init potential energy:  -sum_i body(i).mass * mju_dot(body(i).pos, gravity)
  d->energy[0] = 0;
  if (!mjDISABLED(mjDSBL_GRAVITY)) {
    for (int i=1; i < m->nbody; i++) {
      d->energy[0] -= m->body_mass[i] * mju_dot3(m->opt.gravity, d->xipos+3*i);
    }
  }

  // add joint-level springs
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i < m->njnt; i++) {
      stiffness = m->jnt_stiffness[i];
      padr = m->jnt_qposadr[i];

      switch ((mjtJoint) m->jnt_type[i]) {
      case mjJNT_FREE:
        mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
        d->energy[0] += 0.5*stiffness*mju_dot3(dif, dif);

        // continue with rotations
        padr += 3;
        mjFALLTHROUGH;

      case mjJNT_BALL:
        // convert quaternion difference into angular "velocity"
        mju_copy4(quat, d->qpos+padr);
        mju_normalize4(quat);
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
    for (int i=0; i < m->ntendon; i++) {
      stiffness = m->tendon_stiffness[i];
      mjtNum length = d->ten_length[i];
      mjtNum displacement = 0;

      // compute spring displacement
      mjtNum lower = m->tendon_lengthspring[2*i];
      mjtNum upper = m->tendon_lengthspring[2*i+1];
      if (length > upper) {
        displacement = upper - length;
      } else if (length < lower) {
        displacement = lower - length;
      }

      d->energy[0] += 0.5*stiffness*displacement*displacement;
    }
  }

  // add flex-level springs for dim=1 (dim>1 requires plugins)
  if (!mjDISABLED(mjDSBL_PASSIVE)) {
    for (int i=0; i < m->nflex; i++) {
      stiffness = m->flex_edgestiffness[i];
      if (m->flex_rigid[i] || stiffness == 0 || m->flex_dim[i] > 1) {
        continue;
      }

      // process non-rigid edges of this flex
      int flex_edgeadr = m->flex_edgeadr[i];
      int flex_edgenum = m->flex_edgenum[i];
      for (int e=flex_edgeadr; e < flex_edgeadr+flex_edgenum; e++) {
        if (!m->flexedge_rigid[e]) {
          mjtNum displacement = m->flexedge_length0[e] - d->flexedge_length[e];
          d->energy[0] += 0.5*stiffness*displacement*displacement;
        };
      }
    }
  }
}



// velocity-dependent energy (kinetic)
void mj_energyVel(const mjModel* m, mjData* d) {
  mj_markStack(d);
  mjtNum *vec = mjSTACKALLOC(d, m->nv, mjtNum);

  // kinetic energy:  0.5 * qvel' * M * qvel
  mj_mulM(m, d, vec, d->qvel);
  d->energy[1] = 0.5*mju_dot(vec, d->qvel, m->nv);

  mj_freeStack(d);
}
