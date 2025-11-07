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
#include "engine/engine_collision_sdf.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_core_util.h"
#include "engine/engine_crossplatform.h"
#include "engine/engine_memory.h"
#include "engine/engine_plugin.h"
#include "engine/engine_ray.h"
#include "engine/engine_sleep.h"
#include "engine/engine_sort.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"



//-------------------------------- utility ---------------------------------------------------------


typedef struct {
  mjtNum criterion;  // criterion for partial sort
  int id;            // index in d->contact
  int flip;          // 0: don't flip the normal, 1: flip the normal
} ContactInfo;

// define ContactSelect: find the k smallest elements of a ContactInfo array
static int ContactInfoCompare(const ContactInfo* a, const ContactInfo* b, void* context) {
  if (a->criterion < b->criterion) return -1;
  if (a->criterion > b->criterion) return 1;

  if (a->id < b->id) return -1;
  if (a->id > b->id) return 1;

  return 0;
}
mjPARTIAL_SORT(ContactSelect, ContactInfo, ContactInfoCompare);


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


// check if a contact body/geom matches a sensor spec (type, id)
static int checkMatch(const mjModel* m, int body, int geom, mjtObj type, int id) {
  if (type == mjOBJ_UNKNOWN) return 1;
  if (type == mjOBJ_SITE)    return 1;  // already passed site filter test
  if (type == mjOBJ_GEOM)    return id == geom;
  if (type == mjOBJ_BODY)    return id == body;
  if (type == mjOBJ_XBODY) {
    // traverse up the tree from body, return true if we land on id
    while (body > id) {
      body = m->body_parentid[body];
    }
    return body == id;
  }

  return 0;
}

//   0: no match
//   1: match, use contact normal
//  -1: match, flip contact normal
static int matchContact(const mjModel* m, const mjData* d, int conid,
                        mjtObj type1, int id1, mjtObj type2, int id2) {
  // no criterion: quick match
  if (type1 == mjOBJ_UNKNOWN && type2 == mjOBJ_UNKNOWN) {
    return 1;
  }

  // site filter
  if (type1 == mjOBJ_SITE) {
    if (!mju_insideGeom(d->site_xpos + 3 * id1, d->site_xmat + 9 * id1,
                        m->site_size + 3 * id1, m->site_type[id1], d->contact[conid].pos)) {
      return 0;
    }
  }

  // get geom, body ids
  int geom1 = d->contact[conid].geom[0];
  int geom2 = d->contact[conid].geom[1];
  int body1 = geom1 >= 0 ? m->geom_bodyid[geom1] : -1;
  int body2 = geom2 >= 0 ? m->geom_bodyid[geom2] : -1;

  // check match of sensor objects with contact objects
  int match11 = checkMatch(m, body1, geom1, type1, id1);
  int match12 = checkMatch(m, body2, geom2, type1, id1);
  int match21 = checkMatch(m, body1, geom1, type2, id2);
  int match22 = checkMatch(m, body2, geom2, type2, id2);

  // if a sensor object is specified, it must be involved in the contact
  if (!match11 && !match12) return 0;
  if (!match21 && !match22) return 0;

  // determine direction
  if (type1 != mjOBJ_UNKNOWN && type2 != mjOBJ_UNKNOWN) {
    // both obj1 and obj2 specified: direction depends on order
    int order_regular = match11 && match22;
    int order_reverse = match12 && match21;
    if (order_regular && !order_reverse) return 1;
    if (order_reverse && !order_regular) return -1;
    if (order_regular && order_reverse)  return 1;  // ambiguous, return 1
  } else if (type1 != mjOBJ_UNKNOWN) {
    // only obj1 specified: normal points away from obj1
    return match11 ? 1 : -1;
  } else if (type2 != mjOBJ_UNKNOWN) {
    // only obj2 specified: normal points towards obj2
    return match22 ? 1 : -1;
  }

  // should not occur, all conditions are covered above
  return 0;
}


// fill in output data for contact sensor for all fields
//   if flg_flip > 0, normal/tangent rotate 180 about frame[2]
//   force/torque flip-z s.t. force is equal-and-opposite in new contact frame
static void copySensorData(const mjModel* m, const mjData* d,
                           mjtNum* data[mjNCONDATA], int id, int flg_flip, int nfound) {
  // found flag
  if (data[mjCONDATA_FOUND]) *data[mjCONDATA_FOUND] = nfound;

  // contact force and torque
  if (data[mjCONDATA_FORCE] || data[mjCONDATA_TORQUE]) {
    mjtNum forcetorque[6];
    mj_contactForce(m, d, id, forcetorque);
    if (data[mjCONDATA_FORCE]) {
      mju_copy3(data[mjCONDATA_FORCE], forcetorque);
      if (flg_flip) data[mjCONDATA_FORCE][2] *= -1;
    }
    if (data[mjCONDATA_TORQUE]) {
      mju_copy3(data[mjCONDATA_TORQUE], forcetorque+3);
      if (flg_flip) data[mjCONDATA_TORQUE][2] *= -1;
    }
  }

  // contact penetration distance
  if (data[mjCONDATA_DIST]) {
    *data[mjCONDATA_DIST] = d->contact[id].dist;
  }

  // contact position
  if (data[mjCONDATA_POS]) {
    mju_copy3(data[mjCONDATA_POS], d->contact[id].pos);
  }

  // contact normal
  if (data[mjCONDATA_NORMAL]) {
    mju_copy3(data[mjCONDATA_NORMAL], d->contact[id].frame);
    if (flg_flip) mju_scl3(data[mjCONDATA_NORMAL], data[mjCONDATA_NORMAL], -1);
  }

  // contact first tangent
  if (data[mjCONDATA_TANGENT]) {
    mju_copy3(data[mjCONDATA_TANGENT], d->contact[id].frame+3);
    if (flg_flip) mju_scl3(data[mjCONDATA_TANGENT], data[mjCONDATA_TANGENT], -1);
  }
}


// compute total wrench about one point, in the global frame
static void total_wrench(mjtNum force[3], mjtNum torque[3], const mjtNum point[3], int n,
                        const mjtNum *wrench, const mjtNum *pos, const mjtNum *frame) {
  mju_zero3(force);
  mju_zero3(torque);

  for (int j = 0; j < n; ++j) {
    // rotate force, torque from contact frame to global frame
    mjtNum force_j[3], torque_j[3];
    mju_mulMatTVec3(force_j, frame + 9*j, wrench + 6*j);
    mju_mulMatTVec3(torque_j, frame + 9*j, wrench + 6*j + 3);

    // add to total force, torque
    mju_addTo3(force, force_j);
    mju_addTo3(torque, torque_j);

    // add induced moment:  torque += (pos - point) x force
    mjtNum diff[3];
    mju_sub3(diff, pos + 3*j, point);
    mjtNum induced_torque[3];
    mju_cross(induced_torque, diff, force_j);
    mju_addTo3(torque, induced_torque);
  }
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

  // sleep filtering
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  for (int i=0; i < nsensor; i++) {
    mjtSensor type = (mjtSensor) m->sensor_type[i];

    // skip sleeping sensor
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_SENSOR, i) == mjS_ASLEEP) {
      continue;
    }

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

      case mjSENS_INSIDESITE:                             // insidesite
        get_xpos_xmat(d, objtype, objid, i, &xpos, &xmat);
        d->sensordata[adr] = mju_insideGeom(d->site_xpos + 3*refid,
                                            d->site_xmat + 9*refid,
                                            m->site_size + 3*refid,
                                            m->site_type[refid],
                                            xpos);
        break;

      case mjSENS_GEOMDIST:                               // signed distance between two geoms
      case mjSENS_GEOMNORMAL:                             // normal direction between two geoms
      case mjSENS_GEOMFROMTO:                             // segment between two geoms
        {
          mjtNum cutoff = m->sensor_cutoff[i];

          // initialize outputs
          mjtNum dist = cutoff;    // collision distance
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
              mjtNum dist_new = mj_geomDistance(m, d, geom1, geom2, cutoff, fromto_new);
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
                           m->sensor_cutoff[i+1]  == cutoff;

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
        // clear result, compute later
        mju_zero(d->sensordata + adr, m->sensor_dim[i]);
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

  // sleep filtering
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  int subtreeVel = 0;
  for (int i=0; i < m->nsensor; i++) {
    // skip sensor plugins -- these are handled after builtin sensor types
    if (m->sensor_type[i] == mjSENS_PLUGIN) {
      continue;
    }

    // skip sleeping sensor
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_SENSOR, i) == mjS_ASLEEP) {
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
        // clear result, compute later
        mju_zero(d->sensordata + adr, m->sensor_dim[i]);
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
  const mjContact* con;

  // disabled sensors: return
  if (mjDISABLED(mjDSBL_SENSOR)) {
    return;
  }

  // sleep filtering
  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;

  // process sensors matching stage
  int rnePost = 0;
  for (int i=0; i < m->nsensor; i++) {
    // skip sleeping sensor
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_SENSOR, i) == mjS_ASLEEP) {
      continue;
    }

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

      case mjSENS_CONTACT:                                // contact
        {
          // local reduce enum for readability
          enum {
            REDUCE_NONE     = 0,
            REDUCE_MINDIST  = 1,
            REDUCE_MAXFORCE = 2,
            REDUCE_NETFORCE = 3,
          };

          // prepare sizes and indices
          int dataspec = m->sensor_intprm[i*mjNSENS];
          int size = mju_condataSize(dataspec);  // size of each slot
          int dim = m->sensor_dim[i];            // total sensor array dimension
          int num = dim / size;                  // number of slots
          int reftype = m->sensor_reftype[i];
          int refid = m->sensor_refid[i];
          int reduce = m->sensor_intprm[i*mjNSENS+1];

          // clear all outputs, prepare data pointers
          mjtNum* ptr = d->sensordata + adr;
          mju_zero(ptr, dim);
          mjtNum* data[mjNCONDATA] = {NULL};
          for (int j=0; j < mjNCONDATA; j++) {
            if (dataspec & (1 << j)) {
              data[j] = ptr;
              ptr += mjCONDATA_SIZE[j];
            }
          }

          // prepare for matching loop
          int nmatch = 0;
          mj_markStack(d);
          ContactInfo *match = mjSTACKALLOC(d, d->ncon, ContactInfo);

          // find matching contacts
          for (int j=0; j < d->ncon; j++) {
            // check match condition
            int match_j = matchContact(m, d, j, objtype, objid, reftype, refid);
            if (!match_j) {
              continue;
            }

            // save id and flip flag
            match[nmatch].id = j;
            match[nmatch].flip = match_j < 0;

            // save sorting criterion, if required
            if (reduce == REDUCE_MINDIST) {
              match[nmatch].criterion = d->contact[j].dist;
            } else if (reduce == REDUCE_MAXFORCE) {
              mjtNum forcetorque[6];
              mj_contactForce(m, d, j, forcetorque);
              match[nmatch].criterion = -mju_dot3(forcetorque, forcetorque);
            }

            // increment number of matching contacts
            nmatch++;
          }

          // number of slots to be filled
          int nslot = mjMIN(num, nmatch);

          // partial sort to get bottom nslot contacts if sorted reduction
          if (reduce == REDUCE_MINDIST || reduce == REDUCE_MAXFORCE) {
            ContactInfo *heap = mjSTACKALLOC(d, nslot, ContactInfo);
            ContactSelect(match, heap, nmatch, nslot, NULL);
          }

          // netforce reduction
          else if (reduce == REDUCE_NETFORCE) {
            mjtNum *wrench = mjSTACKALLOC(d, nmatch * 6, mjtNum);
            mjtNum *pos = mjSTACKALLOC(d, nmatch * 3, mjtNum);
            mjtNum *frame = mjSTACKALLOC(d, nmatch * 9, mjtNum);

            // precompute wrenches, positions, and frames, maybe flip wrench
            for (int j=0; j < nmatch; j++) {
              int conid = match[j].id;
              mj_contactForce(m, d, conid, wrench + 6*j);
              mju_copy3(pos + 3*j, d->contact[conid].pos);
              mju_copy(frame + 9*j, d->contact[conid].frame, 9);
              if (match[j].flip) {
                mju_scl(wrench + 6*j , wrench + 6*j, -1, 6);
              }
            }

            // compute point: force-weighted centroid of contact positions
            mjtNum point[3] = {0};
            mjtNum total_force = 0;
            for (int j=0; j < nmatch; j++) {
              mjtNum weight = mju_norm3(wrench + 6*j);
              mju_addToScl3(point, pos + 3*j, weight);
              total_force += weight;
            }
            mju_scl3(point, point, 1.0 / mjMAX(total_force, mjMINVAL));

            // compute total wrench about point, in the global frame
            mjtNum force[3], torque[3];
            total_wrench(force, torque, point, nmatch, wrench, pos, frame);

            // write data to slot 0
            if (data[mjCONDATA_FOUND])   *data[mjCONDATA_FOUND] = nmatch;
            if (data[mjCONDATA_FORCE])   mju_copy3(data[mjCONDATA_FORCE], force);
            if (data[mjCONDATA_TORQUE])  mju_copy3(data[mjCONDATA_TORQUE], torque);
            if (data[mjCONDATA_DIST])    *data[mjCONDATA_DIST] = 0;
            if (data[mjCONDATA_POS])     mju_copy3(data[mjCONDATA_POS], point);
            if (data[mjCONDATA_NORMAL])  data[mjCONDATA_NORMAL][0] = 1;
            if (data[mjCONDATA_TANGENT]) data[mjCONDATA_TANGENT][1] = 1;

            // done with this sensor
            mj_freeStack(d);
            break;
          }

          // copy data into slots, increment pointers
          for (int j=0; j < nslot; j++) {
            copySensorData(m, d, data, match[j].id, match[j].flip, nmatch);
            for (int k=0; k < mjNCONDATA; k++) {
              if (data[k]) data[k] += size;
            }
          }

          mj_freeStack(d);
        }
        break;

      case mjSENS_TACTILE:                                // tactile
        {
          mj_markStack(d);

          // get parent weld id
          int mesh_id = m->sensor_objid[i];
          int geom_id = m->sensor_refid[i];
          int parent_body = m->geom_bodyid[geom_id];
          int parent_weld = m->body_weldid[parent_body];
          int nchannel = m->sensor_dim[i] / m->mesh_vertnum[mesh_id];

          // clear sensordata and distance matrix
          mjtNum* sensordata = d->sensordata + m->sensor_adr[i];
          mju_zero(sensordata, m->sensor_dim[i]);

          // count contacts and get contact geom ids
          // TODO: use a more efficient C version of unordered_set
          int* contact_geom_ids = mj_stackAllocInt(d, d->ncon);
          int ncontact = 0;
          for (int k = 0; k < d->ncon; k++) {
            int body1 = m->body_weldid[m->geom_bodyid[d->contact[k].geom1]];
            int body2 = m->body_weldid[m->geom_bodyid[d->contact[k].geom2]];
            if (body1 == parent_weld) {
              int add = 1;
              for (int j = 0; j < ncontact; j++) {
                if (contact_geom_ids[j] == d->contact[k].geom2) {
                  add = 0;
                  break;
                }
              }
              if (add) {
                contact_geom_ids[ncontact] = d->contact[k].geom2;
                ncontact++;
              }
            }
            if (body2 == parent_weld) {
              int add = 1;
              for (int j = 0; j < ncontact; j++) {
                if (contact_geom_ids[j] == d->contact[k].geom1) {
                  add = 0;
                  break;
                }
              }
              if (add) {
                contact_geom_ids[ncontact] = d->contact[k].geom1;
                ncontact++;
              }
            }
          }

          // no contacts, return
          if (ncontact == 0) {
            mj_freeStack(d);
            break;
          }

          // all of the quadrature points are contact points
          int ncon = m->mesh_vertnum[mesh_id];

          // get site frame
          mjtNum* geom_pos = d->geom_xpos + 3*geom_id;
          mjtNum* geom_mat = d->geom_xmat + 9*geom_id;

          // allocate contact forces and positions
          mjtNum* forcesT = mj_stackAllocNum(d, ncon*3);
          mju_zero(forcesT, ncon*3);

          // iterate over colliding geoms
          for (int g = 0; g < ncontact; g++) {
            int geom = contact_geom_ids[g];
            int body = m->geom_bodyid[geom];

            // get sdf plugin of the geoms
            int sdf_instance[2] = {-1, -1};
            mjtGeom geomtype[2] = {mjGEOM_SDF, mjGEOM_SPHERE};
            const mjpPlugin* sdf_ptr[2] = {NULL, NULL};
            if (m->geom_type[geom] == mjGEOM_SDF) {
              sdf_instance[0] = m->geom_plugin[geom];
              sdf_ptr[0] = mjc_getSDF(m, geom);
            } else if (m->geom_type[geom] == mjGEOM_MESH) {
              sdf_instance[0] = m->geom_dataid[geom];
              geomtype[0] = (mjtGeom)m->geom_type[geom];
            } else {
              sdf_instance[0] = geom;
              geomtype[0] = (mjtGeom)m->geom_type[geom];
            }

            // skip mesh geoms not having an octree
            if (geomtype[0] == mjGEOM_MESH &&
                m->mesh_octadr[m->geom_dataid[geom]] == -1) {
              continue;
            }

            // set SDF parameters
            mjSDF geom_sdf;
            geom_sdf.id = &sdf_instance[0];
            geom_sdf.type = mjSDFTYPE_SINGLE;
            geom_sdf.plugin = &sdf_ptr[0];
            geom_sdf.geomtype = &geomtype[0];

            // get forces in mesh coordinates
            int node = 0;
            float* mesh_vert = m->mesh_vert + 3*m->mesh_vertadr[mesh_id];
            float* mesh_normal = m->mesh_normal + 3*m->mesh_normaladr[mesh_id];
            for (int j = 0; j < ncon; j++) {
              // position in site frame
              mjtNum pos[3] = {mesh_vert[3 * j + 0], mesh_vert[3 * j + 1],
                               mesh_vert[3 * j + 2]};

              // position in global frame
              mjtNum xpos[3];
              mju_mulMatVec3(xpos, geom_mat, pos);
              mju_addTo3(xpos, geom_pos);

              // position in other geom frame
              mjtNum lpos[3];
              mju_sub3(tmp, xpos, d->geom_xpos + 3*geom);
              mju_mulMatTVec3(lpos, d->geom_xmat + 9*geom, tmp);

              // SDF plugins are in the original mesh frame
              if (sdf_ptr[0] != NULL) {
                mjtNum mesh_mat[9];
                mju_quat2Mat(mesh_mat, m->mesh_quat + 4 * m->geom_dataid[geom]);
                mju_mulMatVec3(lpos, mesh_mat, lpos);
                mju_addTo3(lpos, m->mesh_pos + 3 * m->geom_dataid[geom]);
              }

              // compute distance
              mjtNum depth = mju_min(mjc_distance(m, d, &geom_sdf, lpos), 0);
              if (depth == 0) {
                node++;
                continue;
              }

              // get velocity in global frame
              mjtNum vel_sensor[6], vel_other[6], vel_rel[3];
              mju_transformSpatial(
                  vel_sensor, d->cvel + 6 * parent_weld, 0, xpos,
                  d->subtree_com + 3 * m->body_rootid[parent_weld], NULL);
              mju_transformSpatial(
                  vel_other, d->cvel + 6 * body, 0, d->geom_xpos + 3 * geom,
                  d->subtree_com + 3 * m->body_rootid[body], NULL);
              mju_sub3(vel_rel, vel_sensor+3, vel_other+3);

              mjtNum normal[3] = {mesh_normal[9 * j + 0], mesh_normal[9 * j + 1],
                                  mesh_normal[9 * j + 2]};
              mjtNum tang1[3] = {mesh_normal[9 * j + 3], mesh_normal[9 * j + 4],
                                 mesh_normal[9 * j + 5]};
              mjtNum tang2[3] = {mesh_normal[9 * j + 6], mesh_normal[9 * j + 7],
                                 mesh_normal[9 * j + 8]};

              // get contact force/torque, rotate into node frame
              mju_rotVecQuat(normal, normal, m->mesh_quat + 4 * mesh_id);
              mju_rotVecQuat(tang1, tang1, m->mesh_quat + 4 * mesh_id);
              mju_rotVecQuat(tang2, tang2, m->mesh_quat + 4 * mesh_id);
              mjtNum force[3];
              mjtNum kMaxDepth = 0.05;
              mjtNum pressure = depth / mju_max(kMaxDepth - depth, mjMINVAL);
              mju_scl3(force, normal, pressure);

              // one row of mat^T * force
              forcesT[0*ncon + node] = mju_dot3(force, normal);
              forcesT[1*ncon + node] = mju_abs(mju_dot3(vel_rel, tang1));
              forcesT[2*ncon + node] = mju_abs(mju_dot3(vel_rel, tang2));
              node++;
            }
          }

          // compute sensor output
          for (int c = 0; c < nchannel; c++) {
            if (!mju_isZero(forcesT + c*ncon, ncon)) {
              mju_addTo(sensordata + c*ncon, forcesT + c*ncon, ncon);
            }
          }

          mj_freeStack(d);
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
        // clear result, compute later
        mju_zero(d->sensordata + adr, m->sensor_dim[i]);
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

  int sleep_filter = mjENABLED(mjENBL_SLEEP) && d->nbody_awake < m->nbody;

  // add joint-level springs
  if (!mjDISABLED(mjDSBL_SPRING)) {
    int nbody = m->nbody;
    for (int b=1; b < nbody; b++) {
      if (sleep_filter && d->body_awake[b] != mjS_AWAKE) continue;

      int jnt_start = m->body_jntadr[b];
      int jnt_end = jnt_start + m->body_jntnum[b];
      for (int j=jnt_start; j < jnt_end; j++) {
        stiffness = m->jnt_stiffness[j];
        if (stiffness == 0) {
          continue;
        }
        padr = m->jnt_qposadr[j];

        switch ((mjtJoint) m->jnt_type[j]) {
        case mjJNT_FREE:
          mju_sub3(dif, d->qpos+padr, m->qpos_spring+padr);
          d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);

          // continue with rotations
          padr += 3;
          mjFALLTHROUGH;

        case mjJNT_BALL:
          // convert quaternion difference into angular "velocity"
          mju_copy4(quat, d->qpos+padr);
          mju_normalize4(quat);
          mju_subQuat(dif, d->qpos + padr, m->qpos_spring + padr);
          d->energy[0] += 0.5 * stiffness * mju_dot3(dif, dif);
          break;

        case mjJNT_SLIDE:
        case mjJNT_HINGE:
          d->energy[0] += 0.5 * stiffness *
                          (d->qpos[padr] - m->qpos_spring[padr]) *
                          (d->qpos[padr] - m->qpos_spring[padr]);
          break;
        }
      }
    }
  }

  // add tendon-level springs
  if (!mjDISABLED(mjDSBL_SPRING)) {
    for (int i=0; i < m->ntendon; i++) {
    // skip sleeping or static tendon
    if (sleep_filter && mj_sleepState(m, d, mjOBJ_TENDON, i) != mjS_AWAKE) {
      continue;
    }

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

  // add flex-level springs for dim=1
  if (!mjDISABLED(mjDSBL_SPRING)) {
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
