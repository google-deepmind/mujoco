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

#include "engine/engine_vis_interact.h"

#include <math.h>
#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "engine/engine_ray.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_misc.h"
#include "engine/engine_util_spatial.h"

// transform pose from room to model space
void mjv_room2model(mjtNum* modelpos, mjtNum* modelquat, const mjtNum* roompos,
                    const mjtNum* roomquat, const mjvScene* scn) {
  mjtNum translate[3], rotate[4], invpos[3], invquat[4];

  // check scale
  if (scn->scale<mjMINVAL) {
    mju_error("mjvScene scale too small in mjv_room2model");
  }

  // enabled: transform
  if (scn->enabletransform) {
    // convert translate, rotate to mjtNum
    mju_f2n(translate, scn->translate, 3);
    mju_f2n(rotate, scn->rotate, 4);

    // invert model pose (without scale)
    mju_negPose(invpos, invquat, translate, rotate);

    // map from room to model space
    mju_mulPose(modelpos, modelquat, invpos, invquat, roompos, roomquat);

    // divide position by scale
    mju_scl3(modelpos, modelpos, 1.0/scn->scale);
  }

  // disabled: copy
  else {
    mju_copy3(modelpos, roompos);
    mju_copy4(modelquat, roomquat);
  }
}



// transform pose from model to room space
void mjv_model2room(mjtNum* roompos, mjtNum* roomquat, const mjtNum* modelpos,
                    const mjtNum* modelquat, const mjvScene* scn) {
  mjtNum translate[3], rotate[4];

  // check scale
  if (scn->scale<mjMINVAL) {
    mju_error("mjvScene scale too small in mjv_model2room");
  }

  // enabled: transform
  if (scn->enabletransform) {
    // convert translate, rotate to mjtNum
    mju_f2n(translate, scn->translate, 3);
    mju_f2n(rotate, scn->rotate, 4);

    // map from model to room space
    mju_mulPose(roompos, roomquat, translate, rotate, modelpos, modelquat);

    // scale position
    mju_scl3(roompos, roompos, scn->scale);
  }

  // disabled: copy
  else {
    mju_copy3(roompos, modelpos);
    mju_copy4(roomquat, modelquat);
  }
}



// get camera info in model space: average left and right OpenGL cameras
void mjv_cameraInModel(mjtNum* headpos, mjtNum* forward, mjtNum* up, const mjvScene* scn) {
  mjtNum pos[3], fwd[3], u[3], quat[4];
  mjtNum modelpos[3], modelquat[4], modelmat[9];

  // check znear
  if (scn->camera[0].frustum_near<mjMINVAL || scn->camera[1].frustum_near<mjMINVAL) {
    mju_error("mjvScene frustum_near too small in mjv_cameraInModel");
  }

  // clear results
  if (headpos) {
    mju_zero3(headpos);
  }
  if (forward) {
    mju_zero3(forward);
  }
  if (up) {
    mju_zero3(up);
  }

  // average over cameras
  for (int n=0; n<2; n++) {
    // convert pos, fwd, u
    mju_f2n(pos, scn->camera[n].pos, 3);
    mju_f2n(fwd, scn->camera[n].forward, 3);
    mju_f2n(u, scn->camera[n].up, 3);

    // normalize just in case
    mju_normalize3(fwd);
    mju_normalize3(u);

    // make orientation matrix: x = left, y = up, z = forward
    mjtNum left[3];
    mju_cross(left, u, fwd);
    mju_normalize3(left);
    mjtNum mat[9] = {
      left[0], u[0], fwd[0],
      left[1], u[1], fwd[1],
      left[2], u[2], fwd[2]
    };
    mju_mat2Quat(quat, mat);

    // convert to model space, make orientation matrix
    mjv_room2model(modelpos, modelquat, pos, quat, scn);
    mju_quat2Mat(modelmat, modelquat);

    // finalize results
    if (headpos) {
      mju_addToScl3(headpos, modelpos, 0.5);
    }
    if (forward) {
      forward[0] += 0.5*modelmat[2];
      forward[1] += 0.5*modelmat[5];
      forward[2] += 0.5*modelmat[8];
    }
    if (up) {
      up[0] += 0.5*modelmat[1];
      up[1] += 0.5*modelmat[4];
      up[2] += 0.5*modelmat[7];
    }
  }

  // normalize forward and up
  if (forward) {
    mju_normalize3(forward);
  }
  if (up) {
    mju_normalize3(up);
  }
}



// get camera info in room space: average left and right OpenGL cameras
void mjv_cameraInRoom(mjtNum* headpos, mjtNum* forward, mjtNum* up, const mjvScene* scn) {
  mjtNum pos[3], fwd[3], u[3];

  // check znear
  if (scn->camera[0].frustum_near<mjMINVAL || scn->camera[1].frustum_near<mjMINVAL) {
    mju_error("mjvScene frustum_near too small in mjv_cameraInRoom");
  }

  // clear results
  if (headpos) {
    mju_zero3(headpos);
  }
  if (forward) {
    mju_zero3(forward);
  }
  if (up) {
    mju_zero3(up);
  }

  // average over cameras
  for (int n=0; n<2; n++) {
    // convert pos, fwd, u
    mju_f2n(pos, scn->camera[n].pos, 3);
    mju_f2n(fwd, scn->camera[n].forward, 3);
    mju_f2n(u, scn->camera[n].up, 3);

    // finalize results
    if (headpos) {
      mju_addToScl3(headpos, pos, 0.5);
    }
    if (forward) {
      mju_addToScl3(forward, fwd, 0.5);
    }
    if (up) {
      mju_addToScl3(up, u, 0.5);
    }
  }

  // normalize
  if (forward) {
    mju_normalize3(forward);
  }
  if (up) {
    mju_normalize3(up);
  }
}



// get frustum height at unit distance from camera; average left and right OpenGL cameras
mjtNum mjv_frustumHeight(const mjvScene* scn) {
  mjtNum height;

  // check znear
  if (scn->camera[0].frustum_near<mjMINVAL || scn->camera[1].frustum_near<mjMINVAL) {
    mju_error("mjvScene frustum_near too small in mjv_frustumHeight");
  }

  // add normalized height for left and right cameras
  height = (scn->camera[0].frustum_top-scn->camera[0].frustum_bottom)/scn->camera[0].frustum_near +
           (scn->camera[1].frustum_top-scn->camera[1].frustum_bottom)/scn->camera[1].frustum_near;

  // average
  return 0.5*height;
}



// rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)
MJAPI void mjv_alignToCamera(mjtNum* res, const mjtNum* vec, const mjtNum* forward) {
  mjtNum xaxis[2], yaxis[2];

  // fotward-aligned y-axis
  mju_copy(yaxis, forward, 2);
  mju_normalize(yaxis, 2);

  // corresponding x-axis
  xaxis[0] = yaxis[1];
  xaxis[1] = -yaxis[0];

  // apply horizontal rotation
  res[0] = vec[0]*xaxis[0] + vec[1]*yaxis[0];
  res[1] = vec[0]*xaxis[1] + vec[1]*yaxis[1];
  res[2] = vec[2];
}



// convert 2D mouse motion to z-aligned 3D world coordinates
static void convert2D(mjtNum* res, int action, mjtNum dx, mjtNum dy, const mjtNum* forward) {
  mjtNum vec[3];

  // construct 3D vector
  switch (action) {
  case mjMOUSE_ROTATE_V:
    vec[0] = dy;
    vec[1] = 0;
    vec[2] = dx;
    break;

  case mjMOUSE_ROTATE_H:
    vec[0] = dy;
    vec[1] = dx;
    vec[2] = 0;
    break;

  case mjMOUSE_MOVE_V:
    vec[0] = dx;
    vec[1] = 0;
    vec[2] = -dy;
    break;

  case mjMOUSE_MOVE_H:
    vec[0] = dx;
    vec[1] = -dy;
    vec[2] = 0;
    break;

  case mjMOUSE_ZOOM:
    break;

  default:
    mju_error_i("Unexpected mouse action %d in convert2D", action);
  }

  // call 3D converter
  mjv_alignToCamera(res, vec, forward);
}



// move camera with mouse; action is mjtMouse
void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                    const mjvScene* scn, mjvCamera* cam) {
  mjtNum headpos[3], forward[3];
  mjtNum vec[3], dif[3], scl;

  // fixed camera: nothing to do
  if (cam->type==mjCAMERA_FIXED) {
    return;
  }

  // process action
  switch (action) {
  case mjMOUSE_ROTATE_V:
  case mjMOUSE_ROTATE_H:
    cam->azimuth -= reldx * 180.0;
    cam->elevation -= reldy * 180.0;
    break;

  case mjMOUSE_MOVE_V:
  case mjMOUSE_MOVE_H:
    // do not move lookat point of tracking camera
    if (cam->type==mjCAMERA_TRACKING) {
      return;
    }

    // get camera info and align
    mjv_cameraInModel(headpos, forward, NULL, scn);
    convert2D(vec, action, reldx, reldy, forward);

    // compute scaling: rendered lookat displacement = mouse displacement
    mju_sub3(dif, cam->lookat, headpos);
    scl = mjv_frustumHeight(scn) * mju_dot3(dif, forward);

    // move lookat point in opposite direction
    mju_addToScl3(cam->lookat, vec, -scl);
    break;

  case mjMOUSE_ZOOM:
    cam->distance -= mju_log(1 + cam->distance/m->stat.extent/3) * reldy * 9 * m->stat.extent;
    break;

  default:
    mju_error_i("Unexpected action %d in mjv_moveCamera", action);
  }

  // clamp camera parameters
  if (cam->azimuth > 180) {
    cam->azimuth -= 360;
  }
  if (cam->azimuth < -180) {
    cam->azimuth += 360;
  }
  if (cam->elevation > 89) {
    cam->elevation = 89;
  }
  if (cam->elevation < -89) {
    cam->elevation = -89;
  }
  if (cam->distance < 0.01*m->stat.extent) {
    cam->distance = 0.01*m->stat.extent;
  }
  if (cam->distance > 100*m->stat.extent) {
    cam->distance = 100*m->stat.extent;
  }
}



// move perturb object with mouse; action is mjtMouse
void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
                     mjtNum reldy, const mjvScene* scn, mjvPerturb* pert) {
  int sel = pert->select;
  mjtNum forward[3], vec[3], scl, q1[4], xiquat[4];

  // get camera info and align
  mjv_cameraInModel(NULL, forward, NULL, scn);
  convert2D(vec, action, reldx, reldy, forward);

  // process action
  switch (action) {
  case mjMOUSE_MOVE_V:
  case mjMOUSE_MOVE_H:
    mju_addToScl3(pert->refpos, vec, pert->scale);
    break;

  case mjMOUSE_ROTATE_V:
  case mjMOUSE_ROTATE_H:
    // normalize vector, get length
    scl = mju_normalize3(vec);

    // make quaternion and apply
    mju_axisAngle2Quat(q1, vec, scl*mjPI*2);
    mju_mulQuat(pert->refquat, q1, pert->refquat);
    mju_normalize4(pert->refquat);

    // compute xiquat
    mju_mulQuat(xiquat, d->xquat+4*sel, m->body_iquat+4*sel);

    // limit rotation relative to selected body
    if (sel>0 && sel<m->nbody) {
      // q2 = neg(selbody) * refquat
      mjtNum q2[4];
      mju_negQuat(q1, xiquat);
      mju_mulQuat(q2, q1, pert->refquat);

      // convert q2 to axis-angle
      mjtNum dif[3];
      mju_quat2Vel(dif, q2, 1);
      scl = mju_normalize3(dif);

      // check limit: +/- 90 deg allowed
      if (scl<-mjPI*0.5 || scl>mjPI*0.5) {
        // clamp angle
        scl = mju_max(-mjPI*0.5, mju_min(mjPI*0.5, scl));

        // reconstruct q2
        mju_axisAngle2Quat(q2, dif, scl);

        // set refquat = selbody * q2_new
        mju_mulQuat(pert->refquat, xiquat, q2);
      }
    }
    break;

  case mjMOUSE_ZOOM:
    break;

  default:
    mju_error_i("Unexpected mouse action %d in mjv_movePerturb", action);
  }
}



// move model with mouse; action is mjtMouse
void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                   const mjtNum roomup[3], mjvScene* scn) {
  mjtNum roomforward[3], roomright[3], camforward[3];
  mjtNum vec[3], scl, quat[4], rotate[4], result[4];

  // transformation disabled: nothing to do
  if (!scn->enabletransform) {
    return;
  }

  // get camera forward in room space
  mjv_cameraInRoom(NULL, camforward, NULL, scn);

  // make orthogonal to roomright
  mju_addScl3(roomforward, camforward, roomup, -mju_dot3(camforward, roomup));
  mju_normalize3(roomforward);

  // compute roomright
  mju_cross(roomright, roomforward, roomup);
  mju_normalize3(roomright);

  // process action
  switch (action) {
  case mjMOUSE_ROTATE_V:
  case mjMOUSE_ROTATE_H:
    // construct rotation vector
    for (int i=0; i<3; i++) {
      if (action==mjMOUSE_ROTATE_V) {
        vec[i] = roomup[i]*reldx + roomright[i]*reldy;
      } else {
        vec[i] = roomforward[i]*reldx + roomright[i]*reldy;
      }
    }

    // make quaternion from angle-axis
    scl = mju_normalize3(vec);
    mju_axisAngle2Quat(quat, vec, scl*mjPI);

    // get current model rotation
    mju_f2n(rotate, scn->rotate, 4);

    // compose rotation, normalize and and set
    mju_mulQuat(result, quat, rotate);
    mju_normalize4(result);
    mju_n2f(scn->rotate, result, 4);
    break;

  case mjMOUSE_MOVE_V:
    for (int i=0; i<3; i++) {
      scn->translate[i] += (float)(roomright[i]*reldx - roomup[i]*reldy) * m->stat.extent;
    }
    break;

  case mjMOUSE_MOVE_H:
    for (int i=0; i<3; i++) {
      scn->translate[i] += (float)(roomright[i]*reldx - roomforward[i]*reldy) * m->stat.extent;
    }
    break;

  case mjMOUSE_ZOOM:
    scn->scale += (float)(mju_log(1 + scn->scale/3) * reldy * 3);
    if (scn->scale<0.01f) {
      scn->scale = 0.01f;
    } else if (scn->scale>100.0f) {
      scn->scale = 100.0f;
    }
    break;

  default:
    mju_error_i("Unexpected action %d in mjv_moveModel", action);
  }
}



// copy perturb pos,quat from selected body; set scale for perturbation
void mjv_initPerturb(const mjModel* m, const mjData* d, const mjvScene* scn, mjvPerturb* pert) {
  int sel = pert->select;
  mjtNum headpos[3], forward[3], dif[3];

  // invalid selected body: return
  if (sel<=0 || sel>=m->nbody) {
    return;
  }

  // compute selection point in world coordinates
  mjtNum selpos[3];
  mju_rotVecMat(selpos, pert->localpos, d->xmat+9*pert->select);
  mju_addTo3(selpos, d->xpos+3*pert->select);

  // copy
  mju_copy3(pert->refpos, selpos);
  mju_mulQuat(pert->refquat, d->xquat + 4*sel, m->body_iquat + 4*sel);

  // get camera info
  mjv_cameraInModel(headpos, forward, NULL, scn);

  // compute scaling: rendered pert->refpos displacement = mouse displacement
  mju_sub3(dif, pert->refpos, headpos);
  pert->scale = mjv_frustumHeight(scn) * mju_dot3(dif, forward);
}



// set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
//  d->qpos written only if flg_paused and subtree root for selected body has free joint
void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
                          int flg_paused) {
  int rootid = 0, sel = pert->select;
  mjtNum pos1[3], quat1[4], pos2[3], quat2[4], refpos[3], refquat[4];
  mjtNum *Rpos, *Rquat, *Cpos, *Cquat;

  // exit if nothing to do
  if (sel<=0 || sel>=m->nbody || !(pert->active | pert->active2)) {
    return;
  }

  // get rootid above selected body
  rootid = m->body_rootid[sel];

  // transform refpos,refquat from I-frame to X-frame of body[sel]
  mju_negPose(pos1, quat1, m->body_ipos+3*sel, m->body_iquat+4*sel);
  mju_mulPose(refpos, refquat, pert->refpos, pert->refquat, pos1, quat1);

  // mocap body
  if (m->body_mocapid[sel]>=0) {
    // copy ref pose into mocap pose
    mju_copy3(d->mocap_pos + 3*m->body_mocapid[sel], refpos);
    mju_copy4(d->mocap_quat + 4*m->body_mocapid[sel], refquat);
  }

  // floating body, paused
  else if (flg_paused && m->body_jntnum[sel]==1 &&
           m->jnt_type[m->body_jntadr[sel]]==mjJNT_FREE) {
    // copy ref pose into qpos
    mju_copy3(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]], refpos);
    mju_copy4(d->qpos + m->jnt_qposadr[m->body_jntadr[sel]] + 3, refquat);
  }

  // child of floating body, paused
  else if (flg_paused && m->body_jntnum[rootid]==1 &&
           m->jnt_type[m->body_jntadr[rootid]]==mjJNT_FREE) {
    // get pointers to root
    Rpos = d->qpos + m->jnt_qposadr[m->body_jntadr[rootid]];
    Rquat = Rpos + 3;

    // get pointers to child
    Cpos = d->xpos + 3*sel;
    Cquat = d->xquat + 4*sel;

    // set root <- ref*neg(child)*root
    mju_negPose(pos1, quat1, Cpos, Cquat);                      // neg(child)
    mju_mulPose(pos2, quat2, pos1, quat1, Rpos, Rquat);         // neg(child)*root
    mju_mulPose(Rpos, Rquat, refpos, refquat, pos2, quat2);     // ref*neg(child)*root
  }
}



// set perturb force,torque in d->xfrc_applied, if selected body is dynamic
void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert) {
  mjtNum xiquat[4], difquat[4], bvel[6], mass, stiffness, *result;
  int sel = pert->select;

  // exit if nothing to do
  if (sel<0 || sel>=m->nbody || !(pert->active | pert->active2)) {
    return;
  }

  // get pointer to body xfrc_applied
  result = d->xfrc_applied + 6*sel;

  // global selbody velocity
  mj_objectVelocity(m, d, mjOBJ_BODY, sel, bvel, 0);

  if (((pert->active | pert->active2) & mjPERT_TRANSLATE)) {
    // compute selection point in world coordinates
    mjtNum selpos[3];
    mju_rotVecMat(selpos, pert->localpos, d->xmat+9*pert->select);
    mju_addTo3(selpos, d->xpos+3*pert->select);

    // spring perturbation force, with critical damping
    stiffness = m->vis.map.stiffness;
    mass = 1.0/mju_max(mjMINVAL, m->body_invweight0[2*sel]);
    mju_sub3(result, pert->refpos, selpos);
    mju_scl3(result, result, stiffness*mass);
    mju_addToScl3(result, bvel+3, -sqrtf(stiffness)*mass);

    // torque w.r.t body com
    mju_subFrom3(selpos, d->xipos+3*pert->select);
    mju_cross(result+3, selpos, result);

    // add critically damped torque (torsional only)
    stiffness = m->vis.map.stiffnessrot;
    mass = 1.0/mju_max(mjMINVAL, m->body_invweight0[2*sel+1]);
    mju_normalize3(selpos);
    mju_addToScl3(result+3, selpos, -sqrtf(stiffness)*mass*mju_dot3(selpos, bvel));
  }

  if (((pert->active | pert->active2) & mjPERT_ROTATE)) {
    // spring perturbation torque, with critical damping
    stiffness = m->vis.map.stiffnessrot;
    mass = 1.0/mju_max(mjMINVAL, m->body_invweight0[2*sel+1]);
    mju_mulQuat(xiquat, d->xquat+4*sel, m->body_iquat+4*sel);
    mju_negQuat(xiquat, xiquat);
    mju_mulQuat(difquat, pert->refquat, xiquat);
    mju_quat2Vel(result+3, difquat, 1.0/(stiffness*mass));
    mju_addToScl3(result+3, bvel, -sqrtf(stiffness)*mass);
  }
}



// return the average of two OpenGL cameras
mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2) {
  mjtNum pos[3], forward[3], up[3], projection, tmp1[3], tmp2[3];
  mjvGLCamera cam;

  // compute pos
  mju_f2n(tmp1, cam1->pos, 3);
  mju_f2n(tmp2, cam2->pos, 3);
  mju_add3(pos, tmp1, tmp2);
  mju_scl3(pos, pos, 0.5);

  // compute forward
  mju_f2n(tmp1, cam1->forward, 3);
  mju_f2n(tmp2, cam2->forward, 3);
  mju_add3(forward, tmp1, tmp2);
  mju_normalize3(forward);

  // compute up, make it orthogonal to forward
  mju_f2n(tmp1, cam1->up, 3);
  mju_f2n(tmp2, cam2->up, 3);
  mju_add3(up, tmp1, tmp2);
  projection = mju_dot3(up, forward);
  mju_addToScl3(up, forward, -projection);
  mju_normalize3(up);

  // assign 3d quantities
  mju_n2f(cam.pos, pos, 3);
  mju_n2f(cam.forward, forward, 3);
  mju_n2f(cam.up, up, 3);

  // average frustum
  cam.frustum_bottom = 0.5f * (cam1->frustum_bottom + cam2->frustum_bottom);
  cam.frustum_top    = 0.5f * (cam1->frustum_top + cam2->frustum_top);
  cam.frustum_center = 0.5f * (cam1->frustum_center + cam2->frustum_center);
  cam.frustum_near   = 0.5f * (cam1->frustum_near + cam2->frustum_near);
  cam.frustum_far    = 0.5f * (cam1->frustum_far + cam2->frustum_far);

  return cam;
}



// Select geom or skin with mouse, return bodyid; -1: none selected.
int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
               mjtNum aspectratio, mjtNum relx, mjtNum rely,
               const mjvScene* scn, mjtNum selpnt[3], int geomid[1], int skinid[1]) {
  // get average camera
  mjvGLCamera cam = mjv_averageCamera(scn->camera, scn->camera+1);

  // get camera pose in model space
  mjtNum pos[3], forward[3], up[3], left[3];
  mjv_cameraInModel(pos, forward, up, scn);
  mju_cross(left, up, forward);
  mju_normalize3(left);

  // compute frustum halfwidth so as to match viewport aspect ratio
  mjtNum halfwidth = 0.5*aspectratio*(cam.frustum_top - cam.frustum_bottom);

  // construct ray
  mjtNum ray[3];
  mju_scl3(ray, forward, cam.frustum_near);
  mju_addToScl3(ray, up, cam.frustum_bottom + rely*(cam.frustum_top-cam.frustum_bottom));
  mju_addToScl3(ray, left, -(cam.frustum_center + (2*relx-1)*halfwidth));
  mju_normalize3(ray);

  // find intersection with geoms
  *geomid = -1;
  mjtNum geomdist = mj_ray(m, d, pos, ray, vopt->geomgroup,
                           vopt->flags[mjVIS_STATIC], -1, geomid);

  // find intersection with skins
  int bodyid = -1;
  mjtNum skindist = -1;
  *skinid = -1;
  if (vopt->flags[mjVIS_SKIN]) {
    for (int i=0; i<m->nskin; i++) {
      // process one skin
      int vertid;
      mjtNum newdist = mju_raySkin(m->skin_facenum[i], m->skin_vertnum[i],
                                   m->skin_face + 3*m->skin_faceadr[i],
                                   scn->skinvert + 3*m->skin_vertadr[i],
                                   pos, ray, &vertid);

      // update if closer intersection found
      if (newdist>=0 && (newdist<skindist || skindist<0)) {
        // assign result
        skindist = newdist;

        // find body with largest weight for this vertex
        float bestweight = -1;
        for (int j=m->skin_boneadr[i];
             j<m->skin_boneadr[i]+m->skin_bonenum[i];
             j++) {
          for (int k=m->skin_bonevertadr[j];
               k<m->skin_bonevertadr[j]+m->skin_bonevertnum[j];
               k++) {
            // get vertex id and weight
            int vid = m->skin_bonevertid[k];
            float vweight = m->skin_bonevertweight[k];

            // update if matching id and bigger weight
            if (vid==vertid && vweight>bestweight) {
              bestweight = vweight;
              bodyid = m->skin_bonebodyid[j];
              *skinid = i;
            }
          }
        }
      }
    }
  }

  // no intersection
  if (geomdist<0 && skindist<0) {
    return -1;
  }

  // geom only, or geom closer than skin
  else if (geomdist>=0 && (skindist<0 || skindist>geomdist)) {
    mju_addScl3(selpnt, pos, ray, geomdist);
    *skinid = -1;
    return m->geom_bodyid[*geomid];
  }

  // skin
  else {
    mju_addScl3(selpnt, pos, ray, skindist);
    *geomid = -1;
    return bodyid;
  }
}
