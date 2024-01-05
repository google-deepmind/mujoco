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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_VIS_INTERACT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_VIS_INTERACT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>

#ifdef __cplusplus
extern "C" {
#endif

// transform pose from room to model space
MJAPI void mjv_room2model(mjtNum* modelpos, mjtNum* modelquat, const mjtNum* roompos,
                          const mjtNum* roomquat, const mjvScene* scn);

// transform pose from model to room space
MJAPI void mjv_model2room(mjtNum* roompos, mjtNum* roomquat, const mjtNum* modelpos,
                          const mjtNum* modelquat, const mjvScene* scn);

// get camera info in model space: average left and right OpenGL cameras
MJAPI void mjv_cameraInModel(mjtNum* headpos, mjtNum* forward, mjtNum* up,
                             const mjvScene* scn);

// get camera info in room space: average left and right OpenGL cameras
MJAPI void mjv_cameraInRoom(mjtNum* headpos, mjtNum* forward, mjtNum* up,
                            const mjvScene* scn);

// get frustum height at unit distance from camera; average left and right OpenGL cameras
MJAPI mjtNum mjv_frustumHeight(const mjvScene* scn);

// rotate 3D vec in horizontal plane by angle between (0,1) and (forward_x,forward_y)
MJAPI void mjv_alignToCamera(mjtNum* res, const mjtNum* vec, const mjtNum* forward);

// move camera with mouse; action is mjtMouse
MJAPI void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                          const mjvScene* scn, mjvCamera* cam);

// move perturb object with mouse; action is mjtMouse
MJAPI void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
                           mjtNum reldy, const mjvScene* scn, mjvPerturb* pert);

// move model with mouse; action is mjtMouse
MJAPI void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
                         const mjtNum roomup[3], mjvScene* scn);

// copy perturb pos,quat from selected body; set scale perturbation
MJAPI void mjv_initPerturb(const mjModel* m, mjData* d, const mjvScene* scn, mjvPerturb* pert);

// set perturb pos,quat in d->mocap when selected body is mocap, and in d->qpos otherwise
//  d->qpos written only if flg_paused and subtree root for selected body has free joint
MJAPI void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
                                int flg_paused);

// set perturb force,torque in d->xfrc_applied, if selected body is dynamic
MJAPI void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert);

// return the average of two OpenGL cameras
MJAPI mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2);

// Select geom, flex or skin with mouse, return bodyid; -1: none selected.
MJAPI int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
                     mjtNum aspectratio, mjtNum relx, mjtNum rely,
                     const mjvScene* scn, mjtNum selpnt[3],
                     int geomid[1], int flexid[1], int skinid[1]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_VIS_INTERACT_H_
