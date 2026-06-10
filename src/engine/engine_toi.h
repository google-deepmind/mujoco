// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_TOI_H_
#define MUJOCO_SRC_ENGINE_ENGINE_TOI_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#include "engine/engine_collision_convex.h"
#include "engine/engine_collision_gjk.h"

#ifdef __cplusplus
extern "C" {
#endif

// motion descriptor for one object in a TOI query
// rigid objects (geoms) move with a constant world-frame velocity: the center translates
// linearly and the object rotates about the moving center (not a constant spatial twist);
// flex elements and vertices move with constant per-vertex linear velocities (nvert > 0),
// which also captures deformation over the query horizon
struct _mjTOIMotion {
  // rigid object (nvert == 0)
  mjtNum vel[6];        // world-frame [rot(3); lin(3)] at the object center
  mjtNum rbound;        // bounding-sphere radius about the object center

  // flex object (nvert > 0): element vertices in element-data order, or a single vertex
  int nvert;            // number of moving vertices: flex_dim+1 for an element, 1 for a vertex
  mjtNum vertpos[12];   // vertex positions at t = 0 (world frame)
  mjtNum vertvel[12];   // vertex velocities (world frame)
};
typedef struct _mjTOIMotion mjTOIMotion;

// compute time of impact between two convex geoms within [0, horizon], assuming constant
// world-frame velocities taken from d:
//   x_c(t) = x_c(0) + t*v,   R(t) = exp(hat(w)*t) * R(0)
// return TOI in [0, horizon]; 0 if already touching/penetrating; -1 if no impact
// on impact, fromto holds witness points on geom1/geom2 at the impact poses
// supported geom types: sphere, capsule, ellipsoid, cylinder, box, mesh
// requires up-to-date poses and velocities (call after mj_forward)
// Nullable: fromto
MJAPI mjtNum mj_geomTOI(const mjModel* m, mjData* d, int geom1, int geom2, mjtNum horizon,
                        mjtNum fromto[6]);

// compute time of impact between two flex elements within [0, horizon], assuming constant
// per-vertex linear velocities taken from d (deformation-aware)
// return TOI in [0, horizon]; 0 if already touching/penetrating; -1 if no impact
// on impact, fromto holds witness points on elem1/elem2 at the impact positions
// requires up-to-date flex vertex positions and velocities (call after mj_forward)
// Nullable: fromto
MJAPI mjtNum mj_flexTOI(const mjModel* m, mjData* d, int flex1, int elem1, int flex2, int elem2,
                        mjtNum horizon, mjtNum fromto[6]);

// conservative-advancement core: obj1/obj2 are initialized CCD objects (geom or flex element
// or flex vertex), each paired with a motion descriptor; rigid obj poses (pos/mat) must hold
// the t = 0 poses on entry; flex objects are rewired internally to read advanced vertex
// positions, so their mjData arrays are never modified; on return the objects hold the
// final-iterate configuration; config->buffer must be allocated via mjc_ccdSize and
// config->dist_cutoff is managed internally
// return TOI in [0, horizon] or -1 if no impact
MJAPI mjtNum mjc_toi(const mjCCDConfig* config, mjCCDStatus* status,
                     mjCCDObj* obj1, const mjTOIMotion* motion1,
                     mjCCDObj* obj2, const mjTOIMotion* motion2,
                     mjtNum horizon, mjtNum tolerance);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_TOI_H_
