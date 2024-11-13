// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include "engine/engine_collision_convex.h"

#ifdef __cplusplus
extern "C" {
#endif

// configuration for convex collision detection
struct _mjCCDConfig {
  int max_iterations;   // the maximum number of iterations for GJK and EPA
  mjtNum tolerance;     // tolerance used by GJK and EPA
  int max_contacts;     // set to max number of contact points to recover
  mjtNum dist_cutoff;   // set to max geom distance to recover
};
typedef struct _mjCCDConfig mjCCDConfig;

// data produced from running GJK and EPA
struct _mjCCDStatus {
  // geom distance information
  mjtNum dist;                  // distance between geoms
  mjtNum x1[3 * mjMAXCONPAIR];  // witness points for geom 1
  mjtNum x2[3 * mjMAXCONPAIR];  // witness points for geom 2
  int nx;                       // number of witness points

  // configurations used
  int max_iterations;           // the maximum number of iterations for GJK and EPA
  mjtNum tolerance;             // tolerance used by GJK and EPA
  int max_contacts;             // set to max number of contact points to recover
  mjtNum dist_cutoff;           // set to max geom distance to recover

  // statistics for debugging purposes
  int gjk_iterations;           // number of iterations that GJK ran
  int epa_iterations;           // number of iterations that EPA ran (negative if EPA did not run)
  mjtNum simplex1[12];          // the simplex that GJK returned for obj1
  mjtNum simplex2[12];          // the simplex that GJK returned for obj2
  mjtNum simplex[12];           // the simplex that GJK returned for the Minkowski difference
  int nsimplex;                 // size of simplex 1 & 2
};
typedef struct _mjCCDStatus mjCCDStatus;

// run general convex collision detection, returns positive for distance, negative for penetration
MJAPI mjtNum mjc_ccd(const mjCCDConfig* config, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
