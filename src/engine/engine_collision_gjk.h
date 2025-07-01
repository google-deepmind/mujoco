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

#include <float.h>
#include <stddef.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#include "engine/engine_collision_convex.h"

#ifdef __cplusplus
extern "C" {
#endif

// numerical max limit
#ifndef mjUSESINGLE
  #define mjMAX_LIMIT DBL_MAX
#else
  #define mjMAX_LIMIT FLT_MAX
#endif

// max number of EPA iterations
#define mjMAX_EPA_ITERATIONS 170

// tolerance for normal alignment of two faces (cosine of 1.6e-3)
#define mjFACE_TOL 0.99999872

// tolerance for edge-face alignment (sine of 1.6e-3)
#define mjEDGE_TOL 0.00159999931

// max number of supported vertices in a polygon face of a mesh
#define mjMAX_POLYVERT 150

// Status of an EPA run
typedef enum {
  mjEPA_NOCONTACT           = -1,
  mjEPA_SUCCESS             = 0,
  mjEPA_P2_INVALID_FACES,
  mjEPA_P2_NONCONVEX,
  mjEPA_P2_ORIGIN_ON_FACE,
  mjEPA_P3_BAD_NORMAL,
  mjEPA_P3_INVALID_V4,
  mjEPA_P3_INVALID_V5,
  mjEPA_P3_MISSING_ORIGIN,
  mjEPA_P3_ORIGIN_ON_FACE,
  mjEPA_P4_MISSING_ORIGIN,
} mjEPAStatus;

// vertex in a polytope
typedef struct {
  mjtNum vert[3];   // v1 - v2; vertex in Minkowski sum making up polytope
  mjtNum vert1[3];  // vertex of polytope in obj1
  mjtNum vert2[3];  // vertex of polytope in obj2
  int index1;       // vertex index in mesh 1
  int index2;       // vertex index in mesh 2
} Vertex;

// configuration for convex collision detection
typedef struct {
  int max_iterations;  // the maximum number of iterations for GJK and EPA
  mjtNum tolerance;    // tolerance used by GJK and EPA
  int max_contacts;    // set to max number of contact points to recover
  mjtNum dist_cutoff;  // set to max geom distance to recover
  void* context;       // opaque data pointer passed to callbacks

  // callback to allocate memory for polytope (only needed for penetration recovery)
  void*(*alloc)(void* context, size_t nbytes);

  // callback to free memory from alloc callback
  void(*free)(void* context, void* buffer);
} mjCCDConfig;

// data produced from running GJK and EPA
typedef struct {
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
  int epa_iterations;           // number of iterations that EPA ran (zero if EPA did not run)
  mjEPAStatus epa_status;       // status of the EPA run
  Vertex simplex[4];
  int nsimplex;
} mjCCDStatus;

// run general convex collision detection, returns positive for distance, negative for penetration
MJAPI mjtNum mjc_ccd(const mjCCDConfig* config, mjCCDStatus* status, mjCCDObj* obj1, mjCCDObj* obj2);
#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_GJK_H_
