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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// collision function pointers and max contact pairs
MJAPI extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];

// collision detection entry point
MJAPI void mj_collision(const mjModel* m, mjData* d);

// applies Separating Axis Theorem for rotated AABBs
MJAPI int mj_collideOBB(const mjtNum aabb1[6], const mjtNum aabb2[6],
                        const mjtNum xpos1[3], const mjtNum xmat1[9],
                        const mjtNum xpos2[3], const mjtNum xmat2[9], mjtNum margin,
                        mjtNum product[36], mjtNum offset[12], mjtByte* initialize);

// is element active (for collisions)
MJAPI int mj_isElemActive(const mjModel* m, int f, int e);

// checks if pair is already present in pair_geom and calls narrow phase
void mj_collideGeomPair(const mjModel* m, mjData* d, int g1, int g2, int merged,
                        int startadr, int pairadr);

// binary search between two bodyflex trees
void mj_collideTree(const mjModel* m, mjData* d, int bf1, int bf2,
                    int merged, int startadr, int pairadr);

// broad phase collision detection; return list of bodyflex pairs
int mj_broadphase(const mjModel* m, mjData* d, int* bfpair, int maxpair);

// test two geoms for collision, apply filters, add to contact list
void mj_collideGeoms(const mjModel* m, mjData* d, int g1, int g2);

// test a plane geom and a flex for collision, add to contact list
void mj_collidePlaneFlex(const mjModel* m, mjData* d, int g, int f);

// test for internal flex collisions, add to contact list
void mj_collideFlexInternal(const mjModel* m, mjData* d, int f);

// test active element self-collisions with SAP
void mj_collideFlexSAP(const mjModel* m, mjData* d, int f);

// test a geom and an elem for collision, add to contact list
void mj_collideGeomElem(const mjModel* m, mjData* d, int g, int f, int e);

// test two elems for collision, add to contact list
void mj_collideElems(const mjModel* m, mjData* d, int f1, int e1, int f2, int e2);

// test element and vertex for collision, add to contact list
void mj_collideElemVert(const mjModel* m, mjData* d, int f, int e, int v);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_
