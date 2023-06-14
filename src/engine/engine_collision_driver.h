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
#elif !defined(__STDC_VERSION__) || __STDC_VERSION__ < 201112L
// No C11 support in Visual Studio 2019 update 7 and earlier.
// However, MSVC allows C++ alignas to be used in C code, so
// we can just skip the #include <stdalign.h>.
#ifndef _MSC_VER
#error "Compiler does not support C11."
#endif
#else
#include <stdalign.h>
#endif

struct mjCollisionTree_ {
  alignas(mjtNum) int node1;
  int node2;
};
typedef struct mjCollisionTree_ mjCollisionTree;

// collision function pointers and max contact pairs
MJAPI extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];

// collision detection entry point
MJAPI void mj_collision(const mjModel* m, mjData* d);

// applies Separating Axis Theorem for rotated AABBs
MJAPI int mj_collideOBB(const mjtNum aabb1[6], const mjtNum aabb2[6],
                        const mjtNum xpos1[3], const mjtNum xmat1[9],
                        const mjtNum xpos2[3], const mjtNum xmat2[9],
                        mjtNum product[36], mjtNum offset[12], mjtByte* initialize);

// broad phase collision detection; return list of body pairs for narrow phase
int mj_broadphase(const mjModel* m, mjData* d, int* bodypair, int maxpair);

// test two geoms for collision, apply filters, add to contact list
//  flg_user disables filters and uses usermargin
void mj_collideGeoms(const mjModel* m, mjData* d,
                     int g1, int g2, int flg_user, mjtNum usermargin);

// number of possible collisions based on filters and geom types
int mj_contactFilter(int contype1, int conaffinity1,
                     int contype2, int conaffinity2);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_DRIVER_H_
