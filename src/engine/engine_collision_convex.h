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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_

// libCCD has an unconditional `#define _CRT_SECURE_NO_WARNINGS` on Windows.
// TODO(stunya): Remove once https://github.com/danfis/libccd/pull/77 is merged
#ifdef _CRT_SECURE_NO_WARNINGS
#undef _CRT_SECURE_NO_WARNINGS
#endif

#include <ccd/vec3.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// ccd general object type
struct _mjtCCD {
  const mjModel* model;
  const mjData* data;
  int geom;
  int meshindex;
  mjtNum margin;
  mjtNum rotate[4];
};
typedef struct _mjtCCD mjtCCD;


// ccd support function
void mjccd_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);


// pairwise collision functions using ccd
int mjc_PlaneConvex   (const mjModel* m, const mjData* d,
                       mjContact* con, int g1, int g2, mjtNum margin);
int mjc_ConvexHField  (const mjModel* m, const mjData* d,
                       mjContact* con, int g1, int g2, mjtNum margin);
int mjc_Convex        (const mjModel* m, const mjData* d,
                       mjContact* con, int g1, int g2, mjtNum margin);


// fix contact frame normal
void mjc_fixNormal(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_
