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

#include <mujoco/mjexport.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#define mjGETINFO_HFIELD \
    const mjtNum* pos1  = d->geom_xpos + 3*g1; \
    const mjtNum* mat1  = d->geom_xmat + 9*g1; \
    const mjtNum* size1 = m->geom_size + 3*g1; \
          mjtNum* pos2  = d->geom_xpos + 3*g2; \
          mjtNum* mat2  = d->geom_xmat + 9*g2;
// mjc_ConvexHField modifies and then restores pos2 and mat2

// minimum number of vertices to use hill-climbing in mesh support
#define mjMESH_HILLCLIMB_MIN 10

#ifdef __cplusplus
extern "C" {
#endif

// internal object type for convex collision detection
struct _mjCCDObj {
  const mjModel* model;
  const mjData* data;
  int geom;
  int geom_type;
  int vertindex;
  int meshindex;
  int flex;
  int elem;
  int vert;
  mjtNum margin;
  mjtNum rotate[4];
  void (*center)(mjtNum res[3], const struct _mjCCDObj* obj);
  void (*support)(mjtNum res[3], struct _mjCCDObj* obj, const mjtNum dir[3]);
  mjtNum prism[6][3];  // for hfield
};
typedef struct _mjCCDObj mjCCDObj;

// initialize a CCD object
MJAPI void mjc_initCCDObj(mjCCDObj* obj, const mjModel* m, const mjData* d, int g, mjtNum margin);

// center function for convex collision algorithms
MJAPI void mjc_center(mjtNum res[3], const mjCCDObj *obj);

// libccd center function
MJAPI void mjccd_center(const void *obj, ccd_vec3_t *center);

// libccd support function
MJAPI void mjccd_support(const void *obj, const ccd_vec3_t *dir, ccd_vec3_t *vec);

// support function for point
void mjc_pointSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]);

// support function for line (capsule)
void mjc_lineSupport(mjtNum res[3], mjCCDObj* obj, const mjtNum dir[3]);

// pairwise geom collision functions using ccd
int mjc_PlaneConvex(const mjModel* m, const mjData* d,
                    mjContact* con, int g1, int g2, mjtNum margin);
int mjc_ConvexHField(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin);
MJAPI int mjc_Convex(const mjModel* m, const mjData* d,
                     mjContact* con, int g1, int g2, mjtNum margin);

// geom-elem or elem-elem or vert-elem collision function using ccd
int mjc_ConvexElem    (const mjModel* m, const mjData* d, mjContact* con,
                       int g1, int f1, int e1, int v1, int f2, int e2, mjtNum margin);

// heightfield-elem collision function using ccd
int mjc_HFieldElem    (const mjModel* m, const mjData* d, mjContact* con,
                       int g, int f, int e, mjtNum margin);

// fix contact frame normal
void mjc_fixNormal(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_CONVEX_H_
