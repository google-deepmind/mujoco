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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_RAY_H_
#define MUJOCO_SRC_ENGINE_ENGINE_RAY_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

MJAPI void mju_multiRayPrepare(const mjModel* m, const mjData* d,
                               const mjtNum pnt[3], const mjtNum* ray_xmat,
                               const mjtByte* geomgroup, mjtByte flg_static,
                               int bodyexclude, mjtNum cutoff, mjtNum* geom_ba,
                               int* geom_eliminate);

// Intersect multiple rays emanating from a single source
// Similar semantics to mj_ray, but vec is an array of (nray x 3) directions.
MJAPI void mj_multiRay(const mjModel* m, mjData* d, const mjtNum pnt[3], const mjtNum* vec,
                       const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                       int* geomid, mjtNum* dist, int nray, mjtNum cutoff);

// intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms on bodyexclude
//  return geomid and distance (x) to nearest surface, or -1 if no intersection
//  geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion
MJAPI mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum* pnt, const mjtNum* vec,
                    const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                    int geomid[1]);

// intersect ray with hfield
MJAPI mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid,
                          const mjtNum* pnt, const mjtNum* vec);

// intersect ray with triangle
MJAPI mjtNum ray_triangle(mjtNum v[][3], const mjtNum* lpnt, const mjtNum* lvec,
                          const mjtNum* b0, const mjtNum* b1);

// intersect ray with mesh
MJAPI mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid,
                        const mjtNum* pnt, const mjtNum* vec);

// intersect ray with pure geom, no meshes or hfields
MJAPI mjtNum mju_rayGeom(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                         const mjtNum* pnt, const mjtNum* vec, int geomtype);

// intersect ray with flex, return nearest vertex id
MJAPI mjtNum mju_rayFlex(const mjModel* m, const mjData* d, int flex_layer, mjtByte flg_vert,
                         mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid,
                         const mjtNum* pnt, const mjtNum* vec, int vertid[1]);

// intersect ray with skin, return nearest vertex id
MJAPI mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert,
                         const mjtNum* pnt, const mjtNum* vec, int vertid[1]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_RAY_H_
