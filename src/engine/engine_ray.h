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

// intersect ray (pnt+x*vec, x>=0) with visible geoms, except geoms on bodyexclude
//  return geomid and distance (x) to nearest surface, or -1 if no intersection
//  geomgroup, flg_static are as in mjvOption; geomgroup==NULL skips group exclusion
MJAPI mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum* pnt, const mjtNum* vec,
                    const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
                    int geomid[1]);

// interect ray with hfield
MJAPI mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid,
                          const mjtNum* pnt, const mjtNum* vec);

// interect ray with mesh
MJAPI mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid,
                        const mjtNum* pnt, const mjtNum* vec);

// interect ray with pure geom, no meshes or hfields
MJAPI mjtNum mju_rayGeom(const mjtNum* pos, const mjtNum* mat, const mjtNum* size,
                         const mjtNum* pnt, const mjtNum* vec, int geomtype);

// interect ray with skin, return nearest vertex id
MJAPI mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert,
                         const mjtNum* pnt, const mjtNum* vec, int vertid[1]);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_RAY_H_
