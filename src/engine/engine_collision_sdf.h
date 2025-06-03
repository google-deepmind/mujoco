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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_COLLISION_SDF_H_
#define MUJOCO_SRC_ENGINE_ENGINE_COLLISION_SDF_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

// get sdf from geom id
MJAPI const mjpPlugin* mjc_getSDF(const mjModel* m, int id);

// signed distance function
MJAPI mjtNum mjc_distance(const mjModel* m, const mjData* d, const mjSDF* s, const mjtNum x[3]);

// gradient of sdf
MJAPI void mjc_gradient(const mjModel* m, const mjData* d, const mjSDF* s, mjtNum gradient[3],
                        const mjtNum x[3]);

// collision between a height field and a signed distance field
int mjc_HFieldSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);

// collision between a mesh and a signed distance field
int mjc_MeshSDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);

// collision between two signed distance fields
int mjc_SDF(const mjModel* m, const mjData* d, mjContact* con, int g1, int g2, mjtNum margin);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_COLLISION_SDF_H_
