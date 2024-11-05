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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_VIS_VISUALIZE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_VIS_VISUALIZE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>

#ifdef __cplusplus
extern "C" {
#endif

// set (type, size, pos, mat) connector-type geom between given points
//  assume that mjv_initGeom was already called to set all other properties
MJAPI void mjv_connector(mjvGeom* geom, int type, mjtNum width,
                         const mjtNum from[3], const mjtNum to[3]);

// initialize given fields when not NULL, set the rest to their default values
MJAPI void mjv_initGeom(mjvGeom* geom, int type, const mjtNum* size,
                        const mjtNum* pos, const mjtNum* mat, const float* rgba);

// update entire scene
MJAPI void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
                           const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn);

// add geoms from selected categories to existing scene
MJAPI void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
                        const mjvPerturb* pert, int catmask, mjvScene* scn);

// make list of lights only
MJAPI void mjv_makeLights(const mjModel* m, const mjData* d, mjvScene* scn);

// update camera only
MJAPI void mjv_updateCamera(const mjModel* m, const mjData* d, mjvCamera* cam, mjvScene* scn);

// update visible flexes only
MJAPI void mjv_updateActiveFlex(const mjModel* m, mjData* d, mjvScene* scn, const mjvOption* opt);

// update skins only
MJAPI void mjv_updateSkin(const mjModel* m, const mjData* d, mjvScene* scn);

// update visible skins only
MJAPI void mjv_updateActiveSkin(const mjModel* m, const mjData* d, mjvScene* scn, const mjvOption* opt);

int mjv_catenary(const mjtNum x0[3], const mjtNum x1[3], const mjtNum gravity[3], mjtNum length,
                 mjtNum* catenary, int ncatenary);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_VIS_VISUALIZE_H_
