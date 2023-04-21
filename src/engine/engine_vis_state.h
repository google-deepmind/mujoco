// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_VIS_STATE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_VIS_STATE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#ifdef __cplusplus
extern "C" {
#endif
// set default scene state
MJAPI void mjv_defaultSceneState(mjvSceneState* scnstate);

// allocate and init scene state
MJAPI void mjv_makeSceneState(const mjModel* m, const mjData* d,
                              mjvSceneState* scnstate, int maxgeom);

// free scene state
MJAPI void mjv_freeSceneState(mjvSceneState* scnstate);

// shallow copy scene state into model and data for use with mjv functions
void mjv_assignFromSceneState(const mjvSceneState* scnstate, mjModel* m, mjData* d);

// update entire scene from a scene state, return the number of new mjWARN_VGEOMFULL warnings
MJAPI int mjv_updateSceneFromState(const mjvSceneState* scnstate, const mjvOption* opt,
                                   const mjvPerturb* pert, mjvCamera* cam, int catmask,
                                   mjvScene* scn);

// update a scene state from model and data
MJAPI void mjv_updateSceneState(const mjModel* m, mjData* d, const mjvOption* opt,
                                mjvSceneState* scnstate);

// move camera with mouse given a scene state; action is mjtMouse
MJAPI void mjv_moveCameraFromState(const mjvSceneState* scnstate, int action,
                                   mjtNum reldx, mjtNum reldy,
                                   const mjvScene* scn, mjvCamera* cam);

// move perturb object with mouse given a scene state; action is mjtMouse
MJAPI void mjv_movePerturbFromState(const mjvSceneState* scnstate, int action,
                                    mjtNum reldx, mjtNum reldy,
                                    const mjvScene* scn, mjvPerturb* pert);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_VIS_STATE_H_
