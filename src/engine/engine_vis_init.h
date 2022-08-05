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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_VIS_INIT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_VIS_INIT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>

#ifdef __cplusplus
extern "C" {
#endif

// strings
MJAPI extern const char* mjLABELSTRING[mjNLABEL];
MJAPI extern const char* mjFRAMESTRING[mjNFRAME];
MJAPI extern const char* mjVISSTRING[mjNVISFLAG][3];
MJAPI extern const char* mjRNDSTRING[mjNRNDFLAG][3];


// set default scene
MJAPI void mjv_defaultScene(mjvScene* scn);

// allocate and init abstract scene
MJAPI void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom);

// free abstract scene
MJAPI void mjv_freeScene(mjvScene* scn);

// set default visualization options
MJAPI void mjv_defaultOption(mjvOption* vopt);

// set default free camera
MJAPI void mjv_defaultFreeCamera(const mjModel* m, mjvCamera* cam);

// set default camera
MJAPI void mjv_defaultCamera(mjvCamera* cam);

// set default perturbation
MJAPI void mjv_defaultPerturb(mjvPerturb* pert);

// set default figure
MJAPI void mjv_defaultFigure(mjvFigure* fig);

// compute rbound for mjvGeom
float mjv_rbound(const mjvGeom* geom);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_VIS_INIT_H_
