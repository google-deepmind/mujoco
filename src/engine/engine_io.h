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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_IO_H_
#define MUJOCO_SRC_ENGINE_ENGINE_IO_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------- initialization ---------------------------------------------------

// Set default options for length range computation.
MJAPI void mj_defaultLROpt(mjLROpt* opt);

// set default solver paramters
MJAPI void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

// set options to default values
MJAPI void mj_defaultOption(mjOption* opt);

// set visual options to default values
MJAPI void mj_defaultVisual(mjVisual* vis);

// set statistics to default values; compute later in compiler
void mj_defaultStatistic(mjStatistic* stat);


//------------------------------- mjModel ----------------------------------------------------------

// allocate mjModel
mjModel* mj_makeModel(int nq, int nv, int nu, int na, int nbody, int njnt,
                      int ngeom, int nsite, int ncam, int nlight,
                      int nmesh, int nmeshvert, int nmeshtexvert, int nmeshface, int nmeshgraph,
                      int nskin, int nskinvert, int nskintexvert, int nskinface,
                      int nskinbone, int nskinbonevert, int nhfield, int nhfielddata,
                      int ntex, int ntexdata, int nmat, int npair, int nexclude,
                      int neq, int ntendon, int nwrap, int nsensor,
                      int nnumeric, int nnumericdata, int ntext, int ntextdata,
                      int ntuple, int ntupledata, int nkey, int nmocap,
                      int nuser_body, int nuser_jnt, int nuser_geom, int nuser_site, int nuser_cam,
                      int nuser_tendon, int nuser_actuator, int nuser_sensor, int nnames);

// copy mjModel; allocate new if dest is NULL
MJAPI mjModel* mj_copyModel(mjModel* dest, const mjModel* src);

// save model to binary file
MJAPI void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz);

// load model from binary MJB file
//  if vfs is not NULL, look up file in vfs before reading from disk
MJAPI mjModel* mj_loadModel(const char* filename, const mjVFS* vfs);

// de-allocate model
MJAPI void mj_deleteModel(mjModel* m);

// size of buffer needed to hold model
MJAPI int mj_sizeModel(const mjModel* m);

// validate reference fields in a model; return null if valid, error message otherwise
MJAPI const char* mj_validateReferences(const mjModel* m);


//------------------------------- mjData -----------------------------------------------------------

// Allocate mjData correponding to given model.
// If the model buffer is unallocated the initial configuration will not be set.
MJAPI mjData* mj_makeData(const mjModel* m);

// Copy mjData.
// m is only required to contain the size fields from MJMODEL_INTS.
MJAPI mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src);

// set data to defaults
MJAPI void mj_resetData(const mjModel* m, mjData* d);

// set data to defaults, fill everything else with debug_value
MJAPI void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value);

// reset data, set fields from specified keyframe
MJAPI void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key);

// mjData stack allocate
MJAPI mjtNum* mj_stackAlloc(mjData* d, int size);

// de-allocate data
MJAPI void mj_deleteData(mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_IO_H_
