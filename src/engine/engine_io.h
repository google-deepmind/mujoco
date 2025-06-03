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
#include <mujoco/mjxmacro.h>

#ifdef __cplusplus
#include <cstddef>
extern "C" {
#else
#include <stddef.h>
#endif

// internal hash map size factor (2 corresponds to a load factor of 0.5)
#define mjLOAD_MULTIPLE 2

//------------------------------- initialization ---------------------------------------------------

// Set default options for length range computation.
MJAPI void mj_defaultLROpt(mjLROpt* opt);

// set default solver parameters
MJAPI void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

// set options to default values
MJAPI void mj_defaultOption(mjOption* opt);

// set visual options to default values
MJAPI void mj_defaultVisual(mjVisual* vis);

// set statistics to default values; compute later in compiler
void mj_defaultStatistic(mjStatistic* stat);


//------------------------------- mjModel ----------------------------------------------------------

// allocate mjModel
void mj_makeModel(mjModel** dest,
    int nq, int nv, int nu, int na, int nbody, int nbvh, int nbvhstatic, int nbvhdynamic,
    int njnt, int ngeom, int nsite, int ncam, int nlight, int nflex, int nflexnode, int nflexvert,
    int nflexedge, int nflexelem, int nflexelemdata, int nflexelemedge, int nflexshelldata,
    int nflexevpair, int nflextexcoord, int nmesh, int nmeshvert, int nmeshnormal,
    int nmeshtexcoord, int nmeshface, int nmeshgraph,  int nmeshpoly, int nmeshpolyvert,
    int nmeshpolymap, int nskin, int nskinvert, int nskintexvert, int nskinface, int nskinbone,
    int nskinbonevert, int nhfield, int nhfielddata, int ntex, int ntexdata, int nmat, int npair,
    int nexclude, int neq, int ntendon, int nwrap, int nsensor, int nnumeric, int nnumericdata, int ntext,
    int ntextdata, int ntuple, int ntupledata, int nkey, int nmocap, int nplugin, int npluginattr,
    int nuser_body, int nuser_jnt, int nuser_geom, int nuser_site, int nuser_cam, int nuser_tendon,
    int nuser_actuator, int nuser_sensor, int nnames, int npaths);

// copy mjModel; allocate new if dest is NULL
MJAPI mjModel* mj_copyModel(mjModel* dest, const mjModel* src);

// copy mjModel, skip large arrays not required for abstract visualization
MJAPI void mjv_copyModel(mjModel* dest, const mjModel* src);

// save model to binary file
MJAPI void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz);

// load binary MJB
mjModel* mj_loadModelBuffer(const void* buffer, int buffer_sz);

// deallocate model
MJAPI void mj_deleteModel(mjModel* m);

// size of buffer needed to hold model
MJAPI int mj_sizeModel(const mjModel* m);

// validate reference fields in a model; return null if valid, error message otherwise
MJAPI const char* mj_validateReferences(const mjModel* m);


//------------------------------- mjData -----------------------------------------------------------

// allocate mjData corresponding to given model, initialize plugins, reset the state
// if the model buffer is unallocated the initial configuration will not be set
MJAPI mjData* mj_makeData(const mjModel* m);

// allocate mjData corresponding to given model, used internally
MJAPI void mj_makeRawData(mjData** dest, const mjModel* m);

// Copy mjData.
// m is only required to contain the size fields from MJMODEL_INTS.
MJAPI mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src);

// copy mjData, skip large arrays not required for abstract visualization
MJAPI mjData* mjv_copyData(mjData* dest, const mjModel* m, const mjData* src);

// set data to defaults
MJAPI void mj_resetData(const mjModel* m, mjData* d);

// set data to defaults, fill everything else with debug_value
MJAPI void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value);

// Reset data. If 0 <= key < nkey, set fields from specified keyframe.
MJAPI void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key);

// mjData arena allocate
MJAPI void* mj_arenaAllocByte(mjData* d, size_t bytes, size_t alignment);

// init plugins
MJAPI void mj_initPlugin(const mjModel* m, mjData* d);

#ifndef ADDRESS_SANITIZER

// mjData mark stack frame
MJAPI void mj_markStack(mjData* d);

// mjData free stack frame
MJAPI void mj_freeStack(mjData* d);

#else

void mj__markStack(mjData* d) __attribute__((noinline));
void mj__freeStack(mjData* d) __attribute__((noinline));

#endif  // ADDRESS_SANITIZER

// returns the number of bytes available on the stack
MJAPI size_t mj_stackBytesAvailable(mjData* d);

// allocate bytes on the stack
MJAPI void* mj_stackAllocByte(mjData* d, size_t bytes, size_t alignment);

// allocate bytes on the stack, with added caller information
MJAPI void* mj_stackAllocInfo(mjData* d, size_t bytes, size_t alignment,
                              const char* caller, int line);

// macro to allocate a stack array of given type, adds caller information
#define mjSTACKALLOC(d, num, type) \
(type*) mj_stackAllocInfo(d, (num) * sizeof(type), _Alignof(type), __func__, __LINE__)

// mjData stack allocate for array of mjtNums
MJAPI mjtNum* mj_stackAllocNum(mjData* d, size_t size);

// mjData stack allocate for array of ints
MJAPI int* mj_stackAllocInt(mjData* d, size_t size);

// deallocate data
MJAPI void mj_deleteData(mjData* d);

// clear arena pointers in mjData
static inline void mj_clearEfc(mjData* d) {
#define X(type, name, nr, nc) d->name = NULL;
  MJDATA_ARENA_POINTERS
#undef X
  d->nefc = 0;
  d->nisland = 0;
  d->contact = (mjContact*) d->arena;
}

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_IO_H_
