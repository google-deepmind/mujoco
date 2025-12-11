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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

// strings
MJAPI extern const char* mjDISABLESTRING[mjNDISABLE];
MJAPI extern const char* mjENABLESTRING[mjNENABLE];
MJAPI extern const char* mjTIMERSTRING[mjNTIMER];

// arrays
MJAPI extern const int mjCONDATA_SIZE[mjNCONDATA];  // TODO(tassa): expose in public header?


//-------------------------- get/set state ---------------------------------------------------------

// return size of state signature
MJAPI int mj_stateSize(const mjModel* m, int sig);

// get state
MJAPI void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, int sig);

// extract a sub-state from a state
MJAPI void mj_extractState(const mjModel* m, const mjtNum* src, int srcsig,
                           mjtNum* dst, int dstsig);

// set state
MJAPI void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, int sig);

// copy state from src to dst
MJAPI void mj_copyState(const mjModel* m, const mjData* src, mjData* dst, int sig);

// copy current state to the k-th model keyframe
MJAPI void mj_setKeyframe(mjModel* m, const mjData* d, int k);

//-------------------------- inertia functions -----------------------------------------------------

// convert sparse inertia matrix M into full matrix
MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

// multiply vector by inertia matrix
MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// multiply vector by (inertia matrix)^(1/2)
MJAPI void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

// add inertia matrix to destination matrix (lower triangle only)
//  destination can be sparse or dense when all int* are NULL
MJAPI void mj_addM(const mjModel* m, mjData* d, mjtNum* dst,
                   int* rownnz, int* rowadr, int* colind);


//-------------------------- perturbations ---------------------------------------------------------

// apply Cartesian force and torque
MJAPI void mj_applyFT(const mjModel* m, mjData* d,
                      const mjtNum force[3], const mjtNum torque[3],
                      const mjtNum point[3], int body, mjtNum* qfrc_target);

// accumulate xfrc_applied in qfrc
void mj_xfrcAccumulate(const mjModel* m, mjData* d, mjtNum* qfrc);


//-------------------------- miscellaneous ---------------------------------------------------------

// returns the smallest distance between two geoms
MJAPI mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2,
                             mjtNum distmax, mjtNum fromto[6]);

// compute velocity by finite-differencing two positions
MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
                               const mjtNum* qpos1, const mjtNum* qpos2);

// integrate qpos with given qvel for given body indices
MJAPI void mj_integratePosInd(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt,
                              const int* index, int nbody);

// integrate position with given velocity
MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

// normalize all quaternions in qpos-type vector
MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

// return 1 if actuator i is disabled, 0 otherwise
MJAPI int mj_actuatorDisabled(const mjModel* m, int i);

// sum all body masses
MJAPI mjtNum mj_getTotalmass(const mjModel* m);

// scale body masses and inertias to achieve specified total mass
MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass);

// version number
MJAPI int mj_version(void);

// current version of MuJoCo as a null-terminated string
MJAPI const char* mj_versionString(void);

// return total size of data fields in a contact sensor bitfield specification
MJAPI int mju_condataSize(int dataSpec);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SUPPORT_H_
