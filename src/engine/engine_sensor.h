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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------------- sensors ---------------------------------------------------------

// position-dependent sensors
MJAPI void mj_sensorPos(const mjModel* m, mjData* d);

// velocity-dependent sensors
MJAPI void mj_sensorVel(const mjModel* m, mjData* d);

// acceleration/force-dependent sensors
MJAPI void mj_sensorAcc(const mjModel* m, mjData* d);


//-------------------------------- energy ----------------------------------------------------------

// position-dependent energy (potential)
MJAPI void mj_energyPos(const mjModel* m, mjData* d);

// velocity-dependent energy (kinetic)
MJAPI void mj_energyVel(const mjModel* m, mjData* d);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SENSOR_H_
