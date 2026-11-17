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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_CALLBACK_H_
#define MUJOCO_SRC_ENGINE_ENGINE_CALLBACK_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>

#ifdef __cplusplus
extern "C" {
#endif

// global callback function pointers
MJAPI extern mjfGeneric  mjcb_passive;
MJAPI extern mjfGeneric  mjcb_control;
MJAPI extern mjfConFilt  mjcb_contactfilter;
MJAPI extern mjfSensor   mjcb_sensor;
MJAPI extern mjfTime     mjcb_time;
MJAPI extern mjfAct      mjcb_act_bias;
MJAPI extern mjfAct      mjcb_act_gain;
MJAPI extern mjfAct      mjcb_act_dyn;

// invoke global callbacks (mjcb_* must be called through these wrappers)
void mj_callPassive(const mjModel* m, mjData* d);
void mj_callControl(const mjModel* m, mjData* d);
int mj_callContactFilter(const mjModel* m, mjData* d, int geom1, int geom2);
void mj_callSensor(const mjModel* m, mjData* d, int stage);
mjtNum mj_callTime(void);
mjtNum mj_callActBias(const mjModel* m, const mjData* d, int id);
mjtNum mj_callActGain(const mjModel* m, const mjData* d, int id);
mjtNum mj_callActDyn(const mjModel* m, const mjData* d, int id);


// reset callbacks to defaults
MJAPI void mj_resetCallbacks(void);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_CALLBACK_H_
