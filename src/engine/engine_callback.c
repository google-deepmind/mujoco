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

#include "engine/engine_callback.h"

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include "engine/engine_crossplatform.h"

//------------------------- global callback pointers -----------------------------------------------

MJAPI mjfGeneric mjcb_passive       = 0;
MJAPI mjfGeneric mjcb_control       = 0;
MJAPI mjfConFilt mjcb_contactfilter = 0;
MJAPI mjfSensor  mjcb_sensor        = 0;
MJAPI mjfTime    mjcb_time          = 0;
MJAPI mjfAct     mjcb_act_bias      = 0;
MJAPI mjfAct     mjcb_act_gain      = 0;
MJAPI mjfAct     mjcb_act_dyn       = 0;


// invoke global callbacks
mjNO_SANITIZE_CFI void mj_callPassive(const mjModel* m, mjData* d) {
  mjcb_passive(m, d);
}


mjNO_SANITIZE_CFI void mj_callControl(const mjModel* m, mjData* d) {
  mjcb_control(m, d);
}


mjNO_SANITIZE_CFI int mj_callContactFilter(const mjModel* m, mjData* d, int geom1, int geom2) {
  return mjcb_contactfilter(m, d, geom1, geom2);
}


mjNO_SANITIZE_CFI void mj_callSensor(const mjModel* m, mjData* d, int stage) {
  mjcb_sensor(m, d, stage);
}


mjNO_SANITIZE_CFI mjtNum mj_callTime(void) {
  return mjcb_time();
}


mjNO_SANITIZE_CFI mjtNum mj_callActBias(const mjModel* m, const mjData* d, int id) {
  return mjcb_act_bias(m, d, id);
}


mjNO_SANITIZE_CFI mjtNum mj_callActGain(const mjModel* m, const mjData* d, int id) {
  return mjcb_act_gain(m, d, id);
}


mjNO_SANITIZE_CFI mjtNum mj_callActDyn(const mjModel* m, const mjData* d, int id) {
  return mjcb_act_dyn(m, d, id);
}


// reset callbacks to defaults
void mj_resetCallbacks(void) {
  mjcb_passive  = 0;
  mjcb_control  = 0;
  mjcb_contactfilter = 0;
  mjcb_sensor   = 0;
  mjcb_time     = 0;
  mjcb_act_bias = 0;
  mjcb_act_gain = 0;
  mjcb_act_dyn  = 0;
}
