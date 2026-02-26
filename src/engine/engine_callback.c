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

//------------------------- global callback pointers -----------------------------------------------

mjfGeneric mjcb_passive  = 0;
mjfGeneric mjcb_control  = 0;
mjfConFilt mjcb_contactfilter = 0;
mjfSensor mjcb_sensor    = 0;
mjfTime mjcb_time        = 0;
mjfAct mjcb_act_bias     = 0;
mjfAct mjcb_act_gain     = 0;
mjfAct mjcb_act_dyn      = 0;



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
// Getter/setter functions for callbacks (Windows DLL export compatibility)
mjfGeneric mju_getCallbackPassive(void) {
  return mjcb_passive;
}

void mju_setCallbackPassive(mjfGeneric handler) {
  mjcb_passive = handler;
}

mjfGeneric mju_getCallbackControl(void) {
  return mjcb_control;
}

void mju_setCallbackControl(mjfGeneric handler) {
  mjcb_control = handler;
}

mjfConFilt mju_getCallbackContactFilter(void) {
  return mjcb_contactfilter;
}

void mju_setCallbackContactFilter(mjfConFilt handler) {
  mjcb_contactfilter = handler;
}

mjfSensor mju_getCallbackSensor(void) {
  return mjcb_sensor;
}

void mju_setCallbackSensor(mjfSensor handler) {
  mjcb_sensor = handler;
}

mjfTime mju_getCallbackTime(void) {
  return mjcb_time;
}

void mju_setCallbackTime(mjfTime handler) {
  mjcb_time = handler;
}

mjfAct mju_getCallbackActDyn(void) {
  return mjcb_act_dyn;
}

void mju_setCallbackActDyn(mjfAct handler) {
  mjcb_act_dyn = handler;
}

mjfAct mju_getCallbackActGain(void) {
  return mjcb_act_gain;
}

void mju_setCallbackActGain(mjfAct handler) {
  mjcb_act_gain = handler;
}

mjfAct mju_getCallbackActBias(void) {
  return mjcb_act_bias;
}

void mju_setCallbackActBias(mjfAct handler) {
  mjcb_act_bias = handler;
}