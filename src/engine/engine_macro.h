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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_MACRO_H_
#define MUJOCO_SRC_ENGINE_ENGINE_MACRO_H_

#include "engine/engine_callback.h"  // IWYU pragma: export

//-------------------------------- utility macros --------------------------------------------------

// mark and free stack
#define mjMARKSTACK int _mark = d->pstack;
#define mjFREESTACK d->pstack = _mark;

// check bitflag
#define mjDISABLED(x) (m->opt.disableflags & (x))
#define mjENABLED(x)  (m->opt.enableflags & (x))

// max and min macros
#define mjMAX(a, b)    (((a) > (b)) ? (a) : (b))
#define mjMIN(a, b)    (((a) < (b)) ? (a) : (b))


//-------------------------- timer macros ----------------------------------------------------------

#define TM_START mjtNum _tm = (mjcb_time ? mjcb_time() : 0);
#define TM_RESTART _tm = (mjcb_time ? mjcb_time() : 0);
#define TM_END(i) {d->timer[i].duration += ((mjcb_time ? mjcb_time() : 0) - _tm); d->timer[i].number++;}
#define TM_START1 mjtNum _tm1 = (mjcb_time ? mjcb_time() : 0);
#define TM_END1(i) {d->timer[i].duration += ((mjcb_time ? mjcb_time() : 0) - _tm1); d->timer[i].number++;}

#endif  // MUJOCO_SRC_ENGINE_ENGINE_MACRO_H_
