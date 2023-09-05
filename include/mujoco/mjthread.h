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

#ifndef MUJOCO_INCLUDE_MJTHREAD_H_
#define MUJOCO_INCLUDE_MJTHREAD_H_

// C API for MuJoCo threading
#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

#include <mujoco/mjexport.h>

// These types are implemented in C++, they're just used as opaque pointers in C
// to provide type safety for functions.
struct mjTask_ {
  char buffer[24];
};
typedef struct mjTask_ mjTask;

struct mjThreadPool_ {
  char buffer[6208];
};
typedef struct mjThreadPool_ mjThreadPool;

typedef void*(*mjStartRoutine_)(void*);
typedef mjStartRoutine_ mjStartRoutine;

#ifdef __cplusplus
}
#endif


#endif  // MUJOCO_INCLUDE_MJTHREAD_H_
