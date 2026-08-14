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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_ARRAY_SAFETY_H_
#define MUJOCO_SRC_ENGINE_ENGINE_ARRAY_SAFETY_H_

#ifdef __cplusplus
#error This file should not be used from C++ code.
#endif

#include <stdio.h>
#include <string.h>

// Evaluates to sizeof(arr) if arr is a char array, and emits a compiler error
// otherwise. In particular, emits a compiler error if arr is a char*.
#define mjSIZEOFARRAY(arr) _Generic(&(arr), char(*)[sizeof(arr)]: sizeof(arr))

#define mjSNPRINTF(dest, ...) snprintf(dest, mjSIZEOFARRAY(dest), __VA_ARGS__)

#define mjSTRNCAT(dest, src) strncat(dest, src, mjSIZEOFARRAY(dest) - strlen(dest) - 1)

#define mjSTRNCPY(dest, src) mju_strncpy(dest, src, mjSIZEOFARRAY(dest))

#endif  // MUJOCO_SRC_ENGINE_ENGINE_ARRAY_SAFETY_H_
