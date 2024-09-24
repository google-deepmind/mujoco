// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SORT_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SORT_H_

#if !defined(__cplusplus)
#include <stddef.h>
#include <stdlib.h>

// sorting functions using q_sort_s/r
#ifdef _WIN32
#define mjQUICKSORT(buf, elnum, elsz, func, context) \
  qsort_s(buf, elnum, elsz, func, context)
#define quicksortfunc(name, context, el1, el2) \
  static int name(void* context, const void* el1, const void* el2)
#else  // assumes POSIX
#ifdef __APPLE__
#define mjQUICKSORT(buf, elnum, elsz, func, context) \
  qsort_r(buf, elnum, elsz, context, func)
#define quicksortfunc(name, context, el1, el2) \
  static int name(void* context, const void* el1, const void* el2)
#else  // non-Apple
#define mjQUICKSORT(buf, elnum, elsz, func, context) \
  qsort_r(buf, elnum, elsz, func, context)
#define quicksortfunc(name, context, el1, el2) \
  static int name(const void* el1, const void* el2, void* context)
#endif
#endif
#else
#include <algorithm>
#include <cstddef>
#include <cstdlib>

// sorting function using std::sort
template <typename T>
void mjQUICKSORT(T* buf, size_t elnum, size_t elsz,
                 int (*compare)(const void* a, const void* b, void* c),
                 void* context) {
  std::sort(buf, buf + elnum, [compare, context](const T& a, const T& b) {
    return compare(&a, &b, context) < 0;
  });
}

#define quicksortfunc(name, context, el1, el2) \
  static int name(const void* el1, const void* el2, void* context)
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SORT_H_
