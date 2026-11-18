// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_THREAD_H_
#define MUJOCO_SRC_ENGINE_ENGINE_THREAD_H_

#include <stdint.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>

#ifdef __cplusplus
extern "C" {
#endif

// dispatch function for mju_dispatch
typedef void (*mjTaskFunc)(const mjModel* m, mjData* d, void* arg, int thread_id, int task_id);

// create a thread pool with nthread worker threads.
MJAPI void mju_threadpool(mjData* d, int nthread);

// internal helper for mju_numThread: return total threads in an active thread pool
MJAPI int mju_numThreadPool(uintptr_t threadpool);

// return total number of threads in the pool (including main thread)
static inline int mju_numThread(const mjData* d) {
  return d->threadpool ? mju_numThreadPool(d->threadpool) : 1;
}

// internal helper for mju_dispatch: dispatch ntask tasks to an active thread pool
MJAPI void mju_dispatchPool(const mjModel* m, mjData* d, mjTaskFunc func, void* arg, int ntask);

// dispatch ntask tasks to the thread pool; passes arg into func along with thread_id and task_id
static inline void mju_dispatch(const mjModel* m, mjData* d, mjTaskFunc func, void* arg,
                                int ntask) {
  // no thread pool or trivial number of tasks: run on main thread
  if (!d->threadpool || ntask < 2) {
    for (int i = 0; i < ntask; i++) {
      func(m, d, arg, 0, i);
    }
    return;
  }
  mju_dispatchPool(m, d, func, arg, ntask);
}

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_THREAD_H_
