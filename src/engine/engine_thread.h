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

// return total number of threads in the pool (including main thread)
MJAPI int mju_numThread(const mjData* d);

// dispatch ntask tasks to the thread pool; passes arg into func along with thread_id and task_id
MJAPI void mju_dispatch(const mjModel* m, mjData* d, mjTaskFunc func, void* arg, int ntask);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_THREAD_H_
