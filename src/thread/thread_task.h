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

#ifndef MUJOCO_SRC_THREAD_THREAD_TASK_H_
#define MUJOCO_SRC_THREAD_THREAD_TASK_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjthread.h>

#ifdef __cplusplus
#include <atomic>
#include <new>
#include <type_traits>
namespace mujoco {
extern "C" {
#endif

// Initialize an mjTask.
MJAPI void mju_defaultTask(mjTask* task);

// Wait for a task to complete.
MJAPI void mju_taskJoin(mjTask* task);

#ifdef __cplusplus
}  // extern "C"

using TaskStatus = std::remove_volatile_t<decltype(mjTask::status)>;
inline std::atomic<TaskStatus>& GetAtomicTaskStatus(mjTask* task) {
  static_assert(sizeof(std::atomic<TaskStatus>) == sizeof(TaskStatus));
  static_assert(alignof(std::atomic<TaskStatus>) == alignof(TaskStatus));
  static_assert(std::atomic<TaskStatus>::is_always_lock_free);
  return *std::launder(reinterpret_cast<std::atomic<TaskStatus>*>(
      const_cast<TaskStatus*>(&task->status)));
}
}  // namespace mujoco
#endif  // __cplusplus

#endif  // MUJOCO_SRC_THREAD_THREAD_TASK_H_
