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
// IWYU pragma: private, include "third_party/mujoco/include/mujoco.h"
// IWYU pragma: friend "third_party/(py/)?mujoco/.*"

#ifndef MUJOCO_SRC_THREAD_TASK_H_
#define MUJOCO_SRC_THREAD_TASK_H_

#ifdef __cplusplus

#include <atomic>
#include <functional>
#include <thread>

namespace mujoco {

class Task {
 public:
  enum Status {
    QUEUED,
    COMPLETE,
  };

  static void Initialize(
      Task* task,
      std::function<void*(void*)> start_routine,
      void* args) {
    // instantiate a task at the pointer passed in
    new(task) Task();
    task->start_routine_ = start_routine;
    task->args_ = args;
    task->status_ = Status::QUEUED;
  }

  void Execute() {
    args_ = start_routine_(args_);
    status_ = Status::COMPLETE;
  }

  void Join() {
    while (status_ != Status::COMPLETE) {
      std::this_thread::yield();
    }
  }

 private:
  std::function<void*(void*)> start_routine_;

  void* args_;

  std::atomic<Status> status_ = Status::QUEUED;
};

}  // namespace mujoco

#endif  // __cplusplus

#endif  // MUJOCO_SRC_THREAD_TASK_H_
