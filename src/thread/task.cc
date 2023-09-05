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

#include "thread/task.h"

#include <mujoco/mjthread.h>
#include <mujoco/mujoco.h>

// waits for a task to complete
void mju_taskJoin(mjTask* task) {
  mujoco::Task* task_ptr = static_cast<mujoco::Task*>(static_cast<void*>(task));
  task_ptr->Join();
}
