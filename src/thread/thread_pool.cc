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

#include "thread/thread_pool.h"

#include <cstddef>

#include <mujoco/mjthread.h>
#include <mujoco/mujoco.h>
#include "thread/task.h"

static constexpr size_t kMaxThreads = 128;

// create a thread pool
mjThreadPool* mju_threadPoolCreate(size_t number_of_threads) {
  mujoco::ThreadPool<kMaxThreads>* thread_pool =
      new mujoco::ThreadPool<kMaxThreads>(number_of_threads);
  return static_cast<mjThreadPool*>(static_cast<void*>(thread_pool));
}

// start a task in the threadpool
void mju_threadPoolEnqueue(
    mjThreadPool* thread_pool, mjTask* task, mjStartRoutine start_routine,
    void* args) {
  mujoco::ThreadPool<kMaxThreads>* thread_pool_ptr =
      static_cast<mujoco::ThreadPool<kMaxThreads>*>(
          static_cast<void*>(thread_pool));
  thread_pool_ptr->Enqueue(
      static_cast<mujoco::Task*>(static_cast<void*>(task)), start_routine,
      args);
}

// shutdown the threadpool and free the memory
void mju_threadPoolDestroy(mjThreadPool* thread_pool) {
  mujoco::ThreadPool<kMaxThreads>* thread_pool_ptr =
      static_cast<mujoco::ThreadPool<kMaxThreads>*>(
          static_cast<void*>(thread_pool));
  thread_pool_ptr->Shutdown();
  delete thread_pool_ptr;
}

