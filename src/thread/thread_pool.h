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

#ifndef MUJOCO_SRC_THREAD_THREAD_POOL_H_
#define MUJOCO_SRC_THREAD_THREAD_POOL_H_

#ifdef __cplusplus

#include <atomic>
#include <cstddef>
#include <functional>
#include <thread>

#include "thread/lockless_queue.h"
#include "thread/task.h"

namespace mujoco {

static constexpr size_t kThreadPoolQueueSize = 640;

template <size_t max_number_of_threads>
class ThreadPool {
 public:
  ThreadPool(size_t number_of_threads)
      : number_of_threads_(number_of_threads) {
    for (int i = 0; i < number_of_threads_; ++i) {
      threads_[i] = std::thread(ThreadPoolWorker, static_cast<void*>(this));
    }
  }

  // start a task in the threadpool
  void Enqueue(
      Task* task, std::function<void*(void*)> start_routine, void* args) {
    Task::Initialize(task, start_routine, args);
    lockless_queue_.push(static_cast<void*>(task));
  }

  // shutdown the threadpool
  void Shutdown() {
    if (shutdown_) {
      return;
    }

    shutdown_ = true;
    Task shutdown_tasks[max_number_of_threads];
    for (int i = 0; i < number_of_threads_; ++i) {
      Enqueue(&shutdown_tasks[i], ShutdownFunction, nullptr);
    }

    for (int i = 0; i < number_of_threads_; ++i) {
      threads_[i].join();
    }
  }

  ~ThreadPool() { Shutdown(); }

 private:
  // method executed by running threads
  static void ThreadPoolWorker(void* arg) {
    ThreadPool<max_number_of_threads>* thread_pool =
        static_cast<ThreadPool<max_number_of_threads>*>(arg);
    while (!thread_pool->shutdown_) {
      Task* task = static_cast<Task*>(thread_pool->lockless_queue_.pop());
      task->Execute();
    }
  }

  // shutdown function passed to running threads to ensure cleans shutdown
  static void* ShutdownFunction(void* args) {
    return NULL;
  }

  // is the thread pool is being shut down
  std::atomic<bool> shutdown_ = false;

  // actual number of running threads in the threadpool
  const size_t number_of_threads_;

  // OS threads that are running in this pool
  std::thread threads_[max_number_of_threads];

  // queue of tasks to execute
  LocklessQueue<void*, kThreadPoolQueueSize> lockless_queue_;
};

}  // namespace mujoco

#endif  // __cplusplus

#endif  // MUJOCO_SRC_THREAD_THREAD_POOL_H_
