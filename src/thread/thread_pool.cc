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

#include <atomic>
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco/mjthread.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_util_errmem.h"
#include "thread/thread_queue.h"
#include "thread/thread_task.h"

namespace mujoco {
namespace {
constexpr size_t kThreadPoolQueueSize = 640;

struct WorkerThread {
  // Shutdown function passed to running threads to ensure clean shutdown.
  static void* ShutdownFunction(void* args) {
    return nullptr;
  }

  // Thread for the worker.
  std::unique_ptr<std::thread> thread_;

  // An mjTask for shutting down this worker.
  mjTask shutdown_task_ {
    &ShutdownFunction,
    nullptr,
    mjTASK_NEW
  };
};
}  // namespace

// Concrete C++ class definition for mjThreadPool.
// (The public mjThreadPool C struct is an opaque one.)
class ThreadPoolImpl : public mjThreadPool {
 public:
  ThreadPoolImpl(int num_worker) : mjThreadPool{num_worker} {
    // initialize worker threads
    for (int i = 0; i < num_worker; ++i) {
      WorkerThread worker{
        std::make_unique<std::thread>(ThreadPoolWorker, this)};
      workers_.push_back(std::move(worker));
    }
  }

  // start a task in the threadpool
  void Enqueue(mjTask* task) {
    if (mjUNLIKELY(GetAtomicTaskStatus(task).exchange(mjTASK_QUEUED) !=
                   mjTASK_NEW)) {
      mjERROR("task->status is not mjTASK_NEW");
    }
    lockless_queue_.push(task);
  }

  // shutdown the threadpool
  void Shutdown() {
    if (shutdown_) {
      return;
    }

    shutdown_ = true;
    std::vector<mjTask> shutdown_tasks(workers_.size());
    for (auto& worker : workers_) {
      Enqueue(&worker.shutdown_task_);
    }

    for (auto& worker : workers_) {
      worker.thread_->join();
    }
  }

  ~ThreadPoolImpl() { Shutdown(); }

 private:
  // method executed by running threads
  static void ThreadPoolWorker(ThreadPoolImpl* thread_pool) {
    while (!thread_pool->shutdown_) {
      auto task = static_cast<mjTask*>(thread_pool->lockless_queue_.pop());
      task->args = task->func(task->args);
      GetAtomicTaskStatus(task).store(mjTASK_COMPLETED);
    }
  }

  // indicates whether the thread pool is being shut down
  std::atomic<bool> shutdown_ = false;

  // OS threads that are running in this pool
  std::vector<WorkerThread> workers_;

  // queue of tasks to execute
  mujoco::LocklessQueue<void*, kThreadPoolQueueSize> lockless_queue_;
};

// create a thread pool
mjThreadPool* mju_threadPoolCreate(size_t number_of_threads) {
  return new ThreadPoolImpl(number_of_threads);
}

// start a task in the threadpool
void mju_threadPoolEnqueue(mjThreadPool* thread_pool, mjTask* task) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  thread_pool_impl->Enqueue(task);
}

// shutdown the threadpool and free the memory
void mju_threadPoolDestroy(mjThreadPool* thread_pool) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  thread_pool_impl->Shutdown();
  delete thread_pool_impl;
}
}  // namespace mujoco
