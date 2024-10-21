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

#include <stdint.h>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include <mujoco/mjthread.h>
#include <mujoco/mujoco.h>
#include "engine/engine_crossplatform.h"
#include "engine/engine_util_errmem.h"
#include "thread/thread_queue.h"
#include "thread/thread_task.h"

namespace mujoco {
namespace {
constexpr size_t kThreadPoolQueueSize = 640;

// Each thread being run will be assigned a worker_id.
//   0: main thread
//   1->n: workers
thread_local size_t worker_id = 0;

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
    for (int i = 0; i < std::min(num_worker, mjMAXTHREAD); ++i) {
      WorkerThread worker{
        std::make_unique<std::thread>(ThreadPoolWorker, this, i)};
      workers_.push_back(std::move(worker));
    }
  }

  size_t NumberOfThreads() {
    return workers_.size();
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

  // registers a worker ID for a given thread
  void RegisterWorker(const size_t input_worker_id) {
    worker_id = input_worker_id;
  }

  // gets the worker id of the current thread
  size_t GetWorkerId() {
    return worker_id;
  }

  void LockAlloc() {
    alloc_mutex_.lock();
  }

  void UnlockAlloc() {
    alloc_mutex_.unlock();
  }

  bool IsThreadPoolBound() {
    return thread_pool_bound_;
  }

  void BindThreadPool() {
    thread_pool_bound_ = true;
  }

  ~ThreadPoolImpl() { Shutdown(); }

 private:
  // method executed by running threads
  static void ThreadPoolWorker(
      ThreadPoolImpl* thread_pool, const size_t thread_index) {
    worker_id = thread_index + 1;
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

  // Mutex to protect arena allocations.
  std::mutex alloc_mutex_;

  // Whether or not a ThreadPool was bound using mju_bindThreadPool.
  bool thread_pool_bound_ = false;
};

// create a thread pool
mjThreadPool* mju_threadPoolCreate(size_t number_of_threads) {
  return reinterpret_cast<mjThreadPool*>(new ThreadPoolImpl(number_of_threads));
}

// gets the number of shards the stack is currently broken into
static size_t GetNumberOfShards(mjData* d) {
  if (!d->threadpool) {
    return 1;
  }
  return mju_threadPoolNumberOfThreads((mjThreadPool*)d->threadpool) + 1;
}

// returns the stack information for the specified thread's shard
mjStackInfo* mju_getStackInfoForThread(mjData* d, size_t thread_id) {
  auto thread_pool = (ThreadPoolImpl*)d->threadpool;
  if (!thread_pool || !thread_pool->IsThreadPoolBound()) {
    mju_error("Thread Pool not bound, use mju_bindThreadPool to add an mjThreadPool to mjData");
  }

  // number of threads running in the threadpool plus the main thread
  size_t number_of_shards = GetNumberOfShards(d);

  // size of entire arena/stack in bytes
  size_t total_arena_size_bytes = d->narena;

  // set the shard cursor to the end of the arena
  uintptr_t end_of_arena_ptr = (uintptr_t)d->arena + total_arena_size_bytes;

  // each thread including the main one will get an equal shard of the stack
  size_t bytes_per_shard = total_arena_size_bytes / (2 * (number_of_shards));

  // ensure the shard is larger than the cache line
  size_t misalignment = bytes_per_shard % mju_getDestructiveInterferenceSize();

  if (misalignment != 0) {
    bytes_per_shard += mju_getDestructiveInterferenceSize() - misalignment;
  }

  if (bytes_per_shard * number_of_shards > total_arena_size_bytes) {
    mju_error("Arena is not large enough for %zu shards", number_of_shards);
  }

  uintptr_t result = (end_of_arena_ptr - (thread_id + 1) * bytes_per_shard);

  // align the end of the shard to be mjStackInfo.
  misalignment = result % alignof(mjStackInfo);
  result -= misalignment;
#ifdef ADDRESS_SANITIZER
  // Ensure StackInfo is always accessible
  ASAN_UNPOISON_MEMORY_REGION((void*)result, sizeof(mjStackInfo));
#endif
  return (mjStackInfo*) result;
}

// shards the stack for each thread
static void  ConfigureMultiThreadedStack(mjData* d) {
  if (!d->threadpool) {
    mju_error("No thread pool specified for multithreaded operation");
  }

  size_t number_of_shards = GetNumberOfShards(d);

  // current top of the stack
  uintptr_t current_limit = (uintptr_t)d->arena + d->narena - d->pstack;

  // set the shard cursor to the end of the arena
  uintptr_t begin_shard_cursor_ptr = (uintptr_t)d->arena + d->narena;

  for (size_t shard_index = 0; shard_index < number_of_shards; ++shard_index) {
    mjStackInfo* end_shard_cursor_ptr = mju_getStackInfoForThread(d, shard_index);
#ifdef ADDRESS_SANITIZER
  // unpoison stack info
  ASAN_UNPOISON_MEMORY_REGION((void*)end_shard_cursor_ptr, sizeof(mjStackInfo));
#endif
    // handle the main thread's stack which may already have data in it
    if (shard_index == 0) {
      // abort if the current stack is already larger than the portion of the stack
      // that would be reserved for the main thread
      if ((uintptr_t)end_shard_cursor_ptr > current_limit) {
        mju_error("mj_bindThreadPool: sharding stack - existing stack larger than shard size: current_size = %zu, "
                  "max_size = %zu", current_limit, (uintptr_t) end_shard_cursor_ptr);
      }
      end_shard_cursor_ptr->top = current_limit;
      end_shard_cursor_ptr->stack_base = d->pbase;
    } else {
      // all other stacks are empty because threads have not been used yet
      end_shard_cursor_ptr->top = begin_shard_cursor_ptr;
      end_shard_cursor_ptr->stack_base = 0;
    }

    end_shard_cursor_ptr->bottom = begin_shard_cursor_ptr;
    end_shard_cursor_ptr->limit = (uintptr_t)end_shard_cursor_ptr + sizeof(mjStackInfo);
    begin_shard_cursor_ptr = (uintptr_t)end_shard_cursor_ptr - 1;
  }
}

// adds a thread pool to mjData and configures it for multi-threaded use.
void mju_bindThreadPool(mjData* d, void* thread_pool) {
  if (d->threadpool) {
    mju_error("Thread Pool already bound to mjData");
  }

  d->threadpool = (uintptr_t) thread_pool;
  ((ThreadPoolImpl*)thread_pool)->BindThreadPool();
  ConfigureMultiThreadedStack(d);
}

// gets the number of running threads in the thread pool.
size_t mju_threadPoolNumberOfThreads(mjThreadPool* thread_pool) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  return thread_pool_impl->NumberOfThreads();
}

size_t mju_threadPoolCurrentWorkerId(mjThreadPool* thread_pool) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  return thread_pool_impl->GetWorkerId();
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

// locks the allocation mutex to protect Stack and Arena allocations
void mju_threadPoolLockAllocMutex(mjThreadPool* thread_pool) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  thread_pool_impl->LockAlloc();
}

// unlocks the allocation mutex to protect Stack and Arena allocations
void mju_threadPoolUnlockAllocMutex(mjThreadPool* thread_pool) {
  auto thread_pool_impl = static_cast<ThreadPoolImpl*>(thread_pool);
  thread_pool_impl->UnlockAlloc();
}

// Get the destructive interference size for the architecture.
size_t mju_getDestructiveInterferenceSize(void) {
  // return std::hardware_destructive_interference_size;
  return 128;
}

}  // namespace mujoco
