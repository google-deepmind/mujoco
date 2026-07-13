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

#include "engine/engine_thread.h"

#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include "engine/engine_memory.h"

// context for thread pool stored on mjData
class ThreadPoolContext {
 public:
  explicit ThreadPoolContext(int nthread) : threads_(nthread) {
    for (int i = 0; i < nthread; i++) {
      threads_[i] = std::thread(&ThreadPoolContext::Worker, this, i + 1);
    }
  }

  // non-copyable, non-movable
  ThreadPoolContext(const ThreadPoolContext&) = delete;
  ThreadPoolContext& operator=(const ThreadPoolContext&) = delete;

  ~ThreadPoolContext() {
    signal_.store(0, std::memory_order_release);
    signal_.notify_all();
    for (auto& thread : threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }

  // dispatch tasks to the thread pool and work on them on the main thread
  void Dispatch(const mjModel* model, mjData* data, mjTaskFunc func, void* arg,
                int ntask) {
    func_ = func;
    model_ = model;
    data_ = data;
    arg_ = arg;
    ntask_ = ntask;
    next_.store(0, std::memory_order_relaxed);
    ndone_.store(0, std::memory_order_relaxed);
    signal_.store(-signal_.load(std::memory_order_relaxed),
                 std::memory_order_release);
    signal_.notify_all();

    // process tasks on main thread
    while (true) {
      int taskId = next_.fetch_add(1, std::memory_order_relaxed);
      if (taskId >= ntask_) {
        break;
      }
      func_(model_, data_, arg_, 0, taskId);
    }

    // busy wait for rest of workers to finish
    int nthread = threads_.size();
    while (ndone_.load(std::memory_order_acquire) < nthread) {
    }
  }

  int ThreadCount() const { return threads_.size(); }

 private:
  // worker loop for each worker thread
  void Worker(int threadId) {
    int status = 1;

    // main loop waiting for next batch of tasks
    while (true) {
      // wait until signal atomic is notified and sign flips
      signal_.wait(status, std::memory_order_acquire);

      // if signal was set to zero, halt
      status = signal_.load(std::memory_order_acquire);
      if (status == 0) {
        return;
      }

      // subloop to process tasks for the current batch
      while (true) {
        int taskId = next_.fetch_add(1, std::memory_order_relaxed);
        if (taskId >= ntask_) {
          break;
        }
        func_(model_, data_, arg_, threadId, taskId);
      }

      // let main thread know this worker is done
      ndone_.fetch_add(1, std::memory_order_release);
    }
  }

  // arguments for the current batch set by Dispatch
  const mjModel* model_;
  mjData* data_;
  mjTaskFunc func_;
  void* arg_;
  int ntask_;  // total number of tasks for workers to do

  // atomic for each worker to grab the next task
  std::atomic<int> next_{0};

  // atomic counter for number of workers who completed their tasks
  alignas(64) std::atomic<int> ndone_{0};

  // alternating signal from -1, 1 to start / halt the worker threads,
  // set to 0 to force all workers to exit
  std::atomic<int> signal_{1};

  std::vector<std::thread> threads_;
};


// create a thread pool with nthread threads
void mju_threadpool(mjData* d, int nthread) {
  if (d->threadpool) {
    ThreadPoolContext* ctx =
        reinterpret_cast<ThreadPoolContext*>(d->threadpool);
    // same size, nothing to do
    if (nthread == ctx->ThreadCount()) {
        return;
    }
    delete ctx;
    d->threadpool = 0;  // null out in case nthread == 0
  }

  if (nthread >= 1) {
    d->threadpool = reinterpret_cast<uintptr_t>(new ThreadPoolContext(nthread));
  }
}


// dispatch ntask tasks to the thread pool; passes arg into func along with
// thread_id and task_id
void mju_dispatch(const mjModel* m, mjData* d, mjTaskFunc func, void* arg,
                  int ntask) {
  // no thread pool or trivial number of tasks: run on main thread
  if (!d->threadpool || ntask < 2) {
    for (int i = 0; i < ntask; i++) {
      func(m, d, arg, 0, i);
    }
    return;
  }

  ThreadPoolContext& ctx = *reinterpret_cast<ThreadPoolContext*>(d->threadpool);

  // lock mjData and mark stack frame, memory will be freed after thread completion
  if (!d->threadlock) {
    mj_markStack(d);
    d->threadlock = true;
  }

  ctx.Dispatch(m, d, func, arg, ntask);

  if (d->threadlock) {
    // update max usage statistics
    d->maxuse_stack = mjMAX(d->maxuse_stack, d->pstack);
    d->maxuse_arena = mjMAX(d->maxuse_arena, d->pstack + d->parena);

    // unlock mjData and free stack used during worker execution
    d->threadlock = false;
    mj_freeStack(d);
  }
}


// return total number of threads in the pool (including main thread)
int mju_numThread(const mjData* d) {
  ThreadPoolContext* ctx = reinterpret_cast<ThreadPoolContext*>(d->threadpool);
  return ctx ? ctx->ThreadCount() + 1 : 1;
}
