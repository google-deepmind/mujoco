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

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#include <immintrin.h>
static inline void CpuPause() { _mm_pause(); }
#elif defined(_MSC_VER) && (defined(_M_ARM) || defined(_M_ARM64))
#include <intrin.h>
static inline void CpuPause() { __yield(); }
#elif defined(__aarch64__) || defined(__arm__)
static inline void CpuPause() { __asm__ __volatile__("yield" ::: "memory"); }
#else
static inline void CpuPause() {}
#endif

// context for thread pool stored on mjData
class ThreadPoolContext {
 public:
  explicit ThreadPoolContext(int nthread) : slots_(nthread + 1), threads_(nthread) {
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

    // partition [0, ntask) into contiguous per-worker slices for cpu L2 cache
    // affinity
    int nthread = threads_.size();
    int nworker = slots_.size();
    int rem = ntask % nworker;
    int start = 0;
    for (int k = 0; k < nworker; k++) {
      int end = start + ntask / nworker + (k < rem ? 1 : 0);
      slots_[k].cursor.store(start, std::memory_order_relaxed);
      slots_[k].end.store(end, std::memory_order_relaxed);
      start = end;
    }

    ndone_.store(0, std::memory_order_relaxed);
    signal_.store(-signal_.load(std::memory_order_relaxed),
                 std::memory_order_release);
    signal_.notify_all();

    // process tasks on main thread
    RunTasks(0);

    // busy wait for rest of workers to finish
    while (ndone_.load(std::memory_order_acquire) < nthread) {
      // hint to the CPU that the main thread is in a spin-wait loop to avoid
      // branch misprediction penalties on exit, reduce memory bus contention,
      // and yield execution resources to sibling hyperthreads so that workers
      // finishing tasks on the same physical core are not blocked
      CpuPause();
    }
  }

  int ThreadCount() const { return threads_.size(); }

 private:
  // per-worker task slice, realigned to a new cache line to avoid false sharing
  struct alignas(64) WorkerSlot {
    std::atomic<int> cursor{0};
    std::atomic<int> end{0};
  };
  static_assert(sizeof(WorkerSlot) == 64, "WorkerSlot must be one cache line");

  // claim own home slice first, then steal around the worker ring
  void RunTasks(int threadId) {
    int nworker = slots_.size();
    for (int d = 0; d < nworker; d++) {
      WorkerSlot& slot = slots_[(threadId + d) % nworker];
      int end = slot.end.load(std::memory_order_relaxed);
      while (slot.cursor.load(std::memory_order_relaxed) < end) {
        int taskId = slot.cursor.fetch_add(1, std::memory_order_relaxed);
        if (taskId >= end) {
          break;
        }
        func_(model_, data_, arg_, threadId, taskId);
      }
    }
  }

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
      RunTasks(threadId);

      // let main thread know this worker is done
      ndone_.fetch_add(1, std::memory_order_release);
    }
  }

  // arguments for the current batch set by Dispatch
  const mjModel* model_;
  mjData* data_;
  mjTaskFunc func_;
  void* arg_;

  // per-worker task slices for home-range execution and ring stealing
  std::vector<WorkerSlot> slots_;

  // atomic counter for number of workers who completed their tasks
  alignas(64) std::atomic<int> ndone_{0};

  // alternating signal from -1, 1 to start / halt the worker threads,
  // set to 0 to force all workers to exit
  alignas(64) std::atomic<int> signal_{1};

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


// internal helper for mju_dispatch: dispatch ntask tasks to an active thread pool
void mju_dispatchPool(const mjModel* m, mjData* d, mjTaskFunc func, void* arg,
                      int ntask) {
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


// internal helper for mju_numThread: return total threads in an active thread pool
int mju_numThreadPool(uintptr_t threadpool) {
  ThreadPoolContext& ctx = *reinterpret_cast<ThreadPoolContext*>(threadpool);
  return ctx.ThreadCount() + 1;
}
