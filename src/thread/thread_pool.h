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

#ifndef MUJOCO_SRC_THREAD_THREAD_POOL_H_
#define MUJOCO_SRC_THREAD_THREAD_POOL_H_

#include <stddef.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjthread.h>
#include <mujoco/mujoco.h>

#ifdef __cplusplus
namespace mujoco {
extern "C" {
#endif

// MultiThreaded Stack will be an approximately 50/50 split of the entire buffer, with a little
// wiggle for alignment and caching concerns. The basic layout is to reuse the existing single
// threaded markers, and then create shards for each thread to use as its stack.
// Not to scale.
// |----------|-----------|-----------|-----------|-----------|----------|-----------|-----------|
// |Used Arena|Free Arena |Shard1     |Shard1     |Shard1     |Shard0    |Shard0     |Shard0     |
// |%%%%%%%%%%|           |StackInfo  |Free Stack |Used Stack |StackInfo |Free Stack |Used Stack |
// |%%%%%%%%%%|           |           |           |%%%%%%%%%%%|          |           |%%%%%%%%%%%|
// |%%%%%%%%%%|           |           |           |%%%%%%%%%%%|          |           |%%%%%%%%%%%|
// |----------|-----------|-----------|-----------|-----------|----------|-----------|-----------|
// d->arena   d->parena   d->pstack   shard1->stack_info      shard1->bottom_of_stack            shard1->bottom_of_stack
//                        shard1->stack_info                  shard0->stack_info     shard0->current_stack
//                                    shard1->top_of_stack               shard1->top_of_stack
//                                                shard1->current_stack
typedef struct {
  uintptr_t bottom;          // First memory address available to the stack
  uintptr_t top;             // Current memory address used by the stack
  uintptr_t limit;             // Top limit of the stack (note this is smaller than bottom, stack grows down)
  uintptr_t stack_base;               // Current stack base for mark and free stack
} mjStackInfo;

// Create a thread pool with the specified number of threads running.
MJAPI mjThreadPool* mju_threadPoolCreate(size_t number_of_threads);

// Returns the stack information for the specified thread's shard.
mjStackInfo* mju_getStackInfoForThread(mjData* d, size_t thread_id);

// Adds a thread pool to mjData and configures it for multi-threaded use.
MJAPI void mju_bindThreadPool(mjData* d, void* thread_pool);

// Gets the number of running threads in the thread pool.
MJAPI size_t mju_threadPoolNumberOfThreads(mjThreadPool* thread_pool);

// Gets the ID of the current thread being executed
MJAPI size_t mju_threadPoolCurrentWorkerId(mjThreadPool* thread_pool);

// Enqueue a task in a thread pool.
MJAPI void mju_threadPoolEnqueue(mjThreadPool* thread_pool, mjTask* task);

// Locks the allocation mutex to protect Arena allocations.
MJAPI void mju_threadPoolLockAllocMutex(mjThreadPool* thread_pool);

// Unlocks the allocation mutex to protect Arena allocations.
MJAPI void mju_threadPoolUnlockAllocMutex(mjThreadPool* thread_pool);

// Destroy a thread pool.
MJAPI void mju_threadPoolDestroy(mjThreadPool* thread_pool);

// Get the destructive interference size for the architecture.
MJAPI size_t mju_getDestructiveInterferenceSize(void);

#ifdef __cplusplus
}  // extern "C"
}  // namespace mujoco
#endif  // __cplusplus

#endif  // MUJOCO_SRC_THREAD_THREAD_POOL_H_
