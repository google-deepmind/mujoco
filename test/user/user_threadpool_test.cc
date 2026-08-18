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

#include "src/user/user_threadpool.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <set>

#include <gtest/gtest.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using user::ThreadPool;
using ThreadPoolTest = MujocoTest;

TEST(ThreadPoolTest, ConstructorCreatesCorrectNumberOfThreads) {
  ThreadPool pool(4);
  EXPECT_EQ(pool.NumThreads(), 4);
}

TEST(ThreadPoolTest, SingleThread) {
  ThreadPool pool(1);
  EXPECT_EQ(pool.NumThreads(), 1);
}

TEST(ThreadPoolTest, ScheduleAndWait) {
  ThreadPool pool(4);
  std::atomic<int> counter{0};

  for (int i = 0; i < 10; ++i) {
    pool.Schedule([&counter]() { counter++; });
  }

  pool.WaitCount(10);
  EXPECT_EQ(counter.load(), 10);
}

TEST(ThreadPoolTest, GetCountReturnsCompletedTasks) {
  ThreadPool pool(2);

  pool.Schedule([]() {});
  pool.Schedule([]() {});
  pool.WaitCount(2);

  EXPECT_GE(pool.GetCount(), 2);
}

TEST(ThreadPoolTest, ResetCountClearsCounter) {
  ThreadPool pool(2);

  pool.Schedule([]() {});
  pool.WaitCount(1);
  EXPECT_GE(pool.GetCount(), 1);

  pool.ResetCount();
  EXPECT_EQ(pool.GetCount(), 0);
}

TEST(ThreadPoolTest, WorkerIdIsValid) {
  ThreadPool pool(4);
  std::atomic<int> valid_ids{0};

  for (int i = 0; i < 10; ++i) {
    pool.Schedule([&valid_ids, &pool]() {
      int id = ThreadPool::WorkerId();
      if (id >= 0 && id < pool.NumThreads()) {
        valid_ids++;
      }
    });
  }

  pool.WaitCount(10);
  EXPECT_EQ(valid_ids.load(), 10);
}

TEST(ThreadPoolTest, WorkerIdOutsidePoolReturnsNegativeOne) {
  EXPECT_EQ(ThreadPool::WorkerId(), -1);
}

TEST(ThreadPoolTest, ParallelExecution) {
  ThreadPool pool(4);
  std::atomic<int> sum{0};
  const int n = 1000;

  for (int i = 0; i < n; ++i) {
    pool.Schedule([&sum]() { sum++; });
  }

  pool.WaitCount(n);
  EXPECT_EQ(sum.load(), n);
}

TEST(ThreadPoolTest, MultipleWaitCounts) {
  ThreadPool pool(2);
  std::atomic<int> counter{0};

  for (int i = 0; i < 5; ++i) {
    pool.Schedule([&counter]() { counter++; });
  }
  pool.WaitCount(5);
  EXPECT_EQ(counter.load(), 5);

  pool.ResetCount();

  for (int i = 0; i < 3; ++i) {
    pool.Schedule([&counter]() { counter++; });
  }
  pool.WaitCount(3);
  EXPECT_EQ(counter.load(), 8);
}

TEST(ThreadPoolTest, ScopedPoolDestruction) {
  std::atomic<int> counter{0};
  {
    ThreadPool pool(2);
    for (int i = 0; i < 5; ++i) {
      pool.Schedule([&counter]() { counter++; });
    }
    pool.WaitCount(5);
  }
  EXPECT_EQ(counter.load(), 5);
}

TEST(ThreadPoolTest, OnlyConfiguredThreadsAreUsed) {
  constexpr int kNumThreads = 4;
  constexpr int kNumTasks = 100;

  ThreadPool pool(kNumThreads);
  std::mutex ids_mutex;
  std::set<int> worker_ids;

  for (int i = 0; i < kNumTasks; ++i) {
    pool.Schedule([&ids_mutex, &worker_ids]() {
      int id = ThreadPool::WorkerId();
      std::lock_guard<std::mutex> lock(ids_mutex);
      worker_ids.insert(id);
    });
  }

  pool.WaitCount(kNumTasks);

  EXPECT_LE(worker_ids.size(), kNumThreads);
  for (int id : worker_ids) {
    EXPECT_GE(id, 0);
    EXPECT_LT(id, kNumThreads);
  }
}

TEST(ThreadPoolTest, BlockingTaskDoesNotPreventOtherTasks) {
  constexpr int kNumThreads = 4;
  ThreadPool pool(kNumThreads);

  std::mutex block_mutex;
  std::condition_variable block_cv;
  bool unblock = false;

  std::atomic<int> fast_tasks_completed{0};
  constexpr int kNumFastTasks = 10;

  pool.Schedule([&block_mutex, &block_cv, &unblock]() {
    std::unique_lock<std::mutex> lock(block_mutex);
    block_cv.wait(lock, [&unblock]() { return unblock; });
  });

  for (int i = 0; i < kNumFastTasks; ++i) {
    pool.Schedule([&fast_tasks_completed]() { fast_tasks_completed++; });
  }

  pool.WaitCount(kNumFastTasks);

  EXPECT_EQ(fast_tasks_completed.load(), kNumFastTasks);

  {
    std::lock_guard<std::mutex> lock(block_mutex);
    unblock = true;
  }
  block_cv.notify_one();

  pool.WaitCount(kNumFastTasks + 1);
}
}  // namespace
}  // namespace mujoco
