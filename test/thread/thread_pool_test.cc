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

#include "src/thread/thread_pool.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include "src/thread/task.h"

namespace mujoco {
namespace {

struct TestFunctionArgs {
  int input;
  // make this atomic to avoid red-herring tsan failures.
  std::atomic<int> output;
};

void* test_function(void* args) {
  TestFunctionArgs* test_function_args = static_cast<TestFunctionArgs*>(args);
  test_function_args->output = test_function_args->input;
  return nullptr;
}

TEST(TestMjThreadPool, TestMjThreadPool10Threads) {
  ThreadPool<10> thread_pool(10);

  constexpr int kTasks = 1000;
  TestFunctionArgs test_function_args[kTasks];
  Task tasks[kTasks];
  for (int i = 0; i < kTasks; ++i) {
    test_function_args[i].input = i;
    thread_pool.Enqueue(
        &tasks[i], test_function, static_cast<void*>(&test_function_args[i]));
  }

  for (int i = 0; i < kTasks; ++i) {
    tasks[i].Join();
  }

  for (int i = 0; i < kTasks; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].output);
  }

  thread_pool.Shutdown();
}

TEST(TestMjThreadPool, TestMjThreadPool100Threads) {
  ThreadPool<100> thread_pool(100);

  constexpr int kTasks = 1000;
  TestFunctionArgs test_function_args[kTasks];
  Task tasks[kTasks];
  for (int i = 0; i < kTasks; ++i) {
    test_function_args[i].input = i;
    thread_pool.Enqueue(
        &tasks[i], test_function, static_cast<void*>(&test_function_args[i]));
  }

  for (int i = 0; i < kTasks; ++i) {
    tasks[i].Join();
  }

  for (int i = 0; i < kTasks; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].output);
  }

  thread_pool.Shutdown();
}

TEST(TestMjThreadPool, TestMjThreadPoolManyWriters) {
  ThreadPool<10> thread_pool(10);

  constexpr int kTasks = 20;
  TestFunctionArgs test_function_args[kTasks];
  Task tasks[kTasks];
  std::unique_ptr<std::thread> enqueue_threads[kTasks];

  // add tasks to the thread pool from many threads
  std::condition_variable start_cv;
  std::mutex start_mutex;
  bool start = false;
  for (int i = 0; i < kTasks; ++i) {
    test_function_args[i].input = i;
    enqueue_threads[i] = std::make_unique<std::thread>([&, i] {
      // synchronize all threads adding to the thread_pool at the same time
      {
        std::unique_lock<std::mutex> lock(start_mutex);
        start_cv.wait(lock, [&] { return start; });
      }
      // enqueue outside the lock, to get some concurrency
      thread_pool.Enqueue(
          &tasks[i], test_function, static_cast<void*>(&test_function_args[i]));
    });
  }
  {
    std::unique_lock<std::mutex> lock(start_mutex);
    start = true;
  }
  start_cv.notify_all();

  for (int i = 0; i < kTasks; ++i) {
    enqueue_threads[i]->join();
  }

  for (int i = 0; i < kTasks; ++i) {
    tasks[i].Join();
  }

  for (int i = 0; i < kTasks; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].output);
  }

  thread_pool.Shutdown();
}

}  // namespace
}  // namespace mujoco
