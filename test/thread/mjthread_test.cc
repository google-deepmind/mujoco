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

#include <mujoco/mjthread.h>

#include <atomic>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/thread/task.h"
#include "src/thread/thread_pool.h"

namespace {

struct TestFunctionArgs_ {
  int input;
  // make this atomic to avoid red-herring tsan failures.
  std::atomic<int> output;
};
typedef struct TestFunctionArgs_ TestFunctionArgs;

void* test_function(void* args) {
  TestFunctionArgs* test_function_args = static_cast<TestFunctionArgs*>(args);
  if (!test_function_args) {
    return nullptr;
  }
  test_function_args->output = test_function_args->input;
  return nullptr;
}

TEST(TestMjThreadPool, EnsureStructClassSizeMatch) {
  EXPECT_EQ(sizeof(mjTask), sizeof(mujoco::Task));
  EXPECT_EQ(sizeof(mjThreadPool), sizeof(mujoco::ThreadPool<128>));
}

TEST(TestMjThreadPool, TestMjThreadPool10Threads) {
  mjThreadPool* thread_pool = mju_threadPoolCreate(10);

  TestFunctionArgs test_function_args[1000];
  mjTask tasks[1000];
  for (int i = 0; i < 1000; ++i) {
    test_function_args[i].input = i;
    mju_threadPoolEnqueue(thread_pool, &tasks[i], test_function,
                          (void*)&test_function_args[i]);
  }

  for (int i = 0; i < 1000; ++i) {
    mju_taskJoin(&tasks[i]);
  }

  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].output);
  }
  mju_threadPoolDestroy(thread_pool);
}

TEST(TestMjThreadPool, TestMjThreadPool100Threads) {
  mjThreadPool* thread_pool = mju_threadPoolCreate(100);

  TestFunctionArgs test_function_args[1000];
  mjTask tasks[1000];
  for (int i = 0; i < 1000; ++i) {
    test_function_args[i].input = i;
    mju_threadPoolEnqueue(thread_pool, &tasks[i], test_function,
                          (void*)&test_function_args[i]);
  }

  for (int i = 0; i < 1000; ++i) {
    mju_taskJoin(&tasks[i]);
  }

  for (int i = 0; i < 1000; ++i) {
    EXPECT_EQ(test_function_args[i].input, test_function_args[i].output);
  }

  mju_threadPoolDestroy(thread_pool);
}

}  // namespace
