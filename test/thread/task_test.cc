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

#include "src/thread/task.h"

#include <gtest/gtest.h>

namespace mujoco {
namespace {

struct TestFunctionArgs {
  int input;
  int output;
};

void* test_function(void* args) {
  TestFunctionArgs* test_function_args = (TestFunctionArgs*)args;
  test_function_args->output = test_function_args->input;
  return nullptr;
}

TEST(TestMjThread, TestMjThread) {
  TestFunctionArgs test_function_args;
  test_function_args.input = 1;
  test_function_args.output = 2;
  Task task;
  Task::Initialize(
      &task, test_function, static_cast<void*>(&test_function_args));
  task.Execute();
  task.Join();
  EXPECT_EQ(test_function_args.input, test_function_args.output);
}

}  // namespace
}  // namespace mujoco
