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

#include "src/thread/thread_queue.h"

#include <cstddef>

#include <gtest/gtest.h>

namespace mujoco {
namespace {

constexpr size_t kBufferCapacity = 640;

TEST(TestMujocoLocklessQueue, TestMujocoLocklessQueue) {
  LocklessQueue<void*, 640> queue;
  EXPECT_TRUE(queue.empty());
  int test_integers[kBufferCapacity];
  for (int h = 0; h < 10; ++h) {
    for (int i = 0; i < kBufferCapacity; ++i) {
      test_integers[i] = i;
      queue.push(&test_integers[i]);
    }
    EXPECT_TRUE(queue.full());

    for (int i = 0; i < kBufferCapacity; ++i) {
      void* output_ptr = queue.pop();
      ASSERT_EQ(output_ptr, &test_integers[i]);
    }
    EXPECT_TRUE(queue.empty());
  }
}

}  // namespace
}  // namespace mujoco
