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
#include "src/engine/engine_util_container.h"

#include <array>
#include <cstddef>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using testing::NotNull;

template <int prev_size, typename T, int N, int capacity>
constexpr int GetExpectedStackUsageBytes() {
  if constexpr (N <= 0) {
    return prev_size;
  } else {
    constexpr auto RoundUpToAlignment = [](int x, int alignment) {
      return alignment * (x / alignment + ((x % alignment) ? 1 : 0));
    };
    constexpr int size_with_arraylist = RoundUpToAlignment(
        prev_size + sizeof(mjArrayList), alignof(mjArrayList));
    constexpr int size_with_buffer = RoundUpToAlignment(
        size_with_arraylist + capacity * sizeof(T), alignof(std::max_align_t));
    return GetExpectedStackUsageBytes<size_with_buffer, T, N - capacity,
                                      2 * capacity>();
  }
}

class TestMjArrayList : public MujocoTest {};

TEST_F(TestMjArrayList, TestMjArrayListSingleThreaded) {
  std::array<char, 1024> error;
  MjModelPtr m = LoadModelFromString("<mujoco/>", error.data(), error.size());
  ASSERT_THAT(m.get(), NotNull()) << "Failed to load model: " << error.data();
  MjDataPtr d = MakeData(m);
  mj_markStack(d.get());

  using DataType = int;
  constexpr int kInitialCapacity = 10;
  mjArrayList* array_list =
      mju_arrayListCreate(d.get(), sizeof(DataType), kInitialCapacity);

  constexpr int kNumElements = 35;
  for (int i = 0; i < kNumElements; ++i) {
    mju_arrayListAdd(array_list, &i);
  }
  EXPECT_EQ(mju_arrayListSize(array_list), kNumElements);

  constexpr int kFrameMarkerSize = 2 * sizeof(size_t) + sizeof(void*);
  constexpr int kExpectedMaxUseStack =
      GetExpectedStackUsageBytes<kFrameMarkerSize, DataType, kNumElements,
                                 kInitialCapacity>();
  EXPECT_EQ(d->maxuse_stack, kExpectedMaxUseStack);

  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(*static_cast<int*>(mju_arrayListAt(array_list, i)), i);
  }

  EXPECT_EQ(mju_arrayListAt(array_list, kNumElements), nullptr);
  EXPECT_EQ(mju_arrayListAt(array_list, 100), nullptr);

  mj_freeStack(d.get());
}

TEST_F(TestMjArrayList, ZeroInitialCapacity) {
  char error[1024];
  MjModelPtr m = LoadModelFromString("<mujoco/>", error, sizeof(error));
  ASSERT_THAT(m.get(), NotNull()) << "Failed to load model: " << error;
  MjDataPtr d = MakeData(m);
  mj_markStack(d.get());
  mjArrayList* array_list =
      mju_arrayListCreate(d.get(), sizeof(double), /*initial_capacity=*/0);
  EXPECT_EQ(mju_arrayListSize(array_list), 0);

  for (int i = 0; i < 35; ++i) {
    double value = i;
    mju_arrayListAdd(array_list, &value);
  }
  EXPECT_EQ(mju_arrayListSize(array_list), 35);

  for (int i = 0; i < 35; ++i) {
    EXPECT_EQ(*static_cast<double*>(mju_arrayListAt(array_list, i)), i);
  }
  EXPECT_EQ(mju_arrayListAt(array_list, 35), nullptr);

  mj_freeStack(d.get());
}

}  // namespace
}  // namespace mujoco
