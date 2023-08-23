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

#include <mujoco/mjdata.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using testing::NotNull;

template <typename T, int N, int Capacity>
constexpr int GetExpectedStackUsageBytes() {
  if constexpr (N <= 0) {
    return 0;
  } else {
    constexpr auto RoundUpToAlignment =
        [](int x) {
          constexpr auto kAlignment = alignof(std::max_align_t);
          return kAlignment * (x / kAlignment + ((x % kAlignment) ? 1 : 0));
        };
    return RoundUpToAlignment(sizeof(mjArrayList)) +
           RoundUpToAlignment(Capacity * sizeof(T)) +
           GetExpectedStackUsageBytes<T, N - Capacity, 2 * Capacity>();
  }
}

TEST(TestMjArrayList, TestMjArrayListSingleThreaded) {
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString("<mujoco/>", error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << "Failed to load model: " << error.data();
  mjData* d = mj_makeData(m);
  mjMARKSTACK;
  using DataType = int;
  constexpr int kInitialCapacity = 10;
  mjArrayList* array_list =
      mju_arrayListCreate(d, sizeof(DataType), kInitialCapacity);

  constexpr int kNumElements = 35;
  for (int i = 0; i < kNumElements; ++i) {
    mju_arrayListAdd(array_list, &i);
  }
  EXPECT_EQ(mju_arrayListSize(array_list), kNumElements);

  constexpr int kExpectedMaxUseStack =
      GetExpectedStackUsageBytes<DataType, kNumElements, kInitialCapacity>() /
      sizeof(mjtNum);
  EXPECT_EQ(d->maxuse_stack, kExpectedMaxUseStack);

  for (int i = 0; i < kNumElements; ++i) {
    EXPECT_EQ(*static_cast<int*>(mju_arrayListAt(array_list, i)), i);
  }

  EXPECT_EQ(mju_arrayListAt(array_list, kNumElements), nullptr);
  EXPECT_EQ(mju_arrayListAt(array_list, 100), nullptr);

  mjFREESTACK;
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST(TestMjArrayList, ZeroInitialCapacity) {
  mjModel* m = LoadModelFromString("<mujoco/>", nullptr, 0);
  ASSERT_THAT(m, NotNull()) << "Failed to load model";
  mjData* d = mj_makeData(m);
  mjMARKSTACK;
  mjArrayList* array_list =
      mju_arrayListCreate(d, sizeof(double), /*initial_capacity=*/0);
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

  mjFREESTACK;
  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
