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

#include <mujoco/mjdata.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

TEST(TestMjArrayList, TestMjArrayListSingleThreaded) {
  constexpr char xml[] = "<mujoco/>";

  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, testing::NotNull()) << "Failed to load model: "
      << error.data();
  mjData* d = mj_makeData(m);
  mjMARKSTACK;
  mjArrayList* array_list = mju_arrayListCreate(d, sizeof(int), 10);

  for (int i = 0; i < 35; ++i) {
    mju_arrayListAdd(array_list, &i);
  }
  EXPECT_EQ(mju_arrayListSize(array_list), 35);

  // Approximately (3 * sizeof(int) + 3 * sizeof(mjArrayList)) / sizeof(mjtNum)
  // However there is padding for alignment/etc.
  EXPECT_EQ(d->maxuse_stack, 54);

  for (int i = 0; i < 35; ++i) {
    EXPECT_EQ(*(int*)mju_arrayListAt(array_list, i), i);
  }

  EXPECT_TRUE(mju_arrayListAt(array_list, 100) == NULL);


  mjFREESTACK;
  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
