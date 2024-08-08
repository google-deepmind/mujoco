// Copyright 2024 DeepMind Technologies Limited
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

#include "src/engine/engine_sort.h"

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>
#include "src/engine/engine_sort.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using EngineSortTest = MujocoTest;

struct IntStruct {
  int value;
};

quicksortfunc(int_compare, context, x, y) {
  int a = *(int*)x;
  int b = *(int*)y;
  if (a < b) {
    return -1;
  } else if (a == b) {
    return 0;
  } else {
    return 1;
  }
}

quicksortfunc(intstruct_compare, context, x, y) {
  IntStruct* a = (IntStruct*)x;
  IntStruct* b = (IntStruct*)y;
  if (a->value < b->value) {
    return -1;
  } else if (a->value == b->value) {
    return 0;
  } else {
    return 1;
  }
}

TEST_F(EngineSortTest, Sort) {
  // test int
  std::vector<int> x = {1, 3, 2};
  mjQUICKSORT(x.data(), x.size(), sizeof(int), int_compare, x.data());
  EXPECT_EQ(x[0], 1);
  EXPECT_EQ(x[1], 2);
  EXPECT_EQ(x[2], 3);

  // test custom struct with mjQUICKSORT
  std::vector<IntStruct> y = {{1}, {3}, {2}};
  mjQUICKSORT(y.data(), y.size(), sizeof(IntStruct), intstruct_compare, NULL);
  EXPECT_EQ(y[0].value, 1);
  EXPECT_EQ(y[1].value, 2);
  EXPECT_EQ(y[2].value, 3);
}

}  // namespace
}  // namespace mujoco
