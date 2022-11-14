// Copyright 2022 DeepMind Technologies Limited
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

// Tests for engine/engine_util_sparse.c

#include "src/engine/engine_util_sparse.h"

#include <gtest/gtest.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using EngineUtilSparseTest = MujocoTest;

TEST_F(EngineUtilSparseTest, MjuDot) {
  mjtNum a[] = {1, 2, 3, 4, 5, 6, 7};
  mjtNum b[] = {7, 0, 6, 0, 0, 5, 0, 0, 0, 4, 0, 0, 0, 3, 0, 0, 2, 0, 1};
  int i[] = {0, 2, 5, 9, 13, 16, 18};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four
  EXPECT_EQ(mju_dotSparse(a, b, 0, i), 0);
  EXPECT_EQ(mju_dotSparse(a, b, 1, i), 7);
  EXPECT_EQ(mju_dotSparse(a, b, 2, i), 7 + 2*6);
  EXPECT_EQ(mju_dotSparse(a, b, 3, i), 7 + 2*6 + 3*5);
  EXPECT_EQ(mju_dotSparse(a, b, 4, i), 7 + 2*6 + 3*5 + 4*4);
  EXPECT_EQ(mju_dotSparse(a, b, 5, i), 7 + 2*6 + 3*5 + 4*4 + 5*3);
  EXPECT_EQ(mju_dotSparse(a, b, 6, i), 7 + 2*6 + 3*5 + 4*4 + 5*3 + 6*2);
  EXPECT_EQ(mju_dotSparse(a, b, 7, i), 7 + 2*6 + 3*5 + 4*4 + 5*3 + 6*2 + 7);
}

}  // namespace
}  // namespace mujoco
