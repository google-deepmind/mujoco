// Copyright 2021 DeepMind Technologies Limited
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

// Tests for engine/engine_util_solve.c.

#include "src/engine/engine_util_solve.h"

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using QCQP2Test = MujocoTest;

TEST_F(QCQP2Test, DegenerateAMatrix) {
  // A 2x2 matrix with determinant zero.
  const mjtNum Ain[9] { 6, -15, 2, -5 };

  // Any values will do for these three inputs.
  const mjtNum bin[3] { -12, 49 };
  const mjtNum d[3] { 11, 31 };
  const mjtNum r = 0.01;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[2] { 999, 999 };

  EXPECT_EQ(mju_QCQP2(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
}

using QCQP3Test = MujocoTest;

TEST_F(QCQP3Test, DegenerateAMatrix) {
  // A 3x3 matrix with determinant zero.
  const mjtNum Ain[9] { 1, 4, -2, -3, -7, 5, 2, -9, 0 };

  // Any values will do for these three inputs.
  const mjtNum bin[3] { -12, 49, 8 };
  const mjtNum d[3] { 11, 31, -23 };
  const mjtNum r = 0.1;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[3] { 999, 999, 999 };

  EXPECT_EQ(mju_QCQP3(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
  EXPECT_EQ(res[2], 0);
}

}  // namespace
}  // namespace mujoco
