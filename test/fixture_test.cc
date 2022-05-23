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

#include "test/fixture.h"

#include <gmock/gmock.h>
#include <gtest/gtest-spi.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

namespace mujoco {
namespace {

// Tests for the MujocoTest test fixture itself.
using MujocoTestTest = MujocoTest;
class MujocoErrorTestGuardTest : public ::testing::Test {};
using ::testing::IsNull;


TEST_F(MujocoTestTest, MjUserWarningFailsTest) {
  EXPECT_NONFATAL_FAILURE(mju_warning("Warning."), "Warning.");
}

TEST_F(MujocoTestTest, MjUserErrorFailsTest) {
  EXPECT_FATAL_FAILURE(mju_error("Error."), "Error.");
}

TEST_F(MujocoErrorTestGuardTest, NestedErrorGuards) {
  {
    MujocoErrorTestGuard guard1;
    {
      MujocoErrorTestGuard guard2;
    }
    EXPECT_FATAL_FAILURE(mju_error("Error."), "Error.");
    EXPECT_NONFATAL_FAILURE(mju_warning("Warning."), "Warning.");
  }
  EXPECT_THAT(mju_user_error, IsNull());
  EXPECT_THAT(mju_user_warning, IsNull());
}

}  // namespace
}  // namespace mujoco
