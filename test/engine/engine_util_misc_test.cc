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

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::HasSubstr;

TEST_F(MujocoTest, PrintsMemoryWarning) {
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 10)), HasSubstr("1K bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 20)), HasSubstr("1M bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 30)), HasSubstr("1G bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 40)), HasSubstr("1T bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 50)), HasSubstr("1P bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 60)), HasSubstr("1E bytes"));
  EXPECT_THAT(mju_warningText(mjWARN_CNSTRFULL, pow(2, 30)+1), HasSubstr("1073741825 bytes"));
}

}  // namespace
}  // namespace mujoco
