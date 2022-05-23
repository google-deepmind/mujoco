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

// Tests for engine/engine_support.c.

#include <string_view>

#include <mujoco/mujoco.h>
#include "test/fixture.h"

#include <gtest/gtest.h>

namespace mujoco {
namespace {

using VersionTest = MujocoTest;

const char *const kExpectedVersionString = "2.2.0";

TEST_F(VersionTest, MjVersion) {
  EXPECT_EQ(mj_version(), mjVERSION_HEADER);
}

TEST_F(VersionTest, MjVersionString) {
  EXPECT_EQ(std::string_view(mj_versionString()), kExpectedVersionString);
}

}  // namespace
}  // namespace mujoco
