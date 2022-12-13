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

// Tests for user/user_model.cc.

#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include "src/cc/array_safety.h"
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::HasSubstr;
using UserCompositeTest = MujocoTest;

// ------------- test automatic inference of nuser_xxx -------------------------

TEST_F(UserCompositeTest, MultipleJointsNotAllowedUnlessParticle) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body>
      <freejoint/>
      <composite type="box" count="7 7 7" spacing="0.04">
        <joint kind="main" solreffix="0.03 1" solimpfix="0 .1 .01"/>
        <joint kind="main" solreffix="0.03 1" solimpfix="0 .1 .01"/>
      </composite>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull());
  EXPECT_THAT(error.data(),
              HasSubstr("Only particles are allowed to have multiple joints"));
}

TEST_F(UserCompositeTest, StretchAndTwistAllowed) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="B10">
      <freejoint/>
      <composite type="rope" count="21 1 1" spacing="0.04">
        <joint kind="main" damping="0.005"/>
        <joint kind="stretch" damping="0.005"/>
        <joint kind="twist" damping="0.005"/>
        <geom type="capsule" size=".01 .015"/>
      </composite>
    </body>
  </worldbody>
  </mujoco>
  )";
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull());
  EXPECT_THAT(warning, HasSubstr("deprecated"));
  mj_deleteModel(m);
}

TEST_F(UserCompositeTest, SpacingGreaterThanGeometry) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <composite type="grid" count="282 2">
      <geom size="8"/>
    </composite>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("Spacing must be larger than geometry size"));
}

TEST_F(UserCompositeTest, SpacingEqualToGeometry) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <composite type="grid" count="282 2" spacing="8">
      <geom size="8"/>
    </composite>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);
  mj_step(m, d);
  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
