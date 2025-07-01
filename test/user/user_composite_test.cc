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

#include <array>
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

// ------------------------ cable tests ---------------------------------------

TEST_F(UserCompositeTest, ShapeCanBeOmitted) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <composite type="cable" count="100 1 1" curve="s">
      <geom type="box" size="1 1 1"/>
    </composite>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, NotNull()) << error.data();
  mj_deleteModel(m);
}

TEST_F(UserCompositeTest, InvalidShape) {
  static constexpr char xml[] = R"(
  <mujoco>
  <worldbody>
    <composite type="cable" count="100 1 1" curve="s s exp">
      <geom type="box" size="1 1 1"/>
    </composite>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(m, IsNull()) << error.data();
  EXPECT_THAT(error.data(),
              HasSubstr("The curve array contains an invalid shape"));
}

}  // namespace
}  // namespace mujoco
