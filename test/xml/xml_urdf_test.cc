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

// Tests for xml/xml_api.cc.

#include <cstddef>
#include <cstring>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

// ---------------------------- test capsule --------------------------------

TEST_F(MujocoTest, ReadsCapsule) {
  static constexpr char urdf[] = R"(
  <robot name="">
  <link name="torso">
    <collision>
      <origin rpy="-1.57080 0.00000 0.00000" xyz="0.00000 0.00000 0.00000"/>
      <geometry>
        <capsule length="0.14000" radius="0.07000"/>
      </geometry>
    </collision>
  </link>
  </robot>
  )";
  std::array<char, 1000> error;
  mjModel* model = LoadModelFromString(urdf, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
