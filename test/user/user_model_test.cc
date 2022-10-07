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
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using UserDataTest = MujocoTest;

static std::vector<mjtNum> GetRow(const mjtNum* array, int ncolumn, int row) {
  return std::vector<mjtNum>(array + ncolumn * row,
                             array + ncolumn * (row + 1));
}

// ------------- test automatic inference of nuser_xxx -------------------------

TEST_F(UserDataTest, AutoNUserBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body user="1 2 3"/>
      <body user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_body, 3);
  EXPECT_THAT(GetRow(m->body_user, m->nuser_body, 1), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->body_user, m->nuser_body, 2), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint user="1 2 3"/>
        <joint user="2 3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_jnt, 3);
  EXPECT_THAT(GetRow(m->jnt_user, m->nuser_jnt, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->jnt_user, m->nuser_jnt, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserGeom) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <geom size="1" user="1 2 3"/>
      <geom size="1" user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_geom, 3);
  EXPECT_THAT(GetRow(m->geom_user, m->nuser_geom, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->geom_user, m->nuser_geom, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserSite) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site user="1 2 3"/>
      <site user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_site, 3);
  EXPECT_THAT(GetRow(m->site_user, m->nuser_site, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->site_user, m->nuser_site, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserCamera) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <camera user="1 2 3"/>
      <camera user="2 3"/>
    </worldbody>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_cam, 3);
  EXPECT_THAT(GetRow(m->cam_user, m->nuser_cam, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->cam_user, m->nuser_cam, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
      <site name="b"/>
    </worldbody>
    <tendon>
      <spatial user="1 2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
      <spatial user="2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_tendon, 3);
  EXPECT_THAT(GetRow(m->tendon_user, m->nuser_tendon, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->tendon_user, m->nuser_tendon, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a" user="1 2 3"/>
      <motor joint="a" user="2 3"/>
    </actuator>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_actuator, 3);
  EXPECT_THAT(GetRow(m->actuator_user, m->nuser_actuator, 0),
              ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->actuator_user, m->nuser_actuator, 1),
              ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

TEST_F(UserDataTest, AutoNUserSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <accelerometer site="a" user="1 2 3"/>
      <gyro site="a" user="2 3"/>
    </sensor>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nuser_sensor, 3);
  EXPECT_THAT(GetRow(m->sensor_user, m->nuser_sensor, 0), ElementsAre(1, 2, 3));
  EXPECT_THAT(GetRow(m->sensor_user, m->nuser_sensor, 1), ElementsAre(2, 3, 0));
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
