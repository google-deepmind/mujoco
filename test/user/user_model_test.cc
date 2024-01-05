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
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

static std::vector<mjtNum> GetRow(const mjtNum* array, int ncolumn, int row) {
  return std::vector<mjtNum>(array + ncolumn * row,
                             array + ncolumn * (row + 1));
}

// ----------------------------- test mjCModel  --------------------------------

using UserCModelTest = MujocoTest;

TEST_F(UserCModelTest, RepeatedNames) {
  static constexpr char xml[] = R"(
   <mujoco>
     <worldbody>
       <body name="body1">
         <joint axis="0 1 0" name="joint1"/>
         <geom size="1" name="geom1"/>
         <geom size="1" name="geom1"/>
        </body>
      </worldbody>
    </mujoco>)";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("repeated name 'geom1' in geom"));
}

// ------------- test automatic inference of nuser_xxx -------------------------

using UserDataTest = MujocoTest;

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

// ------------- test duplicate names ------------------------------------------
TEST_F(UserDataTest, DuplicateNames) {
  static const char* const kFilePath = "user/testdata/load_twice.xml";
  const std::string xml_path = GetTestDataFilePath(kFilePath);

  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());

  EXPECT_THAT(m, NotNull()) << error.data();
  EXPECT_THAT(m->nmesh, 2);

  for (int i = 0; i < m->nmesh; i++) {
    char mesh_name[mjMAXUINAME] = "";
    util::strcat_arr(mesh_name, m->names + m->name_meshadr[i]);
    EXPECT_THAT(std::string(mesh_name), "cube_" + std::to_string(i));
  }

  mj_deleteModel(m);
}

// ------------- test fusestatic -----------------------------------------------

using FuseStaticTest = MujocoTest;
TEST_F(FuseStaticTest, FuseStaticEquivalent) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="%s"/>
    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0"/>
        <body>
          <geom size="0.5" pos="0 1 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::string fuse = absl::StrFormat(xml_template, "true");
  std::string no_fuse = absl::StrFormat(xml_template, "false");

  mjModel* m_fuse = LoadModelFromString(fuse.c_str());
  mjModel* m_no_fuse = LoadModelFromString(no_fuse.c_str());
  ASSERT_THAT(m_fuse, NotNull());
  ASSERT_THAT(m_no_fuse, NotNull());

  EXPECT_EQ(m_fuse->nbody, 2) << "Expecting a world body and one other body";
  EXPECT_EQ(m_no_fuse->nbody, 3) << "Expecting a world body and two others";

  mjData* d_fuse = mj_makeData(m_fuse);
  mjData* d_no_fuse = mj_makeData(m_no_fuse);

  mj_step(m_fuse, d_fuse);
  mj_step(m_no_fuse, d_no_fuse);

  EXPECT_THAT(d_fuse->qvel[0], DoubleNear(d_no_fuse->qvel[0], 1e-17))
      << "Velocity should be the same after 1 step";
  EXPECT_NE(d_fuse->qvel[0], 0);

  mj_deleteData(d_fuse);
  mj_deleteData(d_no_fuse);
  mj_deleteModel(m_fuse);
  mj_deleteModel(m_no_fuse);
}

}  // namespace
}  // namespace mujoco
