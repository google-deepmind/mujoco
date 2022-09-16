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

// Tests for xml/xml_native_reader.cc.

#include <array>
#include <cstddef>
#include <limits>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/cc/array_safety.h"
#include "src/engine/engine_util_errmem.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::string;
using ::testing::Eq;
using ::testing::HasSubstr;
using ::testing::IsNan;
using ::testing::IsNull;
using ::testing::NotNull;

using UserDataTest = MujocoTest;

TEST_F(UserDataTest, InvalidNUserBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_body="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_body"));
}

TEST_F(UserDataTest, InvalidNUserJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_jnt="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_jnt"));
}

TEST_F(UserDataTest, InvalidNUserGeom) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_geom="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_geom"));
}

TEST_F(UserDataTest, InvalidNUserSite) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_site="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_site"));
}

TEST_F(UserDataTest, InvalidNUserCamera) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_cam="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_cam"));
}

TEST_F(UserDataTest, InvalidNUserTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_tendon="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_tendon"));
}

TEST_F(UserDataTest, InvalidNUserActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_actuator="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_actuator"));
}

TEST_F(UserDataTest, InvalidNUserSensor) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_sensor="-2"/>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_sensor"));
}

TEST_F(UserDataTest, CanParseInf) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" pos="5e-1 -INF iNf"/>
        <geom size="1" pos="inF -inf Inf"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  const double inf = std::numeric_limits<double>::infinity();
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->geom_pos[0], 0.5);
  EXPECT_EQ(model->geom_pos[1], -inf);
  EXPECT_THAT(model->geom_pos[2], inf);
  EXPECT_EQ(model->geom_pos[3], inf);
  EXPECT_EQ(model->geom_pos[4], -inf);
  EXPECT_EQ(model->geom_pos[5], inf);
  mj_deleteModel(model);
}

TEST_F(UserDataTest, CanParseNanAndRaisesWarning) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" pos="nan NaN NAN"/>
        <geom size="1" pos="1.0 0.0 nAn"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(warning, HasSubstr("XML contains a 'NaN'"));
  EXPECT_THAT(model->geom_pos[0], IsNan());
  EXPECT_THAT(model->geom_pos[1], IsNan());
  EXPECT_THAT(model->geom_pos[2], IsNan());
  EXPECT_EQ(model->geom_pos[3], 1);
  EXPECT_EQ(model->geom_pos[4], 0);
  EXPECT_THAT(model->geom_pos[5], IsNan());
  mj_deleteModel(model);
}

TEST_F(UserDataTest, InvalidArrayElement) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1.0 0.0 0.0 [[1]]"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("problem reading attribute 'axisangle'"));
}

TEST_F(UserDataTest, InvalidArrayLength) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0 0 0 asd"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("has too much data"));
}

TEST_F(UserDataTest, InvalidNumber) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0.1.2.3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("problem reading attribute"));
}

TEST_F(UserDataTest, AllowsSpaces) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1" axisangle="1 0   0 0 "/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(UserDataTest, InvalidDoubleOrientation) {
  std::string prefix = "<mujoco><worldbody><";
  std::string suffix = "/></worldbody></mujoco>";
  std::vector<std::string> orientations = {
    R"( quat="0 1 0 0" )",
    R"( euler="1.7 2.9 0.1" )",
    R"( zaxis="1.7 2.9 0.1" )",
    R"( axisangle="1.7 2.9 0.1 0" )",
    R"( xyaxes="1.7 2.9 0.1 0.4 1.4 0.6" )",
  };
  std::vector<std::string> fields = {
    "geom", "body", "camera", "site"
  };
  for (auto const& field : fields) {
    for (auto const& orient1 : orientations) {
      for (auto const& orient2 : orientations) {
        if (orient1 == orient2) continue;
        std::string xml = prefix + field + orient1 + orient2 + suffix;
        std::array<char, 1024> error;
        mjModel* model = LoadModelFromString(xml.c_str(), error.data(), error.size());
        ASSERT_THAT(model, IsNull());
        EXPECT_THAT(error.data(), HasSubstr("multiple orientation specifiers for the same field"));
      }
    }
  }
}

TEST_F(UserDataTest, InvalidInertialOrientation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <inertial pos="0 0 0" mass="1" quat="1 0 0 0" fullinertia="1 1 1 0 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("multiple orientation specifiers for the same field"));
}

TEST_F(UserDataTest, ReadShellParameter) {
  static constexpr char xml[] = R"(
  <mujoco>
    <asset>
      <mesh name="example_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        face="0 2 1  2 0 3" />
    </asset>
    <worldbody>
      <geom type="mesh" mesh="example_mesh" shellinertia="true"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(UserDataTest, ReadsDamper) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" ctrlrange="0 1"/>
      <general joint="jnt" ctrllimited="true" ctrlrange="0 1" gaintype="affine" biastype="none"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_THAT(model->actuator_gaintype[0], Eq(mjGAIN_AFFINE));
  EXPECT_THAT(model->actuator_gaintype[1], Eq(mjGAIN_AFFINE));
  EXPECT_THAT(model->actuator_biastype[0], Eq(mjBIAS_NONE));
  EXPECT_THAT(model->actuator_biastype[1], Eq(mjBIAS_NONE));
  mj_deleteModel(model);
}

TEST_F(UserDataTest, RequiresPoisitiveDamping) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="-1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("damping coefficient cannot be negative"));
}

TEST_F(UserDataTest, RequiresControlRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("required attribute missing: 'ctrlrange'"));
}

TEST_F(UserDataTest, PositiveControlRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0" range="-10 10"/>
      </body>
    </worldbody>
    <actuator>
      <damper joint="jnt" kv="1" ctrlrange="-1 0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("control range cannot be negative"));
}

TEST_F(UserDataTest, ReadsSkinGroups) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <composite prefix="B0" type="box" count="4 4 4" spacing=".2">
          <geom size=".1" group="2"/>
          <skin group="4"/>
        </composite>
      </body>
      <body>
        <composite prefix="B1" type="box" count="4 4 4" spacing=".2">
          <geom size=".1" group="4"/>
          <skin group="2"/>
        </composite>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  int geomid1 = mj_name2id(model, mjOBJ_GEOM, "B0G0_0_0");
  int geomid2 = mj_name2id(model, mjOBJ_GEOM, "B1G0_0_0");
  EXPECT_THAT(model->geom_group[geomid1], 2);
  EXPECT_THAT(model->skin_group[0], 4);
  EXPECT_THAT(model->geom_group[geomid2], 4);
  EXPECT_THAT(model->skin_group[1], 2);
  mj_deleteModel(model);
}

TEST_F(UserDataTest, InvalidSkinGroup) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <composite prefix="B0" type="box" count="6 6 6" spacing=".2">
          <geom size=".1"/>
          <skin group="6"/>
        </composite>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("skin group must be between 0 and 5\nElement 'skin', line 7"));
  mj_deleteModel(model);
}

// ------------- test relative frame sensor parsing ----------------------------

using RelativeFrameSensorParsingTest = MujocoTest;

TEST_F(RelativeFrameSensorParsingTest, RefNameButNoType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objname="sensorized" objtype="body" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("but reftype is missing"));
}

TEST_F(RelativeFrameSensorParsingTest, RefTypeButNoName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objname="sensorized" objtype="body" reftype="site"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("attribute missing: 'refname'"));
}

// ------------- test actlimited parsing ---------------------------------------

using ActuatorTest = MujocoTest;

TEST_F(ActuatorTest, InvalidActlimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="hinge" actlimited="invalid" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("unrecognized attribute"));
}

TEST_F(ActuatorTest, IncompleteActlimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actlimited="true" dyntype="filter" actrange="-1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("attribute 'actrange' does not have enough data"));
}

TEST_F(ActuatorTest, ReadsByte) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="filter" actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(*(model->actuator_actlimited), (mjtByte)(1 & 0xFF));
  mj_deleteModel(model);
}

// ------------- test intvelocity parsing ---------------------------------------

using IntegratedVelocityTest = MujocoTest;

TEST_F(IntegratedVelocityTest, CheckEquivalence) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge" kp="2.5" actrange="-1.57 1.57"/>
      <general joint="hinge" actlimited="true" actrange="-1.57 1.57" dyntype="integrator"
        biastype="affine" gainprm="2.5" biasprm="0 -2.5 0"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, testing::NotNull());
  // same actlimited
  EXPECT_EQ(model->actuator_actlimited[0], 1);
  EXPECT_EQ(model->actuator_actlimited[1], 1);
  // same dyntype
  EXPECT_EQ(model->actuator_dyntype[0], mjDYN_INTEGRATOR);
  EXPECT_EQ(model->actuator_dyntype[1], mjDYN_INTEGRATOR);
  // same biastype
  EXPECT_EQ(model->actuator_biastype[0], mjBIAS_AFFINE);
  EXPECT_EQ(model->actuator_biastype[1], mjBIAS_AFFINE);
  // same gaintype
  EXPECT_EQ(model->actuator_gaintype[0], mjGAIN_FIXED);
  EXPECT_EQ(model->actuator_gaintype[1], mjGAIN_FIXED);
  // same gainprm
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 2.5);
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN], 2.5);
  // same biasprm
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[0], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[1], -2.5);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[2], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS + 1], -2.5);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[mjNBIAS + 2], 0.0);
  // same actrange
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 0], -1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 1], 1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 2], -1.57);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 3], 1.57);
  mj_deleteModel(model);
}

TEST_F(IntegratedVelocityTest, CheckDefaultsIfNotSpecified) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, testing::NotNull());
  // check that by default kp = 1
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 1.0);
  // check that biasprm is (0, -1, 0)
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[0], 0.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[1], -1.0);
  EXPECT_DOUBLE_EQ(model->actuator_biasprm[2], 0.0);
  mj_deleteModel(model);
}

TEST_F(IntegratedVelocityTest, NoActrangeThrowsError) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid activation range for actuator"));
}

TEST_F(IntegratedVelocityTest, DefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <default>
      <intvelocity kp="5"/>
      <default class="withactrange">
        <intvelocity kp="1" actrange="-1 1"/>
      </default>
    </default>
    <worldbody>
      <body>
        <joint name="hinge1"/>
        <joint name="hinge2"/>
        <geom type="box" size=".025 .025 .025"/>
      </body>
    </worldbody>
    <actuator>
      <intvelocity joint="hinge1" actrange="0 1"/>
      <intvelocity joint="hinge2" class="withactrange"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, testing::NotNull());
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[0], 5);
  EXPECT_DOUBLE_EQ(model->actuator_gainprm[mjNGAIN], 1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 0], 0);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 1], 1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 2], -1);
  EXPECT_DOUBLE_EQ(model->actuator_actrange[0 + 3], 1);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
