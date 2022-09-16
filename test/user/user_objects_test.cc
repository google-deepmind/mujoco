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

// Tests for user/user_objects.cc.

#include <array>
#include <cstddef>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

// ------------------------ test keyframes -------------------------------------

using KeyframeTest = MujocoTest;

constexpr char kKeyframePath[] = "user/testdata/keyframe.xml";

TEST_F(KeyframeTest, CheckValues) {
  const std::string xml_path = GetTestDataFilePath(kKeyframePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  EXPECT_EQ(model->nkey, 7);
  EXPECT_EQ(model->key_time[0 * 1], 0.1);
  EXPECT_EQ(model->key_qpos[1 * model->nq], 0.2);
  EXPECT_EQ(model->key_qvel[2 * model->nv], 0.3);
  EXPECT_EQ(model->key_act[3 * model->na], 0.4);
  EXPECT_THAT(AsVector(model->key_ctrl + 4*model->nu, model->nu),
              ElementsAre(0.5, 0.6));
  EXPECT_THAT(AsVector(model->key_mpos + 3*model->nmocap*5, 3),
              ElementsAre(.1, .2, .3));
  EXPECT_THAT(AsVector(model->key_mquat + 4*model->nmocap*6, 4),
              ElementsAre(.5, .5, .5, .5));
  mj_deleteModel(model);
}

TEST_F(KeyframeTest, ResetDataKeyframe) {
  const std::string xml_path = GetTestDataFilePath(kKeyframePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  mj_resetDataKeyframe(model, data, 0);
  EXPECT_EQ(data->time, 0.1);

  mj_resetDataKeyframe(model, data, 1);
  EXPECT_EQ(data->qpos[0], 0.2);

  mj_resetDataKeyframe(model, data, 2);
  EXPECT_EQ(data->qvel[0], 0.3);

  mj_resetDataKeyframe(model, data, 3);
  EXPECT_EQ(data->act[0], 0.4);

  mj_resetDataKeyframe(model, data, 4);
  EXPECT_EQ(data->ctrl[0], 0.5);
  EXPECT_EQ(data->ctrl[1], 0.6);

  mj_resetDataKeyframe(model, data, 5);
  EXPECT_THAT(AsVector(data->mocap_pos, 3), ElementsAre(.1, .2, .3));

  mj_resetDataKeyframe(model, data, 6);
  EXPECT_THAT(AsVector(data->mocap_quat, 4), ElementsAre(.5, .5, .5, .5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(KeyframeTest, BadSize) {
  static constexpr char xml[] = R"(
  <mujoco>
    <keyframe>
      <key qpos="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = LoadModelFromString(xml, error, error_sz);
  EXPECT_THAT(model, ::testing::IsNull());
  EXPECT_THAT(error, HasSubstr("invalid qpos size, expected length 0"));
}

// ------------- test relative frame sensor compilation-------------------------

using RelativeFrameSensorParsingTest = MujocoTest;

TEST_F(RelativeFrameSensorParsingTest, RefTypeNotRequired) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  mj_deleteModel(model);
}

TEST_F(RelativeFrameSensorParsingTest, ReferenceBodyFound) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="xbody" objname="sensorized"
                reftype="xbody" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml, 0, 0);
  ASSERT_THAT(model, NotNull());
  ASSERT_EQ(model->sensor_reftype[0], mjOBJ_XBODY);
  ASSERT_EQ(model->sensor_refid[0], 1);
  mj_deleteModel(model);
}

TEST_F(RelativeFrameSensorParsingTest, MissingRefname) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="body" refname=""/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("missing name of reference frame"));
}

TEST_F(RelativeFrameSensorParsingTest, BadRefName) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="body" refname="wrong_name"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("unrecognized name of reference frame"));
}

TEST_F(RelativeFrameSensorParsingTest, BadRefType) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <light name="reference"/>
      <body name="sensorized"/>
    </worldbody>
    <sensor>
      <framepos objtype="body" objname="sensorized"
                reftype="light" refname="reference"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(error.data(), HasSubstr("reference frame object must be"));
}

// ------------- test capsule inertias -----------------------------------------

static const char* const kCapsuleInertiaPath =
    "user/testdata/capsule_inertia.xml";

using MjCGeomTest = MujocoTest;

static constexpr int kSphereBodyId = 1, kCylinderBodyId = 2,
                     kCapsuleBodyId = 3, kCapsuleGeomId = 2;

TEST_F(MjCGeomTest, CapsuleMass) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  // Mass of capsule should equal mass of cylinder + mass of sphere.
  mjtNum sphere_cylinder_mass =
      model->body_mass[kSphereBodyId] + model->body_mass[kCylinderBodyId];
  mjtNum capsule_mass = model->body_mass[kCapsuleBodyId];
  EXPECT_DOUBLE_EQ(sphere_cylinder_mass, capsule_mass);
  mj_deleteModel(model);
}

TEST_F(MjCGeomTest, CapsuleInertiaZ) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  // z-inertia of capsule should equal sphere + cylinder z-inertia.
  mjtNum sphere_cylinder_z_inertia =
      model->body_inertia[3*kSphereBodyId + 2] +
      model->body_inertia[3*kCylinderBodyId + 2];
  mjtNum capsule_z_inertia = model->body_inertia[3*kCapsuleBodyId + 2];
  EXPECT_DOUBLE_EQ(sphere_cylinder_z_inertia, capsule_z_inertia);
  mj_deleteModel(model);
}

TEST_F(MjCGeomTest, CapsuleInertiaX) {
  const std::string xml_path = GetTestDataFilePath(kCapsuleInertiaPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);

  // The CoM of a solid hemisphere is 3/8*radius away from from the disk.
  mjtNum hs_com = model->geom_size[3*kCapsuleGeomId] * 3 / 8;

  // The mass of the two hemispherical end-caps is just the mass of the sphere.
  mjtNum sphere_mass = model->body_mass[1];

  // x-inertia of capsule should equal sphere + cylinder x-inertias, with
  // corrections from shifting the hemispheres using parallel axis theorem.
  mjtNum sphere_cylinder_x_inertia = model->body_inertia[3*kSphereBodyId] +
                                     model->body_inertia[3*kCylinderBodyId];

  // Parallel axis-theorem #1: translate the hemispheres in to the origin.
  mjtNum translate_in = hs_com;
  sphere_cylinder_x_inertia -= sphere_mass * translate_in*translate_in;

  // Parallel axis-theorem #2: translate the hemispheres out to the end caps.
  mjtNum cylinder_half_length = model->geom_size[3*kCapsuleGeomId + 1];
  mjtNum translate_out = cylinder_half_length + hs_com;
  sphere_cylinder_x_inertia += sphere_mass * translate_out*translate_out;

  // Compare native capsule inertia and computed inertia.
  mjtNum capsule_x_inertia = model->body_inertia[3*kCapsuleBodyId];
  EXPECT_DOUBLE_EQ(sphere_cylinder_x_inertia, capsule_x_inertia);
  mj_deleteModel(model);
}

// ------------- test quaternion normalization----------------------------------

using QuatNorm = MujocoTest;

TEST_F(QuatNorm, QuatNotNormalized) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site quat="1 2 2 4"/>
      <camera quat="1 2 2 4"/>
      <body quat="1 2 2 4">
        <geom quat="1 2 2 4" size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(AsVector(m->body_quat+4, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->geom_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->site_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  EXPECT_THAT(AsVector(m->cam_quat, 4), ElementsAre(1./5, 2./5, 2./5, 4./5));
  mj_deleteModel(m);
}

// ------------- test actuator order -------------------------------------------

using ActuatorTest = MujocoTest;

TEST_F(ActuatorTest, BadOrder) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="filter"/>
      <general joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  size_t error_sz = 1024;
  mjModel* model = LoadModelFromString(xml, error, error_sz);
  EXPECT_THAT(model, ::testing::IsNull());
  EXPECT_THAT(error, HasSubstr("stateless actuators must come before"));
}


// ------------- test actlimited and actrange fields ---------------------------

using ActRangeTest = MujocoTest;

TEST_F(ActRangeTest, ActRangeParsed) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true" actrange="-1 1.5"/>
    </actuator>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml, nullptr, 0);
  EXPECT_EQ(m->actuator_actlimited[0], 1);
  EXPECT_EQ(m->actuator_actrange[0], -1);
  EXPECT_EQ(m->actuator_actrange[1], 1.5);

  mj_deleteModel(m);
}

TEST_F(ActRangeTest, ActRangeBad) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true" actrange="1 -1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid activation range"));
}

TEST_F(ActRangeTest, ActRangeUndefined) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general dyntype="integrator" joint="hinge" actlimited="true"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("invalid activation range"));
}

TEST_F(ActRangeTest, ActRangeNoDyntype) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="hinge"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("actrange specified but dyntype is 'none'"));
}

TEST_F(ActRangeTest, ActRangeDefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <default>
      <general dyntype="integrator" actlimited="true" actrange="-1 1"/>
      <default class="dclass">
        <general actlimited="false" actrange="2 3"/>
      </default>
    </default>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide"/>
      <general joint="slide" class="dclass"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());

  // first actuator
  EXPECT_THAT(model->actuator_actlimited[0], 1);
  EXPECT_THAT(model->actuator_actrange[0], -1);
  EXPECT_THAT(model->actuator_actrange[1], 1);

  // // second actuator
  EXPECT_THAT(model->actuator_actlimited[1], 0);
  EXPECT_THAT(model->actuator_actrange[2], 2);
  EXPECT_THAT(model->actuator_actrange[3], 3);

  mj_deleteModel(model);
}

// ------------- test nuser_xxx fields -----------------------------------------

using UserDataTest = MujocoTest;

TEST_F(UserDataTest, NBodyTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_body="2"/>
    <worldbody>
      <body user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_body"));
}

TEST_F(UserDataTest, NJointTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_jnt="2"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint user="1 2 3"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_jnt"));
}

TEST_F(UserDataTest, NGeomTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_geom="2"/>
    <worldbody>
      <geom size="1" user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_geom"));
}

TEST_F(UserDataTest, NSiteTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_site="2"/>
    <worldbody>
      <site user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_site"));
}

TEST_F(UserDataTest, NCameraTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_cam="2"/>
    <worldbody>
      <camera user="1 2 3"/>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_cam"));
}

TEST_F(UserDataTest, NTendonTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_tendon="2"/>
    <worldbody>
      <site name="a"/>
      <site name="b"/>
    </worldbody>
    <tendon>
      <spatial user="1 2 3">
        <site site="a"/>
        <site site="b"/>
      </spatial>
    </tendon>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_tendon"));
}

TEST_F(UserDataTest, NActuatorTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_actuator="2"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a" user="1 2 3"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_actuator"));
}

TEST_F(UserDataTest, NSensorTooSmall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <size nuser_sensor="2"/>
    <worldbody>
      <site name="a"/>
    </worldbody>
    <sensor>
      <accelerometer site="a" user="1 2 3"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("nuser_sensor"));
}

// ------------- test for auto parsing of *limited fields -------------

using LimitedTest = MujocoTest;

constexpr char kKeyAutoLimits[] = "user/testdata/auto_limits.xml";

// check joint limit values when automatically inferred based on range
TEST_F(LimitedTest, JointLimited) {
  const std::string xml_path = GetTestDataFilePath(kKeyAutoLimits);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());

  // see `user/testdata/auto_limits.xml` for expected values
  for (int i=0; i < model->njnt; i++) {
    EXPECT_EQ(model->jnt_limited[i], (mjtByte)model->jnt_user[i]);
  }

  mj_deleteModel(model);
}

TEST_F(LimitedTest, ErrorIfLimitedMissingOnJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint user="1" range="0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("limited"));
}

TEST_F(LimitedTest, ExplicitLimitedFalseIsOk) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint user="1" limited="false" range="0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();

  EXPECT_EQ(model->jnt_limited[0], 0);
  mj_deleteModel(model);
}

TEST_F(LimitedTest, ErrorIfLimitedMissingOnTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint type="slide"/>
        <geom size="1"/>
        <site name="s1"/>
      </body>
      <site name="s2"/>
    </worldbody>
    <tendon>
      <spatial range="-1 1">
        <site site="s1"/>
        <site site="s2"/>
      </spatial>
    </tendon>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("limited"));
  EXPECT_THAT(error.data(), HasSubstr("tendon"));
}

TEST_F(LimitedTest, ErrorIfForceLimitedMissingOnActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="false"/>
    <worldbody>
      <body>
        <joint type="slide" name="J1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="J1" forcerange="0 100"/>
    </actuator>
  </mujoco>

  )";
  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(error.data(), HasSubstr("forcelimited"));
  EXPECT_THAT(error.data(), HasSubstr("forcerange"));
  EXPECT_THAT(error.data(), HasSubstr("actuator"));
}

}  // namespace
}  // namespace mujoco
