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
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/strings/str_format.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;
using ::testing::Pointwise;

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

TEST_F(UserCModelTest, SameFrame) {
  static constexpr char xml[] = R"(
   <mujoco>
     <default>
      <geom type="box" size="1 2 3"/>
     </default>

     <worldbody>
       <body name="body1">
         <geom name="none"       mass="0" pos="1 1 1" euler="10 10 10"/>
         <geom name="body"       mass="0"/>
         <geom name="inertia"    mass="1" pos="3 2 1" euler="20 30 40"/>
         <geom name="bodyrot"    mass="0" pos="1 1 1"/>
         <geom name="inertiarot" mass="0" euler="20 30 40"/>
        </body>
      </worldbody>
    </mujoco>)";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(model, NotNull()) << error.data();
  EXPECT_EQ(model->geom_sameframe[0], mjSAMEFRAME_NONE);
  EXPECT_EQ(model->geom_sameframe[1], mjSAMEFRAME_BODY);
  EXPECT_EQ(model->geom_sameframe[2], mjSAMEFRAME_INERTIA);
  EXPECT_EQ(model->geom_sameframe[3], mjSAMEFRAME_BODYROT);
  EXPECT_EQ(model->geom_sameframe[4], mjSAMEFRAME_INERTIAROT);

  // make data, get geom_xpos
  mjData* data = mj_makeData(model);
  mj_kinematics(model, data);
  auto geom_xpos = AsVector(data->geom_xpos, model->ngeom*3);
  auto geom_xmat = AsVector(data->geom_xmat, model->ngeom*9);

  // set all geom_sameframe to 0, call kinematics again
  for (int i = 0; i < model->ngeom; i++) {
    model->geom_sameframe[i] = mjSAMEFRAME_NONE;
  }
  mj_resetData(model, data);
  mj_kinematics(model, data);
  auto geom_xpos2 = AsVector(data->geom_xpos, model->ngeom*3);
  auto geom_xmat2 = AsVector(data->geom_xmat, model->ngeom*9);

  // expect them to be equal
  constexpr double eps = 1e-6;
  EXPECT_THAT(geom_xpos, Pointwise(DoubleNear(eps), geom_xpos2));
  EXPECT_THAT(geom_xmat, Pointwise(DoubleNear(eps), geom_xmat2));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(UserCModelTest, ActuatorSparsity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="a"/>
        <body>
          <geom size="1"/>
          <joint name="b"/>
        </body>
      </body>
    </worldbody>
    <actuator>
      <motor joint="a"/>
      <motor joint="b"/>
    </actuator>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_EQ(m->nJmom, 2);
  mj_deleteModel(m);
}

TEST_F(UserCModelTest, NestedZeroMassBodiesOK) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <body>
          <body>
            <body>
              <geom size="1"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

TEST_F(UserCModelTest, NestedZeroMassBodiesWithJointOK) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <freejoint/>
        <body>
          <body>
            <body>
              <joint/>
              <geom size="1"/>
            </body>
            <body>
              <geom size="1"/>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mj_deleteModel(model);
}

TEST_F(UserCModelTest, NestedZeroMassBodiesFail) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <body>
          <freejoint/>
          <body>
            <body>
            </body>
          </body>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, IsNull());
  EXPECT_THAT(
      error,
      HasSubstr(
          "mass and inertia of moving bodies must be larger than mjMINVAL"));
  mj_deleteModel(model);
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
  static const char* const kFilePath = "user/testdata/malformed_duplicated.xml";
  const std::string xml_path = GetTestDataFilePath(kFilePath);

  std::array<char, 1024> error;
  mjModel* m = mj_loadXML(xml_path.c_str(), 0, error.data(), error.size());

  EXPECT_THAT(m, IsNull());
  EXPECT_STREQ(error.data(), "Error: repeated name 'cube' in mesh");
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
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body>
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
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

  EXPECT_EQ(m_no_fuse->body_contype[2], 1);
  EXPECT_EQ(m_no_fuse->body_conaffinity[2], 1);
  EXPECT_EQ(m_fuse->body_contype[1], 1);
  EXPECT_EQ(m_fuse->body_conaffinity[1], 1);

  EXPECT_EQ(m_no_fuse->body_bvhnum[2], 3);
  EXPECT_EQ(m_fuse->body_bvhnum[1], 3);

  mjData* d_fuse = mj_makeData(m_fuse);
  mjData* d_no_fuse = mj_makeData(m_no_fuse);

  mj_step(m_fuse, d_fuse);
  mj_step(m_no_fuse, d_no_fuse);

  EXPECT_THAT(d_fuse->qvel[0], DoubleNear(d_no_fuse->qvel[0], 2e-17))
      << "Velocity should be the same after 1 step";
  EXPECT_NE(d_fuse->qvel[0], 0);

  mj_deleteData(d_fuse);
  mj_deleteData(d_no_fuse);
  mj_deleteModel(m_fuse);
  mj_deleteModel(m_no_fuse);
}

TEST_F(FuseStaticTest, FuseStaticActuatorReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>

    <actuator>
      <adhesion body="referenced" ctrlrange="0 1"/>
    </actuator>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
  mj_deleteModel(m);
}

TEST_F(FuseStaticTest, FuseStaticLightReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <light mode="targetbody" target="referenced"/>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
  mj_deleteModel(m);
}

TEST_F(FuseStaticTest, FuseStaticForceSensorReferencedBody) {
  static constexpr char xml_template[] = R"(
  <mujoco>
    <compiler fusestatic="true"/>

    <worldbody>
      <body>
        <joint axis="1 0 0"/>
        <geom size="0.5" pos="1 0 0" contype="0" conaffinity="0"/>
        <body name="not_referenced">
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
        <body name="referenced">
          <site name="force"/>
          <geom size="0.5" pos="0 1 0" contype="1" conaffinity="1"/>
          <geom size="0.5" pos="0 -2 0" contype="1" conaffinity="1"/>
        </body>
      </body>
    </worldbody>

    <sensor>
      <force site="force"/>
    </sensor>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml_template, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  EXPECT_EQ(m->nbody, 3) << "Expecting a world body and two others";
  mj_deleteModel(m);
}

// ------------- test discardvisual --------------------------------------------

using DiscardVisualTest = MujocoTest;
TEST_F(DiscardVisualTest, DiscardVisualKeepsInertia) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler discardvisual="true"/>

    <asset>
      <mesh name="visual_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 1  0 3 2" />

      <mesh name="collision_mesh"
        vertex="0 0 0  1 0 0  0 1 0  0 0 1"
        normal="1 0 0  0 1 0  0 0 1  0.707 0 0.707"
        face="0 2 1  0 3 2" />
    </asset>

    <worldbody>
      <body>
        <geom type="mesh" mesh="visual_mesh" contype="0" conaffinity="0"/>
      </body>
      <body>
        <geom type="mesh" mesh="collision_mesh"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  std::array<char, 1024> error;
  mjModel* model = LoadModelFromString(xml, error.data(), error.size());
  EXPECT_THAT(model, NotNull()) << error.data();
  EXPECT_THAT(model->nmesh, 1);
  EXPECT_THAT(model->body_inertia[3], model->body_inertia[6]);
  EXPECT_THAT(model->body_inertia[4], model->body_inertia[7]);
  EXPECT_THAT(model->body_inertia[5], model->body_inertia[8]);
  mj_deleteModel(model);
}

TEST_F(DiscardVisualTest, DiscardVisualEquivalent) {
  char error[1024];
  size_t error_sz = 1024;

  static const char* const kDiscardvisualPath =
      "user/testdata/discardvisual.xml";
  static const char* const kDiscardvisualFalsePath =
      "user/testdata/discardvisual_false.xml";

  const std::string xml_path1 = GetTestDataFilePath(kDiscardvisualPath);
  mjModel* model1 = mj_loadXML(xml_path1.c_str(), 0, error, error_sz);
  EXPECT_THAT(model1, NotNull()) << error;

  const std::string xml_path2 = GetTestDataFilePath(kDiscardvisualFalsePath);
  mjModel* model2 = mj_loadXML(xml_path2.c_str(), 0, error, error_sz);
  EXPECT_THAT(model2, NotNull()) << error;

  EXPECT_THAT(model1->nq, model2->nq);
  EXPECT_THAT(model1->nmat, 0);
  EXPECT_THAT(model1->ntex, 0);
  EXPECT_THAT(model2->ngeom-model1->ngeom, 3);
  EXPECT_THAT(model2->nmesh-model1->nmesh, 2);
  EXPECT_THAT(model1->npair, model2->npair);
  EXPECT_THAT(model1->nsensor, model2->nsensor);
  EXPECT_THAT(model1->nwrap, model2->nwrap);

  for (int i = 0; i < model1->ngeom; i++) {
    std::string name = std::string(model1->names + model1->name_geomadr[i]);
    EXPECT_NE(name.find("kept"), std::string::npos);
    EXPECT_EQ(name.find("discard"), std::string::npos);
  }

  for (int i = 0; i < model1->npair; i++) {
    int adr1 = model1->name_geomadr[model1->pair_geom1[i]];
    int adr2 = model2->name_geomadr[model2->pair_geom1[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
    adr1 = model1->name_geomadr[model1->pair_geom2[i]];
    adr2 = model2->name_geomadr[model2->pair_geom2[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  for (int i = 0; i < model1->nsensor; i++) {
    int adr1 = model1->name_geomadr[model1->sensor_objid[i]];
    int adr2 = model2->name_geomadr[model2->sensor_objid[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  for (int i = 0; i < model1->nwrap; i++) {
    int adr1 = model1->name_geomadr[model1->wrap_objid[i]];
    int adr2 = model2->name_geomadr[model2->wrap_objid[i]];
    EXPECT_STREQ(model1->names + adr1, model2->names + adr2);
  }

  mjData *d1 = mj_makeData(model1);
  mjData *d2 = mj_makeData(model2);
  for (int i = 0; i < 100; i++) {
    mj_step(model1, d1);
    mj_step(model2, d2);
  }

  for (int i = 0; i < model1->nq; i++) {
    EXPECT_THAT(d1->qpos[i], d2->qpos[i]);
  }

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteData(d1);
  mj_deleteData(d2);
}

// ------------- test lengthrange ----------------------------------------------

using LengthRangeTest = MujocoTest;

TEST_F(LengthRangeTest, LengthRangeThreading) {
  char error[1024];
  size_t error_sz = 1024;
  std::string field = "";

  static const char* const kLengthrangePath =
      "user/testdata/lengthrange.xml";

  const std::string xml_path1 = GetTestDataFilePath(kLengthrangePath);
  mjSpec* spec = mj_parseXML(xml_path1.c_str(), 0, error, error_sz);
  EXPECT_THAT(spec, NotNull()) << error;
  mjModel* model1 = mj_compile(spec, 0);
  EXPECT_THAT(model1, NotNull()) << error;

  // model is such that the lengthrange for first actuator is [1, sqrt(5)]
  EXPECT_THAT(model1->actuator_lengthrange[0], DoubleNear(1.0, 1e-3));
  EXPECT_THAT(model1->actuator_lengthrange[1],
              DoubleNear(std::sqrt(5.0), 1e-3));

  // recompile without threads
  ASSERT_EQ(spec->compiler.usethread, 1);
  spec->compiler.usethread = 0;
  mjModel* model2 = mj_compile(spec, 0);
  EXPECT_THAT(model2, NotNull()) << error;

  // expect threaded and unthreaded models to be identical
  EXPECT_LE(CompareModel(model1, model2, field), 0)
      << "Threaded and unthreaded lengthrange models are different!\n"
      << "Different field: " << field << '\n';

  mj_deleteModel(model1);
  mj_deleteModel(model2);
  mj_deleteSpec(spec);
}

// ----------------------------- test modeldir  --------------------------------

TEST_F(MujocoTest, Modeldir) {
  static constexpr char cube[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "meshdir/cube.obj", cube, sizeof(cube));

  // child with the asset
  mjSpec* child = mj_makeSpec();
  mjsMesh* mesh = mjs_addMesh(child, 0);
  mjsFrame* frame = mjs_addFrame(mjs_findBody(child, "world"), 0);
  mjsGeom* geom = mjs_addGeom(mjs_findBody(child, "world"), 0);
  mjs_setString(child->meshdir, "meshdir");
  mjs_setString(mesh->file, "cube.obj");
  mjs_setString(mesh->name, "cube");
  mjs_setString(geom->meshname, "cube");
  mjs_setFrame(geom->element, frame);
  geom->type = mjGEOM_MESH;

  // parent attaching the child
  mjSpec* spec = mj_makeSpec();
  mjs_setDeepCopy(spec, true);
  mjs_setString(spec->meshdir, "asset");
  mjs_attach(mjs_findBody(spec, "world")->element, frame->element, "_", "");
  mjModel* model = mj_compile(spec, vfs.get());
  EXPECT_THAT(model, NotNull());

  mj_deleteSpec(child);
  mj_deleteSpec(spec);
  mj_deleteModel(model);
  mj_deleteVFS(vfs.get());
}

TEST_F(MujocoTest, NestedMeshDir) {
  static constexpr char cube[] = R"(
  v -1 -1  1
  v  1 -1  1
  v -1  1  1
  v  1  1  1
  v -1  1 -1
  v  1  1 -1
  v -1 -1 -1
  v  1 -1 -1)";

  static constexpr char child_xml[] = R"(
  <mujoco>
    <compiler meshdir="child_meshdir"/>

    <asset>
      <mesh name="m" file="child_mesh.obj"/>
    </asset>

    <worldbody>
      <body name="child">
        <geom type="mesh" mesh="m"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char parent_xml[] = R"(
  <mujoco>
    <compiler meshdir="parent_meshdir"/>

    <asset>
      <mesh name="m" file="parent_mesh.obj"/>
      <model name="child" file="child.xml"/>
    </asset>

    <worldbody>
      <body name="parent">
        <geom type="mesh" mesh="m"/>
        <attach model="child" body="child" prefix="child_"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  static constexpr char grandparent_xml[] = R"(
  <mujoco>
    <compiler meshdir="grandparent_meshdir"/>

    <asset>
      <mesh name="m" file="grandparent_mesh.obj"/>
      <model name="parent" file="parent.xml"/>
    </asset>

    <worldbody>
      <geom type="mesh" mesh="m"/>
      <attach model="parent" body="parent" prefix="parent_"/>
    </worldbody>
  </mujoco>
  )";

  auto vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(vfs.get());
  mj_addBufferVFS(vfs.get(), "child_meshdir/child_mesh.obj", cube,
                  sizeof(cube));
  mj_addBufferVFS(vfs.get(), "child.xml", child_xml, sizeof(child_xml));
  mj_addBufferVFS(vfs.get(), "parent_meshdir/parent_mesh.obj", cube,
                  sizeof(cube));
  mj_addBufferVFS(vfs.get(), "parent.xml", parent_xml, sizeof(parent_xml));
  mj_addBufferVFS(vfs.get(), "grandparent_meshdir/grandparent_mesh.obj", cube,
                  sizeof(cube));

  std::array<char, 1024> error;
  mjModel* child_model = LoadModelFromString(child_xml, error.data(),
                                             error.size(), vfs.get());
  EXPECT_THAT(child_model, NotNull()) << error.data();
  mj_deleteModel(child_model);

  mjModel* parent_model = LoadModelFromString(parent_xml, error.data(),
                                              error.size(), vfs.get());
  EXPECT_THAT(parent_model, NotNull()) << error.data();
  mj_deleteModel(parent_model);

  mjModel* grandparent_model = LoadModelFromString(
      grandparent_xml, error.data(), error.size(), vfs.get());
  EXPECT_THAT(grandparent_model, NotNull()) << error.data();
  mj_deleteModel(grandparent_model);

  mj_deleteVFS(vfs.get());
}

TEST_F(MujocoTest, ConvertSpringdamper) {
  static constexpr char xml[] = R"(
    <mujoco>
    <worldbody>
      <body>
        <joint axis="0 1 0" springdamper="1 1"/>
        <geom size="0.2 0.2 0.2" type="box"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> err;
  mjSpec* spec = mj_parseXMLString(xml, 0, err.data(), err.size());
  ASSERT_THAT(spec, NotNull()) << err.data();
  mjModel* model = mj_compile(spec, 0);
  ASSERT_THAT(model, NotNull()) << err.data();
  std::array<char, 1024> str;
  mj_saveXMLString(spec, str.data(), str.size(), err.data(), err.size());
  EXPECT_THAT(str.data(), HasSubstr("damping"));
  EXPECT_THAT(str.data(), HasSubstr("stiffness"));
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
