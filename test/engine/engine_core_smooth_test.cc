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

// Tests for engine/engine_core_smooth.c.

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
using CoreSmoothTest = MujocoTest;

static std::vector<mjtNum> GetVector(const mjtNum* array, int length) {
  return std::vector<mjtNum>(array, array + length);
}

// --------------------------- mj_kinematics -----------------------------------

TEST_F(CoreSmoothTest, MjKinematicsWorldXipos) {
  constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
    </worldbody>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, testing::NotNull());
  mjData* data = mj_makeData(model);

  mj_resetDataDebug(model, data, 'd');
  mj_kinematics(model, data);
  EXPECT_THAT(GetVector(&data->xipos[0], 3), ElementsAre(0, 0, 0));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// --------------------------- connect constraint ------------------------------

TEST_F(CoreSmoothTest, RnePostConnectForceSlide) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/connect/force_slide.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(data->sensordata[i], model->sensor_user[i], 1e-6);
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostConnectForceSlideRotated) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/connect/force_slide_rotated.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(data->sensordata[i], model->sensor_user[i], 1e-6);
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostConnectForceFree) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/connect/force_free.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(data->sensordata[i], model->sensor_user[i], 1e-6);
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostConnectTorque) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/connect/torque_free.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(data->sensordata[i], model->sensor_user[i], 1e-6);
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostConnectMultipleConstraints) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/connect/multiple_constraints.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int i=0; i < 3; i++) {
    EXPECT_NEAR(data->sensordata[i], model->sensor_user[i], 1e-6);
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


// --------------------------- weld constraint ---------------------------------

TEST_F(CoreSmoothTest, RnePostWeldForceFree) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/weld/force_free.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int sensor_index=0; sensor_index < model->nsensor; sensor_index++) {
    for (int i=0; i < 3; i++) {
      EXPECT_NEAR(
          data->sensordata[model->sensor_adr[sensor_index] + i],
          model->sensor_user[model->nuser_sensor*sensor_index + i],
          1e-6);
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostWeldForceFreeRotatoed) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/weld/force_free_rotated.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int sensor_index=0; sensor_index < model->nsensor; sensor_index++) {
    for (int i=0; i < 3; i++) {
      EXPECT_NEAR(
          data->sensordata[model->sensor_adr[sensor_index] + i],
          model->sensor_user[model->nuser_sensor*sensor_index + i],
          1e-6);
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFree) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int sensor_index=0; sensor_index < model->nsensor; sensor_index++) {
    for (int i=0; i < 3; i++) {
      EXPECT_NEAR(
          data->sensordata[model->sensor_adr[sensor_index] + i],
          model->sensor_user[model->nuser_sensor*sensor_index + i],
          1e-6);
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFreeRotated) {
  static const char* const kModelFilePath =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free_rotated.xml";
  const std::string xml_path = GetTestDataFilePath(kModelFilePath);
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  mjData* data = mj_makeData(model);
  // settle physics:
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  for (int sensor_index=0; sensor_index < model->nsensor; sensor_index++) {
    for (int i=0; i < 3; i++) {
      EXPECT_NEAR(
          data->sensordata[model->sensor_adr[sensor_index] + i],
          model->sensor_user[model->nuser_sensor*sensor_index + i],
          1e-6);
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}


// ----------------------------- fluidshape --------------------------------

using EllipsoidFluidTest = MujocoTest;

TEST_F(EllipsoidFluidTest, GeomsEquivalentToBodies) {
  static constexpr char two_bodies_xml[] = R"(
  <mujoco>
    <option wind="5 5 0" density="10"/>
    <worldbody>
      <body>
        <freejoint/>
        <body>
          <geom type="box" size=".1 .01 0.01" pos="0.1 0 0" euler="40 0 0" fluidshape="ellipsoid"/>
        </body>
        <body>
          <geom type="box" size=".1 .01 0.01" pos="-.1 0 0" euler="0 20 0" fluidshape="ellipsoid"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  mjModel* m2 = LoadModelFromString(two_bodies_xml);
  mjData* d2 = mj_makeData(m2);
  for (int i = 0; i < 6; i++) {
    d2->qvel[i] = (mjtNum) i+1;
  }
  d2->qpos[3] = 0.5;
  d2->qpos[4] = 0.5;
  d2->qpos[5] = 0.5;
  d2->qpos[6] = 0.5;

  static constexpr char one_body_xml[] = R"(
  <mujoco>
    <option wind="5 5 0" density="10"/>
    <worldbody>
      <body pos="1 2 3">
        <freejoint/>
        <geom type="box" size=".1 .01 0.01" pos="0.1 0 0" euler="40 0 0" fluidshape="ellipsoid"/>
        <geom type="box" size=".1 .01 0.01" pos="-.1 0 0" euler="0 20 0" fluidshape="ellipsoid"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  mjModel* m1 = LoadModelFromString(one_body_xml);
  mjData* d1 = mj_makeData(m1);
  for (int i = 0; i < 6; i++) {
    d1->qvel[i] = (mjtNum) i+1;
  }
  d1->qpos[3] = 0.5;
  d1->qpos[4] = 0.5;
  d1->qpos[5] = 0.5;
  d1->qpos[6] = 0.5;

  const mjtNum tol = 1e-14;  // tolerance for floating point numbers

  EXPECT_EQ(m1->nv, m2->nv);

  mj_forward(m2, d2);
  mj_forward(m1, d1);
  for (int i = 0; i < m1->nv; i++) {
    EXPECT_NEAR(d2->qfrc_passive[i], d1->qfrc_passive[i], tol);
  }

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(EllipsoidFluidTest, DefaultsPropagate) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option wind="5 5 0" density="10"/>
    <default>
      <geom fluidshape="ellipsoid" fluidcoef="2 3 4 5 6"/>
      <default class="test_class">
        <geom fluidshape="none" fluidcoef="5 4 3 2 1"/>
      </default>
    </default>
    <worldbody>
      <body>
        <freejoint/>
        <geom type="box" size=".1 .01 0.01" pos="0.1 0 0" class="test_class"/>
        <geom type="box" size=".1 .01 0.01" pos="-0.1 0 0"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml);
  EXPECT_THAT(GetVector(model->geom_fluid, 6),
              ElementsAre(0, 0, 0, 0, 0, 0));
  EXPECT_THAT(GetVector(model->geom_fluid + mjNFLUID, 6),
              ElementsAre(1, 2, 3, 4, 5, 6));
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
