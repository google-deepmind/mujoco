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
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_spatial.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::NotNull;
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
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  mj_resetDataDebug(model, data, 'd');
  mj_kinematics(model, data);
  EXPECT_THAT(GetVector(&data->xipos[0], 3), ElementsAre(0, 0, 0));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// --------------------------- connect constraint ------------------------------

// test that bodies hanging on connects lead to expected force sensor readings
void TestConnect(const char* const filepath) {
  const std::string xml_path = GetTestDataFilePath(filepath);
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


TEST_F(CoreSmoothTest, RnePostConnectForceSlide) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_slide.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostConnectForceSlideRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_slide_rotated.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostConnectForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/force_free.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostConnectTorque) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/torque_free.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostConnectMultipleConstraints) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/connect/multiple_constraints.xml";
  TestConnect(kModelFilePath);
}


// --------------------------- weld constraint ---------------------------------

// test that bodies attached with welds lead to expected force sensor readings
void TestWeld(const char* const filepath) {
  const std::string xml_path = GetTestDataFilePath(filepath);
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


TEST_F(CoreSmoothTest, RnePostWeldForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_free.xml";
  TestWeld(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostWeldForceFreeRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_free_rotated.xml";
  TestWeld(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free.xml";
  TestWeld(kModelFilePath);
}


TEST_F(CoreSmoothTest, RnePostWeldForceTorqueFreeRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/force_torque_free_rotated.xml";
  TestWeld(kModelFilePath);
}


TEST_F(CoreSmoothTest, WeldRatioForceFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_force_free.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, WeldRatioForceSlide) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_force_slide.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, WeldRatioTorqueFree) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_torque_free.xml";
  TestConnect(kModelFilePath);
}


TEST_F(CoreSmoothTest, WeldRatioForceSlideRotated) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_force_slide_rotated.xml";
  TestConnect(kModelFilePath);
}

TEST_F(CoreSmoothTest, WeldRatioMultipleConstraints) {
  constexpr char kModelFilePath[] =
      "engine/testdata/core_smooth/rne_post/weld/tfratio0_multiple_constraints.xml";
  TestConnect(kModelFilePath);
}

// --------------------------- site actuators ----------------------------------

// Test Cartesian position control using site transmission with refsite
TEST_F(CoreSmoothTest, RefsiteBringsToPose) {
  constexpr char kRefsitePath[] = "engine/testdata/refsite.xml";
  const std::string xml_path = GetTestDataFilePath(kRefsitePath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, 0, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  // set pose target in ctrl (3 positions, 3 rotations)
  mjtNum targetpos[] = {.01, .02, .03};
  mjtNum targetrot[] = {.1, .2, .3};
  mju_copy3(data->ctrl, targetpos);
  mju_copy3(data->ctrl+3, targetrot);

  // step for 5 seconds
  while (data->time < 5) {
    mj_step(model, data);
  }

  // get site IDs
  int refsite_id = mj_name2id(model, mjOBJ_SITE, "reference");
  int site_id = mj_name2id(model, mjOBJ_SITE, "end_effector");

  // check that position matches target to within 1e-5 length units
  double tol_pos = 1e-5;
  mjtNum relpos[3];
  mju_sub3(relpos, data->site_xpos+3*site_id, data->site_xpos+3*refsite_id);
  EXPECT_THAT(relpos, Pointwise(DoubleNear(tol_pos), targetpos));

  // check that orientation matches target to within 1e-3 radians
  double tol_rot = 1e-3;
  mjtNum site_xquat[4], refsite_xquat[4], relrot[3];
  mju_mat2Quat(refsite_xquat, data->site_xmat+9*refsite_id);
  mju_mat2Quat(site_xquat, data->site_xmat+9*site_id);
  mju_subQuat(relrot, site_xquat, refsite_xquat);
  EXPECT_THAT(relrot, Pointwise(DoubleNear(tol_rot), targetrot));

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
