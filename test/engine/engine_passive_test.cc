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
using ::testing::NotNull;
using CoreSmoothTest = MujocoTest;

static std::vector<mjtNum> GetVector(const mjtNum* array, int length) {
  return std::vector<mjtNum>(array, array + length);
}


// ------------------------ ellipsoid fluid model ------------------------------

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
        <freejoint align="false"/>
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


// ------------------------------ tendons --------------------------------------

using TendonTest = MujocoTest;

// check tendon spring deadband using example model
TEST_F(TendonTest, SpringrangeDeadband) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/tendon_springlength.xml");
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data = mj_makeData(model);

  // initial state outside deadband: spring is active
  mj_forward(model, data);
  mjtNum expected_force = model->tendon_stiffness[0] *
      (model->tendon_lengthspring[1] - data->ten_length[0]);
  EXPECT_EQ(expected_force, data->qfrc_passive[0]);

  // put body inside deadband: spring is inactive
  data->qpos[0] = -1;
  mj_forward(model, data);
  EXPECT_EQ(0, data->qfrc_passive[0]);

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
