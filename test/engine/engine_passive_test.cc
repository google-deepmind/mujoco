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

#include <limits>
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

using PassiveTest = MujocoTest;

TEST_F(PassiveTest, DisableFlags) {
  static constexpr char flex_xml[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body gravcomp="1">
        <joint type="slide" springref="1" stiffness="10" damping="1"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <keyframe>
      <key qvel="-1"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(flex_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);
  EXPECT_FLOAT_EQ(d->qacc[0], 11);

  m->opt.disableflags = mjDSBL_DAMPER;
  mj_forward(m, d);
  EXPECT_FLOAT_EQ(d->qacc[0], 10);

  m->opt.disableflags = mjDSBL_SPRING;
  mj_forward(m, d);
  EXPECT_FLOAT_EQ(d->qacc[0], 1);

  m->opt.disableflags = mjDSBL_SPRING | mjDSBL_DAMPER;
  mj_forward(m, d);
  EXPECT_EQ(d->qacc[0], -10);

  mj_deleteData(d);
  mj_deleteModel(m);
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

  char error[1024];
  mjModel* m2 = LoadModelFromString(two_bodies_xml, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
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

  mjModel* m1 = LoadModelFromString(one_body_xml, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
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

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  EXPECT_THAT(AsVector(model->geom_fluid, 6),
              ElementsAre(0, 0, 0, 0, 0, 0));
  EXPECT_THAT(AsVector(model->geom_fluid + mjNFLUID, 6),
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

// -------------------------------- flex ------------------------------------

using ElasticityTest = MujocoTest;

TEST_F(ElasticityTest, FlexCompatibility) {
  static constexpr char flex_xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="parent">
        <flexcomp name="soft" type="grid" count="3 3 3"
                  radius="0.01" dim="3"mass="1">
            <pin id="2"/>
            <elasticity young="5e4" poisson="0.2"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(flex_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;

  mjData* d = mj_makeData(m);
  mj_deleteData(d);
  mj_deleteModel(m);
}

// -------------------------------- shell -----------------------------------
TEST_F(ElasticityTest, ElasticEnergyShell) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp type="grid" count="8 8 1" spacing="1 1 1"
              radius=".025" name="test" dim="2">
      <elasticity young="2" poisson="0" thickness="1"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);

  // check that a plane is in the kernel of the energy
  for (mjtNum scale = 1; scale < 4; scale++) {
    for (int e = 0; e < m->flex_edgenum[0]; e++) {
      int* edge = m->flex_edge + 2*(m->flex_edgeadr[0] + e);
      int* flap = m->flex_edgeflap + 2*(m->flex_edgeadr[0] + e);
      int v[4] = {edge[0], edge[1], flap[0], flap[1]};
      if (v[3]== -1) {
        continue;
      }
      mjtNum energy = 0;
      mjtNum volume = 1./2.;
      for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
          for (int x = 0; x < 3; x++) {
            mjtNum elongation1 = scale * d->flexvert_xpos[3*v[i]+x];
            mjtNum elongation2 = scale * d->flexvert_xpos[3*v[j]+x];
            energy += m->flex_bending[17*e+4*i+j] * elongation1 * elongation2;
          }
        }
      }
      EXPECT_NEAR(
        4*energy/volume, 0, std::numeric_limits<float>::epsilon());
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(ElasticityTest, CurvedShell) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
  <worldbody>
    <body name="v0" pos="-0.5 -0.5 -0.5">
      <inertial pos="0 0 0" mass="0.125" diaginertia="1e-4 1e-4 1e-4"/>
      <joint axis="1 0 0" type="slide"/>
      <joint axis="0 1 0" type="slide"/>
      <joint axis="0 0 1" type="slide"/>
    </body>
    <body name="v1" pos="-0.5 0.5 -0.5">
      <inertial pos="0 0 0" mass="0.125" diaginertia="1e-4 1e-4 1e-4"/>
      <joint axis="1 0 0" type="slide"/>
      <joint axis="0 1 0" type="slide"/>
      <joint axis="0 0 1" type="slide"/>
    </body>
    <body name="v2" pos="0.5 -0.5 -0.5">
      <inertial pos="0 0 0" mass="0.125" diaginertia="1e-4 1e-4 1e-4"/>
      <joint axis="1 0 0" type="slide"/>
      <joint axis="0 1 0" type="slide"/>
      <joint axis="0 0 1" type="slide"/>
    </body>
    <body name="v3" pos="-0.5 -0.5 0.5">
      <inertial pos="0 0 0" mass="0.125" diaginertia="1e-4 1e-4 1e-4"/>
      <joint axis="1 0 0" type="slide"/>
      <joint axis="0 1 0" type="slide"/>
      <joint axis="0 0 1" type="slide"/>
    </body>
  </worldbody>
  <deformable>
    <flex name="test" radius="0.025" flatskin="true" body="v0 v1 v2 v3"
          element="0 2 3 0 3 1">
      <elasticity young="2" thickness="1" elastic2d="bend"/>
    </flex>
  </deformable>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_kinematics(m, d);
  mj_flex(m, d);
  mj_passive(m, d);

  // v1 force component is in-plane along v1-v0 edge (y-axis)
  EXPECT_NEAR(d->qfrc_spring[3], 0, 1e-6);
  EXPECT_NEAR(d->qfrc_spring[5], 0, 1e-6);

  // v2 force component is in-plane along v2-v0 edge (x-axis)
  EXPECT_NEAR(d->qfrc_spring[7], 0, 1e-6);
  EXPECT_NEAR(d->qfrc_spring[8], 0, 1e-6);

  // v3 force component is in-plane along v3-v0 edge (z-axis)
  EXPECT_NEAR(d->qfrc_spring[9], 0, 1e-6);
  EXPECT_NEAR(d->qfrc_spring[10], 0, 1e-6);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// -------------------------------- membrane -----------------------------------
TEST_F(ElasticityTest, ElasticEnergyMembrane) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp type="grid" count="8 8 1" spacing="1 1 1"
              radius=".025" name="test" dim="2">
      <elasticity young="2" poisson="0" thickness="1" elastic2d="stretch"/>
      <edge equality="false"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  mj_kinematics(m, d);
  mj_flex(m, d);
  mjtNum* metric = m->flex_stiffness + 21 * m->flex_elemadr[0];

  // check that if the entire geometry is rescaled by a factor "scale", then
  // trace(strain^2) = 2*scale^2

  for (mjtNum scale = 1; scale < 4; scale++) {
    for (int t = 0; t < m->flex_elemnum[0]; t++) {
      mjtNum energy = 0;
      mjtNum volume = 1./2.;
      int idx = 0;
      for (int e1 = 0; e1 < 3; e1++) {
        for (int e2 = e1; e2 < 3; e2++) {
          int idx1 = m->flex_elemedge[3*t+e1 + m->flex_elemedgeadr[0]];
          int idx2 = m->flex_elemedge[3*t+e2 + m->flex_elemedgeadr[0]];
          mjtNum elong1 =
              scale * m->flexedge_length0[idx1] * m->flexedge_length0[idx1];
          mjtNum elong2 =
              scale * m->flexedge_length0[idx2] * m->flexedge_length0[idx2];
          energy += metric[21*t+idx++] * elong1 * elong2 * (e1 == e2 ? 1. : 2.);
        }
      }
      EXPECT_NEAR(
        4*energy/volume, 2*scale*scale, std::numeric_limits<float>::epsilon());
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// -------------------------------- solid -----------------------------------
TEST_F(ElasticityTest, ElasticEnergySolid) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
  <worldbody>
    <flexcomp type="grid" count="8 8 8" spacing="1 1 1"
              radius=".025" name="test" dim="3">
      <elasticity young="2" poisson="0"/>
      <edge equality="false"/>
    </flexcomp>
  </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  mj_kinematics(m, d);
  mj_flex(m, d);
  mjtNum* metric = m->flex_stiffness + 21 * m->flex_elemadr[0];

  // check that if the entire geometry is rescaled by a factor "scale", then
  // trace(strain^2) = 3*scale^2

  for (mjtNum scale = 1; scale < 4; scale++) {
    for (int t = 0; t < m->flex_elemnum[0]; t++) {
      mjtNum energy = 0;
      mjtNum volume = 1./6.;
      int idx = 0;
      for (int e1 = 0; e1 < 6; e1++) {
        for (int e2 = e1; e2 < 6; e2++) {
          int idx1 = m->flex_elemedge[6*t+e1 + m->flex_elemedgeadr[0]];
          int idx2 = m->flex_elemedge[6*t+e2 + m->flex_elemedgeadr[0]];
          mjtNum elong1 =
              scale * m->flexedge_length0[idx1] * m->flexedge_length0[idx1];
          mjtNum elong2 =
              scale * m->flexedge_length0[idx2] * m->flexedge_length0[idx2];
          energy += metric[21*t+idx++] * elong1 * elong2 * (e1 == e2 ? 1. : 2.);
        }
      }
      EXPECT_NEAR(
        energy/volume, 3*scale*scale, std::numeric_limits<float>::epsilon());
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
