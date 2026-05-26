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
  EXPECT_MJTNUM_EQ(d->qacc[0], 11);

  m->opt.disableflags = mjDSBL_DAMPER;
  mj_forward(m, d);
  EXPECT_MJTNUM_EQ(d->qacc[0], 10);

  m->opt.disableflags = mjDSBL_SPRING;
  mj_forward(m, d);
  EXPECT_MJTNUM_EQ(d->qacc[0], 1);

  m->opt.disableflags = mjDSBL_SPRING | mjDSBL_DAMPER;
  mj_forward(m, d);
  EXPECT_EQ(d->qacc[0], -10);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, GravcompNestedBody) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body pos="0 0 2">
        <freejoint/>
        <body gravcomp="1.2">
          <geom size="0.2" mass="1"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  mj_forward(m, d);

  EXPECT_GT(d->qacc[2], 0);
  EXPECT_NEAR(d->qacc[2], 2.0, 0.1);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessSlide) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" stiffness="10 5 1"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <keyframe>
      <key qpos="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);
  EXPECT_MJTNUM_EQ(d->qfrc_spring[0], -48);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessAntiSymmetric) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" stiffness="10 5 1"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <keyframe>
      <key qpos="-2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);
  EXPECT_MJTNUM_EQ(d->qfrc_spring[0], 8);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" name="j"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <tendon>
      <fixed>
        <joint joint="j" coef="1"/>
      </fixed>
    </tendon>

    <keyframe>
      <key qpos="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  m->tendon_stiffness[0] = 10;
  m->tendon_stiffnesspoly[0] = 5;
  m->tendon_stiffnesspoly[1] = 1;

  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);
  EXPECT_MJTNUM_EQ(d->qfrc_spring[0], -48);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessEnergy) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.0001">
      <flag energy="enable"/>
    </option>

    <worldbody>
      <body>
        <joint type="slide" stiffness="10 5 1"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <keyframe>
      <key qpos="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);
  mjtNum total_energy = d->energy[0] + d->energy[1];

  for (int i = 0; i < 100; i++) {
    mj_step(m, d);
    EXPECT_NEAR(d->energy[0] + d->energy[1], total_energy, 0.002);
  }

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

  // tolerance for floating point numbers
  constexpr mjtNum tol = MjTol(1e-14, 1e-5);

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
      constexpr mjtNum tol = MjTol(std::numeric_limits<float>::epsilon(), 1e-5);
      EXPECT_NEAR(4*energy/volume, 2*scale*scale, tol);
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
      constexpr mjtNum tol = MjTol(std::numeric_limits<float>::epsilon(), 1e-4);
      EXPECT_NEAR(energy/volume, 3*scale*scale, tol);
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolynomialStiffnessJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" stiffness="2 3 4"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qpos="0.5"/>
    </keyframe>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);

  mjtNum x = 0.5;
  mjtNum a = 2, b = 3, c = 4;
  mjtNum expected = -(a + b * x + c * x * x) * x;
  EXPECT_NEAR(d->qfrc_spring[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolynomialStiffnessNegativeDisplacement) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" stiffness="2 3 4"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qpos="-0.5"/>
    </keyframe>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);

  mjtNum x = -0.5;
  mjtNum a = 2, b = 3, c = 4;
  mjtNum expected = -(a + b * x + c * x * x) * x;
  EXPECT_NEAR(d->qfrc_spring[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessFixedTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" name="j"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <tendon>
      <fixed stiffness="10 5 1">
        <joint joint="j" coef="1"/>
      </fixed>
    </tendon>

    <keyframe>
      <key qpos="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);

  mjtNum x = d->ten_length[0] - m->tendon_lengthspring[1];
  mjtNum expected = -(10 + 5*x + 1*x*x) * x;
  EXPECT_NEAR(d->qfrc_spring[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolyStiffnessSpatialTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <site name="s0"/>
      <body>
        <joint type="slide" name="j"/>
        <geom size="1" mass="1"/>
        <site name="s1"/>
      </body>
    </worldbody>

    <tendon>
      <spatial stiffness="10 5 1">
        <site site="s0"/>
        <site site="s1"/>
      </spatial>
    </tendon>

    <keyframe>
      <key qpos="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);

  mjtNum x = d->ten_length[0] - m->tendon_lengthspring[1];
  mjtNum expected = -x * (10 + 5*x + 1*x*x);
  EXPECT_NEAR(d->qfrc_spring[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}



TEST_F(PassiveTest, PolynomialDampingJoint) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" damping="2 3 4"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="0.5"/>
    </keyframe>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);

  mjtNum v = 0.5;
  mjtNum a = 2, b = 3, c = 4;
  mjtNum expected = -(a * v + b * v * mju_abs(v) + c * v * v * v);
  EXPECT_NEAR(d->qfrc_damper[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolynomialDampingNegativeVelocity) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" damping="2 3 4"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="-0.5"/>
    </keyframe>
  </mujoco>
  )";
  mjModel* m = LoadModelFromString(xml);
  ASSERT_THAT(m, NotNull());
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);

  mjtNum v = -0.5;
  mjtNum a = 2, b = 3, c = 4;
  mjtNum expected = -(a * v + b * v * mju_abs(v) + c * v * v * v);
  EXPECT_NEAR(d->qfrc_damper[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(PassiveTest, PolynomialDampingTendon) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint type="slide" name="j"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <tendon>
      <fixed damping="10 5 1">
        <joint joint="j" coef="1"/>
      </fixed>
    </tendon>

    <keyframe>
      <key qvel="2"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);
  mj_resetDataKeyframe(m, d, 0);

  mj_forward(m, d);

  mjtNum v = d->ten_velocity[0];
  mjtNum expected = -(10*v + 5*v*mju_abs(v) + 1*v*v*v);
  EXPECT_NEAR(d->qfrc_damper[0], expected, 1e-12);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// shell-mode (elastic2d=stretch) flexcomp must have zero passive spring forces
// at rest (initial configuration); any nonzero force indicates a rotation
// mismatch between compile-time reference positions and runtime corotation.
TEST_F(ElasticityTest, ShellModeZeroForceAtRest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <flexcomp type="grid" count="8 8 8" spacing=".07 .07 .07" pos="0 0 1"
                dim="3" cellcount="1 1 1" radius=".001" rgba="0 .7 .7 1"
                mass="5" name="softbody" dof="trilinear">
        <elasticity young="1e4" poisson="0.1" damping="0.01"
                    elastic2d="stretch" thickness="0.02"/>
        <contact selfcollide="none" internal="false"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  mj_forward(m, d);

  // all spring forces should be zero at rest
  for (int i = 0; i < m->nv; i++) {
    EXPECT_NEAR(d->qfrc_spring[i], 0, 1e-10)
        << "nonzero spring force at DOF " << i;
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// interpolated shell bending must produce zero spring forces at rest
TEST_F(ElasticityTest, InterpBendingZeroForceAtRest) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <flexcomp type="grid" count="8 8 8" spacing=".07 .07 .07" pos="0 0 1"
                dim="3" cellcount="2 2 1" radius=".001" rgba="0 .7 .7 1"
                mass="5" name="softbody" dof="trilinear">
        <elasticity young="1e4" poisson="0.1" damping="0"
                    elastic2d="bend" thickness="0.02"/>
        <contact selfcollide="none" internal="false"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  // verify bending data was compiled
  const mjtNum* bdata = m->flex_bending + m->flex_bendingadr[0];
  int nedge = (int)bdata[0];
  EXPECT_GT(nedge, 0) << "no bending edges compiled";

  mj_forward(m, d);

  // all spring forces should be zero at rest
  for (int i = 0; i < m->nv; i++) {
    EXPECT_NEAR(d->qfrc_spring[i], 0, 1e-10)
        << "nonzero spring force at DOF " << i;
  }

  // verify per-edge bending data
  int n_flat = 0, n_corner = 0;
  for (int e = 0; e < nedge; e++) {
    const mjtNum* edata = bdata + 1 + e * 10;
    mjtNum stiffness = edata[6];
    mjtNum dn0[3] = {edata[7], edata[8], edata[9]};
    mjtNum dn0_norm = mju_norm3(dn0);

    // stiffness must be positive
    EXPECT_GT(stiffness, 0) << "edge " << e << " has non-positive stiffness";

    if (dn0_norm < 1e-10) {
      // intra-surface edge: coplanar faces, zero normal jump
      n_flat++;
    } else {
      // corner edge: 90° between perpendicular face normals, |dn0| = sqrt(2)
      n_corner++;
      EXPECT_NEAR(dn0_norm, mju_sqrt(2.0), 1e-10)
          << "corner edge " << e << " has unexpected |dn0|=" << dn0_norm;
    }
  }

  // for a 2x2x1 box: 12 intra-surface + 20 corner = 32 edges
  EXPECT_GT(n_flat, 0) << "no intra-surface edges found";
  EXPECT_GT(n_corner, 0) << "no corner edges found";
  EXPECT_EQ(n_flat + n_corner, nedge);

  mj_deleteData(d);
  mj_deleteModel(m);
}

// interpolated shell bending must produce zero forces after a rigid rotation
TEST_F(ElasticityTest, InterpBendingRigidRotationInvariance) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <flexcomp type="grid" count="8 2 12" spacing=".025 .05 .025" pos="0 0 1"
                dim="3" cellcount="6 1 6" radius=".001" rgba="0 .7 .7 1"
                mass="5" name="softbody" dof="trilinear">
        <elasticity young="1e5" poisson="0.3" damping="0"
                    elastic2d="bend" thickness="0.03"/>
        <contact selfcollide="none" internal="false"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  // compute geometric center from body positions (skip world body)
  mjtNum center[3] = {0, 0, 0};
  int nnodes = 0;
  for (int b = 1; b < m->nbody; b++) {
    center[0] += m->body_pos[3*b + 0];
    center[1] += m->body_pos[3*b + 1];
    center[2] += m->body_pos[3*b + 2];
    nnodes++;
  }
  ASSERT_GT(nnodes, 0);
  center[0] /= nnodes; center[1] /= nnodes; center[2] /= nnodes;

  // rotation: 45 degrees about (1,1,1)/sqrt(3)
  mjtNum angle = 45 * 3.14159265358979 / 180.0;
  mjtNum sa = mju_sin(angle / 2), ca = mju_cos(angle / 2);
  mjtNum inv_sqrt3 = 1.0 / mju_sqrt(3.0);
  mjtNum quat[4] = {ca, sa * inv_sqrt3, sa * inv_sqrt3, sa * inv_sqrt3};
  mjtNum neg_quat[4];
  mju_negQuat(neg_quat, quat);

  // apply rigid rotation via slide joint displacements:
  // new_pos = center + R * (body_pos - center)
  // qpos = new_pos - body_pos
  for (int b = 1; b < m->nbody; b++) {
    mjtNum rel[3] = {m->body_pos[3*b+0] - center[0],
                     m->body_pos[3*b+1] - center[1],
                     m->body_pos[3*b+2] - center[2]};
    mjtNum rotated[3];
    mju_rotVecQuat(rotated, rel, neg_quat);

    // each body has 3 slide joints (x, y, z)
    for (int j = 0; j < m->body_jntnum[b] && j < 3; j++) {
      int jid = m->body_jntadr[b] + j;
      int qadr = m->jnt_qposadr[jid];
      int axis = -1;
      for (int a = 0; a < 3; a++) {
        if (m->jnt_axis[3*jid + a] != 0) { axis = a; break; }
      }
      if (axis >= 0) {
        d->qpos[qadr] =
            (center[axis] + rotated[axis]) - m->body_pos[3 * b + axis];
      }
    }
  }

  mj_forward(m, d);

  // spring forces should still be zero after rigid rotation
  constexpr mjtNum tol = MjTol(1e-6, 1e-3);
  for (int i = 0; i < m->nv; i++) {
    EXPECT_NEAR(d->qfrc_spring[i], 0, tol)
        << "nonzero spring force at DOF " << i << " after rigid rotation";
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
