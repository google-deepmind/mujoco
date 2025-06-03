// Copyright 2022 DeepMind Technologies Limited
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

// Tests for plugin-related functionalities.

#include <cmath>
#include <limits>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ElasticityTest = PluginTest;

// -------------------------------- flex ------------------------------------
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
            energy += m->flex_bending[16*e+4*i+j] * elongation1 * elongation2;
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

// -------------------------------- membrane -----------------------------------
TEST_F(PluginTest, ElasticEnergyMembrane) {
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

// -------------------------------- cable -----------------------------------
TEST_F(ElasticityTest, CantileverIntoCircle) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <extension>
      <plugin plugin="mujoco.elasticity.cable"/>
    </extension>
    <worldbody>
      <geom type="plane" size="0 0 1" quat="1 0 0 0"/>
      <site name="reference" pos="0 0 0"/>
      <composite type="cable" curve="s" count="41 1 1" size="1" offset="0 0 1" initial="none">
        <plugin plugin="mujoco.elasticity.cable">
          <config key="twist" value="1e6"/>
          <config key="bend" value="1e9"/>
        </plugin>
        <joint kind="main" damping="2"/>
        <geom type="capsule" size=".005" density="1"/>
      </composite>
    </worldbody>
    <contact>
      <exclude body1="B_first" body2="B_last"/>
    </contact>
    <sensor>
      <framepos objtype="site" objname="S_last"/>
    </sensor>
    <actuator>
      <motor site="S_last" gear="0 0 0 0 1 0" ctrllimited="true" ctrlrange="0 4"/>
    </actuator>
  </mujoco>
  )";

  char error[1024] = {0};

  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mjData* d = mj_makeData(m);

  // see Oliver Weeger, Sai-Kit Yeung, Martin L. Dunn, "Isogeometric collocation methods for Cosserat rods and rod
  // structures", section 7.1 (DOI: j.cma.2016.05.009), the torque for achieving an angle phi is phi * E * Iy.
  mjtNum Iy = mjPI * pow(0.005, 4) / 4;
  mjtNum torque = 2 * mjPI * 1e9 * Iy;
  for (int i=0; i < 1300; i++) {
    if (i < 300) {
      d->ctrl[0] += torque / 300;
    }
    mj_step(m, d);
  }
  EXPECT_NEAR(d->sensordata[0], 0, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(d->sensordata[1], 0, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(d->sensordata[2], 1, std::numeric_limits<float>::epsilon());
  mj_deleteData(d);
  mj_deleteModel(m);
}

TEST_F(ElasticityTest, InvalidTxtAttribute) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="one"/>
          <config key="bend" value="1"/>
        </instance>
      </plugin>
    </extension>
    <worldbody>
      <geom type="plane" size="0 0 1" quat="1 0 0 0"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};

  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::IsNull());
}

TEST_F(ElasticityTest, InvalidMixedAttribute) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="1"/>
          <config key="bend" value="1 is not a number"/>
        </instance>
      </plugin>
    </extension>
    <worldbody>
      <geom type="plane" size="0 0 1" quat="1 0 0 0"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};

  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::IsNull());
}

TEST_F(ElasticityTest, ValidAttributes) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <plugin plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="0.0"/>
          <config key="bend" value=" 0  "/>
        </instance>
      </plugin>
    </extension>
    <worldbody>
      <geom type="plane" size="0 0 1" quat="1 0 0 0"/>
    </worldbody>
  </mujoco>
  )";

  char error[1024] = {0};

  mjModel* m = LoadModelFromString(cantilever_xml, error, sizeof(error));
  ASSERT_THAT(m, testing::NotNull()) << error;
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
