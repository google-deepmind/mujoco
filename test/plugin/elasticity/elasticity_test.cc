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

#include <array>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

class PluginTest : public MujocoTest {
 public:
  // load plugin library
  PluginTest() : MujocoTest() {
    #if defined(_WIN32) || defined(__CYGWIN__)
      mj_loadPluginLibrary((
        std::string(std::getenv("MUJOCO_PLUGIN_DIR")) +
        std::string("\\elasticity.dll")).c_str());
    #else
      #if defined(__APPLE__)
        mj_loadPluginLibrary((
          std::string(std::getenv("MUJOCO_PLUGIN_DIR")) +
          std::string("/libelasticity.dylib")).c_str());
      #else
        mj_loadPluginLibrary((
          std::string(std::getenv("MUJOCO_PLUGIN_DIR")) +
          std::string("/libelasticity.so")).c_str());
      #endif
    #endif
  }
};

// -------------------------------- cable -----------------------------------

TEST_F(PluginTest, CantileverIntoCircle) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <extension>
      <required plugin="mujoco.elasticity.cable"/>
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

TEST_F(PluginTest, InvalidTxtAttribute) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <required plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="one"/>
          <config key="bend" value="1"/>
        </instance>
      </required>
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

TEST_F(PluginTest, InvalidMixedAttribute) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <required plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="1"/>
          <config key="bend" value="1 is not a number"/>
        </instance>
      </required>
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

TEST_F(PluginTest, ValidAttributes) {
  static constexpr char cantilever_xml[] = R"(
  <mujoco>
    <extension>
      <required plugin="mujoco.elasticity.cable">
        <instance name="invalid">
          <config key="twist" value="0.0"/>
          <config key="bend" value=" 0  "/>
        </instance>
      </required>
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
