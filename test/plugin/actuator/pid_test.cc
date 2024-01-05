// Copyright 2023 DeepMind Technologies Limited
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

// Tests for the PID controller plugin

#include <string_view>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/cleanup/cleanup.h>
#include <absl/strings/string_view.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using PidTest = PluginTest;
using ::testing::DoubleNear;
using ::testing::HasSubstr;
using ::testing::IsNull;
using ::testing::NotNull;

TEST_F(PidTest, PGain) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1"><config key="kp" value="4.0"/></instance>
    </plugin>
  </extension>

  <option gravity="0 0 0"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1" />
    <position joint="j2" kp="4.0" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  EXPECT_EQ(m->plugin_statenum[0], 0)
      << "Plugin should have no state variables";
  EXPECT_EQ(m->actuator_actnum[0], 0)
      << "Plugin should have no activation variables";

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // apply the same ctrl to both actuators and see if the same qpos results
  d->ctrl[0] = 1.0;
  d->ctrl[1] = 1.0;

  mj_step(m, d);

  EXPECT_EQ(d->actuator_force[0], d->actuator_force[1]);
  EXPECT_EQ(d->qfrc_actuator[0], d->qfrc_actuator[1]);
  EXPECT_EQ(d->qpos[0], d->qpos[1]);
}

TEST_F(PidTest, PGainWithFilterExact) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1"><config key="kp" value="4.0"/></instance>
    </plugin>
  </extension>

  <option gravity="0 0 0"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1"
        dyntype="filterexact" dynprm="0.1" actearly="true"/>
    <general joint="j2" gainprm="4.0 0 0" biastype="affine" biasprm="0 -4.0 0"
        dyntype="filterexact" dynprm="0.1" actearly="true"/>
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  EXPECT_EQ(m->plugin_statenum[0], 0)
      << "Plugin should have no state variables";
  EXPECT_EQ(m->actuator_actnum[0], 1)
      << "Plugin should have one activation variable";

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // apply the same ctrl to both actuators and see if the same qpos results
  d->ctrl[0] = 1.0;
  d->ctrl[1] = 1.0;

  for (int i = 0; i < 2; i++) {
    mj_step(m, d);

    EXPECT_EQ(d->actuator_force[0], d->actuator_force[1]);
    EXPECT_EQ(d->qfrc_actuator[0], d->qfrc_actuator[1]);
    EXPECT_EQ(d->qpos[0], d->qpos[1]);
  }
}

TEST_F(PidTest, SlewMaxRate) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="kp" value="4.0"/>
        <config key="slewmax" value="0.75"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 0" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1" />
    <position joint="j2" kp="4.0" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  // having a slew rate means that there should be one extra state variable
  // for the plugin.
  EXPECT_EQ(m->actuator_actnum[0], 1);

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // going from ctrl=0.0 to ctrl=1.0 immediately should be equivalent to slowly
  // incrementing the setpoint
  d->ctrl[0] = d->ctrl[1] = 0.0;
  mj_step(m, d);

  mjtNum max_slew_rate = 0.75;
  for (int i = 0; i < 2; i++) {
    d->ctrl[0] = 1.0;
    d->ctrl[1] = d->time * max_slew_rate;
    mj_step(m, d);

    EXPECT_EQ(d->actuator_force[0], d->actuator_force[1])
        << "actuator_force mismatch at step " << i;
    EXPECT_EQ(d->qfrc_actuator[0], d->qfrc_actuator[1])
        << "qfrc_actuator mismatch at step " << i;
    EXPECT_EQ(d->qpos[0], d->qpos[1]) << "qpos mismatch at step " << i;
  }
}

TEST_F(PidTest, IntegratedVelocitySlewMaxRate) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="kp" value="4.0"/>
        <config key="slewmax" value="0.75"/>
      </instance>
      <instance name="pid2">
        <config key="kp" value="4.0"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 0" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1" />
    <!-- make an integrated velocity controller using the PID plugin -->
    <plugin joint="j2" plugin="mujoco.pid" instance="pid2"
        dyntype="integrator" dynprm="1 0 0" actearly="true"/>
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  // having a slew rate means that there should be one extra state variable
  // for the plugin.
  EXPECT_EQ(m->actuator_actnum[0], 1);
  // The integrated-velocity controller should have one activation variable too.
  EXPECT_EQ(m->actuator_actnum[1], 1);

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // going from ctrl=0.0 to ctrl=1.0 immediately should be equivalent to using
  // an integrated-velocity controller, with actearly = true
  d->ctrl[0] = d->ctrl[1] = 0.0;
  mj_step(m, d);

  mjtNum max_slew_rate = 0.75;
  for (int i = 0; i < 2; i++) {
    d->ctrl[0] = 1.0;
    d->ctrl[1] = max_slew_rate;
    mj_step(m, d);

    EXPECT_EQ(d->act[0], d->act[1])
        << "act mismatch at step " << i;
    EXPECT_EQ(d->actuator_force[0], d->actuator_force[1])
        << "actuator_force mismatch at step " << i;
    EXPECT_EQ(d->qfrc_actuator[0], d->qfrc_actuator[1])
        << "qfrc_actuator mismatch at step " << i;
    EXPECT_EQ(d->qpos[0], d->qpos[1]) << "qpos mismatch at step " << i;
  }
}

TEST_F(PidTest, SlewMaxRateUsesFirstCtrl) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="kp" value="4.0"/>
        <config key="slewmax" value="0.75"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 0" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator><plugin joint="j1" plugin="mujoco.pid" instance="pid1" /></actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // When starting with ctrl = 1.0, there shouldn't be a slew rate restriction
  d->ctrl[0] = 1.0;
  mj_forward(m, d);
  EXPECT_EQ(d->actuator_force[0], 4.0);
}

TEST_F(PidTest, ITerm) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid2">
        <config key="kp" value="40.0"/>
        <config key="ki" value="40"/>
        <config key="kd" value="4"/>
      </instance>
      <instance name="pid3">
        <config key="kp" value="40.0"/>
        <config key="ki" value="40"/>
        <config key="kd" value="4"/>
        <config key="imax" value="5"/>
      </instance>
      <!-- intentionally put the plugin instances out of order, to test
           any accidentaly dependency on the plugin definition order matching
           the actuator order.
      -->
      <instance name="pid1">
        <config key="kp" value="40.0"/>
        <config key="kd" value="4"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 -10" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
    <body pos="0.08 0 0" >
      <joint name="j3" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1" />
    <plugin joint="j2" plugin="mujoco.pid" instance="pid2" />
    <plugin joint="j3" plugin="mujoco.pid" instance="pid3" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  // only the actuators with an I term should have state.
  EXPECT_EQ(m->actuator_actnum[0], 0);
  EXPECT_EQ(m->actuator_actnum[1], 1);
  EXPECT_EQ(m->actuator_actnum[2], 1);

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // when applying a constant 1.0 control for a while:
  // - the PD controller should settle to 1.0 - m*g / kp
  // - the unclamped PID controller should reach 1.0
  // - the clamped PID controller should reach 1.0 - (m*g - imax) / kp
  d->ctrl[0] = d->ctrl[1] = d->ctrl[2] = 1.0;
  for (int i = 0; i < 10000; i++) {
    mj_step(m, d);
  }

  EXPECT_THAT(d->qpos[0], DoubleNear(1.0 - 10 / 40.0, 1e-5));
  EXPECT_THAT(d->qpos[1], DoubleNear(1.0, 1e-5));
  EXPECT_THAT(d->qpos[2], DoubleNear(1.0 - (10 - 0.125 * 40.0) / 40.0, 1e-5));
}

TEST_F(PidTest, FiniteDifferencing) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid">
        <config key="ki" value="40"/>
        <config key="slewmax" value="0.75"/>
      </instance>
    </plugin>
  </extension>

  <worldbody>
    <body><joint name="j"/><geom size="0.01"/></body>
  </worldbody>

  <actuator>
    <plugin joint="j" plugin="mujoco.pid" instance="pid" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  // actuators with an I term and max slew rate should have 2 activation
  // variables.
  EXPECT_EQ(m->actuator_actnum[0], 2);

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  d->ctrl[0] = 1.0;
  mj_step(m, d);
  EXPECT_NE(d->act[0], 0);
  mjtNum state_before_finite_differencing = d->act[0];

  std::vector<mjtNum> A((2*m->nv+m->na) * (2*m->nv+m->na), 0);
  std::vector<mjtNum> B((2*m->nv+m->na) * m->nu, 0);
  std::vector<mjtNum> C((m->nsensordata) * (2*m->nv+m->na), 0);
  std::vector<mjtNum> D((m->nsensordata) * m->nu, 0);
  mjd_transitionFD(m, d, /*eps=*/1e-3, /*flg_centered=*/true, A.data(),
                   B.data(), C.data(), D.data());

  EXPECT_EQ(d->act[0], state_before_finite_differencing)
      << "mjd_transitionFD should not change actuator plugin state.";
}

TEST_F(PidTest, CtrlClamp) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <compiler autolimits="true"/>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid">
        <config key="kp" value="40.0"/>
        <config key="ki" value="40"/>
        <config key="kd" value="4"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 0" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid"
        ctrlrange="0.25 0.75" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // when applying a constant 1.0 control for a while, control should be clamped
  // to 0.75, and the body should reach that point.
  d->ctrl[0] = 1.0;
  for (int i = 0; i < 10000; i++) {
    mj_step(m, d);
  }
  EXPECT_THAT(d->qpos[0], DoubleNear(0.75, 1e-5));

  // when applying 0, it should be clamped to 0.25
  d->ctrl[0] = 0.0;
  for (int i = 0; i < 10000; i++) {
    mj_step(m, d);
  }
  EXPECT_THAT(d->qpos[0], DoubleNear(0.25, 1e-5));
}

TEST_F(PidTest, CopyData) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="kp" value="40.0"/>
        <config key="kd" value="4"/>
      </instance>
      <instance name="pid2">
        <config key="kp" value="40.0"/>
        <config key="ki" value="40"/>
        <config key="kd" value="4"/>
        <config key="slewmax" value="3.14" />
        <config key="imax" value="1"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 -10" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide" axis="0 0 1" />
      <geom size="0.01" mass="1"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid1" />
    <plugin joint="j2" plugin="mujoco.pid" instance="pid2" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  mjData* d1 = mj_makeData(m);
  absl::Cleanup d1_deleter = [d1] { mj_deleteData(d1); };

  d1->ctrl[0] = d1->ctrl[1] = -1.0;

  for (int i = 0; i < 3; i++) {
    mj_step(m, d1);
  }
  mjData* d2 = mj_copyData(nullptr, m, d1);
  absl::Cleanup d2_deleter = [d2] { mj_deleteData(d2); };

  mj_step(m, d1);
  mj_step(m, d2);

  EXPECT_EQ(d1->qpos[0], d2->qpos[0]);
  EXPECT_EQ(d1->qpos[1], d2->qpos[1]);
}

TEST_F(PidTest, MultipleActuatorsSamePlugin) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid">
        <config key="kp" value="4.0"/>
        <config key="slewmax" value="0.75"/>
      </instance>
    </plugin>
  </extension>

  <option gravity="0 0 0" timestep="0.001"/>

  <worldbody>
    <body>
      <joint name="j1" type="slide"/>
      <geom size="0.01"/>
    </body>
    <body pos="0.04 0 0" >
      <joint name="j2" type="slide"/>
      <geom size="0.01"/>
    </body>
  </worldbody>

  <actuator>
    <plugin joint="j1" plugin="mujoco.pid" instance="pid" />
    <plugin joint="j2" plugin="mujoco.pid" instance="pid" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  absl::Cleanup m_deleter = [m] { mj_deleteModel(m); };

  // having a slew rate means that there should be one extra state variable
  // for the plugin.
  EXPECT_EQ(m->actuator_actnum[0], 1);
  EXPECT_EQ(m->actuator_actnum[1], 1);

  mjData* d = mj_makeData(m);
  absl::Cleanup d_deleter = [d] { mj_deleteData(d); };

  // Set different ctrls for the two actuators, and check that they're
  // independent.
  d->ctrl[0] = 1.0;
  d->ctrl[1] = -1.0;

  for (int i = 0; i < 2; i++) {
    mj_step(m, d);

    EXPECT_EQ(d->actuator_force[0], -d->actuator_force[1])
        << "actuator_force mismatch at step " << i;
    EXPECT_EQ(d->qfrc_actuator[0], -d->qfrc_actuator[1])
        << "qfrc_actuator mismatch at step " << i;
    EXPECT_EQ(d->qpos[0], -d->qpos[1]) << "qpos mismatch at step " << i;
  }
}

TEST_F(PidTest, InvalidClamp) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="ki" value="40"/>
        <!-- imax should be nonnegative -->
        <config key="imax" value="-0.01"/>
      </instance>
    </plugin>
  </extension>

  <worldbody>
    <body><joint name="j1" /><geom size="0.01" /></body>
  </worldbody>

  <actuator><plugin joint="j1" plugin="mujoco.pid" instance="pid1" /></actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  EXPECT_THAT(m, IsNull());

  // TODO: b/303654852 - ensure that the compilation error includes "imax"
  EXPECT_THAT(std::string_view(error), HasSubstr("plugin"));
}

TEST_F(PidTest, InvalidSlew) {
  constexpr absl::string_view kModelXml = R"(
  <mujoco>
  <extension>
    <plugin plugin="mujoco.pid">
      <instance name="pid1">
        <config key="kp" value="40"/>
        <!-- slewmax must be nonnegative -->
        <config key="slewmax" value="-0.1"/>
      </instance>
    </plugin>
  </extension>

  <worldbody>
    <body><joint name="j" /><geom size="0.01" /></body>
  </worldbody>

  <actuator>
    <plugin joint="j" plugin="mujoco.pid" instance="pid1" />
  </actuator>
  </mujoco>
  )";

  char error[1024] = {0};
  mjModel* m = LoadModelFromString(kModelXml, error, sizeof(error));
  ASSERT_THAT(m, IsNull());

  // TODO: b/303654852 - ensure that the compilation error includes "slewmax"
  EXPECT_THAT(std::string_view(error), HasSubstr("plugin"));
}
}  // namespace
}  // namespace mujoco
