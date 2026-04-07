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

// Tests for engine/engine_forward.c.

#include "src/engine/engine_forward.h"
#include "src/engine/engine_derivative.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mujoco.h>
#include <mujoco/mjxmacro.h>
#include "src/cc/array_safety.h"
#include "src/engine/engine_callback.h"
#include "src/engine/engine_core_util.h"
#include "src/engine/engine_io.h"
#include "test/fixture.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

namespace mujoco {
namespace {

static const char* const kEnergyConservingPendulumPath =
    "engine/testdata/derivative/energy_conserving_pendulum.xml";

// helper for precision-aware checks in macros (e.g. MJDATA_POINTERS)
template <typename T>
void ExpectNear(T a, T b) {
  EXPECT_EQ(a, b);
}

template <>
void ExpectNear<mjtNum>(mjtNum a, mjtNum b) {
  EXPECT_EQ(a, b);
}


static const char* const kDampedActuatorsPath =
    "engine/testdata/derivative/damped_actuators.xml";
static const char* const kJointForceClamp =
    "engine/testdata/actuation/joint_force_clamp.xml";
static const char* const kTendonForceClamp =
    "engine/testdata/actuation/tendon_force_clamp.xml";

using ::testing::Pointwise;

using ::testing::Ne;
using ::testing::HasSubstr;
using ::testing::NotNull;
using ::testing::Gt;

// --------------------------- activation limits -------------------------------

struct ActLimitedTestCase {
  std::string test_name;
  mjtIntegrator integrator;
};

using ParametrizedForwardTest = ::testing::TestWithParam<ActLimitedTestCase>;

TEST_P(ParametrizedForwardTest, ActLimited) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="slide" gainprm="100" biasprm="0 -100" biastype="affine"
      dynprm="10" dyntype="integrator"
      actlimited="true" actrange="-1 1"/>
    </actuator>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  model->opt.integrator = GetParam().integrator;

  data->ctrl[0] = 1.0;
  // integrating up from 0, we will hit the clamp after 99 steps
  for (int i=0; i < 200; i++) {
    mj_step(model, data);
    // always greater than lower bound
    EXPECT_GT(data->act[0], -1);
    // after 99 steps we hit the upper bound
    if (i < 99) EXPECT_LT(data->act[0], 1);
    if (i >= 99) EXPECT_NEAR(data->act[0], 1, MjTol(0, 5e-6));
  }

  data->ctrl[0] = -1.0;
  // integrating down from 1, we will hit the clamp after 199 steps
  for (int i=0; i < 300; i++) {
    mj_step(model, data);
    // always smaller than upper bound
    EXPECT_LT(data->act[0], model->actuator_actrange[1]);
    // after 199 steps we hit the lower bound
    if (i < 199) EXPECT_GT(data->act[0], model->actuator_actrange[0]);
    if (i >= 199) {
      EXPECT_NEAR(data->act[0], model->actuator_actrange[0], MjTol(0.0, 5e-6));
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

INSTANTIATE_TEST_SUITE_P(
    ParametrizedForwardTest, ParametrizedForwardTest,
    testing::ValuesIn<ActLimitedTestCase>({
        {"Euler", mjINT_EULER},
        {"Implicit", mjINT_IMPLICIT},
        {"RK4", mjINT_RK4},
    }),
    [](const testing::TestParamInfo<ParametrizedForwardTest::ParamType>& info) {
      return info.param.test_name;
    });

// --------------------------- damping actuator --------------------------------

using ForwardTest = MujocoTest;

TEST_F(ForwardTest, DamperDampens) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="jnt" type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt"/>
      <damper joint="jnt" kv="1000" ctrlrange="0 100"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // move the joint
  data->ctrl[0] = 100.0;
  data->ctrl[1] = 0.0;
  for (int i=0; i < 100; i++)
    mj_step(model, data);

  // stop the joint with damping
  data->ctrl[0] = 0.0;
  data->ctrl[1] = 100.0;
  for (int i=0; i < 1000; i++)
    mj_step(model, data);

  EXPECT_LE(data->qvel[0], std::numeric_limits<double>::epsilon());
  mj_deleteData(data);
  mj_deleteModel(model);
}

static const char* const kArmatureEquivalencePath =
    "engine/testdata/armature_equivalence.xml";

// test that adding joint armature is equivalent to a coupled rotating mass with
// a gear ratio enforced by an equality
TEST_F(ForwardTest, ArmatureEquivalence) {
  const std::string xml_path = GetTestDataFilePath(kArmatureEquivalencePath);
  char error[1000];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // with actuators
  mjtNum qpos_mse = 0;
  int nstep = 0;
  while (data->time < 4) {
    data->ctrl[0] = data->ctrl[1] = mju_sin(2*data->time);
    mj_step(model, data);
    nstep++;
    mjtNum err = data->qpos[0] - data->qpos[2];
    qpos_mse += err * err;
  }
  EXPECT_LT(mju_sqrt(qpos_mse/nstep), 1e-3);

  // no actuators
  model->opt.disableflags |= mjDSBL_ACTUATION;
  qpos_mse = 0;
  nstep = 0;
  mj_resetData(model, data);
  while (data->time < 4) {
    mj_step(model, data);
    nstep++;
    mjtNum err = data->qpos[0] - data->qpos[2];
    qpos_mse += err * err;
  }
  EXPECT_LT(mju_sqrt(qpos_mse/nstep), 1e-3);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// --------------------------- implicit integrator -----------------------------

using ImplicitIntegratorTest = MujocoTest;

// Disabling implicit joint damping works as expected
TEST_F(ImplicitIntegratorTest, EulerDampDisable) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option>
      <flag eulerdamp="disable"/>
    </option>

    <worldbody>
      <body>
        <joint axis="1 0 0" damping="2"/>
        <geom type="capsule" size=".01" fromto="0 0 0 0 .1 0"/>
        <body pos="0 .1 0">
          <joint axis="0 1 0" damping="1"/>
          <geom type="capsule" size=".01" fromto="0 0 0 .1 0 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // step once, call mj_forward, save qvel and qacc
  mj_step(model, data);
  mj_forward(model, data);
  std::vector<mjtNum> qvel = AsVector(data->qvel, model->nv);
  std::vector<mjtNum> qacc = AsVector(data->qacc, model->nv);

  // second step
  mj_step(model, data);

  // compute finite-difference acceleration
  std::vector<mjtNum> qacc_fd(model->nv);
  for (int i=0; i < model->nv; i++) {
    qacc_fd[i] = (data->qvel[i] - qvel[i]) / model->opt.timestep;
  }
  // expect finite-differenced qacc to match to high precision
  EXPECT_THAT(qacc_fd, Pointwise(MjNear(1e-14, 1e-6), qacc));

  // reach the same initial state
  mj_resetData(model, data);
  mj_step(model, data);

  // second step again, but with implicit integration of joint damping
  model->opt.disableflags &= ~mjDSBL_EULERDAMP;
  mj_step(model, data);

  // compute finite-difference acceleration difference
  std::vector<mjtNum> dqacc(model->nv);
  for (int i=0; i < model->nv; i++) {
    dqacc[i] = (data->qvel[i] - qvel[i]) / model->opt.timestep;
  }
  // expect finite-differenced qacc to not match
  EXPECT_GT(mju_norm(dqacc.data(), model->nv), 1);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Reducing timesteps reduces the difference between implicit/explicit
TEST_F(ImplicitIntegratorTest, EulerDampLimit) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint axis="1 0 0" damping="2"/>
        <geom type="capsule" size=".01" fromto="0 0 0 0 .1 0"/>
        <body pos="0 .1 0">
          <joint axis="0 1 0" damping="1"/>
          <geom type="capsule" size=".01" fromto="0 0 0 .1 0 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  mjtNum diff_norm_prev = -1;
  for (const mjtNum dt : {1e-2, 1e-3, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8}) {
    // set timestep
    model->opt.timestep = dt;

    // step twice with implicit damping, save qvel
    model->opt.disableflags &= ~mjDSBL_EULERDAMP;
    mj_resetData(model, data);
    mj_step(model, data);
    mj_step(model, data);
    std::vector<mjtNum> qvel_imp = AsVector(data->qvel, model->nv);

    // step once, step again without implicit damping, save qvel
    mj_resetData(model, data);
    mj_step(model, data);
    model->opt.disableflags |= mjDSBL_EULERDAMP;
    mj_step(model, data);
    std::vector<mjtNum> qvel_exp = AsVector(data->qvel, model->nv);

    mjtNum diff_norm = 0;
    for (int i=0; i < model->nv; i++) {
      diff_norm += (qvel_imp[i] - qvel_exp[i]) * (qvel_imp[i] - qvel_exp[i]);
    }
    diff_norm = mju_sqrt(diff_norm);

    if (diff_norm_prev != -1){
      EXPECT_LT(diff_norm, diff_norm_prev);
    }

    diff_norm_prev = diff_norm;
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Euler and implicit should be equivalent if there is only joint damping
TEST_F(ImplicitIntegratorTest, EulerImplicitEquivalent) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint axis="1 0 0" damping="2"/>
        <geom type="capsule" size=".01" fromto="0 0 0 0 .1 0"/>
        <body pos="0 .1 0">
          <joint axis="0 1 0" damping="1"/>
          <geom type="capsule" size=".01" fromto="0 0 0 .1 0 0"/>
        </body>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // step 10 times with Euler, save copy of qpos as vector
  for (int i=0; i < 10; i++) {
    mj_step(model, data);
  }
  std::vector<mjtNum> qposEuler = AsVector(data->qpos, model->nq);

  // reset, step 10 times with implicit
  mj_resetData(model, data);
  model->opt.integrator = mjINT_IMPLICIT;
  for (int i=0; i < 10; i++) {
    mj_step(model, data);
  }

  // expect qpos vectors to be numerically different
#ifndef mjUSESINGLE
  EXPECT_THAT(AsVector(data->qpos, model->nq), Pointwise(Ne(), qposEuler));
#endif

  // expect qpos vectors to be similar to high precision
  EXPECT_THAT(AsVector(data->qpos, model->nq),
              Pointwise(MjNear(1e-14, 1e-6), qposEuler));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Joint and actuator damping should integrate identically under implicit
TEST_F(ImplicitIntegratorTest, JointActuatorEquivalent) {
  const std::string xml_path = GetTestDataFilePath(kDampedActuatorsPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // take 1000 steps with Euler
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  // expect corresponding joint values to be significantly different
#ifndef mjUSESINGLE
  EXPECT_GT(fabs(data->qpos[0]-data->qpos[2]), 1e-4);
  EXPECT_GT(fabs(data->qpos[1]-data->qpos[3]), 1e-4);
#endif

  // reset, take 10 steps with implicit
  mj_resetData(model, data);
  model->opt.integrator = mjINT_IMPLICIT;
  for (int i=0; i < 10; i++) {
    mj_step(model, data);
  }

  // expect corresponding joint values to be insignificantly different
  EXPECT_LT(fabs(data->qpos[0]-data->qpos[2]), MjTol(1e-16, 1e-6));
  EXPECT_LT(fabs(data->qpos[1]-data->qpos[3]), MjTol(1e-16, 1e-6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Energy conservation: RungeKutta > implicit > Euler
TEST_F(ImplicitIntegratorTest, EnergyConservation) {
  const std::string xml_path =
      GetTestDataFilePath(kEnergyConservingPendulumPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  const int nstep = 500;  // number of steps to take

  // take nstep steps with Euler, measure energy (potential + kinetic)
  model->opt.integrator = mjINT_EULER;
  for (int i=0; i < nstep; i++) {
    mj_step(model, data);
  }
  mjtNum energyEuler = data->energy[0] + data->energy[1];

  // take nstep steps with implicit, measure energy
  model->opt.integrator = mjINT_IMPLICIT;
  mj_resetData(model, data);
  for (int i=0; i < nstep; i++) {
    mj_step(model, data);
  }
  mjtNum energyImplicit = data->energy[0] + data->energy[1];

  // take nstep steps with 4th order Runge-Kutta, measure energy
  model->opt.integrator = mjINT_RK4;
  mj_resetData(model, data);
  for (int i=0; i < nstep; i++) {
    mj_step(model, data);
  }
  mjtNum energyRK4 = data->energy[0] + data->energy[1];

  // energy was measured: expect all energies to be nonzero
  EXPECT_NE(energyEuler, 0);
  EXPECT_NE(energyImplicit, 0);
  EXPECT_NE(energyRK4, 0);

  // test conservation: perfectly conserved energy would remain 0.0
  // expect RK4 to be better than implicit
  EXPECT_LT(fabs(energyRK4), fabs(energyImplicit));
  // expect implicit to be better than Euler
  EXPECT_LT(fabs(energyImplicit), fabs(energyEuler));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ForwardTest, ControlClamping) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="slide" type="slide" axis="1 0 0"/>
      </body>
    </worldbody>
    <actuator>
      <motor name="unclamped" joint="slide"/>
      <motor name="clamped" joint="slide" ctrllimited="true" ctrlrange="-1 1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // for the unclamped actuator, ctrl={1, 2} produce different accelerations
  data->ctrl[0] = 1;
  mj_forward(model, data);
  mjtNum qacc1 = data->qacc[0];
  data->ctrl[0] = 2;
  mj_forward(model, data);
  mjtNum qacc2 = data->qacc[0];
  EXPECT_NE(qacc1, qacc2);

  // for the clamped actuator, ctrl={1, 2} produce identical accelerations
  data->ctrl[1] = 1;
  mj_forward(model, data);
  qacc1 = data->qacc[0];
  data->ctrl[1] = 2;
  mj_forward(model, data);
  qacc2 = data->qacc[0];
  EXPECT_EQ(qacc1, qacc2);

  // data->ctrl[1] remains pristine
  EXPECT_EQ(data->ctrl[1], 2);

  // install warning handler
  static char warning[1024];
  warning[0] = '\0';
  mju_user_warning = [](const char* msg) {
    util::strcpy_arr(warning, msg);
  };

  // for the unclamped actuator, huge raises warning
  data->ctrl[0] = 10*mjMAXVAL;
  mj_forward(model, data);
  EXPECT_THAT(warning,
              HasSubstr("Nan, Inf or huge value in CTRL at ACTUATOR 0"));

  // for the clamped actuator, huge does not raise warning
  mj_resetData(model, data);
  warning[0] = '\0';
  data->ctrl[1] = 10*mjMAXVAL;
  mj_forward(model, data);
  EXPECT_EQ(warning[0], '\0');

  // for the clamped actuator, NaN raises warning
  mj_resetData(model, data);
  data->ctrl[1] = std::numeric_limits<double>::quiet_NaN();
  mj_forward(model, data);
  EXPECT_THAT(warning,
              HasSubstr("Nan, Inf or huge value in CTRL at ACTUATOR 1"));

  mj_deleteData(data);
  mj_deleteModel(model);
}

void control_callback(const mjModel* m, mjData *d) {
  d->ctrl[0] = 2;
}

TEST_F(ForwardTest, MjcbControlDisabled) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="hinge"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // install global control callback
  mjcb_control = control_callback;

  // call forward
  mj_forward(model, data);
  // expect that callback was used
  EXPECT_EQ(data->ctrl[0], 2.0);

  // reset, disable actuation, call forward
  mj_resetData(model, data);
  model->opt.disableflags |= mjDSBL_ACTUATION;
  mj_forward(model, data);
  // expect that callback was not used
  EXPECT_EQ(data->ctrl[0], 0.0);

  // remove global control callback
  mjcb_control = nullptr;

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ForwardTest, gravcomp) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 -10" />
    <worldbody>
      <body>
        <joint type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
      <body pos="3 0 0" gravcomp="1">
        <joint type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
      <body pos="6 0 0" gravcomp="2">
        <joint type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data = mj_makeData(model);
  while (data->time < 1) { mj_step(model, data); }

  mjtNum dist = 0.5*mju_norm3(model->opt.gravity)*(data->time*data->time);

  // expect that body 1 moved down, allowing some slack from our estimate
  EXPECT_NEAR(data->qpos[0], -dist, 0.011);

  // expect that body 2 does not move
  EXPECT_EQ(data->qpos[1], 0.0);

  // expect that body 3 moves up the same distance that body 0 moved down
  EXPECT_EQ(data->qpos[0], -data->qpos[2]);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test disabling of equality constraints
TEST_F(ForwardTest, eq_active) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="vertical" type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <equality>
      <joint joint1="vertical"/>
    </equality>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data = mj_makeData(model);

  // simulate for 1 second
  while (data->time < 1) {
    mj_step(model, data);
  }

  // expect that the body has barely moved
  EXPECT_LT(mju_abs(data->qpos[0]), 0.001);

  // turn the equality off, simulate for another second
  data->eq_active[0] = 0;
  while (data->time < 2) {
    mj_step(model, data);
  }

  // expect that the body has fallen about 5m
  EXPECT_LT(data->qpos[0], -4.5);
  EXPECT_GT(data->qpos[0], -5.5);

  // turn the equality back on, simulate for another second
  data->eq_active[0] = 1;
  while (data->time < 3) {
    mj_step(model, data);
  }

  // expect that the body has snapped back
  EXPECT_LT(mju_abs(data->qpos[0]), 0.001);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// test that normalized and denormalized quats give the same result
TEST_F(ForwardTest, NormalizeQuats) {
#ifdef mjUSESINGLE
  GTEST_SKIP() << "Skipping in float32: exact mjData comparison infeasible.";
#endif
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicit">
      <flag warmstart="disable" energy="enable"/>
    </option>
    <worldbody>
      <body name="free">
        <freejoint/>
        <geom size="1" pos=".1 .2 .3"/>
      </body>
      <body pos="3 0 0">
        <joint name="ball" type="ball" stiffness="100" range="0 10"/>
        <geom size="1" pos=".1 .2 .3"/>
      </body>
    </worldbody>
    <sensor>
      <ballquat joint="ball"/>
      <framequat objtype="body" objname="free"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data_u = mj_makeData(model);

  // we'll compare all the memory, so unpoison it first
  #ifdef MEMORY_SANITIZER
    __msan_unpoison(data_u->buffer, data_u->nbuffer);
    __msan_unpoison(data_u->arena, data_u->narena);
  #endif

  // set quats to denormalized values, non-zero velocities
  for (int i = 3; i < model->nq; i++) data_u->qpos[i] = i;
  for (int i = 0; i < model->nv; i++) data_u->qvel[i] = 0.1*i;

  // copy data and normalize quats
  mjData* data_n = mj_copyData(nullptr, model, data_u);
  mj_normalizeQuat(model, data_n->qpos);

  // call forward, expect quats to be untouched
  mj_forward(model, data_u);
  for (int i = 3; i < model->nq; i++) {
    EXPECT_EQ(data_u->qpos[i], (mjtNum)i);
  }

  // expect that the ball joint limit is active
  EXPECT_EQ(data_u->nl, 1);

  // step both models
  mj_step(model, data_u);
  mj_step(model, data_n);

  // expect everything to match
  #define X(type, name, nr, nc)                                 \
    for (int i = 0; i < model->nr; i++)                         \
      for (int j = 0; j < nc; j++)                              \
        ExpectNear(data_n->name[i*nc+j], data_u->name[i*nc+j]);
  MJDATA_POINTERS;
  #undef X

  // repeat the above with RK4 integrator
  model->opt.integrator = mjINT_RK4;

  // reset data, unpoison
  mj_resetData(model, data_u);
  #ifdef MEMORY_SANITIZER
    __msan_unpoison(data_u->buffer, data_u->nbuffer);
    __msan_unpoison(data_u->arena, data_u->narena);
  #endif

  // set quats to un-normalized values, non-zero velocities
  for (int i = 3; i < model->nq; i++) data_u->qpos[i] = i;
  for (int i = 0; i < model->nv; i++) data_u->qvel[i] = 0.1*i;

  // copy data and normalize quats
  mj_copyData(data_n, model, data_u);
  mj_normalizeQuat(model, data_n->qpos);

  // step both models
  mj_step(model, data_u);
  mj_step(model, data_n);

  // expect everything to match
  #define X(type, name, nr, nc)                                 \
    for (int i = 0; i < model->nr; i++)                         \
      for (int j = 0; j < nc; j++)                              \
        ExpectNear(data_n->name[i*nc+j], data_u->name[i*nc+j]);
  MJDATA_POINTERS;
  #undef X

  mj_deleteData(data_n);
  mj_deleteData(data_u);
  mj_deleteModel(model);
}

// test that normalized and denormalized quats give the same result
TEST_F(ForwardTest, MocapQuats) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body name="mocap" mocap="true" quat="1 1 1 1">
        <geom size="1"/>
      </body>
    </worldbody>
    <sensor>
      <framequat objtype="body" objname="mocap"/>
    </sensor>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // expect mocap_quat to be normalized (by the compiler)
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(data->mocap_quat[i], 0.5, MjTol(0, 1e-6));
    EXPECT_NEAR(data->xquat[4+i], 0.5, MjTol(0, 1e-6));
  }

  // write denormalized quats to mocap_quat, call forward again
  for (int i = 0; i < 4; i++) {
    data->mocap_quat[i] = 1;
  }
  mj_forward(model, data);

  // expect mocap_quat to remain denormalized, but xquat to be normalized
  for (int i = 0; i < 4; i++) {
    EXPECT_NEAR(data->mocap_quat[i], 1, MjTol(0, 1e-6));
    EXPECT_NEAR(data->xquat[4+i], 0.5, MjTol(0, 1e-6));
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// user defined 2nd-order activation dynamics: frequency-controlled oscillator
//  note that scalar mjcb_act_dyn callbacks are expected to return act_dot, but
//  since we have a vector output we write into act_dot directly
mjtNum oscillator(const mjModel* m, const mjData *d, int id) {
  // check that actnum == 2
  if (m->actuator_actnum[id] != 2) {
    mju_error("callback expected actnum == 2");
  }

  // get pointers to activations (inputs) and their derivatives (outputs)
  mjtNum* act = d->act + m->actuator_actadr[id];
  mjtNum* act_dot = d->act_dot + m->actuator_actadr[id];

  // harmonic oscillator with controlled frequency
  mjtNum frequency = 2*mjPI*d->ctrl[id];
  act_dot[0] = -act[1] * frequency;
  act_dot[1] = act[0] * frequency;

  return 0;  // ignored by caller
}

TEST_F(ForwardTest, MjcbActDynSecondOrderExpectsActnum) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="1e-4"/>
    <worldbody>
      <body>
        <geom size="1"/>
        <joint name="hinge"/>
      </body>
    </worldbody>
    <actuator>
      <general joint="hinge" dyntype="user" actdim="2"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // install global dynamics callback
  mjcb_act_dyn = oscillator;

  // for two arbitrary frequencies, compare actuator force as output by the
  // user-defined oscillator and analytical sine function
  for (mjtNum frequency : {1.5, 0.7}) {
    mj_resetData(model, data);
    data->ctrl[0] = frequency;  // set desired oscillation frequency
    data->act[0] = 1;  // initialise activation

    // simulate and compare to sine function
    while (data->time < 1) {
      mjtNum expected_force = mju_sin(2*mjPI*data->time*frequency);
      mj_step(model, data);
      EXPECT_NEAR(data->actuator_force[0], expected_force, .01);
    }
  }

  // uninstall global dynamics callback
  mjcb_act_dyn = nullptr;

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ------------------------------ actuators -----------------------------------

using ActuatorTest = MujocoTest;

TEST_F(ActuatorTest, ExpectedAdhesionForce) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 -1"/>

    <worldbody>
      <body name="static">
        <!-- small increase to size to ensure contact -->
        <geom size=".02001" pos=" .01  .01 .07"/>
        <geom size=".02001" pos="-.01  .01 .07"/>
        <geom size=".02001" pos=" .01 -.01 .07"/>
        <geom size=".02001" pos="-.01 -.01 .07"/>
      </body>
      <body name="free">
        <freejoint/>
        <geom type="box" size=".05 .05 .05" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <adhesion body="static" ctrlrange="0 2"/>
      <adhesion body="free" ctrlrange="0 2"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // iterate over cone type
  for (mjtCone cone : {mjCONE_ELLIPTIC, mjCONE_PYRAMIDAL}) {
    // set cone
    model->opt.cone = cone;
    // iterate over condim
    for (int condim : {1, 3, 4, 6}) {
      // set condim
      for (int id=0; id < model->ngeom; id++) {
        model->geom_condim[id] = condim;
      }
      // iterate over actuators
      for (int id=0; id < 2; id++) {
        // set ctrl > 1, expect free body to not fall
        mj_resetData(model, data);
        data->ctrl[id] = 1.01;
        for (int i = 0; i < 100; i++) {
          mj_step(model, data);
        }
        // moved down at most 10 microns
        EXPECT_GT(data->qpos[2], -1e-5);

        // set ctrl < 1, expect free body to fall below 1cm
        mj_resetData(model, data);
        data->ctrl[id] = 0.99;
        for (int i = 0; i < 100; i++) {
          mj_step(model, data);
        }
        // fell lower than 1cm
        EXPECT_LT(data->qpos[2], -0.01);
      }
    }
  }
  mj_deleteData(data);
  mj_deleteModel(model);
}

// Actuator force clamping at joints
TEST_F(ActuatorTest, ActuatorForceClamping) {
  const std::string xml_path = GetTestDataFilePath(kJointForceClamp);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 10;
  mj_forward(model, data);

  // expect clamping as specified in the model
  EXPECT_NEAR(data->actuator_force[0], 1, MjTol(0, 1e-6));
  EXPECT_NEAR(data->qfrc_actuator[0], 0.4, MjTol(0, 1e-6));

  // simulate for 2 seconds to gain velocity
  while (data->time < 2) {
    mj_step(model, data);
  }

  // activate damper, expect force to be clamped at lower bound
  data->ctrl[1] = 1;
  mj_forward(model, data);
  EXPECT_NEAR(data->qfrc_actuator[0], -0.4, MjTol(0, 1e-6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Apply gravity compensation via actuators
TEST_F(ActuatorTest, ActuatorGravcomp) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body gravcomp="1">
        <joint name="joint" type="slide" axis="0 0 1"
               actuatorfrcrange="-2 2" actuatorgravcomp="true"/>
        <geom type="box" size=".05 .05 .05" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <motor name="actuator" joint="joint"/>
    </actuator>

    <sensor>
      <actuatorfrc actuator="actuator"/>
      <jointactuatorfrc joint="joint"/>
    </sensor>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  mj_forward(model, data);

  // expect force clamping as specified in the model
  EXPECT_EQ(data->actuator_force[0], 0);
  EXPECT_EQ(data->qfrc_actuator[0], 2);
  EXPECT_EQ(data->qfrc_passive[0], 0);
  EXPECT_EQ(data->sensordata[0], 0);
  EXPECT_EQ(data->sensordata[1], 2);

  // reduce gravity so gravcomp is not clamped
  model->opt.gravity[2] = -1;
  mj_forward(model, data);
  EXPECT_EQ(data->actuator_force[0], 0);
  EXPECT_EQ(data->qfrc_actuator[0], 1);
  EXPECT_EQ(data->qfrc_passive[0], 0);
  EXPECT_EQ(data->sensordata[0], 0);
  EXPECT_EQ(data->sensordata[1], 1);

  // add control, see that it adds up
  data->ctrl[0] = 0.5;
  mj_forward(model, data);
  EXPECT_EQ(data->actuator_force[0], 0.5);
  EXPECT_EQ(data->qfrc_actuator[0], 1.5);
  EXPECT_EQ(data->qfrc_passive[0], 0);
  EXPECT_EQ(data->sensordata[0], 0.5);
  EXPECT_EQ(data->sensordata[1], 1.5);

  // add larger control, expect clamping
  data->ctrl[0] = 1.5;
  mj_forward(model, data);
  EXPECT_EQ(data->actuator_force[0], 1.5);
  EXPECT_EQ(data->qfrc_actuator[0], 2);
  EXPECT_EQ(data->qfrc_passive[0], 0);
  EXPECT_EQ(data->sensordata[0], 1.5);
  EXPECT_EQ(data->sensordata[1], 2);

  // disable actgravcomp, expect gravcomp as a passive force
  model->jnt_actgravcomp[0] = 0;
  mj_forward(model, data);
  EXPECT_EQ(data->actuator_force[0], 1.5);
  EXPECT_EQ(data->qfrc_actuator[0], 1.5);
  EXPECT_EQ(data->qfrc_passive[0], 1);
  EXPECT_EQ(data->sensordata[0], 1.5);
  EXPECT_EQ(data->sensordata[1], 1.5);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Check that dampratio works as expected
TEST_F(ActuatorTest, DampRatio) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicitfast"/>

    <worldbody>
      <body>
        <joint name="slide1" axis="1 0 0" type="slide"/>
        <geom size=".05"/>
      </body>

      <body pos="0 0 -.15">
        <joint name="slide2" axis="1 0 0" type="slide"/>
        <geom size=".05"/>
      </body>
    </worldbody>

    <actuator>
      <position name="slightly underdamped" joint="slide1" kp="10" dampratio="0.99"/>
      <position name="slightly overdamped"  joint="slide2" kp="10" dampratio="1.01"/>
    </actuator>
  </mujoco>
  )";
  mjModel* model = LoadModelFromString(xml);
  mjData* data = mj_makeData(model);

  data->qpos[0] = data->qpos[1] = -0.1;

  mjtNum under_damped = data->qpos[0];
  mjtNum over_damped = data->qpos[1];
  while (data->time < 10) {
    mj_step(model, data);
    under_damped = mju_max(under_damped, data->qpos[0]);
    over_damped = mju_max(over_damped, data->qpos[1]);
  }

  // expect slightly underdamped to slightly overshoot
  EXPECT_GT(under_damped, 0);
  EXPECT_LT(under_damped, 1e-6);

  // expect slightly overdamped to slightly undershoot
  EXPECT_LT(over_damped, 0);
  EXPECT_GT(over_damped, -1e-6);

  mj_deleteData(data);
  mj_deleteModel(model);
}


// Check dampratio for actuators with nontrivial transmission
TEST_F(ActuatorTest, DampRatioTendon) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/actuation/tendon_dampratio.xml");
  char error[1000];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 1;
  data->ctrl[1] = 4;

  while (data->time < 1) {
    mj_step(model, data);
  }

  // expect first and second fingers to move together
  double tol  = 1e-10;
  EXPECT_THAT(AsVector(data->qpos, 4),
              Pointwise(MjNear(tol, tol), AsVector(data->qpos + 4, 4)));
  EXPECT_THAT(AsVector(data->qvel, 4),
              Pointwise(MjNear(tol, tol), AsVector(data->qvel + 4, 4)));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ----------------------- DC motor actuators ----------------------------------

using DCMotorTest = MujocoTest;

TEST_F(DCMotorTest, IntVelocityEquivalence) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option integrator="implicit"/>
    <worldbody>
      <body pos="0 0 0">
        <joint name="slide1" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
      <body pos="0 1 0">
        <joint name="slide2" type="slide" axis="1 0 0"/>
        <geom size=".1"/>
      </body>
    </worldbody>
    <actuator>
      <!--
        Equivalence mapping:
        intvelocity force: F = kp * \int(ctrl - v) - kv * v
        dcmotor force:     F = (V*K - K^2*v) / R
        where V = ki * \int(ctrl - v)     (since kp=0, kd=0)

        Setting K=1, R=0.2, ki=2 yields:
        F = (2 * \int(ctrl - v) - v) / 0.2
          = 10 * \int(ctrl - v) - 5 * v

        This perfectly matches intvelocity with kp=10, kv=5.
      -->
      <intvelocity name="intvel" joint="slide1" kp="10" kv="5" actrange="-0.01 0.01"/>
      <dcmotor name="dcmotor" joint="slide2" motorconst="1" resistance="0.2" input="velocity" controller="0 2 0 0 0.01"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Apply a time-varying velocity command
  while (data->time < 1.0) {
    data->ctrl[0] = mju_sin(20 * data->time);
    data->ctrl[1] = mju_sin(20 * data->time);
    mj_step(model, data);

    // Both actuators should integrate identical states
    EXPECT_MJTNUM_EQ(data->act[0], data->act[1]);

    // Both bodies should move identically
    EXPECT_NEAR(data->qpos[0], data->qpos[1], MjTol(1e-14, 1e-7));
    EXPECT_NEAR(data->qvel[0], data->qvel[1], MjTol(1e-14, 1e-7));
    EXPECT_NEAR(data->qacc[0], data->qacc[1], MjTol(1e-14, 1e-6));

    // Both actuators should produce identical force
    EXPECT_NEAR(data->actuator_force[0], data->actuator_force[1],
                MjTol(1e-14, 1e-6));
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatelessSteadyState) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double K = 0.05;
  double R = 2.0;
  double V = 12.0;
  double omega = 3.0;

  data->ctrl[0] = V;
  data->qvel[0] = omega;
  mj_forward(model, data);

  double expected_force = K / R * (V - K * omega);
  EXPECT_NEAR(data->actuator_force[0], expected_force, MjTol(1e-12, 1e-5));
  EXPECT_EQ(model->actuator_actnum[0], 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CurrentFilterConverges) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.0001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="1000"/>
        <geom size="1" mass="100"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               inductance="0.01 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  ASSERT_EQ(model->actuator_actnum[0], 1);

  double K = 0.05;
  double R = 2.0;
  double V = 12.0;

  data->ctrl[0] = V;
  for (int i = 0; i < 10000; i++) {
    mj_step(model, data);
  }

  double omega = data->qvel[0];
  double i_ss = V / R - K / R * omega;
  double expected_force = K * i_ss;

  EXPECT_NEAR(data->act[0], i_ss, MjTol(1e-6, 1e-4));
  EXPECT_NEAR(data->actuator_force[0], expected_force, MjTol(1e-6, 1e-4));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CurrentFilterExactIntegration) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="10000"/>
        <geom size="1" mass="10000"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               inductance="0.01 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double R = 2.0;
  double te = 0.01 / R;
  double V = 12.0;

  data->ctrl[0] = V;
  mj_step(model, data);

  double h = model->opt.timestep;
  double exact_current = V / R * (1 - mju_exp(-h / te));
  EXPECT_NEAR(data->act[0], exact_current, MjTol(1e-10, 1e-4));

  double euler_current = V / R * h / te;
  EXPECT_GT(std::abs(data->act[0] - euler_current),
            std::abs(data->act[0] - exact_current));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CoggingTorque) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               cogging="0.1 6 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double A = 0.1, Np = 6, phi = 0;
  double K = 0.05, R = 2.0;
  double V = 5.0;
  double pos = 1.0;

  data->ctrl[0] = V;
  data->qpos[0] = pos;
  mj_forward(model, data);

  double electrical_force = K / R * V;
  double cogging = A * mju_sin(Np * pos + phi);
  EXPECT_NEAR(data->actuator_force[0], electrical_force + cogging,
              MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CoggingBypassesSaturation) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               saturation="0.001 0" cogging="0.1 6 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double A = 0.1, Np = 6, phi = 0;
  double pos = 1.0;

  data->ctrl[0] = 100.0;
  data->qpos[0] = pos;
  mj_forward(model, data);

  double cogging = A * mju_sin(Np * pos + phi);
  EXPECT_NEAR(model->actuator_forcerange[1], 0.001, MjTol(1e-12, 1e-5));
  EXPECT_GT(mju_abs(data->actuator_force[0]), 0.001);
  EXPECT_NEAR(data->actuator_force[0], 0.001 + cogging, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, LuGreViscousFriction) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               lugre="100 1 0.01 0.5 0.7 10"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  ASSERT_EQ(model->actuator_actnum[0], 1);

  double sigma1 = 1, sigma2 = 0.01;
  double K = 0.05, R = 2.0;
  double omega = 2.0;

  data->ctrl[0] = 0;
  data->qvel[0] = omega;
  mj_forward(model, data);

  EXPECT_MJTNUM_EQ(model->actuator_damping[0], sigma2);
  double electrical_force = K / R * (0 - K * omega);
  double z = data->act[model->actuator_actadr[0]];
  double z_dot = data->act_dot[model->actuator_actadr[0]];
  double lugre_force = 100 * z + sigma1 * z_dot;
  EXPECT_NEAR(data->actuator_force[0], electrical_force - lugre_force,
              MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, ThermalRiseAndFall) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="10000"/>
        <geom size="1" mass="10000"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               thermal="10 5 0 0 25 25"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  int adr = model->actuator_actadr[0];
  ASSERT_EQ(model->actuator_actnum[0], 1);
  EXPECT_EQ(data->act[adr], 0);

  double R = 2.0, V = 10.0;
  double RT = 10.0, C = 5.0;
  double h = model->opt.timestep;
  double P = V * V / R;

  data->ctrl[0] = V;

  mj_step(model, data);
  double dT1 = h * P / C;
  EXPECT_NEAR(data->act[adr], dT1, MjTol(1e-11, 1e-4));

  mj_step(model, data);
  double dT2 = dT1 + h * (P - dT1 / RT) / C;
  EXPECT_NEAR(data->act[adr], dT2, MjTol(1e-11, 1e-4));

  data->ctrl[0] = 0;
  mj_step(model, data);
  double dT3 = dT2 + h * (0 - dT2 / RT) / C;
  EXPECT_NEAR(data->act[adr], dT3, MjTol(1e-11, 1e-4));
  EXPECT_LT(data->act[adr], dT2);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, ThermalSteadyState) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="10000"/>
        <geom size="1" mass="10000"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               thermal="0.1 0.1 0 0 25 25"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double R = 2.0, V = 10.0;
  double RT = 0.1;
  double dT_ss = RT * V * V / R;

  data->ctrl[0] = V;
  for (int i = 0; i < 10000; i++) {
    mj_step(model, data);
  }

  int adr = model->actuator_actadr[0];
  EXPECT_NEAR(data->act[adr], dT_ss, 1e-4);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, ThermalAffectsForce) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               thermal="0.1 0.1 0 0.004 25 25"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  double K = 0.05, R = 2.0, V = 10.0;
  double alpha = 0.004;
  int adr = model->actuator_actadr[0];

  data->ctrl[0] = V;
  data->act[adr] = 0;
  mj_forward(model, data);
  double force_cold = data->actuator_force[0];
  EXPECT_NEAR(force_cold, K / R * V, MjTol(1e-12, 1e-5));

  double dT = 50;
  data->act[adr] = dT;
  mj_forward(model, data);
  double R_hot = R * (1 + alpha * dT);
  double force_hot = data->actuator_force[0];
  EXPECT_NEAR(force_hot, K / R_hot * V, MjTol(1e-12, 1e-5));
  EXPECT_LT(force_hot, force_cold);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Temperature slot must be correctly offset past slew and integral states.
TEST_F(DCMotorTest, ThermalAffectsForceWithController) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               input="position" controller="1.0 1.0 0 5.0 0"
               thermal="0.1 0.1 0 0.004 25 25"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // slot order: slew(0), integral(1), temperature(2)
  ASSERT_EQ(model->actuator_actnum[0], 3);
  int adr = model->actuator_actadr[0];
  int temp_adr = adr + 2;  // temperature is slot 2

  double K = 0.05, R = 2.0, alpha = 0.004;
  double dT = 50;
  data->act[adr]      = 1.0;   // slew state = ctrl: no rate-limiting applied
  data->act[adr + 1]  = 0.0;   // integral state x_I = 0
  data->act[temp_adr] = dT;    // temperature rise above ambient
  data->ctrl[0] = 1.0;         // position setpoint = 1.0, qpos = 0, error = 1.0
  mj_forward(model, data);

  // u_eff = ctrl = 1.0 (no slew applied since act[slew] == ctrl)
  // V = kp*(u_eff - length) + ki*x_I - kd*omega = 1.0*1.0 + 1.0*0.0 - 0*0 = 1.0
  // R(T) = 2.0 * (1 + 0.004 * 50) = 2.4
  // stateless (no te): force = K/R(T) * V = 0.05/2.4 * 1.0
  double R_hot = R * (1 + alpha * dT);
  EXPECT_NEAR(data->actuator_force[0], K / R_hot * 1.0, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatelessPositionMode) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" input="position" controller="2.0 0 0.5 0 0"
               motorconst="0.05" resistance="2.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Position target 5.0, current pos 0.0, current vel 0.0
  data->ctrl[0] = 5.0;
  mj_forward(model, data);

  // V = Kp * (u - theta) = 2.0 * 5.0 = 10.0
  // force = K / R * V + bias = (0.05 / 2.0) * 10.0 + 0 = 0.25
  EXPECT_NEAR(data->actuator_force[0], 0.25, MjTol(1e-12, 1e-5));

  // Velocity penalty
  data->qvel[0] = 2.0;
  mj_forward(model, data);
  // V = 10.0 - Kd * omega = 10.0 - (0.5 * 2.0) = 9.0
  // bias = - K^2 / R * omega = -0.0025 / 2.0 * 2.0 = -0.0025
  // force = K / R * V + bias = 0.225 - 0.0025 = 0.2225
  EXPECT_NEAR(data->actuator_force[0], 0.2225, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatelessVelocityMode) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" input="velocity" controller="3.0 0 0 0 0"
               motorconst="0.05" resistance="2.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Velocity target 4.0, current vel 1.0
  data->ctrl[0] = 4.0;
  data->qvel[0] = 1.0;
  mj_forward(model, data);

  // V = Kp * (u - omega) = 3.0 * (4.0 - 1.0) = 9.0
  // bias = - K^2 / R * omega = -0.0025 / 2.0 * 1.0 = -0.00125
  // force = K / R * V + bias = (0.05 / 2.0) * 9.0 - 0.00125 = 0.22375
  EXPECT_NEAR(data->actuator_force[0], 0.22375, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatefulPositionMode) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" input="position" controller="2.0 0.5 0.1 10.0 5.0"
               motorconst="0.05" resistance="2.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Controller states: 1 for slew, 1 for ki -> actnum = 2
  ASSERT_EQ(model->actuator_actnum[0], 2);
  int adr = model->actuator_actadr[0];

  // Current states
  double u_prev = 1.0;
  double x_I = 2.0;
  data->act[adr] = u_prev;
  data->act[adr+1] = x_I;

  // target 5.0 position, current 0.0
  data->ctrl[0] = 5.0;
  data->qvel[0] = 0.5;
  mj_forward(model, data);

  // slew bounding: s = 10.0, dt = 0.001. max_change = 0.01
  // Target = 5.0. It is upper bounded by u_prev + 0.01 = 1.01
  EXPECT_NEAR(data->act_dot[adr], 10.0, MjTol(1e-12, 1e-5));

  // PI error: error = u_eff - length = 1.01 - 0.0 = 1.01
  EXPECT_NEAR(data->act_dot[adr+1], 1.01, MjTol(1e-12, 1e-5));

  // V = Kp(u_eff - length) + Ki * x_I - Kd * omega
  // V = 2.0 * 1.01 + 0.5 * 2.0 - 0.1 * 0.5 = 2.97
  // bias = - K^2/R * omega = -(0.05)^2 / 2.0 * 0.5 = -0.000625
  // force = K/R * V + bias = 0.025 * 2.97 - 0.000625 = 0.073625
  EXPECT_NEAR(data->actuator_force[0], 0.073625, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatefulPositionWithCurrentMode) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" input="position" controller="2.0 0.5 0.1 10.0 5.0"
               motorconst="0.05" resistance="2.0" inductance="1.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Controller states: slew (0), ki (1), current (2). actnum = 3
  ASSERT_EQ(model->actuator_actnum[0], 3);
  int adr = model->actuator_actadr[0];

  double u_prev = 1.0;
  double x_I = 2.0;
  double current = 0.5;
  data->act[adr] = u_prev;
  data->act[adr+1] = x_I;
  data->act[adr+2] = current;

  // Target 5.0 position, velocity 0.5
  data->ctrl[0] = 5.0;
  data->qvel[0] = 0.5;
  mj_forward(model, data);

  // Slew bounding: max_change = 0.01, u_eff = 1.01
  EXPECT_NEAR(data->act_dot[adr], 10.0, MjTol(1e-12, 1e-5));

  // PI error: error = u_eff - length = 1.01
  EXPECT_NEAR(data->act_dot[adr+1], 1.01, MjTol(1e-12, 1e-5));

  // Voltage computation:
  // V = Kp(u_eff - length) + Ki * x_I - Kd * omega
  // V = 2.0 * 1.01 + 0.5 * 2.0 - 0.1 * 0.5 = 2.97

  // Current filter:
  // t_e = L / R = 1.0 / 2.0 = 0.5
  // di/dt = (V/R - K/R * omega - i) / t_e
  // di/dt = (2.97/2.0 - 0.05/2.0 * 0.5 - 0.5) / 0.5
  // di/dt = (1.485 - 0.0125 - 0.5) / 0.5 = 0.9725 / 0.5 = 1.945
  EXPECT_NEAR(data->act_dot[adr+2], 1.945, MjTol(1e-12, 1e-5));

  // Force is just K * current since current is stateful
  EXPECT_NEAR(data->actuator_force[0], 0.05 * 0.5, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, StatefulVelocityMode) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" input="velocity" controller="3.0 1.0 0 0 2.0"
               motorconst="0.05" resistance="2.0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Controller states: 1 for ki (no slew)
  ASSERT_EQ(model->actuator_actnum[0], 1);
  int adr = model->actuator_actadr[0];

  double x_I = 2.0;    // Exactly at Imax limit (Imax = 2.0)
  data->act[adr] = x_I;

  // target vel 4.0, current vel 1.0
  data->ctrl[0] = 4.0;
  data->qvel[0] = 1.0;
  mj_forward(model, data);

  // integrate command directly: error = target = 4.0
  // since x_I == Imax (2.0) and error (4.0) > 0, act_dot should be clamped to 0
  EXPECT_NEAR(data->act_dot[adr], 0.0, MjTol(1e-12, 1e-5));

  // V = Kp * (u_eff - omega) + Ki * (x_I - length)
  // V = 3.0 * (4.0 - 1.0) + 1.0 * (2.0 - 0.0) = 9.0 + 2.0 = 11.0
  // bias = - K^2/R * omega = -(0.05)^2 / 2.0 * 1.0 = -0.00125
  // force = K/R * V + bias = 0.025 * 11.0 - 0.00125 = 0.275 - 0.00125 = 0.27375
  EXPECT_NEAR(data->actuator_force[0], 0.27375, MjTol(1e-12, 1e-5));

  // repeat with non-zero joint position
  data->qpos[0] = 1.5;
  mj_forward(model, data);

  // V = 3.0 * (4.0 - 1.0) + 1.0 * (2.0 - 1.5) = 9.0 + 0.5 = 9.5
  // force = K/R * V + bias = 0.025 * 9.5 - 0.00125 = 0.2375 - 0.00125 = 0.23625
  EXPECT_NEAR(data->actuator_force[0], 0.23625, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CurrentPlusThermal) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="10000"/>
        <geom size="1" mass="10000"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               inductance="0.01 0" thermal="10 5 0 0.004 25 25"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  ASSERT_EQ(model->actuator_actnum[0], 2);
  int adr = model->actuator_actadr[0];

  double K = 0.05, R = 2.0, V = 12.0;
  double te = 0.01 / R;
  double RT = 10.0, C = 5.0;

  double current = 3.0;
  double dT = 10.0;
  data->act[adr] = dT;
  data->act[adr+1] = current;
  data->ctrl[0] = V;
  mj_forward(model, data);

  EXPECT_NEAR(data->actuator_force[0], K * current, MjTol(1e-12, 1e-5));

  double R_hot = R * (1 + 0.004 * dT);
  double T_dot = (R_hot * current * current - dT / RT) / C;
  EXPECT_NEAR(data->act_dot[adr], T_dot, MjTol(1e-10, 1e-4));

  double omega = data->qvel[0];
  double i_dot = (V/R_hot - K/R_hot*omega - current) / te;
  EXPECT_NEAR(data->act_dot[adr+1], i_dot, MjTol(1e-10, 1e-3));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, CurrentRateLimit) {
  // Verifies that saturation:current_rate clamps di/dt.
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint" damping="10000"/>
        <geom size="1" mass="10000"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               inductance="0.01 0" saturation="0 0 100 0"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  ASSERT_EQ(model->actuator_actnum[0], 1);
  int adr = model->actuator_actadr[0];

  double V = 12.0;
  double dimax = 100.0;   // A/s rate limit

  // unclamped: i_dot = (V/R - 0 - 0) / te = 6 / 0.005 = 1200 A/s >> dimax
  data->act[adr] = 0;   // current = 0
  data->ctrl[0] = V;
  mj_forward(model, data);

  // i_dot should be clipped to +dimax
  EXPECT_NEAR(data->act_dot[adr], dimax, MjTol(1e-12, 1e-5));

  // reverse: large negative drive
  data->ctrl[0] = -V;
  mj_forward(model, data);

  // i_dot should be clipped to -dimax
  EXPECT_NEAR(data->act_dot[adr], -dimax, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, LuGreExactIntegration) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1" mass="1e6"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               lugre="100 1 0.01 0.5 0.7 10"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  ASSERT_EQ(model->actuator_actnum[0], 1);
  int adr = model->actuator_actadr[0];

  double sigma0 = 100, F_C = 0.5, F_S = 0.7, v_S = 10;
  double z0 = 0.002;
  double v = 0.5;
  double h = model->opt.timestep;

  data->act[adr] = z0;
  data->qvel[0] = v;

  double ratio = v / v_S;
  double g_v = F_C + (F_S - F_C) * mju_exp(-ratio*ratio);
  double a = -sigma0 * std::abs(v) / g_v;
  double exp_ah = mju_exp(a * h);
  double int_h = (exp_ah - 1) / a;
  double z_new = exp_ah * z0 + int_h * v;

  mj_step(model, data);
  EXPECT_NEAR(data->act[adr], z_new, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, LuGreSteadyState) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.001"/>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1" mass="1e6"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               lugre="100 1 0.01 0.5 0.7 10"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  int adr = model->actuator_actadr[0];

  double sigma0 = 100, sigma2 = 0.01;
  double F_C = 0.5, F_S = 0.7, v_S = 10;
  double K = 0.05, R = 2.0;
  double v = 0.5;

  data->qvel[0] = v;
  data->ctrl[0] = 0;
  for (int i = 0; i < 10000; i++) {
    mj_step(model, data);
  }

  double ratio = v / v_S;
  double g_v = F_C + (F_S - F_C) * mju_exp(-ratio*ratio);
  double z_ss = g_v / sigma0;
  EXPECT_NEAR(data->act[adr], z_ss, 1e-4);

  EXPECT_MJTNUM_EQ(model->actuator_damping[0], sigma2);
  double back_emf = K * K / R * data->qvel[0];
  double lugre_ss = g_v;
  EXPECT_NEAR(data->actuator_force[0], -back_emf - lugre_ss, 1e-3);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(DCMotorTest, LuGreBristleSpring) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <dcmotor joint="joint" motorconst="0.05" resistance="2.0"
               lugre="100 1 0.01 0.5 0.7 10"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  int adr = model->actuator_actadr[0];
  double sigma0 = 100;
  double X = 0.01;

  data->act[adr] = X;
  data->ctrl[0] = 0;
  mj_forward(model, data);

  EXPECT_NEAR(data->actuator_force[0], -sigma0 * X, MjTol(1e-12, 1e-5));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ----------------------- filterexact actuators -------------------------------

using FilterExactTest = MujocoTest;

TEST_F(FilterExactTest, ApproximatesContinuousTime) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true"/>
    <worldbody>
      <body name="box">
        <joint name="slide" type="slide" axis="1 0 0" />
        <geom type="box" size=".05 .05 .05" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <general joint="slide" dyntype="filter" gainprm="1.1" />
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  const mjtNum kSimulationTime = 1.0;

  // compute act with a small timestep to approximate continuous integration
  model->opt.timestep = 0.001;
  mj_resetData(model, data);
  data->ctrl[0] = 1.0;
  data->act[0] = 0.0;
  for (int i = 0; i < std::round(kSimulationTime / model->opt.timestep); i++) {
    mj_step(model, data);
  }
  mjtNum continuous_act = data->act[0];

  // compute again with a larger timestep, introducing integration error
  model->opt.timestep = 0.01;
  mj_resetData(model, data);
  data->ctrl[0] = 1.0;
  data->act[0] = 0.0;
  for (int i = 0; i < std::round(kSimulationTime / model->opt.timestep); i++) {
    mj_step(model, data);
  }
  mjtNum discrete_act = data->act[0];

  // compute a third time with exact integration
  model->actuator_dyntype[0] = mjDYN_FILTEREXACT;
  mj_resetData(model, data);
  data->ctrl[0] = 1.0;
  data->act[0] = 0.0;
  for (int i = 0; i < std::round(kSimulationTime / model->opt.timestep); i++) {
    mj_step(model, data);
  }
  mjtNum exactfilter_act = data->act[0];

  // expect exact integration to be closer to the small-timestep result
  EXPECT_THAT(std::abs(continuous_act - discrete_act),
              Gt(5*std::abs(continuous_act - exactfilter_act)))
      << "Using filterexact should make the error at least 5 times smaller";

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(FilterExactTest, TimestepIndependent) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true"/>
    <worldbody>
      <body name="box">
        <joint name="slide" type="slide" axis="1 0 0" />
        <geom type="box" size=".05 .05 .05" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <general joint="slide" dyntype="filterexact" dynprm="0.9" gainprm="1.1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  const mjtNum kSimulationTime = 1.0;

  // first, compute act based on a small timestep and exact integration
  model->opt.timestep = 0.01;
  mj_resetData(model, data);
  data->ctrl[0] = 1.0;
  data->act[0] = 0.0;
  for (int i = 0; i < std::round(kSimulationTime / model->opt.timestep); i++) {
    mj_step(model, data);
  }
  mjtNum small_timestep_act = data->act[0];

  // now change the timestep to a much larger timestep
  model->opt.timestep = 0.1;
  mj_resetData(model, data);
  data->ctrl[0] = 1.0;
  data->act[0] = 0.0;
  for (int i = 0; i < std::round(kSimulationTime / model->opt.timestep); i++) {
    mj_step(model, data);
  }
  mjtNum large_timestep_act = data->act[0];

  EXPECT_NEAR(small_timestep_act, large_timestep_act, MjTol(1e-14, 1e-6))
      << "exact integration should be independent of timestep to machine "
         "precision.";

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(FilterExactTest, ActEqualsCtrlWhenTauIsZero) {
  static constexpr char xml[] = R"(
  <mujoco>
    <compiler autolimits="true"/>
    <worldbody>
      <body name="box">
        <joint name="slide" type="slide" axis="1 0 0" />
        <geom type="box" size=".05 .05 .05" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <general joint="slide" dyntype="filterexact" dynprm="0" gainprm="1.1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);
  data->ctrl[0] = 0.5;
  data->act[0] = 0.0;
  mj_step(model, data);
  EXPECT_EQ(data->act[0], data->ctrl[0]);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ----------------------- actearly actuator attribute -------------------------

using ActEarlyTest = MujocoTest;

TEST_F(ActEarlyTest, RemovesOneStepDelay) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/actuation/actearly.xml");
  char error[1000];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  ASSERT_EQ(model->nu % 2, 0) << "number of actuators should be even";
  ASSERT_EQ(model->nu, model->na) << "all actuators should be stateful";
  ASSERT_EQ(model->nq, model->nu);
  EXPECT_GT(model->nu, 0);

  // actuators are ordered in pairs with actearly=true and actearly=false
  for (int i = 0; i < model->na / 2; i++) {
    EXPECT_TRUE(model->actuator_actearly[2*i]);
    EXPECT_FALSE(model->actuator_actearly[2*i + 1]);
  }

  mjData* data = mj_makeData(model);

  // set all controls to the same value and make one step
  mju_fill(data->ctrl, 0.5, model->nu);
  mj_step(model, data);

  for (int i = 0; i < model->na / 2; i++) {
    EXPECT_EQ(data->act[2 * i], data->act[2 * i + 1])
        << "act should be the same after first step for "
        << mj_id2name(model, mjOBJ_ACTUATOR, 2 * i);

    EXPECT_EQ(data->act_dot[2 * i], data->act_dot[2 * i + 1])
        << "act_dot should be the same after first step for "
        << mj_id2name(model, mjOBJ_ACTUATOR, 2 * i);
  }

  for (int i = 0; i < 100; i++) {
    std::vector<mjtNum> last_qfrc(data->qfrc_actuator,
                                  data->qfrc_actuator + model->nu);
    mj_step(model, data);
    for (int j = 0; j < model->nu / 2; j++) {
      // this is true for torque actuators
      EXPECT_NEAR(last_qfrc[2 * j], data->qfrc_actuator[2 * j + 1], MjTol(1e-3, 1e-1))
          << "there should be a 1 step delay between qfrc for "
          << mj_id2name(model, mjOBJ_ACTUATOR, 2 * j);
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ActEarlyTest, DoesntChangeStateInMjForward) {
  const std::string xml_path =
      GetTestDataFilePath("engine/testdata/actuation/actearly.xml");
  char error[1000];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data = mj_makeData(model);

  // set all controls to the same value and make one step
  mju_fill(data->ctrl, 0.5, model->nu);
  mj_forward(model, data);

  for (int i = 0; i < model->na; i++) {
    EXPECT_EQ(data->act[i], 0)
        << "act should not change with mj_forward."
        << mj_id2name(model, mjOBJ_ACTUATOR, i);
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ActuatorTest, DisableActuator) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <motor joint="slide" gear="2"  group="0"/>
      <position joint="slide" kp="1" group="1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 1.0;
  data->ctrl[1] = 1.0;

  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], 3.0);

  model->opt.disableactuator = 1 << 0;
  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], 1.0);

  model->opt.disableactuator = 1 << 1;
  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], 2.0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ActuatorTest, DisableActuatorOutOfRange) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="slide" type="slide" axis="1 0 0"/>
        <geom size="1" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <motor joint="slide" gear="-1" group="-1"/>
      <motor joint="slide" gear="5"  group="0"/>
      <motor joint="slide" gear="31" group="31"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  data->ctrl[0] = 1.0;
  data->ctrl[1] = 1.0;
  data->ctrl[2] = 1.0;

  // all actuators active
  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], 35.0);

  // set all bits of disableactuator, only group 1 is disabled
  model->opt.disableactuator = ~0;
  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], 30.0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ActuatorTest, TendonActuatorForceRange) {
  const std::string xml_path = GetTestDataFilePath(kTendonForceClamp);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  EXPECT_EQ(model->tendon_actfrclimited[0], 0);
  EXPECT_EQ(model->tendon_actfrcrange[0], 0);
  EXPECT_EQ(model->tendon_actfrcrange[1], 0);

  EXPECT_EQ(model->tendon_actfrclimited[1], 1);
  EXPECT_EQ(model->tendon_actfrcrange[2], -1);
  EXPECT_EQ(model->tendon_actfrcrange[3], 1);

  EXPECT_EQ(model->tendon_actfrclimited[2], 1);
  EXPECT_EQ(model->tendon_actfrcrange[4], -10);
  EXPECT_EQ(model->tendon_actfrcrange[5], 10);

  EXPECT_EQ(model->tendon_actfrclimited[3], 1);
  EXPECT_EQ(model->tendon_actfrcrange[6], 0);
  EXPECT_EQ(model->tendon_actfrcrange[7], 1);

  data->ctrl[0] = 1;
  data->ctrl[1] = 1;
  data->ctrl[2] = 1;

  data->ctrl[3] = -1;
  data->ctrl[4] = 1;

  data->ctrl[5] = -20;
  data->ctrl[6] = 5;
  data->ctrl[7] = -5;

  mj_forward(model, data);

  EXPECT_NEAR(data->actuator_force[0], 1, 1e-6);
  EXPECT_NEAR(data->actuator_force[1], 1, 1e-6);
  EXPECT_NEAR(data->actuator_force[2], 1, 1e-6);
  EXPECT_NEAR(data->actuator_force[3], -1, 1e-6);
  EXPECT_NEAR(data->actuator_force[4], 1, 1e-6);
  EXPECT_NEAR(data->actuator_force[5], -10, 1e-6);
  EXPECT_NEAR(data->actuator_force[6], 5, 1e-6);
  EXPECT_NEAR(data->actuator_force[7], -5, 1e-6);

  EXPECT_EQ(data->sensordata[0], 3);
  EXPECT_EQ(data->sensordata[1], 0);
  EXPECT_EQ(data->sensordata[2], -10);
  EXPECT_EQ(data->sensordata[3], 0);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// ----------------------------- actuator delays -------------------------------

TEST_F(ForwardTest, ActuatorDelay) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size="0.1" mass="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="slide" delay="0.02" nsample="2"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // delay = 0.02 seconds, timestep = 0.01, so ndelay = ceil(0.02/0.01) = 2
  EXPECT_EQ(model->actuator_history[0], 2);

  // set ctrl to a nonzero value
  data->ctrl[0] = 10.0;

  // step once: the new ctrl is appended but won't be read for 2 timesteps
  mj_step(model, data);
  // actuator_force should still be 0 (delayed value from buffer init)
  EXPECT_NEAR(data->actuator_force[0], 0.0, 1e-10);

  // step again
  mj_step(model, data);
  // still reading old values
  EXPECT_NEAR(data->actuator_force[0], 0.0, 1e-10);

  // step a third time - now the delayed ctrl should arrive
  mj_step(model, data);
  // actuator_force should now be 10.0
  EXPECT_NEAR(data->actuator_force[0], 10.0, 1e-10);

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Test actuator delay with linear interpolation (interp=1)
// Uses delay = 1.5*timestep so interpolation is meaningful
TEST_F(ForwardTest, ActuatorDelayLinearInterp) {
  constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01"/>
    <worldbody>
      <body>
        <joint name="slide" type="slide"/>
        <geom size="0.1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="slide" delay="0.015" nsample="3" interp="linear"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // delay = 0.015 seconds = 1.5*timestep, nsample=3, interp=1 (linear)
  EXPECT_EQ(model->actuator_history[0], 3);
  EXPECT_EQ(model->actuator_history[1], 1);  // interp=1 (linear)
  EXPECT_NEAR(model->actuator_delay[0], 0.015, MjTol(1e-10, 5e-6));

  // Set increasing ctrl values
  // Buffer has samples at times: -0.02, -0.01, 0 with values 0, 0, 0
  // After step 0 at time=0.01: buffer has times -0.01, 0, 0.01 with values 0, 0, ctrl[0]
  // Read at time 0.01 - 0.015 = -0.005: interpolate between t=-0.01 and t=0
  // Since both values are 0, expected actuator_force = 0

  data->ctrl[0] = 10.0;
  mj_step(model, data);
  EXPECT_NEAR(data->actuator_force[0], 0.0, MjTol(1e-10, 5e-6)) << "step 0";

  // After step 1 at time=0.02: buffer has times 0, 0.01, 0.02 with values 0, 10, 20
  // Read at time 0.02 - 0.015 = 0.005: interpolate between t=0 (val=0) and t=0.01 (val=10)
  // Expected: 0 * 0.5 + 10 * 0.5 = 5

  data->ctrl[0] = 20.0;
  mj_step(model, data);
  EXPECT_NEAR(data->actuator_force[0], 5.0, MjTol(1e-10, 5e-6)) << "step 1";

  // After step 2 at time=0.03: buffer has times 0.01, 0.02, 0.03 with values 10, 20, 30
  // Read at 0.03 - 0.015 = 0.015: interpolate between t=0.01 (val=10) and t=0.02 (val=20)
  // Expected: 10 * 0.5 + 20 * 0.5 = 15

  data->ctrl[0] = 30.0;
  mj_step(model, data);
  EXPECT_NEAR(data->actuator_force[0], 15.0, MjTol(1e-10, 5e-6)) << "step 2";

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(ForwardTest, FlexTrilinearInstability) {
  // model parameters matches user's trilinear.xml
  constexpr char xml[] = R"(
  <mujoco model="stability_test">
      <option gravity="0 0 -9.81" iterations="100" solver="CG" tolerance="1e-10"
              timestep="0.002" integrator="implicitfast">
          <flag warmstart="disable" island="disable"/>
      </option>
      <worldbody>
          <geom name="floor" size="0 0 .05" type="plane" condim="3"/>
          <flexcomp name="bed" type="grid" count="17 17 3" spacing="0.05 0.05 0.05"
                    pos="0 0 0.05" radius="0.0005" dim="3" mass="10" dof="trilinear">
              <contact condim="3" solref="0.005 1" solimp=".99 .99 .001" selfcollide="none"/>
              <elasticity young="865067.00" poisson="0.1" damping="1"/>
          </flexcomp>
          <body name="box" pos="0.05 0.05 0.5">
              <freejoint/>
              <geom name="box_geom" type="box" size="0.04 0.04 0.04" mass="0.5"
                    solref="0.001 1" solimp="0.99 0.99 0.01"/>
          </body>
      </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  mjData* data = mj_makeData(model);

  // flex stiffness sign checks
  // verify correct sign of flex stiffness derivatives before simulation
  int nv = model->nv;
  mjtNum h = model->opt.timestep;

  // create a test vector
  std::vector<mjtNum> v(nv), Mv(nv), flex_Kv(nv);
  for (int i = 0; i < nv; i++) v[i] = mju_Halton(i, 2) - 0.5;
  mjtNum vnorm = mju_norm(v.data(), nv);
  for (int i = 0; i < nv; i++) v[i] /= vnorm;

  mj_forward(model, data);

  // compute M*v and stiffness contributions
  mj_mulM(model, data, Mv.data(), v.data());

  // note: we use mjd_flexInterp_mulK here (unscaled by h^2) to check raw
  // stiffness logic similar to what we expect in the solver now
  mjtNum* v_copy = (mjtNum*)mju_malloc(nv * sizeof(mjtNum));
  mju_copy(v_copy, v.data(), nv);
  mju_zero(flex_Kv.data(), nv);

  // using mulKD for legacy check consistency, but we know it applies h^2+h*d
  // scaling; actually, let's stick to the high-level property checks from
  // FlexStiffnessSign which used mulKD
  mjd_flexInterp_mulKD(model, data, flex_Kv.data(), v.data(), h);

  // compute v^T*M*v and v^T*scale*K*v
  mjtNum vMv = mju_dot(v.data(), Mv.data(), nv);
  // mulKD returns -scale*K*v, so -flex_Kv = +scale*K*v
  mjtNum vKv = -mju_dot(v.data(), flex_Kv.data(), nv);

  // assertions from FlexStiffnessSign
  EXPECT_GT(vKv, 0) << "Stiffness contribution should be positive";
  EXPECT_GT(vMv + vKv, vMv) << "Full Hessian should exceed M alone";

  mju_free(v_copy);

  // stability simulation
  // run for steps to catch instability
  for (int i = 0; i < 2000; ++i) {
    mj_step(model, data);

    for (int j = 0; j < model->nq; ++j) {
      if (mju_abs(data->qpos[j]) > 1000.0) {
        ADD_FAILURE() << "Instability detected at step " << i << " dof " << j
                      << " val " << data->qpos[j];
        return;  // Exit early
      }
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Verify that flex damping does not affect rigid body motion
TEST_F(ForwardTest, FlexDampingRigidMotion) {
  constexpr char xml[] = R"(
  <mujoco>
      <option gravity="0 0 0" timestep="0.01" integrator="implicitfast"/>
      <worldbody>
          <flexcomp name="flex" type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
                    pos="0 0 0" euler="45 45 45" radius="0.01" dim="3" mass="1" dof="trilinear">
              <contact selfcollide="none"/>
              <elasticity young="1e5" poisson="0.3" damping="10"/>
          </flexcomp>
      </worldbody>
  </mujoco>
  )";
  char error[1024];
  mjModel* model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // Set initial rigid rotation velocity about Z axis
  // Center of mass is roughly at 0 0 0 because pos="0 0 0" and symmetric grid.
  // v = w x r. Let w = (1, 1, 1).
  mjtNum w[3] = {10.0, 10.0, 10.0};
  for (int i = 0; i < model->nv / 3; ++i) {
    int qpos_adr = model->jnt_qposadr[i];
    int qvel_adr = model->jnt_dofadr[i];
    mjtNum* pos = data->qpos + qpos_adr;
    mjtNum* vel = data->qvel + qvel_adr;

    mjtNum r[3] = {pos[0], pos[1], pos[2]};
    mju_cross(vel, w, r);
  }

  mj_forward(model, data);
  mjtNum initial_energy = data->energy[0] + data->energy[1];

  // Run a few steps
  for (int i = 0; i < 10; ++i) {
    mj_step(model, data);
  }

  mj_forward(model, data);
  mjtNum final_energy = data->energy[0] + data->energy[1];

  // Expect energy conservation.
  // With the bug, damping force acts on rigid rotation, dissipating energy.
  EXPECT_NEAR(final_energy, initial_energy, 1e-6 * initial_energy)
      << "Energy decayed significantly (" << initial_energy << " -> "
      << final_energy << ")";

  mj_deleteData(data);
  mj_deleteModel(model);
}

// verify that implicit integrator respects parent-flex coupling
TEST_F(ForwardTest, FlexParentCoupling) {
  static const char* const kXml = R"(
  <mujoco>
    <option integrator="implicit" timestep="0.01"/>
    <worldbody>
      <body name="parent" pos="0 0 0">
        <freejoint/>
        <geom size=".1" mass="0.1"/>
        <flexcomp name="flex" type="grid" count="3 3 3" spacing="1 1 1"
                  radius=".01" dim="3" mass="100" dof="trilinear" pos="1 1 1">
          <contact selfcollide="none"/>
          <elasticity young="1e4" poisson="0.3" damping="50"/>
        </flexcomp>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  mjData* data = mj_makeData(model);

  // set state: parent moving, flex deformed
  // this ensures both H_fp (coupling) and qacc_parent are non-trivial
  // Run with Euler (timestep 1e-6)
  model->opt.timestep = 1e-6;
  model->opt.integrator = mjINT_EULER;
  mj_resetData(model, data);
  data->qvel[0] = 1.0;
  data->qpos[7] += 0.01;
  data->qfrc_applied[0] = 10000.0;  // Apply large force to parent
  mj_step(model, data);             // Step integrates
  std::vector<mjtNum> qvel_euler(model->nv);
  mju_copy(qvel_euler.data(), data->qvel, model->nv);

  // Run with Implicit (timestep 1e-6)
  model->opt.integrator = mjINT_IMPLICIT;
  mj_resetData(model, data);
  data->qvel[0] = 1.0;
  data->qpos[7] += 0.01;
  data->qfrc_applied[0] = 10000.0;
  mj_step(model, data);  // Step integrates
  std::vector<mjtNum> qvel_implicit(model->nv);
  mju_copy(qvel_implicit.data(), data->qvel, model->nv);

  // Check agreement
  double max_diff = 0;
  for (int i = 0; i < model->nv; ++i) {
    double diff = mju_abs(qvel_euler[i] - qvel_implicit[i]);
    if (diff > max_diff) max_diff = diff;
  }

  EXPECT_LT(max_diff, MjTol(2e-5, 5e-3))
      << "Implicit integrator should match Euler at small timestep";

  mj_deleteData(data);
  mj_deleteModel(model);
}


TEST_F(ForwardTest, TrilinearPinnedParentWithFreejoint) {
  static constexpr char xml[] = R"(
  <mujoco>
  <option integrator="implicitfast"/>
  <worldbody>
    <body>
      <joint type="free"/>
      <geom type="box" size="0.13 0.18 0.036" pos="0 0 0.036"/>
      <body name="parent">
        <flexcomp name="test" type="grid"
                  count="3 3 3" spacing=".1 .02 .1" radius="0.001"
                  pos="0 0 0.1" dof="trilinear" xyaxes="0 1 0 0 0 1" mass="10" dim="3">
          <contact selfcollide="none"/>
          <elasticity young="1e5" poisson="0.3" damping="0.1"/>
          <pin id="0 2 4 6"/>
        </flexcomp>
      </body>
    </body>
  </worldbody>
  </mujoco>
  )";
  std::array<char, 1024> error;
  mjModel* m = LoadModelFromString(xml, error.data(), error.size());
  ASSERT_THAT(m, NotNull()) << error.data();
  mjData* d = mj_makeData(m);

  int parent_id = mj_name2id(m, mjOBJ_BODY, "parent");
  ASSERT_GT(parent_id, 0);

  EXPECT_EQ(m->nflexnode, 8);
  EXPECT_EQ(m->body_dofnum[parent_id], 0) << "parent body should have 0 DOFs";

  int freejoint_body = m->body_parentid[parent_id];
  EXPECT_EQ(m->body_dofnum[freejoint_body], 6) << "freejoint body has 6 DOFs";

  mj_resetData(m, d);
  mj_forward(m, d);

  for (int i = 0; i < 500; i++) {
    mj_step(m, d);

    ASSERT_FALSE(mju_isBad(d->qpos[0]))
        << "Simulation became unstable at step " << i;
    ASSERT_FALSE(mju_isBad(d->qvel[0]))
        << "Velocity became unstable at step " << i;

    for (int j = 0; j < m->nq; j++) {
      ASSERT_LT(mju_abs(d->qpos[j]), 100.0)
          << "Position exploded at step " << i << ", qpos[" << j
          << "]=" << d->qpos[j];
    }
    for (int j = 0; j < m->nv; j++) {
      ASSERT_LT(mju_abs(d->qvel[j]), 1000.0)
          << "Velocity exploded at step " << i << ", qvel[" << j
          << "]=" << d->qvel[j];
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
}

// -------------------- actuator damping and armature --------------------------

using ActuatorDampingTest = MujocoTest;

TEST_F(ActuatorDampingTest, SingleActuatorJointDamping) {
  // actuator damping=3 with gear=2 should produce same force as
  // joint damping=12 (3*2^2=12)
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="2" damping="3"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_joint[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"
               damping="12"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_joint, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_forward(m1, d1);

  mj_resetDataKeyframe(m2, d2, 0);
  mj_forward(m2, d2);

  EXPECT_EQ(d1->qfrc_passive[0], d2->qfrc_passive[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, SingleActuatorTendonDamping) {
  // actuator damping through tendon transmission
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <tendon>
      <fixed name="ten">
        <joint joint="jnt" coef="1"/>
      </fixed>
    </tendon>
    <actuator>
      <motor tendon="ten" gear="2" damping="3"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_tendon[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <tendon>
      <fixed name="ten" damping="12">
        <joint joint="jnt" coef="1"/>
      </fixed>
    </tendon>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_tendon, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_forward(m1, d1);

  mj_resetDataKeyframe(m2, d2, 0);
  mj_forward(m2, d2);

  EXPECT_EQ(d1->qfrc_passive[0], d2->qfrc_passive[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, SingleActuatorArmature) {
  // actuator armature=0.5 with gear=3 should equal
  // joint armature=4.5 (0.5*3^2=4.5)
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="3" armature="0.5"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_joint[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"
               armature="4.5"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_joint, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_forward(m1, d1);

  mj_resetDataKeyframe(m2, d2, 0);
  mj_forward(m2, d2);

  EXPECT_EQ(d1->qacc[0], d2->qacc[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, MultipleActuatorsAccumulate) {
  // two actuators: damping=2 gear=3, damping=1 gear=4
  // equivalent joint damping: 2*9 + 1*16 = 34
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="3" damping="2"/>
      <motor joint="jnt" gear="4" damping="1"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_joint[] = R"(
  <mujoco>
    <option gravity="0 0 0"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"
               damping="34"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_joint, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_forward(m1, d1);

  mj_resetDataKeyframe(m2, d2, 0);
  mj_forward(m2, d2);

  EXPECT_EQ(d1->qfrc_passive[0], d2->qfrc_passive[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, DampingSimulationEquivalence) {
  // actuator damping=5 gear=2 should match joint damping=20 over time
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="2" damping="5"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_joint[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="0 0 1"
               damping="20"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_joint, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_resetDataKeyframe(m2, d2, 0);
  for (int i = 0; i < 100; i++) {
    mj_step(m1, d1);
    mj_step(m2, d2);
  }

  EXPECT_MJTNUM_EQ(d1->qpos[0], d2->qpos[0]);
  EXPECT_MJTNUM_EQ(d1->qvel[0], d2->qvel[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, ArmatureSimulationEquivalence) {
  // actuator armature=2 gear=3 should match joint armature=18 over time
  static constexpr char xml_actuator[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="0 0 1"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="3" armature="2"/>
    </actuator>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  static constexpr char xml_joint[] = R"(
  <mujoco>
    <option gravity="0 0 -10"/>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="0 0 1"
               armature="18"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <keyframe>
      <key qvel="1"/>
    </keyframe>
  </mujoco>
  )";
  char error[1024];
  mjModel* m1 = LoadModelFromString(xml_actuator, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  mjModel* m2 = LoadModelFromString(xml_joint, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << error;
  mjData* d2 = mj_makeData(m2);

  mj_resetDataKeyframe(m1, d1, 0);
  mj_resetDataKeyframe(m2, d2, 0);
  for (int i = 0; i < 100; i++) {
    mj_step(m1, d1);
    mj_step(m2, d2);
  }

  EXPECT_MJTNUM_EQ(d1->qpos[0], d2->qpos[0]);
  EXPECT_MJTNUM_EQ(d1->qvel[0], d2->qvel[0]);

  mj_deleteData(d1);
  mj_deleteModel(m1);
  mj_deleteData(d2);
  mj_deleteModel(m2);
}

TEST_F(ActuatorDampingTest, UtilityFunctionValues) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="5" damping="7" armature="3"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  mjtNum poly[mjNPOLY] = {0};
  EXPECT_EQ(mj_actuatorDamping(m, mjOBJ_JOINT, 0, poly), 175);
  EXPECT_EQ(mj_actuatorArmature(m, mjOBJ_JOINT, 0), 75);

  mj_deleteModel(m);
}


TEST_F(ActuatorDampingTest, NonlinearDamping) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="jnt" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="jnt" gear="3" damping="2 0.5 0.1"/>
    </actuator>
  </mujoco>
  )";
  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  // linear damping: 2 * gear^2 = 18
  mjtNum poly0[mjNPOLY] = {0};
  EXPECT_EQ(mj_actuatorDamping(m, mjOBJ_JOINT, 0, poly0), 18);

  // poly coefficients scaled by gear^2
  mjtNum poly[mjNPOLY] = {0};
  mj_actuatorDamping(m, mjOBJ_JOINT, 0, poly);
  EXPECT_MJTNUM_EQ(poly[0], 0.5 * 9);  // 4.5
  EXPECT_MJTNUM_EQ(poly[1], 0.1 * 9);  // 0.9

  mj_deleteModel(m);
}

TEST_F(ActuatorDampingTest, DampingVsKvGearScaling) {
  // Single model with two parallel bodies: one using kv, one using damping.
  // Both produce the same joint-space damping force:
  //   kv:      qfrc_actuator contribution = -kv * gear^2 * qvel
  //   damping: qfrc_passive  contribution = -damping * gear^2 * qvel
  static constexpr char xml[] = R"(
  <mujoco>
    <option gravity="0 0 0" integrator="implicitfast"/>
    <worldbody>
      <body name="kv_body">
        <joint name="jnt_kv" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
      <body name="damp_body" pos="5 0 0">
        <joint name="jnt_damp" type="slide" axis="1 0 0"/>
        <geom size="1"/>
      </body>
    </worldbody>
    <actuator>
      <position joint="jnt_kv" kp="0" kv="5" gear="3"/>
      <position joint="jnt_damp" kp="0" damping="5" gear="3"/>
    </actuator>
    <keyframe>
      <key qvel="1 1"/>
    </keyframe>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // check forces at initial state
  mj_resetDataKeyframe(m, d, 0);
  mj_forward(m, d);

  // kv force arrives via qfrc_actuator, damping via qfrc_passive
  mjtNum frc_kv = d->qfrc_actuator[0];
  mjtNum frc_damp = d->qfrc_passive[1];
  EXPECT_NEAR(frc_kv, frc_damp, MjTol(1e-12, 1e-5));

  // expected force = -5 * 3^2 * 1 = -45
  EXPECT_NEAR(frc_damp, -45, MjTol(1e-12, 1e-5));

  // simulate and check trajectory equivalence
  mj_resetDataKeyframe(m, d, 0);
  for (int i = 0; i < 100; i++) {
    mj_step(m, d);
  }

  EXPECT_NEAR(d->qpos[0], d->qpos[1], MjTol(1e-12, 1e-5))
      << "position trajectory mismatch";
  EXPECT_NEAR(d->qvel[0], d->qvel[1], MjTol(1e-12, 1e-5))
      << "velocity trajectory mismatch";

  mj_deleteData(d);
  mj_deleteModel(m);
}

}  // namespace
}  // namespace mujoco
