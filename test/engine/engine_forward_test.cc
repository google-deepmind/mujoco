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
#include "src/engine/engine_io.h"
#include "test/fixture.h"

#ifdef MEMORY_SANITIZER
  #include <sanitizer/msan_interface.h>
#endif

namespace mujoco {
namespace {

static const char* const kEnergyConservingPendulumPath =
    "engine/testdata/derivative/energy_conserving_pendulum.xml";
static const char* const kDampedActuatorsPath =
    "engine/testdata/derivative/damped_actuators.xml";
static const char* const kJointForceClamp =
    "engine/testdata/actuation/joint_force_clamp.xml";
static const char* const kTendonForceClamp =
    "engine/testdata/actuation/tendon_force_clamp.xml";

using ::testing::Pointwise;
using ::testing::DoubleNear;
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

  mjModel* model = LoadModelFromString(xml);
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
    if (i >= 99) EXPECT_EQ(data->act[0], 1);
  }

  data->ctrl[0] = -1.0;
  // integrating down from 1, we will hit the clamp after 199 steps
  for (int i=0; i < 300; i++) {
    mj_step(model, data);
    // always smaller than upper bound
    EXPECT_LT(data->act[0], model->actuator_actrange[1]);
    // after 199 steps we hit the lower bound
    if (i < 199) EXPECT_GT(data->act[0], model->actuator_actrange[0]);
    if (i >= 199) EXPECT_EQ(data->act[0], model->actuator_actrange[0]);
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
  mjModel* model = LoadModelFromString(xml);
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

  mjModel* model = LoadModelFromString(xml);
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
  EXPECT_THAT(qacc_fd, Pointwise(DoubleNear(1e-14), qacc));

  // reach the the same initial state
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

  mjModel* model = LoadModelFromString(xml);
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
TEST_F(ImplicitIntegratorTest, EulerImplicitEqivalent) {
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

  mjModel* model = LoadModelFromString(xml);
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
  EXPECT_THAT(AsVector(data->qpos, model->nq), Pointwise(Ne(), qposEuler));

  // expect qpos vectors to be similar to high precision
  EXPECT_THAT(AsVector(data->qpos, model->nq),
              Pointwise(DoubleNear(1e-14), qposEuler));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Joint and actuator damping should integrate identically under implicit
TEST_F(ImplicitIntegratorTest, JointActuatorEqivalent) {
  const std::string xml_path = GetTestDataFilePath(kDampedActuatorsPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  // take 1000 steps with Euler
  for (int i=0; i < 1000; i++) {
    mj_step(model, data);
  }
  // expect corresponding joint values to be significantly different
  EXPECT_GT(fabs(data->qpos[0]-data->qpos[2]), 1e-4);
  EXPECT_GT(fabs(data->qpos[1]-data->qpos[3]), 1e-4);

  // reset, take 1000 steps with implicit
  mj_resetData(model, data);
  model->opt.integrator = mjINT_IMPLICIT;
  for (int i=0; i < 10; i++) {
    mj_step(model, data);
  }

  // expect corresponding joint values to be insignificantly different
  EXPECT_LT(fabs(data->qpos[0]-data->qpos[2]), 1e-16);
  EXPECT_LT(fabs(data->qpos[1]-data->qpos[3]), 1e-16);

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
  mjModel* model = LoadModelFromString(xml);
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
  mjModel* model = LoadModelFromString(xml);
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());

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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());

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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());

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
  MJDATA_POINTERS_PREAMBLE(model)
  #define X(type, name, nr, nc)                                 \
    for (int i = 0; i < model->nr; i++)                         \
      for (int j = 0; j < nc; j++)                              \
        EXPECT_EQ(data_n->name[i*nc+j], data_u->name[i*nc+j]);
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
        EXPECT_EQ(data_n->name[i*nc+j], data_u->name[i*nc+j]);
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());

  mjData* data = mj_makeData(model);
  mj_forward(model, data);

  // expect mocap_quat to be normalized (by the compiler)
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(data->mocap_quat[i], 0.5);
    EXPECT_EQ(data->xquat[4+i], 0.5);
  }

  // write denormalized quats to mocap_quat, call forward again
  for (int i = 0; i < 4; i++) {
    data->mocap_quat[i] = 1;
  }
  mj_forward(model, data);

  // expect mocap_quat to remain denormalized, but xquat to be normalized
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(data->mocap_quat[i], 1);
    EXPECT_EQ(data->xquat[4+i], 0.5);
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
  mjModel* model = LoadModelFromString(xml);
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
  mjModel* model = LoadModelFromString(xml);
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
  EXPECT_EQ(data->actuator_force[0], 1);
  EXPECT_EQ(data->qfrc_actuator[0], 0.4);

  // simulate for 2 seconds to gain velocity
  while (data->time < 2) {
    mj_step(model, data);
  }

  // activate damper, expect force to be clamped at lower bound
  data->ctrl[1] = 1;
  mj_forward(model, data);
  EXPECT_EQ(data->qfrc_actuator[0], -0.4);

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
              Pointwise(DoubleNear(tol), AsVector(data->qpos + 4, 4)));
  EXPECT_THAT(AsVector(data->qvel, 4),
              Pointwise(DoubleNear(tol), AsVector(data->qvel + 4, 4)));

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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
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

  EXPECT_THAT(small_timestep_act, DoubleNear(large_timestep_act, 1e-14))
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
  mjModel* model = LoadModelFromString(xml);
  ASSERT_THAT(model, NotNull());
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
      EXPECT_THAT(last_qfrc[2 * j],
                  DoubleNear(data->qfrc_actuator[2 * j + 1], 1e-3))
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
  mjModel* model = LoadModelFromString(xml);
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
  mjModel* model = LoadModelFromString(xml);
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

}  // namespace
}  // namespace mujoco
