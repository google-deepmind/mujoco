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

// Tests for engine/engine_derivative.c.

#include "src/engine/engine_derivative.h"

#include <cstddef>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "src/engine/engine_derivative_fd.h"
#include "src/engine/engine_forward.h"
#include "src/engine/engine_io.h"
#include "src/engine/engine_util_blas.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::vector;
using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Each;
using ::testing::NotNull;
using DerivativeTest = MujocoTest;

// errors smaller than this are ignored
static const mjtNum absolute_tolerance = 1e-9;

// corrected relative error
static mjtNum RelativeError(mjtNum a, mjtNum b) {
  mjtNum nominator = mjMAX(0, mju_abs(a-b) - absolute_tolerance);
  mjtNum denominator = (mju_abs(a) + mju_abs(b) + absolute_tolerance);
  return nominator / denominator;
}

// expect two 2D arrays to have elementwise relative error smaller than eps
// return maximum absolute error
static mjtNum CompareMatrices(mjtNum* Actual, mjtNum* Expected,
                              int nrow, int ncol, mjtNum eps) {
  mjtNum max_error = 0;
  for (int i=0; i < nrow; i++) {
    for (int j=0; j < ncol; j++) {
      mjtNum actual = Actual[i*ncol+j];
      mjtNum expected = Expected[i*ncol+j];
      EXPECT_LT(RelativeError(actual, expected), eps)
          << "error at position (" << i << ", " << j << ")"
          << "\nexpected = " << expected
          << "\nactual   = " << actual
          << "\ndiff     = " << expected-actual;
      max_error = mjMAX(mju_abs(actual-expected), max_error);
    }
  }
  return max_error;
}

static const char* const kEnergyConservingPendulumPath =
    "engine/testdata/derivative/energy_conserving_pendulum.xml";
static const char* const kTumblingThinObjectPath =
    "engine/testdata/derivative/tumbling_thin_object.xml";
static const char* const kTumblingThinObjectEllipsoidPath =
    "engine/testdata/derivative/tumbling_thin_object_ellipsoid.xml";
static const char* const kDampedActuatorsPath =
    "engine/testdata/derivative/damped_actuators.xml";
static const char* const kDamperActuatorsPath =
    "engine/testdata/actuation/damper.xml";
static const char* const kDampedPendulumPath =
    "engine/testdata/derivative/damped_pendulum.xml";
static const char* const kLinearPath =
    "engine/testdata/derivative/linear.xml";
static const char* const kModelPath = "testdata/model.xml";

// compare analytic and finite-difference d_smooth/d_qvel
TEST_F(DerivativeTest, SmoothDvel) {
  // run test on all models
  for (const char* local_path : {kEnergyConservingPendulumPath,
                                 kTumblingThinObjectPath,
                                 kDampedActuatorsPath,
                                 kDamperActuatorsPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int nD = model->nD;
    mjData* data = mj_makeData(model);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      // set sparsity
      model->opt.jacobian = sparsity;

      // take 100 steps so we have some velocities, then call forward
      mj_resetData(model, data);
      if (model->nu) {
        data->ctrl[0] = 0.1;
      }
      for (int i=0; i < 100; i++) {
        mj_step(model, data);
      }
      mj_forward(model, data);

      // construct sparse structure in d->D_xxx, compute analytical qDeriv
      mju_zero(data->qDeriv, nD);
      mjd_smooth_vel(model, data, /*flg_bias=*/true);

      // expect derivatives to be non-zero, make copy of qDeriv as a vector
      EXPECT_GT(mju_norm(data->qDeriv, nD), 0);
      vector<mjtNum> qDerivAnalytic = AsVector(data->qDeriv, nD);

      // compute finite-difference derivatives
      mjtNum eps = 1e-7;
      mju_zero(data->qDeriv, nD);
      mjd_smooth_velFD(model, data, eps);

      // expect FD and analytic derivatives to be numerically different
      EXPECT_NE(mju_norm(data->qDeriv, nD),
                mju_norm(qDerivAnalytic.data(), nD));

      // expect FD and analytic derivatives to be similar to eps precision
      EXPECT_THAT(AsVector(data->qDeriv, nD),
                  Pointwise(DoubleNear(eps), qDerivAnalytic));
    }
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

// disabled actuators do not contribute to d_qfrc_actuator/d_qvel
TEST_F(DerivativeTest, DisabledActuators) {
  // model with only a position actuator
  static constexpr char xml1[] = R"(
  <mujoco>
    <option integrator="implicitfast"/>

    <worldbody>
      <body>
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <position joint="joint" group="1" kp="2000" kv="200"/>
    </actuator>
  </mujoco>
  )";

  char error[1024];
  mjModel* m1 = LoadModelFromString(xml1, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << error;
  mjData* d1 = mj_makeData(m1);

  d1->ctrl[0] = 6;
  while (d1->time < 1)
    mj_step(m1, d1);

  // model with a position actuator and an intvelocity actuator
  static constexpr char xml2[] = R"(
  <mujoco>
    <option integrator="implicitfast" actuatorgroupdisable="2"/>

    <worldbody>
      <body>
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <position joint="joint" group="1" kp="2000" kv="200"/>
      <intvelocity joint="joint" group="2" kp="2000" kv="200" actrange="-6 6"/>
    </actuator>
  </mujoco>
  )";

  mjModel* m2 = LoadModelFromString(xml2);
  mjData* d2 = mj_makeData(m2);

  d2->ctrl[0] = 6;
  d2->ctrl[1] = 6;

  while (d2->time < 1)
    mj_step(m2, d2);

  // expect same qvel in both models
  EXPECT_EQ(d1->qvel[0], d2->qvel[0]);

  mj_deleteData(d2);
  mj_deleteModel(m2);
  mj_deleteData(d1);
  mj_deleteModel(m1);
}

// actuator order has no effect
TEST_F(DerivativeTest, ActuatorOrder) {
  // model with stateful actuator first
  static constexpr char xml1[] = R"(
  <mujoco>
    <option integrator="implicitfast"/>

    <worldbody>
      <body>
        <joint name="0" type="slide" range="-1 1"/>
        <geom size=".1"/>
      </body>
      <body pos="1 0 0">
        <joint name="1" type="slide" range="-1 1"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <muscle joint="0" ctrlrange="0 6"/>
      <damper joint="1" kv="200" ctrlrange="0 6"/>
    </actuator>
  </mujoco>
  )";

  char error[1024];
  mjModel* m1 = LoadModelFromString(xml1, error, sizeof(error));
  ASSERT_THAT(m1, NotNull()) << "Failed to load model: " << error;
  mjData* d1 = mj_makeData(m1);

  d1->ctrl[0] = 6;
  d1->ctrl[1] = 6;

  while (d1->time < 1)
    mj_step(m1, d1);

  // model with stateful actuator second
  static constexpr char xml2[] = R"(
  <mujoco>
    <option integrator="implicitfast"/>

    <worldbody>
      <body>
        <joint name="0" type="slide" range="-1 1"/>
        <geom size=".1"/>
      </body>
      <body pos="1 0 0">
        <joint name="1" type="slide" range="-1 1"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <damper joint="1" kv="200" ctrlrange="0 6"/>
      <muscle joint="0" ctrlrange="0 6"/>
    </actuator>
  </mujoco>
  )";

  mjModel* m2 = LoadModelFromString(xml2, error, sizeof(error));
  ASSERT_THAT(m2, NotNull()) << "Failed to load model: " << error;
  mjData* d2 = mj_makeData(m2);

  d2->ctrl[0] = 6;
  d2->ctrl[1] = 6;

  while (d2->time < 1)
    mj_step(m2, d2);

  // expect same qvel in both models
  EXPECT_EQ(d1->qvel[0], d2->qvel[0]);
  EXPECT_EQ(d1->qvel[1], d2->qvel[1]);

  mj_deleteData(d2);
  mj_deleteModel(m2);
  mj_deleteData(d1);
  mj_deleteModel(m1);
}

// compare analytic and fin-diff d_qfrc_passive/d_qvel
TEST_F(DerivativeTest, PassiveDvel) {
  for (const char* local_path : {kTumblingThinObjectPath,
                                 kTumblingThinObjectEllipsoidPath}) {
    // load model
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int nD = model->nD;
    mjData* data = mj_makeData(model);
    // allocate Jacobians
    mjtNum* qDerivAnalytic = (mjtNum*) mju_malloc(sizeof(mjtNum)*nD);
    mjtNum* qDerivFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*nD);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      // set sparsity
      model->opt.jacobian = sparsity;

      // take 100 steps so we have some velocities, then call forward
      mj_resetData(model, data);
      for (int i=0; i < 100; i++) {
        mj_step(model, data);
      }
      mj_forward(model, data);

      // get analytic derivatives
      mju_copy(qDerivAnalytic, data->qDeriv, nD);

      // clear qDeriv, get finite-difference derivatives
      mju_zero(data->qDeriv, nD);
      mju_zero(qDerivFD, nD);
      mjtNum eps = 1e-6;
      mjd_passive_velFD(model, data, eps);

      // expect FD and analytic derivatives to be similar to tol precision
      mjtNum tol = 1e-4;
      EXPECT_THAT(AsVector(data->qDeriv, nD),
                  Pointwise(DoubleNear(tol), AsVector(qDerivAnalytic, nD)));
    }

    mju_free(qDerivFD);
    mju_free(qDerivAnalytic);
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

// ----------------------- derivatives of mj_step() ----------------------------

// mj_stepSkip computes the same next state as mj_step
TEST_F(DerivativeTest, StepSkip) {
  const std::string xml_path = GetTestDataFilePath(kDampedPendulumPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);
  int nq = model->nq;
  int nv = model->nv;

  // disable warm-starts so we don't need to save qacc_warmstart
  model->opt.disableflags |= mjDSBL_WARMSTART;

  for (const mjtIntegrator integrator : {mjINT_EULER,
                                         mjINT_IMPLICIT,
                                         mjINT_IMPLICITFAST}) {
    model->opt.integrator = integrator;

    // reset, take 20 steps
    mj_resetData(model, data);
    for (int i=0; i < 20; i++) {
      mj_step(model, data);
    }

    // denormalize the quat, just to see that it doesn't make a difference
    for (int j=0; j < model->njnt; j++) {
      if (model->jnt_type[j] == mjJNT_BALL) {
        int adr = model->jnt_qposadr[j];
        for (int k=0; k < 4; k++) {
          data->qpos[adr + k] *= 8;
        }
      }
    }

    // save state
    vector<mjtNum> qpos = AsVector(data->qpos, nq);
    vector<mjtNum> qvel = AsVector(data->qvel, nv);

    // take one more step, save next state
    mj_step(model, data);
    vector<mjtNum> qpos_next = AsVector(data->qpos, nq);
    vector<mjtNum> qvel_next = AsVector(data->qvel, nv);

    // reset state, take step again, compare (assert mj_step is deterministic)
    mju_copy(data->qpos, qpos.data(), nq);
    mju_copy(data->qvel, qvel.data(), nv);
    mj_step(model, data);
    EXPECT_THAT(AsVector(data->qpos, nq), Pointwise(Eq(), qpos_next));
    EXPECT_THAT(AsVector(data->qvel, nv), Pointwise(Eq(), qvel_next));

    // reset state, change ctrl, call mj_stepSkip, save next state
    mju_copy(data->qpos, qpos.data(), nq);
    mju_copy(data->qvel, qvel.data(), nv);
    data->ctrl[0] = 1;
    mj_stepSkip(model, data, mjSTAGE_VEL, 0);  // skipping both POS and VEL
    vector<mjtNum> qpos_next_dctrl = AsVector(data->qpos, nq);
    vector<mjtNum> qvel_next_dctrl = AsVector(data->qvel, nv);

    // reset state (ctrl remains unchanged), call full mj_step, compare
    mju_copy(data->qpos, qpos.data(), nq);
    mju_copy(data->qvel, qvel.data(), nv);
    mj_step(model, data);
    EXPECT_THAT(AsVector(data->qpos, nq), Pointwise(Eq(), qpos_next_dctrl));
    EXPECT_THAT(AsVector(data->qvel, nv), Pointwise(Eq(), qvel_next_dctrl));

    // reset state, change velocity, call mj_stepSkip, save next state
    mju_copy(data->qpos, qpos.data(), nq);
    mju_copy(data->qvel, qvel.data(), nv);
    data->qvel[0] += 1;
    mj_stepSkip(model, data, mjSTAGE_POS, 0);  // skipping POS
    vector<mjtNum> qpos_next_dvel = AsVector(data->qpos, nq);
    vector<mjtNum> qvel_next_dvel = AsVector(data->qvel, nv);

    // reset state, change velocity, call full mj_step, compare
    mju_copy(data->qpos, qpos.data(), nq);
    mju_copy(data->qvel, qvel.data(), nv);
    data->qvel[0] += 1;
    mj_step(model, data);
    EXPECT_THAT(AsVector(data->qpos, nq), Pointwise(Eq(), qpos_next_dvel));
    EXPECT_THAT(AsVector(data->qvel, nv), Pointwise(Eq(), qvel_next_dvel));
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Analytic transition matrices for linear dynamical system xn = A*x + B*u
//   given modified mass matrix H (`data->qH`) and
//   Ac = H^-1 [diag(-stiffness) diag(-damping)]
//   we have
//   A  = eye(2*nv) + dt [dt*Ac + [zeros(3) eye(3)]; Ac]
//   given the moment arm matrix K (`data->actuator_moment`) and Bc = H^-1 K
//   B  = dt*[Bc*dt; Bc]
static void LinearSystem(const mjModel* m, mjData* d, mjtNum* A, mjtNum* B) {
  int nv = m->nv, nu = m->nu;
  mjtNum dt = m->opt.timestep;
  mj_markStack(d);

  // === state-transition matrix A
  if (A) {
    mjtNum *Ac = mj_stackAllocNum(d, 2*nv*nv);
    // Ac = H^-1 [diag(-stiffness) diag(-damping)]
    mju_zero(Ac, 2*nv*nv);
    for (int i=0; i < nv; i++) {
      Ac[i*nv + i]  = -m->jnt_stiffness[i];
      Ac[nv*nv + i*nv + i] = -m->dof_damping[i];
    }
    mj_solveLD(Ac, d->qH, d->qHDiagInv, nv, 2*nv,
               m->M_rownnz, m->M_rowadr, m->M_colind, nullptr);

    // A = [dt*Ac; Ac]
    mju_transpose(A, Ac, 2*nv, nv);
    mju_scl(A, A, dt, nv*2*nv);
    mju_transpose(A+2*nv*nv, Ac, 2*nv, nv);

    // Add eye(nv) to top right quadrant of A
    for (int i=0; i < nv; i++) {
      A[i*2*nv + nv + i] += 1;
    }

    // A *= dt
    mju_scl(A, A, dt, 2*nv*2*nv);

    // A += eye(2*nv)
    for (int i=0; i < 2*nv; i++) {
      A[i*2*nv + i] += 1;
    }
  }

  // === control-transition matrix B
  if (B) {
    mjtNum *Bc = mj_stackAllocNum(d, nu*nv);
    mjtNum *BcT = mj_stackAllocNum(d, nv*nu);
    mju_sparse2dense(Bc, d->actuator_moment, nu, nv, d->moment_rownnz,
                     d->moment_rowadr, d->moment_colind);
    mj_solveLD(Bc, d->qH, d->qHDiagInv, nv, nu,
               m->M_rownnz, m->M_rowadr, m->M_colind, nullptr);
    mju_transpose(BcT, Bc, nu, nv);
    mju_scl(B, BcT, dt*dt, nu*nv);
    mju_scl(B+nu*nv, BcT, dt, nu*nv);
  }

  mj_freeStack(d);
}

// compare FD derivatives to analytic derivatives of linear dynamical system
TEST_F(DerivativeTest, LinearSystem) {
  const std::string xml_path = GetTestDataFilePath(kLinearPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);
  int nv = model->nv, nu = model->nu;

  // set ctrl, integrate for 20 steps
  data->ctrl[0] =  .1;
  data->ctrl[1] = -.1;
  for (int i=0; i < 20; i++) {
    mj_step(model, data);
  }

  // analytic A and B
  mjtNum* A = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*2*nv);
  mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);

  LinearSystem(model, data, A, B);

  // uncomment for debugging:
  // PrintMatrix(A, 2*nv, 2*nv);
  // PrintMatrix(B, 2*nv, nu);

  // forward differenced A and B
  mjtNum eps = 1e-6;
  mjtNum* AFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*2*nv);
  mjtNum* BFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);

  mjd_transitionFD(model, data, eps, /*centered=*/0,
                   AFD, BFD, nullptr, nullptr);

  // uncomment for debugging:
  // PrintMatrix(AFD, 2*nv, 2*nv);
  // PrintMatrix(BFD, 2*nv, nu);

  // expect FD and analytic derivatives to be similar to eps precision
  CompareMatrices(A, AFD, 2*nv, 2*nv, eps);
  CompareMatrices(B, BFD, 2*nv, nu, eps);

  // central differenced A and B
  mjtNum* AFDc = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*2*nv);
  mjtNum* BFDc = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);
  mjd_transitionFD(model, data, eps, /*centered=*/1,
                   AFDc, BFDc, nullptr, nullptr);

  // expect central derivatives to be equal to forward differences
  CompareMatrices(AFD, AFDc, 2*nv, 2*nv, eps);
  CompareMatrices(BFD, BFDc, 2*nv, nu, eps);

  mju_free(BFDc);
  mju_free(AFDc);
  mju_free(BFD);
  mju_free(AFD);
  mju_free(B);
  mju_free(A);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// check ctrl derivatives at the range limit
TEST_F(DerivativeTest, ClampedCtrlDerivatives) {
  const std::string xml_path = GetTestDataFilePath(kLinearPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);
  int nv = model->nv, nu = model->nu;

  // set ctrl, integrate for 20 steps
  data->ctrl[0] =  .1;
  data->ctrl[1] = -.1;
  for (int i=0; i < 20; i++) {
    mj_step(model, data);
  }

  // analytic B
  mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);

  LinearSystem(model, data, nullptr, B);

  // forward differenced A and B
  mjtNum eps = 1e-6;
  mjtNum* BFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);

  // set ctrl to the limits, request forward differences
  data->ctrl[0] =  1;
  data->ctrl[1] = -1;
  mjd_transitionFD(model, data, eps, /*centered=*/0,
                   nullptr, BFD, nullptr, nullptr);
  // expect FD and analytic derivatives to be similar to eps precision
  CompareMatrices(B, BFD, 2*nv, nu, eps);

  // ctrl remains at limits, request central differences
  mjd_transitionFD(model, data, eps, /*centered=*/1,
                   nullptr, BFD, nullptr, nullptr);
  // expect FD and analytic derivatives to be similar to eps precision
  CompareMatrices(B, BFD, 2*nv, nu, eps);

  // set ctrl beyond limits, request forward differences
  data->ctrl[0] =  2;
  data->ctrl[1] = -2;
  mjd_transitionFD(model, data, eps, /*centered=*/0,
                   nullptr, BFD, nullptr, nullptr);
  // expect derivatives to be 0
  EXPECT_THAT(AsVector(BFD, 2*nv*nu), Each(Eq(0.0)));

  // expect ctrl to remain unchanged (despite internal clamping)
  EXPECT_EQ(data->ctrl[0],  2.0);
  EXPECT_EQ(data->ctrl[1], -2.0);

  // ctrl remains beyond limits, request centered differences
  mjd_transitionFD(model, data, eps, /*centered=*/1,
                   nullptr, BFD, nullptr, nullptr);
  // expect derivatives to be 0
  EXPECT_THAT(AsVector(BFD, 2*nv*nu), Each(Eq(0.0)));

  mju_free(BFD);
  mju_free(B);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// compare FD sensor derivatives to analytic derivatives
TEST_F(DerivativeTest, SensorDerivatives) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <general name="actuator" joint="joint" gainprm="3"/>
    </actuator>

    <sensor>
      <jointpos joint="joint"/>
      <jointvel joint="joint"/>
      <actuatorfrc actuator="actuator"/>
    </sensor>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml);
  int nv = model->nv, nu = model->nu, ns = model->nsensordata;
  mjData* data = mj_makeData(model);

  // finite differenced C and D
  mjtNum eps = 1e-6;
  mjtNum* CFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*ns*2*nv);
  mjtNum* DFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*ns*nu);
  mjd_transitionFD(model, data, eps, /*centered=*/0,
                   nullptr, nullptr, CFD, DFD);

  // expected analytic C and D
  mjtNum C[6] = {
    1, 0,
    0, 1,
    0, 0
  };

  mjtNum D[3] = {
    0,
    0,
    3,
  };

  // compare expected and actual values
  CompareMatrices(CFD, C, ns, 2*nv, eps);
  CompareMatrices(DFD, D, ns, nu, eps);

  mju_free(DFD);
  mju_free(CFD);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// if sensor derivatives aren't requested, don't compute sensors
TEST_F(DerivativeTest, SensorSkip) {
  static constexpr char xml[] = R"(
  <mujoco>
    <worldbody>
      <body>
        <joint name="joint" type="slide"/>
        <geom size=".1"/>
      </body>
    </worldbody>

    <actuator>
      <general name="actuator" joint="joint" gainprm="3"/>
    </actuator>

    <sensor>
      <jointpos joint="joint"/>
    </sensor>
  </mujoco>
  )";

  mjModel* model = LoadModelFromString(xml);
  int nv = model->nv, nu = model->nu;
  mjData* data = mj_makeData(model);

  // set a sentinel value in the sensor
  data->sensordata[0] = 1337;

  // finite differenced B
  mjtNum eps = 1e-6;
  mjtNum* BFD = (mjtNum*) mju_malloc(sizeof(mjtNum)*2*nv*nu);
  mjd_transitionFD(model, data, eps, /*centered=*/0,
                   nullptr, BFD, nullptr, nullptr);

  EXPECT_EQ(data->sensordata[0], 1337) << "sensors should not be recomputed";

  mju_free(BFD);
  mj_deleteData(data);
  mj_deleteModel(model);
}

// derivatives don't mutate the state
TEST_F(DerivativeTest, NoStateMutation) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  ASSERT_THAT(model, NotNull());
  mjData* data0 = mj_makeData(model);
  mjData* data = mj_makeData(model);
  int nv = model->nv, nu = model->nu, na = model->na, ns = model->nsensordata;

  // set time
  data->time = data0->time = 0.5;

  for (int i=0; i < nv; i++) {
    data->qpos[i] = data0->qpos[i] = (mjtNum) i+1;
    data->qvel[i] = data0->qvel[i] = (mjtNum) i+2;
  }

  // set ctrl
  for (int i=0; i < nu; i++) {
    data->ctrl[i] = data0->ctrl[i] = (mjtNum) i+1;
  }

  // set act
  for (int i=0; i < na; i++) {
    data->act[i] = data0->act[i] = (mjtNum) i+1;
  }


  // allocate Jacobians, call derivatives
  int ndx = nv+nv+na;
  mjtNum* A = (mjtNum*) mju_malloc(sizeof(mjtNum)*ndx*ndx);
  mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*ndx*nu);
  mjtNum* C = (mjtNum*) mju_malloc(sizeof(mjtNum)*ns*ndx);
  mjtNum* D = (mjtNum*) mju_malloc(sizeof(mjtNum)*ns*nu);
  mjtNum eps = 1e-6;
  mjd_transitionFD(model, data, eps, /*centered=*/0, A, B, C, D);

  // compare states in data and data0
  EXPECT_EQ(data->time, data0->time);
  EXPECT_EQ(AsVector(data->qpos, model->nq), AsVector(data0->qpos, model->nq));
  EXPECT_EQ(AsVector(data->qvel, nv), AsVector(data0->qvel, nv));
  EXPECT_EQ(AsVector(data->act, na), AsVector(data0->act, na));
  EXPECT_EQ(AsVector(data->ctrl, nu), AsVector(data0->ctrl, nu));

  mju_free(D);
  mju_free(C);
  mju_free(B);
  mju_free(A);
  mj_deleteData(data);
  mj_deleteData(data0);
  mj_deleteModel(model);
}

// compare dense and sparse derivatives of qfrc_bias (RNE)
TEST_F(DerivativeTest, DenseSparseRneEquivalent) {
  // run test on all models
  for (const char* local_path : {kEnergyConservingPendulumPath,
                                 kTumblingThinObjectPath,
                                 kDampedActuatorsPath,
                                 kDamperActuatorsPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int nD = model->nD;
    mjtNum* qDeriv = (mjtNum*) mju_malloc(sizeof(mjtNum)*nD);
    mjData* data = mj_makeData(model);

    // take 100 steps so we have some velocities, then call forward
    mj_resetData(model, data);
    if (model->nu) {
      data->ctrl[0] = 0.1;
    }
    for (int i=0; i < 100; i++) {
      mj_step(model, data);
    }
    mj_forward(model, data);

    // compute qDeriv with sparse function, make local copy
    mjd_smooth_vel(model, data, /*flg_bias=*/1);
    mju_copy(qDeriv, data->qDeriv, nD);

    // re-compute with dense function
    mju_zero(data->qDeriv, model->nD);
    mjd_actuator_vel(model, data);
    mjd_passive_vel(model, data);
    mjd_rne_vel_dense(model, data);

    // expect dense and sparse derivatives to be similar to eps precision
    mjtNum eps = 1e-12;
    EXPECT_THAT(AsVector(data->qDeriv, nD),
                Pointwise(DoubleNear(eps), AsVector(qDeriv, nD)));

    mj_deleteData(data);
    mju_free(qDeriv);
    mj_deleteModel(model);
  }
}

// compare FD inverse derivatives to analytic derivatives of linear system
TEST_F(DerivativeTest, LinearSystemInverse) {
  const std::string xml_path = GetTestDataFilePath(kLinearPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);

  int nv = model->nv;
  int ns = model->nsensordata;
  int nM = model->nM;

  vector<mjtNum> DfDq(nv*nv);
  vector<mjtNum> DfDv(nv*nv);
  vector<mjtNum> DfDa(nv*nv);
  vector<mjtNum> DsDq(nv*ns);
  vector<mjtNum> DsDv(nv*ns);
  vector<mjtNum> DsDa(nv*ns);
  vector<mjtNum> DmDq(nv*nM);

  // call mj_forward to get accelerations at initial state
  mj_forward(model, data);

  // get derivatives
  mjtNum eps = 1e-6;
  mjtByte flg_actuation = 0;
  mjd_inverseFD(model, data, eps, flg_actuation,
                DfDq.data(), DfDv.data(), DfDa.data(),
                DsDq.data(), DsDv.data(), DsDa.data(),
                DmDq.data());

  // expect that position derivatives are the stiffnesses
  vector<mjtNum> DfDq_expect = {model->jnt_stiffness[0], 0, 0,
                                0, model->jnt_stiffness[1], 0,
                                0, 0, model->jnt_stiffness[2]};
  EXPECT_THAT(DfDq, Pointwise(DoubleNear(eps), DfDq_expect));

  // expect that velocity derivatives are the dampings
  vector<mjtNum> DfDv_expect = {model->dof_damping[0], 0, 0,
                                0, model->dof_damping[1], 0,
                                0, 0, model->dof_damping[2]};
  EXPECT_THAT(DfDv, Pointwise(DoubleNear(eps), DfDv_expect));

  // expect that acceleration derivatives are the mass matrix
  vector<mjtNum> DfDa_expect(nv*nv, 0);
  mj_fullM(model, DfDa_expect.data(), data->qM);
  EXPECT_THAT(DfDa, Pointwise(DoubleNear(eps), DfDa_expect));

  // expect that sensor derivatives w.r.t position only see sensor 1 at dof 0
  vector<mjtNum> DsDq_expect(nv*ns, 0);
  int dof_index = 0;
  int sensordata_index = model->sensor_adr[1];
  DsDq_expect[dof_index*ns + sensordata_index] = 1;
  EXPECT_THAT(DsDq, Pointwise(DoubleNear(eps), DsDq_expect));

  // expect that sensor derivatives w.r.t velocity only see sensor 0 at dof 1
  vector<mjtNum> DsDv_expect(nv*ns, 0);
  dof_index = 1;
  sensordata_index = model->sensor_adr[0];
  DsDv_expect[dof_index*ns + sensordata_index] = 1;
  EXPECT_THAT(DsDv, Pointwise(DoubleNear(eps), DsDv_expect));

  // expect that sensor derivatives w.r.t acceleration see the accelerometer
  // in the y-axis, affected by both dof 0 and dof 1
  vector<mjtNum> DsDa_expect(nv*ns, 0);
  dof_index = 0;
  sensordata_index = model->sensor_adr[2] + 1;
  DsDa_expect[dof_index*ns + sensordata_index] = 1;
  dof_index = 1;
  DsDa_expect[dof_index*ns + sensordata_index] = 1;
  EXPECT_THAT(DsDa, Pointwise(DoubleNear(eps), DsDa_expect));

  // expect that mass matrix derivatives are zero
  vector<mjtNum> DmDq_expect(nv*nM, 0);
  EXPECT_THAT(DmDq, Pointwise(DoubleNear(eps), DmDq_expect));

  mj_deleteData(data);
  mj_deleteModel(model);
}

// utility: generate two random quaternions with a given angle difference
void randomQuatPair(mjtNum qa[4], mjtNum qb[4], mjtNum angle, int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // sample qa = qb
  for (int i=0; i < 4; i++) {
    qa[i] = qb[i] = dist(rng);
  }
  mju_normalize4(qa);
  mju_normalize4(qb);

  // integrate qb in random direction by angle
  mjtNum dir[3];
  for (int i=0; i < 3; i++) {
    dir[i] = dist(rng);
  }
  mju_normalize3(dir);
  mju_quatIntegrate(qb, dir, angle);
}

// utility: finite-difference Jacobians of mju_subQuat
static void subQuatFD(mjtNum Da[9], mjtNum Db[9],
                      const mjtNum qa[4], const mjtNum qb[4], mjtNum eps) {
  // subQuat
  mjtNum y[3];
  mju_subQuat(y, qa, qb);

  mjtNum dq[3];   // nudge input direction
  mjtNum dqa[4];  // nudged qa input
  mjtNum dqb[4];  // nudged qb input
  mjtNum dy[3];   // nudged output
  mjtNum DaT[9];  // Da transposed
  mjtNum DbT[9];  // Db transposed

  for (int i = 0; i < 3; i++) {
    // perturbation
    mju_zero3(dq);
    dq[i] = 1.0;

    // Jacobian: d_y / d_qa
    mju_copy4(dqa, qa);
    mju_quatIntegrate(dqa, dq, eps);
    mju_subQuat(dy, dqa, qb);

    mju_sub3(DaT + i * 3, dy, y);
    mju_scl3(DaT + i * 3, DaT + i * 3, 1.0 / eps);

    // Jacobian: d_y / d_qb
    mju_copy4(dqb, qb);
    mju_quatIntegrate(dqb, dq, eps);
    mju_subQuat(dy, qa, dqb);

    mju_sub3(DbT + i * 3, dy, y);
    mju_scl3(DbT + i * 3, DbT + i * 3, 1.0 / eps);
  }

  // transpose result
  mju_transpose(Da, DaT, 3, 3);
  mju_transpose(Db, DbT, 3, 3);
}

TEST_F(DerivativeTest, SubQuat) {
  const int nrepeats = 10;  // number of repeats
  const mjtNum eps = 1e-7;  // epsilon for finite-differencing and comparison

  int seed = 1;
  for (int i = 0; i < nrepeats; i++) {
    for (mjtNum angle : {0.0, 1e-9, 1e-5, 1e-2, 1.0, 4.0}) {
      // random quaternions
      mjtNum qa[4];
      mjtNum qb[4];

      // make random quaternion pair with given relative angle
      randomQuatPair(qa, qb, angle, seed++);

      // analytic Jacobians
      mjtNum Da[9];  // d_subQuat(qa, qb) / d_qa
      mjtNum Db[9];  // d_subQuat(qa, qb) / d_qb
      mjd_subQuat(qa, qb, Da, Db);

      // finite-differenced Jacobians
      mjtNum DaFD[9];
      mjtNum DbFD[9];
      subQuatFD(DaFD, DbFD, qa, qb, eps);

      // expect numerical equality
      EXPECT_THAT(AsVector(DaFD, 9),
                  Pointwise(DoubleNear(eps), AsVector(Da, 9)));
      EXPECT_THAT(AsVector(DbFD, 9),
                  Pointwise(DoubleNear(eps), AsVector(Db, 9)));
    }
  }
}

// utility: random quaternion, 3D velocity
static void randomQuatVel(mjtNum quat[4], mjtNum vel[3], int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // sample quat
  for (int i=0; i < 4; i++) {
    quat[i] = dist(rng);
  }
  mju_normalize4(quat);

  // sample vel
  for (int i=0; i < 3; i++) {
    vel[i] = dist(rng);
  }
}

// utility: finite-difference Jacobians of mju_quatIntegrate
void mjd_quatIntegrateFD(mjtNum Dquat[9], mjtNum Ds[9],
                         mjtNum Dvel[9], mjtNum Dh[3],
                         const mjtNum quat[4], const mjtNum vel[3],
                         mjtNum h, mjtNum eps) {
  // compute y, output of mju_quatIntegrate(quat, vel, h)
  mjtNum y[4] = {quat[0], quat[1], quat[2], quat[3]};
  mju_quatIntegrate(y, vel, h);

  mjtNum dx[3];      // nudged tangent-space input
  mjtNum dq[4];      // quat output
  mjtNum dy[3];      // nudged tangent-space output
  mjtNum DquatT[9];  // Dquat transposed
  mjtNum DsT[9];     // Ds transposed
  mjtNum DvelT[9];   // Dvel transposed

  for (int i = 0; i < 3; i++) {
    // perturbation
    mju_zero3(dx);
    dx[i] = 1.0;

    // d_y / d_quat
    mju_copy4(dq, quat);
    mju_quatIntegrate(dq, dx, eps);  // nudge dq
    mju_quatIntegrate(dq, vel, h);  // compute nudged
    mju_subQuat(dy, dq, y);          // subtract
    mju_scl3(DquatT + i * 3, dy, 1.0 / eps);

    // d_y / d_sv (scaled velocity)
    mju_copy4(dq, quat);
    mjtNum dsv[3] = {vel[0]*h, vel[1]*h, vel[2]*h};
    mju_addToScl3(dsv, dx, eps);      // nudge dsv
    mju_quatIntegrate(dq, dsv, 1.0);  // compute nudged
    mju_subQuat(dy, dq, y);           // subtract
    mju_scl3(DsT + i * 3, dy, 1.0 / eps);

    // d_y / d_v (unscaled velocity)
    mju_copy4(dq, quat);
    mjtNum dv[3] = {vel[0], vel[1], vel[2]};
    mju_addToScl3(dv, dx, eps);    // nudge dv
    mju_quatIntegrate(dq, dv, h);  // compute nudged
    mju_subQuat(dy, dq, y);        // subtract
    mju_scl3(DvelT + i * 3, dy, 1.0 / eps);
  }

  // d_y / d_h (unscaled velocity)
  mju_copy4(dq, quat);
  mju_quatIntegrate(dq, vel, h + eps);  // compute nudged
  mju_subQuat(dy, dq, y);               // subtract
  mju_scl3(Dh, dy, 1.0 / eps);

  // transpose
  mju_transpose(Dquat, DquatT, 3, 3);
  mju_transpose(Ds, DsT, 3, 3);
  mju_transpose(Dvel, DsT, 3, 3);
}

TEST_F(DerivativeTest, quatIntegrate) {
  const int nrepeats = 10;  // number of repeats
  const mjtNum eps = 1e-7;  // epsilon for finite-differencing and comparison

  int seed = 1;
  for (int i = 0; i < nrepeats; i++) {
    for (mjtNum h : {0.0, 1e-9, 1e-5, 1e-2, 1.0, 4.0}) {
      // make random quaternion and velocity
      mjtNum quat[4];
      mjtNum vel[3];
      randomQuatVel(quat, vel, seed++);

      // analytic Jacobians
      mjtNum Dquat[9];  // d_quatIntegrate(quat, vel, h) / d_quat
      mjtNum Dvel[9];   // d_quatIntegrate(quat, vel, h) / d_vel
      mjtNum Dh[3];     // d_quatIntegrate(quat, vel, h) / d_h
      mjd_quatIntegrate(vel, h, Dquat, Dvel, Dh);

      // finite-differenced Jacobians
      mjtNum DquatFD[9];
      mjtNum DsFD[9];
      mjtNum DvelFD[9];
      mjtNum DhFD[3];
      mjd_quatIntegrateFD(DquatFD, DsFD, DvelFD, DhFD, quat, vel, h, eps);

      // expect numerical equality of un/scaled velocity derivatives
      EXPECT_THAT(AsVector(DvelFD, 9), Pointwise(DoubleNear(eps), DsFD));

      // expect numerical equality of analytic and FD derivatives
      EXPECT_THAT(AsVector(DquatFD, 9), Pointwise(DoubleNear(eps), Dquat));
      EXPECT_THAT(AsVector(DvelFD, 9), Pointwise(DoubleNear(eps), Dvel));
      EXPECT_THAT(AsVector(DhFD, 3), Pointwise(DoubleNear(eps), Dh));
    }
  }
}

// implicit integration is better than Euler with active forcerange clamping
TEST_F(DerivativeTest, ForcerangeClampedDerivative) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="0.01" integrator="implicitfast"/>

    <worldbody>
      <geom name="plane" type="plane" size="2 2 0.1"/>
      <light pos="0 0 3"/>
      <body name="1" pos="0 0 1">
        <joint name="1" type="slide" axis="1 0 0"/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <position joint="1" kp="10000" kv="1000" forcerange="-10 10"/>
    </actuator>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;

  mjtNum dt_small = 1e-4;
  mjtNum dt_large = 1e-2;
  mjtNum duration = 1.0;

  mjData* d_gt = mj_makeData(m);
  mjData* d_implicit = mj_makeData(m);
  mjData* d_euler = mj_makeData(m);

  mj_resetData(m, d_gt);
  mj_resetData(m, d_implicit);
  mj_resetData(m, d_euler);

  d_gt->ctrl[0] = 0.5;
  d_implicit->ctrl[0] = 0.5;
  d_euler->ctrl[0] = 0.5;

  mjtNum error_implicit = 0;
  mjtNum error_euler = 0;
  int nsteps_large = static_cast<int>(duration / dt_large);
  int substeps = static_cast<int>(dt_large / dt_small);

  m->opt.timestep = dt_large;

  m->opt.integrator = mjINT_IMPLICITFAST;
  mj_resetData(m, d_gt);
  d_gt->ctrl[0] = 0.5;
  m->opt.timestep = dt_small;
  m->opt.integrator = mjINT_EULER;

  for (int i = 0; i < nsteps_large; i++) {
    // ground truth: small steps with Euler
    m->opt.integrator = mjINT_EULER;
    m->opt.timestep = dt_small;
    for (int j = 0; j < substeps; j++) {
      mj_step(m, d_gt);
    }

    // euler at large timestep
    m->opt.timestep = dt_large;
    mj_step(m, d_euler);

    // implicitfast at large timestep
    m->opt.integrator = mjINT_IMPLICITFAST;
    mj_step(m, d_implicit);

    // accumulate errors
    mjtNum diff_implicit = d_gt->qpos[0] - d_implicit->qpos[0];
    mjtNum diff_euler = d_gt->qpos[0] - d_euler->qpos[0];
    error_implicit += diff_implicit * diff_implicit;
    error_euler += diff_euler * diff_euler;
  }

  // expect implicitfast to be more accurate than Euler
  EXPECT_LT(error_implicit, error_euler)
      << "implicitfast should be more accurate than Euler at large timestep "
      << "when forcerange derivatives are correctly handled";

  mj_deleteData(d_euler);
  mj_deleteData(d_implicit);
  mj_deleteData(d_gt);
  mj_deleteModel(m);
}

// implicit derivatives should use next activation when actearly is set
TEST_F(DerivativeTest, ActearlyDerivative) {
  static constexpr char xml[] = R"(
  <mujoco>
    <option timestep="1" integrator="implicitfast"/>

    <worldbody>
      <body>
        <joint name="early" type="slide"/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body>
      <body pos="1 0 0">
        <joint name="late" type="slide"/>
        <geom type="sphere" size="0.1" mass="1"/>
      </body>
    </worldbody>

    <actuator>
      <general joint="early" dyntype="integrator" gaintype="affine"
               gainprm="1 0 1" actearly="true"/>
      <general joint="late" dyntype="integrator" gaintype="affine"
               gainprm="1 0 1" actearly="false"/>
    </actuator>
  </mujoco>
  )";

  char error[1024];
  mjModel* m = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(m, NotNull()) << error;
  mjData* d = mj_makeData(m);

  // set identical ctrl with zero initial activation
  d->ctrl[0] = 1.0;
  d->ctrl[1] = 1.0;
  d->act[0] = 0.0;
  d->act[1] = 0.0;

  // step computes derivatives during implicit integration
  mj_step(m, d);

  // both should have same act_dot
  EXPECT_EQ(d->act_dot[0], d->act_dot[1]);

  // with actearly=true and nonzero act_dot, derivative should differ
  // because actearly uses next activation: act + act_dot*dt
  // for our model: next_act = 0 + 1*1 = 1, current_act = 0
  // derivative adds gain_vel * act to qDeriv diagonal
  // for independent bodies, D is diagonal, so diag[i] is at D_rowadr[i]
  int diag0 = m->D_rowadr[0];  // first joint's diagonal
  int diag1 = m->D_rowadr[1];  // second joint's diagonal
  EXPECT_NE(d->qDeriv[diag0], d->qDeriv[diag1])
      << "actearly=true should use next activation in derivative";

  // verify specific values: gain_vel=1, next_act=1, current_act=0
  EXPECT_NEAR(d->qDeriv[diag0], 1.0, 1e-10)
      << "actearly=true should use next_act=1";
  EXPECT_NEAR(d->qDeriv[diag1], 0.0, 1e-10)
      << "actearly=false should use current_act=0";

  mj_deleteData(d);
  mj_deleteModel(m);
}

// Utility: Rotate flex grid
void RotateFlexGrid(mjModel* model, mjData* data, const char* flex_name,
                    double angle) {
  int flex_id = mj_name2id(model, mjOBJ_FLEX, flex_name);
  ASSERT_NE(flex_id, -1);
  int node_adr = model->flex_nodeadr[flex_id];
  int* node_bodies = model->flex_nodebodyid + node_adr;
  int nodenum = model->flex_nodenum[flex_id];

  // Make deterministic quaternion for rotation inside helper
  mjtNum quat[4] = {1, 0, 0, 0};
  if (angle != 0) {
    mjtNum vel[3] = {1, 1, 1};
    mju_normalize3(vel);
    mju_quatIntegrate(quat, vel, angle);
  }

  // reset first to get initial positions
  mj_resetData(model, data);
  mj_forward(model, data);  // Compute initial xpos

  // Update qpos
  for (int i = 0; i < nodenum; i++) {
    int bodyid = node_bodies[i];

    // Only process nodes with valid bodies (FlexInterpDamping assumes this)
    if (bodyid >= 0) {
      mjtNum xpos0[3];
      mju_copy3(xpos0, data->xpos + 3 * bodyid);  // Initial absolute position

      mjtNum xpos_new[3];
      mju_rotVecQuat(xpos_new, xpos0, quat);  // Rotate absolute position

      mjtNum delta[3];
      mju_sub3(delta, xpos_new, xpos0);

      // Find the qpos address for this node/body
      int jnt = model->body_jntadr[bodyid];
      if (jnt >= 0) {
        int qadr = model->jnt_qposadr[jnt];
        mju_addTo3(data->qpos + qadr, delta);
      }
    }
  }
}

// compare analytic and fin-diff d_qfrc_passive/d_qvel for flex interp
// Combined test for verify mjd_flexInterp_mulK (stiffness) and damping
TEST_F(DerivativeTest, FlexInterpDerivatives) {
  static const char* const kXml = R"(
  <mujoco>
    <option integrator="implicit"/>
    <worldbody>
      <flexcomp name="flex" type="grid" count="3 3 3" spacing="0.1 0.2 0.3"
                radius=".01" dim="3" mass="1" dof="trilinear">
        <contact selfcollide="none"/>
        <elasticity young="1e4" poisson="0.3" damping="50"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int nD = model->nD;
  int nv = model->nv;
  ASSERT_EQ(model->nq, 24);  // 8 corners * 3 dofs

  mjData* data = mj_makeData(model);

  // iterate over rotations
  for (mjtNum angle : {0.0, 0.5, 1.0, mjPI / 2, mjPI, 2.0 * mjPI}) {
    RotateFlexGrid(model, data, "flex", angle);
    mj_forward(model, data);

    // part 1: stiffness verification
    {
      std::vector<mjtNum> vec(nv);
      std::vector<mjtNum> res(nv);
      mju_zero(vec.data(), nv);
      // use deterministic random perturbation to verify full stiffness matrix
      // behavior
      for (int i = 0; i < nv; i++) {
        vec[i] = mju_Halton(i, 2) - 0.5;
      }

      // use addH to compute K * vec
      // addH adds (h^2*K + h*D) to H
      // if we set h=1, damping=0, we get K added to H
      mjtNum save_damping = model->flex_damping[0];
      model->flex_damping[0] = 0;

      std::vector<mjtNum> H(nv * nv, 0);
      std::vector<int> dof_indices(nv);
      for (int i = 0; i < nv; i++) dof_indices[i] = i;

      // assemble K into H
      mjd_flexInterp_addH(model, data, H.data(), dof_indices.data(), nv, 1.0);

      // restore damping
      model->flex_damping[0] = save_damping;

      // compute res = K * vec
      mju_mulMatVec(res.data(), H.data(), vec.data(), nv, nv);

      // finite difference of mj_passive for stiffness
      double eps = 1e-6;
      mjData* data_perturbed = mj_copyData(NULL, model, data);

      // apply perturbation
      mju_addToScl(data_perturbed->qpos, vec.data(), eps, nv);

      // recompute geometry/passive
      mj_forward(model, data_perturbed);

      // compute FD estimate of K * vec
      // qfrc_passive = -dV/dq => d(qfrc)/dq = -K
      // (qfrc_new - qfrc)/eps ~= -K * vec
      std::vector<mjtNum> fd_res(nv);
      for (int i = 0; i < nv; ++i) {
        fd_res[i] =
            -(data_perturbed->qfrc_passive[i] - data->qfrc_passive[i]) / eps;
      }

      // compare analytical result (H*vec) with FD result
      for (int i = 0; i < nv; ++i) {
        EXPECT_NEAR(res[i], fd_res[i], 5e-3)
            << "Stiffness Mismatch at DOF " << i;
      }

      mj_deleteData(data_perturbed);

      // check symmetry: K[i,j] == K[j,i]
      std::vector<mjtNum>& K_full = H;
      mjtNum max_asymmetry = 0;
      for (int i = 0; i < nv; i++) {
        for (int j = 0; j < i; j++) {
          mjtNum diff = mju_abs(K_full[i * nv + j] - K_full[j * nv + i]);
          max_asymmetry = mju_max(max_asymmetry, diff);
        }
      }
      EXPECT_LT(max_asymmetry, 1e-10)
          << "K matrix is not symmetric at angle " << angle;

      // check positive semi-definiteness: v^T * K * v >= 0
      for (int trial = 0; trial < 5; trial++) {
        std::vector<mjtNum> v(nv);
        for (int i = 0; i < nv; i++) {
          v[i] = mju_Halton(i + trial * nv, 3) - 0.5;
        }
        mjtNum vKv = 0;
        for (int i = 0; i < nv; i++) {
          for (int j = 0; j < nv; j++) {
            vKv += v[i] * K_full[i * nv + j] * v[j];
          }
        }
        EXPECT_GE(vKv, -1e-8) << "K matrix is not PSD at angle " << angle;
      }
    }

    // part 2: damping verification
    {
      // set velocity non-zero to test damping
      data->qvel[0] = 1.0;

      mj_forward(model, data);

      // get analytic derivatives (without Flex Damping currently)
      std::vector<mjtNum> qDerivAnalytic(nD);
      mju_zero(data->qDeriv, nD);
      mjd_passive_vel(model, data);
      mju_copy(qDerivAnalytic.data(), data->qDeriv, nD);

      // finite-difference derivatives
      std::vector<mjtNum> qDerivFD(nD);
      mju_zero(data->qDeriv, nD);
      mjtNum eps = 1e-6;

      mjd_passive_velFD(model, data, eps);
      mju_copy(qDerivFD.data(), data->qDeriv, nD);

      // check that we have non-zero damping (FD should find it)
      EXPECT_GT(mju_norm(qDerivFD.data(), nD), 1e-3);

      // compute expected flex damping using mjd_flexInterp_addH
      // D = 4*H(0.5) - H(1)
      vector<int> dof_indices(nv);
      for (int i = 0; i < nv; i++) dof_indices[i] = i;

      vector<mjtNum> H1(nv * nv, 0);
      mjd_flexInterp_addH(model, data, H1.data(), dof_indices.data(), nv, 1.0);

      vector<mjtNum> H2(nv * nv, 0);
      mjd_flexInterp_addH(model, data, H2.data(), dof_indices.data(), nv, 0.5);

      vector<mjtNum> D(nv * nv);
      for (int i = 0; i < nv * nv; i++) {
        D[i] = 4.0 * H2[i] - H1[i];
      }

      // subtract D from qDerivAnalytic using sparse indexing
      // d(force)/d(vel) = -D
      for (int i = 0; i < nv; i++) {
        int rownnz = model->D_rownnz[i];
        int rowadr = model->D_rowadr[i];
        for (int k = 0; k < rownnz; k++) {
          int index = rowadr + k;
          int j = model->D_colind[index];
          qDerivAnalytic[index] -= D[i * nv + j];
        }
      }

      // expect FD and corrected analytic derivatives to match
      mjtNum tol = 1e-4;
      EXPECT_THAT(qDerivAnalytic, Pointwise(DoubleNear(tol), qDerivFD))
          << "Damping Mismatch at angle: " << angle;
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

// Test Jacobian under deformation to highlight approximation error
TEST_F(DerivativeTest, FlexInterpDerivativesDeformed) {
  static const char* const kXml = R"(
  <mujoco>
    <option integrator="implicit"/>
    <worldbody>
      <flexcomp name="flex" type="grid" count="3 3 3" spacing="0.1 0.2 0.3"
                radius=".01" dim="3" mass="1" dof="trilinear">
        <contact selfcollide="none"/>
        <elasticity young="1e4" poisson="0.3" damping="0"/>
      </flexcomp>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  mjModel* model = LoadModelFromString(kXml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int nv = model->nv;

  mjData* data = mj_makeData(model);

  // Apply rotation
  RotateFlexGrid(model, data, "flex", 1.0);  // 1 radian rotation

  // Apply deformation (stretch along X)
  // qpos is initialized by RotateFlexGrid.
  // Add a random perturbation to qpos that represents deformation.
  // We use a deterministic sequence to ensure reproducibility.
  std::vector<mjtNum> deformation(nv);
  for (int i = 0; i < nv; i++) {
    // Large deformation to make sure terms are significant
    deformation[i] = (mju_Halton(i, 3) - 0.5) * 0.2;
  }
  mju_addTo(data->qpos, deformation.data(), nv);

  mj_forward(model, data);

  // 1. Compute Analytic Jacobian (Approximate)
  // We use mjd_flexInterp_addH to get K_approx
  std::vector<mjtNum> H_approx(nv * nv, 0);
  std::vector<int> dof_indices(nv);
  for (int i = 0; i < nv; i++) dof_indices[i] = i;

  // h=1, damping=0 => adds K to H
  mjd_flexInterp_addH(model, data, H_approx.data(), dof_indices.data(), nv,
                      1.0);

  // 2. Compute Finite Difference Jacobian (Ground Truth)
  // qfrc_passive = -dV/dq
  // d(qfrc)/dq = -K_true
  std::vector<mjtNum> K_true(nv * nv, 0);
  mjtNum eps = 1e-6;

  for (int i = 0; i < nv; i++) {
    mjData* data_p = mj_copyData(NULL, model, data);
    data_p->qpos[i] += eps;
    mj_forward(model, data_p);

    for (int j = 0; j < nv; j++) {
      // d(force_j)/d(q_i)
      mjtNum df = data_p->qfrc_passive[j] - data->qfrc_passive[j];
      // K_true[j, i] = -df/eps
      K_true[j * nv + i] = -df / eps;
    }
    mj_deleteData(data_p);
  }

  // 3. Compare and check for significant mismatch
  mjtNum max_error = 0;
  for (int i = 0; i < nv * nv; i++) {
    max_error = mju_max(max_error, mju_abs(H_approx[i] - K_true[i]));
  }

  // We expect significant error because of deformation + rotation.
  // The missing term (geometric stiffness) is proportional to stress.
  // We assert that the error is relatively large to confirm the approximation
  // exists.
  EXPECT_GT(max_error, 1e-3)
      << "Jacobian approximation should differ from FD when deformed";

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
