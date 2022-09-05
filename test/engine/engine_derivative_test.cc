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

#include <cmath>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "src/engine/engine_derivative.h"
#include "src/engine/engine_io.h"
#include "src/engine/engine_support.h"
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_errmem.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::Eq;
using ::testing::Each;
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

// utility function for matrix printing
static void PrintMatrix(mjtNum* mat, int nrow, int ncol) {
  std::cerr.precision(5);
  std::cerr << "\n";
  for (int r=0; r < nrow; r++) {
    for (int c=0; c < ncol; c++) {
      std::cerr << std::fixed << std::setw(9) << mat[c + r*ncol] << " ";
    }
    std::cerr << "\n";
  }
}


std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
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
    "engine/testdata/damper.xml";
static const char* const kDampedPendulumPath =
    "engine/testdata/derivative/damped_pendulum.xml";
static const char* const kLinearPath =
    "engine/testdata/derivative/linear.xml";
// compare analytic and finite-difference d_smooth/d_qvel
TEST_F(DerivativeTest, SmoothDvel) {
  // run test on all models
  for (const char* local_path : {kEnergyConservingPendulumPath,
                                 kTumblingThinObjectPath,
                                 kDampedActuatorsPath,
                                 kDamperActuatorsPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
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
      mj_makeMSparse(model, data,
                     data->D_rownnz, data->D_rowadr, data->D_colind);
      mjd_smooth_vel(model, data);

      // expect derivatives to be non-zero, make copy of qDeriv as a vector
      EXPECT_GT(mju_norm(data->qDeriv, model->nD), 0);
      std::vector<mjtNum> qDerivAnalytic = AsVector(data->qDeriv, model->nD);

      // compute finite-difference derivatives
      mjtNum eps = 1e-7;
      mjd_smooth_velFD(model, data, eps);

      // expect FD and analytic derivatives to be numerically different
      EXPECT_NE(mju_norm(data->qDeriv, model->nD),
                mju_norm(qDerivAnalytic.data(), model->nD));

      // expect FD and analytic derivatives to be similar to eps precision
      EXPECT_THAT(AsVector(data->qDeriv, model->nD),
                  Pointwise(DoubleNear(eps), qDerivAnalytic));
    }
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

// compare analytic and fin-diff d_qfrc_passive/d_qvel
TEST_F(DerivativeTest, PassiveDvel) {
  for (const char* local_path : {kTumblingThinObjectPath,
                                 kTumblingThinObjectEllipsoidPath}) {
    // load model
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    int nv = model->nv;
    mjData* data = mj_makeData(model);
    // allocate Jacobians
    mjtNum* DfDv_analytic = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv*nv);
    mjtNum* DfDv_FD = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv*nv);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      // set sparsity
      model->opt.jacobian = sparsity;

      // take 100 steps so we have some velocities, then call forward
      mj_resetData(model, data);
      for (int i=0; i < 100; i++) {
        mj_step(model, data);
      }
      mj_forward(model, data);

      // clear DfDv, get analytic derivatives
      mju_zero(DfDv_analytic, nv*nv);
      mjd_passive_vel(model, data, DfDv_analytic);

      // clear DfDv, get finite-difference derivatives
      mju_zero(DfDv_FD, nv*nv);
      mjtNum eps = 1e-6;
      mjd_passive_velFD(model, data, eps, DfDv_FD);

      // expect FD and analytic derivatives to be similar to tol precision
      mjtNum tol = 1e-4;
      CompareMatrices(DfDv_analytic, DfDv_FD, nv, nv, tol);
    }

    mju_free(DfDv_FD);
    mju_free(DfDv_analytic);
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

  // disable warmstarts so we don't need to save qacc_warmstart
  model->opt.disableflags |= mjDSBL_WARMSTART;

  for (const mjtIntegrator integrator : {mjINT_EULER, mjINT_IMPLICIT}) {
    model->opt.integrator = integrator;

    // reset, take 20 steps, save initial state
    mj_resetData(model, data);
    for (int i=0; i < 20; i++) {
      mj_step(model, data);
    }
    std::vector<mjtNum> qpos = AsVector(data->qpos, nq);
    std::vector<mjtNum> qvel = AsVector(data->qvel, nv);

    // take one more step, save next state
    mj_step(model, data);
    std::vector<mjtNum> qpos_next = AsVector(data->qpos, nq);
    std::vector<mjtNum> qvel_next = AsVector(data->qvel, nv);

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
    std::vector<mjtNum> qpos_next_dctrl = AsVector(data->qpos, nq);
    std::vector<mjtNum> qvel_next_dctrl = AsVector(data->qvel, nv);

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
    std::vector<mjtNum> qpos_next_dvel = AsVector(data->qpos, nq);
    std::vector<mjtNum> qvel_next_dvel = AsVector(data->qvel, nv);

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
  mjMARKSTACK;

  // === state-transition matrix A
  if (A) {
    mjtNum *Ac = mj_stackAlloc(d, 2*nv*nv);
    // Ac = H^-1 [diag(-stiffness) diag(-damping)]
    mju_zero(Ac, 2*nv*nv);
    for (int i=0; i < nv; i++) {
      Ac[i*nv + i]  = -m->jnt_stiffness[i];
      Ac[nv*nv + i*nv + i] = -m->dof_damping[i];
    }
    mj_solveLD(m, Ac, Ac, 2*nv, d->qH, d->qHDiagInv);

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
    mjtNum *Bc = mj_stackAlloc(d, nu*nv);
    mjtNum *BcT = mj_stackAlloc(d, nv*nu);
    mj_solveLD(m, Bc, d->actuator_moment, nu, d->qH, d->qHDiagInv);
    mju_transpose(BcT, Bc, nu, nv);
    mju_scl(B, BcT, dt*dt, nu*nv);
    mju_scl(B+nu*nv, BcT, dt, nu*nv);
  }

  mjFREESTACK;
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

  // expect ctrl to remain unchanged (despite intenal clamping)
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


}  // namespace
}  // namespace mujoco
