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
using DerivativeTest = MujocoTest;

// errors smaller than this are ignored
static const mjtNum absolute_tolerance = 1e-7;

// corrected relative error
static mjtNum RelativeError(mjtNum a, mjtNum b) {
  mjtNum nominator = mjMAX(0, mju_abs(a-b) - absolute_tolerance);
  mjtNum denominator = (mju_abs(a) + mju_abs(b) + absolute_tolerance);
  return nominator / denominator;
}

// expect two 2D arrays to have elementwise relative error smaller than eps
static void CompareMatrices(mjtNum* Actual, mjtNum* Expected,
                               int nrow, int ncol, mjtNum eps) {
  for (int i=0; i<nrow; i++) {
    for (int j=0; j<ncol; j++) {
      mjtNum actual = Actual[i*ncol+j];
      mjtNum expected = Expected[i*ncol+j];
      EXPECT_LT(RelativeError(actual, expected), eps)
          << "error at position (" << i << ", " << j << ")"
          << "\nexpected = " << expected
          << "\nactual   = " << actual
          << "\ndiff     = " << expected-actual;
    }
  }
}

std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

static const char* const kEnergyConservingPendulumPath =
    "engine/testdata/derivative/energy_conserving_pendulum.xml";
static const char* const kTumblingThinObjectPath =
    "engine/testdata/derivative/tumbling_thin_object.xml";
static const char* const kDampedActuatorsPath =
    "engine/testdata/derivative/damped_actuators.xml";

// compare analytic and finite-difference d_smooth/d_qvel
TEST_F(DerivativeTest, SmoothDvel) {
  // run test on all models
  for (const char* local_path : {kEnergyConservingPendulumPath,
                                 kTumblingThinObjectPath,
                                 kDampedActuatorsPath}) {
    const std::string xml_path = GetTestDataFilePath(local_path);
    mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
    mjData* data = mj_makeData(model);

    for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
      // set sparsity
      model->opt.jacobian = sparsity;

      // take 100 steps so we have some velocities, then call forward
      mj_resetData(model, data);
      for (int i=0; i<100; i++) {
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
  const std::string xml_path = GetTestDataFilePath(kTumblingThinObjectPath);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  int nv = model->nv;
  mjData* data = mj_makeData(model);
  // allocate d_qfrc_passive/d_qvel Jacobians
  mjtNum* DfDv_analytic = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv*nv);
  mjtNum* DfDv_FD = (mjtNum*) mju_malloc(sizeof(mjtNum)*nv*nv);

  for (mjtJacobian sparsity : {mjJAC_DENSE, mjJAC_SPARSE}) {
    // set sparsity
    model->opt.jacobian = sparsity;

    // take 100 steps so we have some velocities, then call forward
    mj_resetData(model, data);
    for (int i=0; i<100; i++) {
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

    // expect FD and analytic derivatives to be similar to eps precision
    CompareMatrices(DfDv_analytic, DfDv_FD, nv, nv, eps);
  }

  mju_free(DfDv_FD);
  mju_free(DfDv_analytic);
  mj_deleteData(data);
  mj_deleteModel(model);

}

}  // namespace
}  // namespace mujoco
