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

// Tests of the entire pipeline that are not easily associated with one file.

#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_io.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

static const char* const kDefaultModel = "testdata/model.xml";

using ::testing::Pointwise;
using ::testing::DoubleNear;
using PipelineTest = MujocoTest;


// Joint and actuator damping should integrate identically under implicit
TEST_F(PipelineTest, SparseDenseEquivalent) {
  const std::string xml_path = GetTestDataFilePath(kDefaultModel);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(model, nullptr) << error;
  mjData* data = mj_makeData(model);

  // set dense jacobian, call mj_forward, save accelerations
  model->opt.jacobian = mjJAC_DENSE;
  mj_forward(model, data);
  std::vector<mjtNum> qacc_dense = AsVector(data->qacc, model->nv);

  // set sparse jacobian, call mj_forward, save accelerations
  model->opt.jacobian = mjJAC_SPARSE;
  mj_forward(model, data);
  std::vector<mjtNum> qacc_sparse = AsVector(data->qacc, model->nv);

  // expect accelerations to be insignificantly different
  mjtNum tol = 1e-11;
  EXPECT_THAT(qacc_dense, Pointwise(DoubleNear(tol), qacc_sparse));
  // TODO: is 1e-12 larger than we expect?
  //       investigate sources of discrepancy, eliminate if possible

  mj_deleteData(data);
  mj_deleteModel(model);
}

// mj_forward should be idempotent when warm starts are disabled
TEST_F(PipelineTest, DeterministicNoWarmstart) {
  const std::string xml_path = GetTestDataFilePath(kDefaultModel);
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, nullptr, 0);
  mjData* data = mj_makeData(model);
  mjData* data2 = mj_makeData(model);

  // disable warmstarts
  model->opt.disableflags |= mjDSBL_WARMSTART;

  int nv = model->nv;

  int kNumSteps = 50;

  for (mjtSolver solver : {mjSOL_NEWTON, mjSOL_PGS, mjSOL_CG}) {
    model->opt.solver = solver;
    mj_resetData(model, data);
    mj_resetData(model, data2);

    for (int step = 0; step < kNumSteps; step++) {
      mj_step(model, data);
      mj_forward(model, data);

      mj_step(model, data2);
      mj_forward(model, data2);

      // test determinism: both models steps did the same thing
      EXPECT_EQ(AsVector(data->qacc, nv), AsVector(data2->qacc, nv));

      // one more mj_forward call on data2
      mj_forward(model, data2);

      // expect that the extra mj_forward call didn't change anything
      EXPECT_EQ(AsVector(data->qacc, nv), AsVector(data2->qacc, nv));
    }
  }

  mj_deleteData(data2);
  mj_deleteData(data);
  mj_deleteModel(model);
}


}  // namespace
}  // namespace mujoco
