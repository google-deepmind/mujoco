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

// Tests for engine/engine_solver.c

#include <algorithm>
#include <cstdlib>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleNear;
using ::testing::NotNull;
using ::std::abs;
using ::std::max;

using SolverTest = MujocoTest;

static const char* const kModelPath =
    "testdata/model.xml";

// compare accelerations produced by CG solver with and without islands
TEST_F(SolverTest, IslandsEquivalent) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  model->opt.solver = mjSOL_CG;                 // use CG solver
  model->opt.jacobian = mjJAC_SPARSE;           // use sparse
  model->opt.tolerance = 0;                     // set tolerance to 0
  model->opt.ls_tolerance = 0;                  // set ls_tolerance to 0

  int nv = model->nv;

  int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  // Below are 3 tolerances associated with 3 different iteration counts,
  // they are only moderately tight, 2x higher than x86-64 failure on Linux,
  // i.e. in that case the test fails with rtol smaller than {5e-3, 5e-4, 5e-5}.
  // The point of this test is to show that CG convergence is actually not very
  // precise, simply changing whether islands are used changes the solution by
  // quite a lot, even at high iteration count and zero {ls_}tolerance.
  // Increasing the iteration count higher than 60 does not improve convergence.
  constexpr int kNumTol = 3;
  mjtNum maxiter[kNumTol] = {30,   40,   60};
  mjtNum rtol[kNumTol] =    {1e-2, 1e-3, 1e-4};

  for (int i = 0; i < kNumTol; ++i) {
    model->opt.iterations = maxiter[i];
    model->opt.ls_iterations = maxiter[i];

    for (bool coldstart : {true, false}) {
      mj_resetDataKeyframe(model, data_noisland, 0);

      if (coldstart) {
        model->opt.disableflags |= mjDSBL_WARMSTART;
      } else {
        model->opt.disableflags &= ~mjDSBL_WARMSTART;
      }

      while (data_noisland->time < .1) {
        mj_getState(model, data_noisland, state, mjSTATE_INTEGRATION);
        mj_setState(model, data_island, state, mjSTATE_INTEGRATION);

        model->opt.enableflags |= mjENBL_ISLAND;  // enable islands
        mj_forward(model, data_island);

        model->opt.enableflags &= ~mjENBL_ISLAND;  // disable islands
        mj_forward(model, data_noisland);

        auto time = std::to_string(data_noisland->time);
        for (int j = 0; j < nv; j++) {
          // increase tolerance for large elements
          mjtNum scale = 0.5 * max(2.0, abs(data_noisland->qacc[j]) +
                                        abs(data_island->qacc[j]));
          EXPECT_THAT(data_noisland->qacc[j],
                      DoubleNear(data_island->qacc[j], scale * rtol[i]))
              << "time: " << time << '\n'
              << "dof: " << j << '\n'
              << "maxiter: " << maxiter[i] << '\n'
              << "rtol: " << scale * rtol[i];
        }

        mj_step(model, data_noisland);
      }
    }
  }

  mj_deleteData(data_noisland);
  mj_deleteData(data_island);
  mju_free(state);
  mj_deleteModel(model);
}

// compare accelerations produced by CG solver with and without islands
TEST_F(SolverTest, IslandsEquivalentForward) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  model->opt.solver = mjSOL_CG;                 // use CG solver
  model->opt.tolerance = 0;                     // set tolerance to 0
  model->opt.ls_tolerance = 0;                  // set ls_tolerance to 0

  mjtNum rtol = 2e-6;

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  for (bool coldstart : {true, false}) {
    mj_resetDataKeyframe(model, data_island, 0);
    mj_resetDataKeyframe(model, data_noisland, 0);

    if (coldstart) {
      model->opt.disableflags |= mjDSBL_WARMSTART;
    } else {
      model->opt.disableflags &= ~mjDSBL_WARMSTART;
    }

    model->opt.enableflags &= ~mjENBL_ISLAND;  // disable islands
    mj_forward(model, data_noisland);

    model->opt.enableflags |= mjENBL_ISLAND;   // enable islands
    mj_forward(model, data_island);
    for (int j = 0; j < model->nv; j++) {
      mjtNum scale = 0.5 * max(2.0, abs(data_noisland->qacc[j]) +
                                    abs(data_island->qacc[j]));
      EXPECT_THAT(data_noisland->qacc[j],
                  DoubleNear(data_island->qacc[j], scale * rtol))
          << "dof: " << j << '\n'
          << "rtol: " << scale * rtol;
    }
  }

  mj_deleteData(data_noisland);
  mj_deleteData(data_island);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
