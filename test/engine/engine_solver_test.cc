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
using ::testing::Pointwise;
using ::std::max;

using SolverTest = MujocoTest;

static const char* const kModelPath = "engine/testdata/solver/model.xml";
static const char* const kHumanoidPath = "engine/testdata/solver/humanoid.xml";

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
  model->opt.ccd_tolerance = 0;                 // set ccd_tolerance to 0

  int nv = model->nv;

  int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  // Below are 3 tolerances associated with 3 different iteration counts,
  // they are only moderately tight, 10x higher than x86-64 failure on Linux,
  // i.e. in that case the test fails with rtol smaller than {5e-3, 5e-4, 5e-5}.
  // The point of this test is to show that CG convergence is actually not very
  // precise, simply changing whether islands are used changes the solution by
  // quite a lot, even at high iteration count and zero {ls_}tolerance.
  // Increasing the iteration count higher than 60 does not improve convergence.
  constexpr int kNumTol = 3;
  mjtNum maxiter[kNumTol] = {30,   40,   60};
  mjtNum rtol[kNumTol] =    {5e-2, 5e-3, 5e-4};

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

        model->opt.disableflags &= ~mjDSBL_ISLAND;  // enable islands
        mj_forward(model, data_island);

        model->opt.disableflags |= mjDSBL_ISLAND;  // disable islands
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

// compare accelerations produced by CG/Newton solver with and without islands
TEST_F(SolverTest, IslandsEquivalentForward) {
  const std::string xml_path = GetTestDataFilePath(kModelPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  int nv = model->nv;

  // set tolerance to 0 so opt.iterations are always run
  model->opt.tolerance = 0;

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  for (bool warmstart : {false, true}) {
    for (mjtJacobian jacobian : {mjJAC_DENSE, mjJAC_SPARSE}) {
      for (mjtSolver solver : {mjSOL_CG, mjSOL_NEWTON}) {
        for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
          if (warmstart) {
            model->opt.disableflags &= ~mjDSBL_WARMSTART;
          } else {
            model->opt.disableflags |= mjDSBL_WARMSTART;
          }
          model->opt.jacobian = jacobian;
          model->opt.solver = solver;
          model->opt.cone = cone;

          // disable islands, reset and step both datas to populate warmstart
          model->opt.disableflags |= mjDSBL_ISLAND;
          mj_resetDataKeyframe(model, data_island, 0);
          mj_resetDataKeyframe(model, data_noisland, 0);
          mj_step(model, data_island);
          mj_step(model, data_noisland);

          // forward with islands disabled
          mj_forward(model, data_noisland);

          // forward with islands enabled
          model->opt.disableflags &= ~mjDSBL_ISLAND;   // enable islands
          mj_forward(model, data_island);

          mjtNum scale = 0.5 * (mju_norm(data_noisland->qacc, nv) +
                                mju_norm(data_island->qacc, nv));
          mjtNum tol = scale * (solver == mjSOL_CG ? 1e-6 : 1e-8);
          EXPECT_THAT(AsVector(data_island->qacc, nv),
                      Pointwise(DoubleNear(scale * tol),
                                AsVector(data_noisland->qacc, nv)))
              << "warmstart: " << warmstart << '\n'
              << "jacobian: " << (jacobian ? "sparse" : "dense") << '\n'
              << "solver: " << (solver == mjSOL_CG ? "CG" : "Newton") << '\n'
              << "cone: " << (cone == 1 ? "elliptic" : "pyramidal");
        }
      }
    }
  }

  mj_deleteData(data_noisland);
  mj_deleteData(data_island);
  mj_deleteModel(model);
}

TEST_F(SolverTest, SolversEquivalent) {
  struct SolverTolerances {
    double newton;
    double cg;
    double pgs_pyramidal;
    double pgs_elliptic;
  };

  // Base relative tolerances are factor of 10 above failure thresholds
  // on Linux, clang, x86-64 (i.e., test just passes with tol_multiplier = 1)
  // TODO: Get float32 tolerances
  const struct {
    const char* path;
    SolverTolerances tolerances;
  } kConfigs[] = {
      {.path = kModelPath,
       .tolerances =
           {
               .newton        = 1e-15,
               .cg            = 1e-7,
               .pgs_pyramidal = 1e-14,
               .pgs_elliptic  = 1e-3,
           }},
      {.path = kHumanoidPath,
       .tolerances =
           {
               .newton        = 1e-15,
               .cg            = 1e-7,
               .pgs_pyramidal = 1e-6,
               .pgs_elliptic  = 1e-9,
           }},
  };

  for (const auto& config : kConfigs) {
    const std::string xml_path = GetTestDataFilePath(config.path);
    char error[1024];
    mjModel* model =
        mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    ASSERT_THAT(model, NotNull()) << error;

    model->opt.tolerance = 0;                     // set tolerance to 0
    model->opt.iterations = 500;                  // set iterations to 500
    model->opt.disableflags |= mjDSBL_WARMSTART;  // disable warmstart
    int nv = model->nv;

    mjData* data = mj_makeData(model);
    mjData* data_truth = mj_makeData(model);

    for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      model->opt.cone = cone;

      // use Newton Dense as ground truth
      model->opt.solver = mjSOL_NEWTON;
      model->opt.jacobian = mjJAC_DENSE;
      mj_resetDataKeyframe(model, data_truth, 0);
      mj_forward(model, data_truth);

      mjtNum scale = mju_norm(data_truth->qfrc_constraint, nv);

      for (mjtSolver solver : {mjSOL_NEWTON, mjSOL_CG, mjSOL_PGS}) {
        double rtol;
        switch (solver) {
          case mjSOL_NEWTON:
            rtol = config.tolerances.newton;
            break;
          case mjSOL_CG:
            rtol = config.tolerances.cg;
            break;
          case mjSOL_PGS:
            rtol = cone == mjCONE_PYRAMIDAL ? config.tolerances.pgs_pyramidal
                                            : config.tolerances.pgs_elliptic;
            break;
        }

        // increase base tolerance to avoid test flakiness
        double tol_multiplier = 1e2;
        double tolerance = scale * rtol * tol_multiplier;

        for (mjtJacobian jacobian : {mjJAC_DENSE, mjJAC_SPARSE}) {
          model->opt.solver = solver;
          model->opt.jacobian = jacobian;

          mj_resetDataKeyframe(model, data, 0);
          mj_forward(model, data);

          const char* cone_str =
              (cone == mjCONE_PYRAMIDAL ? "pyramidal" : "elliptic");
          const char* solver_str =
              (solver == mjSOL_NEWTON ? "Newton"
                                      : (solver == mjSOL_CG ? "CG" : "PGS"));
          const char* jacobian_str =
              (jacobian == mjJAC_DENSE ? "dense" : "sparse");

          EXPECT_THAT(AsVector(data->qfrc_constraint, nv),
                      Pointwise(DoubleNear(tolerance),
                                AsVector(data_truth->qfrc_constraint, nv)))
              << "model: " << config.path << "\n"
              << "cone: " << cone_str << "\n"
              << "solver: " << solver_str << "\n"
              << "jacobian: " << jacobian_str << "\n"
              << "tolerance: " << tolerance;
        }
      }
    }

    mj_deleteData(data_truth);
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

}  // namespace
}  // namespace mujoco
