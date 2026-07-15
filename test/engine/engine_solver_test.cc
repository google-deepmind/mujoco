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
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::std::max;
using ::testing::NotNull;

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
  model->opt.disableflags |= mjDSBL_MULTICCD;   // disable multiccd

  int nv = model->nv;

  int state_size = mj_stateSize(model, mjSTATE_INTEGRATION);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);

  mjData* data_island = mj_makeData(model);
  mjData* data_noisland = mj_makeData(model);

  constexpr int kNumTol = 3;
  mjtNum maxiter[kNumTol] = {30,   40,   60};
  // Below are 3 tolerances associated with 3 different iteration counts.
  // Tolerances are set to be ~12x higher than failure thresholds.
  // The point of this test is to show that CG convergence is actually not very
  // precise, simply changing whether islands are used changes the solution by
  // quite a lot, even at high iteration count and zero {ls_}tolerance.
  // Increasing the iteration count higher than 60 does not improve convergence.
  mjtNum rtol[kNumTol] = {
      MjTol(1e-1, 1.2e-1),
      MjTol(3e-2, 2e-2),
      MjTol(1.3e-5, 2.8e-3)
  };

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

      mjtNum max_ratio = 0;
      mjtNum worst_diff = 0;
      mjtNum worst_scale = 1.0;
      mjtNum worst_expected = 0;
      mjtNum worst_actual = 0;
      std::string worst_time = "";
      int worst_dof = -1;

      while (data_noisland->time < .1) {
        mj_getState(model, data_noisland, state, mjSTATE_INTEGRATION);
        mj_setState(model, data_island, state, mjSTATE_INTEGRATION);

        model->opt.disableflags &= ~mjDSBL_ISLAND;  // enable islands
        mj_forward(model, data_island);

        model->opt.disableflags |= mjDSBL_ISLAND;  // disable islands
        mj_forward(model, data_noisland);

        for (int j = 0; j < nv; j++) {
          mjtNum diff = std::abs(data_noisland->qacc[j] - data_island->qacc[j]);
          mjtNum scale = 0.5 * max(static_cast<mjtNum>(2.0),
                                   std::abs(data_noisland->qacc[j]) +
                                       std::abs(data_island->qacc[j]));
          mjtNum ratio = diff / scale;
          if (ratio > max_ratio) {
            max_ratio = ratio;
            worst_diff = diff;
            worst_scale = scale;
            worst_expected = data_island->qacc[j];
            worst_actual = data_noisland->qacc[j];
            worst_time = std::to_string(data_noisland->time);
            worst_dof = j;
          }
        }

        mj_step(model, data_noisland);
      }

      // Assert once per condition with the worst offender.
      // rtol[i] is already scaled by MjTolScale() at initialization.
      mjtNum allowed_tol = worst_scale * rtol[i];
      EXPECT_NEAR(worst_actual, worst_expected, allowed_tol)
          << "Worst offender info:\n"
          << "time: " << worst_time << '\n'
          << "dof: " << worst_dof << '\n'
          << "maxiter: " << maxiter[i] << '\n'
          << "coldstart: " << coldstart << '\n'
          << "rtol: " << worst_scale * rtol[i] << '\n'
          << "actual diff: " << worst_diff << " (allowed: " << allowed_tol
          << ")";
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

          mjtNum max_diff = 0;
          mjtNum worst_expected = 0;
          mjtNum worst_actual = 0;
          int worst_idx = -1;
          mjtNum scale = 0.5 * (mju_norm(data_noisland->qacc, nv) +
                                mju_norm(data_island->qacc, nv));
          mjtNum rtol = solver == mjSOL_CG ? MjTol(1e-8, 1e-4)
                                           : MjTol(1e-13, 1e-3);
          mjtNum worst_allowed = scale * rtol;

          for (int j = 0; j < nv; j++) {
            mjtNum diff =
                std::abs(data_island->qacc[j] - data_noisland->qacc[j]);
            if (diff > max_diff) {
              max_diff = diff;
              worst_expected = data_noisland->qacc[j];
              worst_actual = data_island->qacc[j];
              worst_idx = j;
            }
          }

          EXPECT_NEAR(worst_actual, worst_expected, worst_allowed)
              << "Worst offender in IslandsEquivalentForward:\n"
              << "idx: " << worst_idx << '\n'
              << "warmstart: " << warmstart << '\n'
              << "jacobian: " << (jacobian ? "sparse" : "dense") << '\n'
              << "solver: " << (solver == mjSOL_CG ? "CG" : "Newton") << '\n'
              << "cone: " << (cone == 1 ? "elliptic" : "pyramidal") << '\n'
              << "actual diff: " << max_diff << " (allowed: " << worst_allowed
              << ")";
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
    mjtNum newton;
    mjtNum cg;
    mjtNum pgs_pyramidal;
    mjtNum pgs_elliptic;
  };

  // Relative tolerances: 10x above failure thresholds on Linux, clang, x86-64.
  // MjTol(f64, f32) selects the appropriate tolerance for the current build.
  const struct {
    const char* path;
    SolverTolerances tolerances;
  } kConfigs[] = {
      {.path = kModelPath,
       .tolerances =
           {
               .newton        = MjTol(1e-13, 1e-5),
               .cg            = MjTol(1e-13, 1e-5),
               .pgs_pyramidal = MjTol(1e-13, 1e-5),
               .pgs_elliptic  = MjTol(1e-3,  1e-3),
           }},
      {.path = kHumanoidPath,
       .tolerances =
           {
               .newton        = MjTol(1e-13, 1e-5),
               .cg            = MjTol(1e-12, 1e-5),
               .pgs_pyramidal = MjTol(1e-12, 1e-5),
               .pgs_elliptic  = MjTol(1e-8,  1e-4),
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
        mjtNum rtol;
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

        mjtNum tolerance = scale * rtol;

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

          mjtNum max_diff = 0;
          mjtNum worst_expected = 0;
          mjtNum worst_actual = 0;
          int worst_idx = -1;

          for (int j = 0; j < nv; j++) {
            mjtNum diff = std::abs(data->qfrc_constraint[j] -
                                   data_truth->qfrc_constraint[j]);
            if (diff > max_diff) {
              max_diff = diff;
              worst_expected = data_truth->qfrc_constraint[j];
              worst_actual = data->qfrc_constraint[j];
              worst_idx = j;
            }
          }

          EXPECT_NEAR(worst_actual, worst_expected, tolerance)
              << "Worst offender in SolversEquivalent:\n"
              << "idx: " << worst_idx << '\n'
              << "model: " << config.path << "\n"
              << "cone: " << cone_str << "\n"
              << "solver: " << solver_str << "\n"
              << "jacobian: " << jacobian_str << "\n"
              << "actual diff: " << max_diff << " (allowed: " << tolerance
              << ")";
        }
      }
    }

    mj_deleteData(data_truth);
    mj_deleteData(data);
    mj_deleteModel(model);
  }
}

TEST_F(SolverTest, EllipticLineSearchPrecisionDiagnostics) {
  std::string xml = R"(
  <mujoco>
    <option cone="elliptic" solver="Newton"/>
    <worldbody>
      <geom name="floor" type="plane" size="10 10 1"/>
      <body name="box" pos="0 0 0.499">
        <joint type="free"/>
        <geom type="box" size="0.5 0.5 0.5" mass="1" friction="0.5"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // Set gravity to 0
  model->opt.gravity[0] = 0;
  model->opt.gravity[1] = 0;
  model->opt.gravity[2] = 0;

  for (double fn : {1e2, 1e4, 1e6, 1e8}) {
    mj_resetData(model.get(), data.get());

    // Apply large downward force
    data->qfrc_applied[2] = -fn;

    // Apply large lateral force (dynamic friction limit is 0.5 * fn)
    double ft = fn * 1.5;
    data->qfrc_applied[0] = ft;

    mj_forward(model.get(), data.get());

    int niter = std::min(data->solver_niter[0], mjNSOLVER);
    for (int i = 0; i < niter; ++i) {
      const mjSolverStat& stat = data->solver[i];
      EXPECT_GE(stat.improvement, -MjTol(1e-5, 100.0));
    }
  }
}

// Newton terminates early when the decrement predicts sub-tolerance improvement
TEST_F(SolverTest, NewtonDecrementTermination) {
  const std::string xml_path = GetTestDataFilePath(kHumanoidPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  model->opt.solver = mjSOL_NEWTON;
  model->opt.disableflags |= mjDSBL_ISLAND;  // monolithic solve
  model->opt.iterations = 100;
  const mjtNum tolerance = MjTol(1e-6, 1e-4);

  int state_size = mj_stateSize(model, mjSTATE_FULLPHYSICS);
  mjtNum* state = (mjtNum*) mju_malloc(sizeof(mjtNum)*state_size);

  mjData* data = mj_makeData(model);
  mjData* data_test = mj_makeData(model);
  mjData* data_deep = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);

  int nfired = 0;
  mjtNum max_leftover = 0;
  for (int step = 0; step < 200; step++) {
    model->opt.tolerance = tolerance;
    mj_step(model, data);
    mj_getState(model, data, state, mjSTATE_FULLPHYSICS);

    // solve with the test tolerance
    mj_setState(model, data_test, state, mjSTATE_FULLPHYSICS);
    mj_forward(model, data_test);

    // reference: tolerance 0 runs until the line search finds no improvement
    model->opt.tolerance = 0;
    mj_setState(model, data_deep, state, mjSTATE_FULLPHYSICS);
    mj_forward(model, data_deep);

    // accuracy: both runs produce identical iterates up to the test run's
    // stopping point, so the cost improvement forgone by early termination is
    // the sum of the deep run's remaining (scaled) improvements
    int niter = data_test->solver_niter[0];
    int niter_deep = std::min(data_deep->solver_niter[0], mjNSOLVER);
    mjtNum leftover = 0;
    for (int i = niter; i < niter_deep; i++) {
      leftover += max(static_cast<mjtNum>(0), data_deep->solver[i].improvement);
    }
    max_leftover = max(max_leftover, leftover);

    // count decrement terminations: the test run stopped while both existing
    // criteria were above tolerance, and the deep run shows that the next
    // iteration would have improved the cost by less than tolerance
    if (niter > 0 && niter < std::min(model->opt.iterations, mjNSOLVER) &&
        data_deep->solver_niter[0] > niter) {
      const mjSolverStat& last = data_test->solver[niter - 1];
      const mjSolverStat& next = data_deep->solver[niter];
      if (last.improvement >= tolerance && last.gradient >= tolerance &&
          next.improvement < tolerance) {
        nfired++;
      }
    }
  }

  EXPECT_LT(max_leftover, 10*tolerance)
      << "early termination forgoes more than a small multiple of tolerance";
  EXPECT_GT(nfired, 0)
      << "no state exercised the Newton decrement termination criterion";

  mj_deleteData(data_deep);
  mj_deleteData(data_test);
  mj_deleteData(data);
  mju_free(state);
  mj_deleteModel(model);
}

// a settled, warmstarted scene certifies convergence and solves in zero iterations
TEST_F(SolverTest, WarmstartZeroIterations) {
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="1 1 .1"/>
      <body pos="0 0 0.1">
        <freejoint/>
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  MjDataPtr data = MakeData(model);
  model->opt.disableflags |= mjDSBL_ISLAND;  // monolithic solve: stats in slot 0
  model->opt.enableflags |= mjENBL_FWDINV;

  int nv = model->nv;
  int state_size = mj_stateSize(model.get(), mjSTATE_FULLPHYSICS);
  std::vector<mjtNum> state(state_size);
  std::vector<mjtNum> qacc(nv), qfrc(nv);

  // float32 cannot resolve the default tolerance: use a resolvable one
  const mjtNum tolerance = MjTol(1e-8, 1e-6);

  for (mjtSolver solver : {mjSOL_CG, mjSOL_NEWTON}) {
    for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
      for (mjtJacobian jacobian : {mjJAC_DENSE, mjJAC_SPARSE}) {
        std::string config = std::string(solver == mjSOL_CG ? "CG" : "Newton") +
                             (cone == mjCONE_ELLIPTIC ? "/elliptic" : "/pyramidal") +
                             (jacobian == mjJAC_SPARSE ? "/sparse" : "/dense");
        model->opt.solver = solver;
        model->opt.cone = cone;
        model->opt.jacobian = jacobian;
        model->opt.tolerance = tolerance;

        // settle the box on the plane
        mj_resetData(model.get(), data.get());
        for (int i=0; i < 500; i++) {
          mj_step(model.get(), data.get());
        }
        mj_getState(model.get(), data.get(), state.data(), mjSTATE_FULLPHYSICS);

        // solve once more: certificate fires, forward/inverse stay consistent
        mj_forward(model.get(), data.get());
        EXPECT_EQ(data->solver_niter[0], 0) << config;

        // thresholds here and below are ~10x above measured, per precision
        EXPECT_LT(data->solver_fwdinv[0], MjTol(1e-12, 1e-4)) << config;
        EXPECT_LT(data->solver_fwdinv[1], MjTol(1e-2, 2e-1)) << config;
        mju_copy(qacc.data(), data->qacc, nv);
        mju_copy(qfrc.data(), data->qfrc_constraint, nv);

        // control arm: tolerance = 0 disables the certificate, full solve from
        // the same state must agree with the skipped solve
        model->opt.tolerance = 0;
        mj_setState(model.get(), data.get(), state.data(), mjSTATE_FULLPHYSICS);
        mj_forward(model.get(), data.get());
        mjtNum dqacc = 0, dqfrc = 0;
        for (int j=0; j < nv; j++) {
          dqacc = max(dqacc, std::abs(qacc[j] - data->qacc[j]));
          dqfrc = max(dqfrc, std::abs(qfrc[j] - data->qfrc_constraint[j]));
        }
        EXPECT_LT(dqacc, MjTol(2e-4, 1.5e-3)) << config;
        EXPECT_LT(dqfrc, MjTol(2e-2, 4e-1)) << config;

        // guard: a perturbed scene does not certify
        model->opt.tolerance = tolerance;
        mj_setState(model.get(), data.get(), state.data(), mjSTATE_FULLPHYSICS);
        data->qfrc_applied[0] = 5;
        mj_forward(model.get(), data.get());
        EXPECT_GT(data->solver_niter[0], 0) << config;
        data->qfrc_applied[0] = 0;
      }
    }
  }
}

// per-island certificates: settled islands solve in zero iterations while
// islands with new loads solve normally
TEST_F(SolverTest, WarmstartZeroIterationsIslands) {
  std::string xml = R"(
  <mujoco>
    <worldbody>
      <geom type="plane" size="2 2 .1"/>
      <body pos="-0.5 0 0.1">
        <freejoint/>
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
      <body pos="0.5 0 0.1">
        <freejoint/>
        <geom type="box" size="0.1 0.1 0.1"/>
      </body>
    </worldbody>
  </mujoco>
  )";

  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  MjDataPtr data = MakeData(model);

  // float32 cannot resolve the default tolerance: use a resolvable one
  model->opt.tolerance = MjTol(1e-8, 1e-6);

  // settle both boxes, islands enabled (default)
  for (int i=0; i < 500; i++) {
    mj_step(model.get(), data.get());
  }

  // both islands certify: zero iterations everywhere
  mj_forward(model.get(), data.get());
  ASSERT_EQ(data->nisland, 2);
  EXPECT_EQ(data->solver_niter[0], 0);
  EXPECT_EQ(data->solver_niter[1], 0);

  // kick the second box: its island solves, the settled island still certifies
  data->qfrc_applied[6] = 5;
  mj_forward(model.get(), data.get());
  ASSERT_EQ(data->nisland, 2);
  int island1 = data->dof_island[0];
  int island2 = data->dof_island[6];
  ASSERT_GE(island1, 0);
  ASSERT_GE(island2, 0);
  ASSERT_NE(island1, island2);
  EXPECT_EQ(data->solver_niter[island1], 0);
  EXPECT_GT(data->solver_niter[island2], 0);
}

// tolerance == 0 disables early termination, including the Newton decrement
TEST_F(SolverTest, ZeroToleranceDisablesTermination) {
  const std::string xml_path = GetTestDataFilePath(kHumanoidPath);
  char error[1024];
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;
  model->opt.solver = mjSOL_NEWTON;
  model->opt.disableflags |= mjDSBL_ISLAND | mjDSBL_WARMSTART;
  model->opt.tolerance = 0;
  model->opt.iterations = 3;

  mjData* data = mj_makeData(model);
  for (mjtCone cone : {mjCONE_PYRAMIDAL, mjCONE_ELLIPTIC}) {
    model->opt.cone = cone;
    mj_resetDataKeyframe(model, data, 0);
    mj_forward(model, data);
    EXPECT_EQ(data->solver_niter[0], 3)
        << "cone: " << (cone == mjCONE_ELLIPTIC ? "elliptic" : "pyramidal");
  }

  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
