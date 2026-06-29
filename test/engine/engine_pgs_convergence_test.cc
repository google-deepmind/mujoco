// Copyright 2026 DeepMind Technologies Limited
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

// PGS with/without Nesterov momentum: compare convergence and timing.
//
// Rolls out Newton ground truth on 2humanoid100.xml, then evaluates PGS with
// and without Nesterov momentum at various iteration budgets.


#include <chrono>  // NOLINT
#include <cstdio>
#include <ratio>  // NOLINT
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

extern "C" thread_local int mj_nesterov_momentum;

namespace mujoco {
namespace {

using ::testing::NotNull;

mjtNum gettm(void) {
  using Clock = std::chrono::steady_clock;
  using Microseconds = std::chrono::duration<mjtNum, std::micro>;
  static const Clock::time_point tm_start = Clock::now();
  return Microseconds(Clock::now() - tm_start).count();
}

using PgsConvergenceTest = MujocoTest;

// get total solver iterations across all islands
int get_total_solver_iters(const mjData* d) {
  int nisland = mjMAX(1, mjMIN(d->nisland, mjNISLAND));
  int total = 0;
  for (int i = 0; i < nisland; i++) {
    total += d->solver_niter[i];
  }
  return total;
}

// run solver benchmark at various iteration counts, print table
void run_benchmark(mjModel* model, mjData* data,
                   const std::vector<mjtNum>& all_qpos,
                   const std::vector<mjtNum>& all_qvel,
                   const std::vector<mjtNum>& all_warmstart,
                   const std::vector<mjtNum>& all_qacc,
                   int nq, int nv, int kStride, int kNumEval,
                   const int* kIterCounts, int kNumIter,
                   const char* label) {
  std::printf("\n  %s:\n", label);
  std::printf("  %6s | %11s | %11s | %10s | %11s\n",
              "Iters", "Mean Err", "Max Err", "Mean Iters", "Solver us");
  std::printf("  %s\n",
              "-------+-------------+-------------+------------+-----------");

  for (int c = 0; c < kNumIter; c++) {
    model->opt.iterations = kIterCounts[c];

    for (int i = 0; i < mjNTIMER; i++) {
      data->timer[i].duration = 0;
      data->timer[i].number = 0;
    }

    int total_iters = 0;
    mjtNum sum_rel_err = 0;
    mjtNum max_rel_err = 0;

    for (int e = 0; e < kNumEval; e++) {
      int idx = e * kStride;

      mju_copy(data->qpos, all_qpos.data() + idx*nq, nq);
      mju_copy(data->qvel, all_qvel.data() + idx*nv, nv);
      mju_copy(data->qacc_warmstart, all_warmstart.data() + idx*nv, nv);

      mj_forward(model, data);
      total_iters += get_total_solver_iters(data);

      mjtNum newton_norm = mju_norm(all_qacc.data() + idx*nv, nv);
      mjtNum err = 0;
      for (int j = 0; j < nv; j++) {
        mjtNum diff = data->qacc[j] - all_qacc[idx*nv + j];
        err += diff * diff;
      }
      mjtNum rel_err = mju_sqrt(err) / mju_max(newton_norm, 1e-10);
      sum_rel_err += rel_err;
      if (rel_err > max_rel_err) {
        max_rel_err = rel_err;
      }
    }

    mjtNum solver_time = data->timer[mjTIMER_CONSTRAINT].duration;
    mjtNum mean_iters = static_cast<mjtNum>(total_iters) / kNumEval;
    std::printf("  %6d | %11.4e | %11.4e | %10.2f | %11.2f\n",
                kIterCounts[c], sum_rel_err / kNumEval, max_rel_err,
                mean_iters, solver_time);
  }
  std::printf("  %s\n",
              "-------+-------------+-------------+------------+-----------");
}

// run pipeline mode: consecutive steps with tolerance, print summary
void run_pipeline(mjModel* model, mjData* data, int kNumSteps,
                  const char* label) {
  std::printf("\n  %s Pipeline mode (mj_step, tolerance = 1e-8):\n", label);

  mj_resetData(model, data);
  for (int i = 0; i < mjNTIMER; i++) {
    data->timer[i].duration = 0;
    data->timer[i].number = 0;
  }

  int pipe_total_iters = 0;
  for (int i = 0; i < kNumSteps; i++) {
    mj_step(model, data);
    pipe_total_iters += get_total_solver_iters(data);
  }

  mjtNum pipe_constraint = data->timer[mjTIMER_CONSTRAINT].duration;
  mjtNum pipe_step = data->timer[mjTIMER_STEP].duration;
  int pipe_step_count = data->timer[mjTIMER_STEP].number;

  std::printf("  Steps/s          : %.0f\n",
              pipe_step_count > 0 ? 1e6 * pipe_step_count / pipe_step : 0.0);
  std::printf("  us/step (total)  : %.1f\n",
              pipe_step_count > 0 ? pipe_step / pipe_step_count : 0.0);
  std::printf("  us/step (constr) : %.1f\n",
              pipe_step_count > 0 ? pipe_constraint / pipe_step_count : 0.0);
  std::printf("  Iters/step       : %.2f\n",
              pipe_step_count > 0
                ? static_cast<mjtNum>(pipe_total_iters) / pipe_step_count
                : 0.0);
}

TEST_F(PgsConvergenceTest, PGSConvergence) {
  static const char* const kPath =
      "engine/testdata/forward/perf/2humanoid100_PGS.xml";
  const std::string xml_path = GetTestDataFilePath(kPath);
  char error[1024];
  mjModel* model =
      mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
  ASSERT_THAT(model, NotNull()) << error;

  // disable islands: monolithic solver makes statistics simpler
  model->opt.disableflags |= mjDSBL_ISLAND;

  int nq = model->nq;
  int nv = model->nv;

  // stronger sideways gravity
  model->opt.gravity[0] = -2;
  model->opt.gravity[1] = -2;
  model->opt.gravity[2] = -10;

  // configure Newton ground truth
  model->opt.solver = mjSOL_NEWTON;
  model->opt.jacobian = mjJAC_SPARSE;
  model->opt.tolerance = 0;
  model->opt.iterations = 10;
  model->opt.disableflags &= ~mjDSBL_WARMSTART;

  mjData* data = mj_makeData(model);
  mjcb_time = gettm;

  // roll out Newton for kNumSteps, save pre-step state and post-step qacc
  constexpr int kNumSteps = 1000;
  std::vector<mjtNum> all_qpos(kNumSteps * nq);
  std::vector<mjtNum> all_qvel(kNumSteps * nv);
  std::vector<mjtNum> all_warmstart(kNumSteps * nv);
  std::vector<mjtNum> all_qacc(kNumSteps * nv);

  std::printf("Rolling out ground truth Newton steps...\n");
  for (int i = 0; i < kNumSteps; i++) {
    mju_copy(all_qpos.data() + i*nq, data->qpos, nq);
    mju_copy(all_qvel.data() + i*nv, data->qvel, nv);
    mju_copy(all_warmstart.data() + i*nv, data->qacc_warmstart, nv);

    mj_step(model, data);

    mju_copy(all_qacc.data() + i*nv, data->qacc, nv);
  }

  // evaluation points
  constexpr int kNumEval = 100;
  constexpr int kStride = kNumSteps / kNumEval;

  // iteration counts to test
  constexpr int kIterCounts[] = {5, 10, 20, 40, 80, 160, 320};
  constexpr int kNumIter = sizeof(kIterCounts) / sizeof(kIterCounts[0]);

  std::printf("\nPGS vs Nesterov PGS: 2humanoid100_PGS.xml\n");
  std::printf("  %d Newton steps, %d evaluation points\n",
              kNumSteps, kNumEval);
  std::printf("  nv = %d, nq = %d, nefc = %d\n", nv, nq, data->nefc);

  // switch to PGS
  model->opt.solver = mjSOL_PGS;
  model->opt.tolerance = 0;
  model->opt.disableflags &= ~mjDSBL_WARMSTART;

  // 1. Row PGS (default)
  mj_nesterov_momentum = 0;
  run_benchmark(model, data, all_qpos, all_qvel, all_warmstart, all_qacc,
                nq, nv, kStride, kNumEval, kIterCounts, kNumIter,
                "Row PGS (Warmstart, tolerance = 0)");

  // 2. Nesterov PGS (via REFSAFE hack)
  mj_nesterov_momentum = 1;
  run_benchmark(model, data, all_qpos, all_qvel, all_warmstart, all_qacc,
                nq, nv, kStride, kNumEval, kIterCounts, kNumIter,
                "Nesterov PGS (Warmstart, tolerance = 0)");

  // 3. Pipeline: Row PGS
  mj_nesterov_momentum = 0;
  model->opt.tolerance = 1e-8;
  model->opt.iterations = 100;
  run_pipeline(model, data, kNumSteps, "Row PGS");

  // 4. Pipeline: Nesterov PGS
  mj_nesterov_momentum = 1;
  model->opt.tolerance = 1e-8;
  model->opt.iterations = 100;
  run_pipeline(model, data, kNumSteps, "Nesterov PGS");

  std::printf("\n");

  // ========== PASS 2: ISLANDS ENABLED ==========
  model->opt.disableflags &= ~mjDSBL_ISLAND;

  // do one forward pass to get nisland
  mj_resetData(model, data);
  mj_forward(model, data);
  std::printf("\n============================================\n");
  std::printf("PASS 2: ISLANDS ENABLED\n");
  std::printf("============================================\n");

  // Pipeline: Row PGS with islands
  mj_nesterov_momentum = 0;
  model->opt.tolerance = 1e-8;
  model->opt.iterations = 100;
  run_pipeline(model, data, kNumSteps, "Row PGS (islands)");

  // Pipeline: Nesterov PGS with islands
  mj_nesterov_momentum = 1;
  model->opt.tolerance = 1e-8;
  model->opt.iterations = 100;
  run_pipeline(model, data, kNumSteps, "Nesterov PGS (islands)");

  // Reset to default
  mj_nesterov_momentum = 1;

  std::printf("\n");
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
