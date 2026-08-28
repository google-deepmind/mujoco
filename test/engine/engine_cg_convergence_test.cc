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

// CG solver convergence benchmark.
//
// Rolls out Newton ground truth on 2humanoid100.xml, then evaluates CG qacc
// error. Pass 1: vary iterations with/without warmstart. Pass 2: vary
// tolerance. Pass 3: consecutive stepping. No assertions, only data.

#include <chrono>  // NOLINT
#include <cstdio>
#include <ratio>  // NOLINT
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::NotNull;

mjtNum gettm(void) {
  using Clock = std::chrono::steady_clock;
  using Microseconds = std::chrono::duration<mjtNum, std::micro>;
  static const Clock::time_point tm_start = Clock::now();
  return Microseconds(Clock::now() - tm_start).count();
}

using CgConvergenceTest = MujocoTest;

TEST_F(CgConvergenceTest, CGConvergence) {
  static const char* const kPath =
      "engine/testdata/island/2humanoid100.xml";
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

  // configure Newton ground truth: tolerance 0, generous iterations, warmstart
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

  for (int i = 0; i < kNumSteps; i++) {
    // save pre-step state
    mju_copy(all_qpos.data() + i*nq, data->qpos, nq);
    mju_copy(all_qvel.data() + i*nv, data->qvel, nv);
    mju_copy(all_warmstart.data() + i*nv, data->qacc_warmstart, nv);

    // step with Newton
    mj_step(model, data);

    // save qacc (set by mj_forward inside mj_step)
    mju_copy(all_qacc.data() + i*nv, data->qacc, nv);
  }

  // evaluation points: 100 evenly spaced
  constexpr int kNumEval = 100;
  constexpr int kStride = kNumSteps / kNumEval;

  // iteration counts to test
  constexpr int kIterCounts[] = {5, 10, 20, 40, 80, 160};
  constexpr int kNumIter = sizeof(kIterCounts) / sizeof(kIterCounts[0]);

  // print header
  std::printf("\nCG Convergence: 2humanoid100.xml\n");
  std::printf("  %d Newton steps, %d evaluation points\n",
              kNumSteps, kNumEval);
  std::printf("  nv = %d, nq = %d\n", nv, nq);
  std::printf("  metric: ||qacc_cg - qacc_newton|| / ||qacc_newton||\n");

  // table header
  std::printf("\n  Warmstart (tolerance = 0):\n");
  std::printf("  %6s | %11s | %11s | %10s | %8s\n",
              "Iters", "Mean Err", "Max Err", "Mean Iters", "LS evals");
  std::printf("  %s\n",
              "-------+-------------+-------------+------------+---------");

  // configure CG: full rollout (tolerance 0), warmstart enabled
  model->opt.solver = mjSOL_CG;
  model->opt.tolerance = 0;
  model->opt.disableflags &= ~mjDSBL_WARMSTART;

  for (int c = 0; c < kNumIter; c++) {
    model->opt.iterations = kIterCounts[c];

    int total_iters = 0;
    int total_neval = 0;
    mjtNum sum_rel_err = 0;
    mjtNum max_rel_err = 0;

    for (int e = 0; e < kNumEval; e++) {
      int idx = e * kStride;

      // restore state
      mju_copy(data->qpos, all_qpos.data() + idx*nq, nq);
      mju_copy(data->qvel, all_qvel.data() + idx*nv, nv);
      mju_copy(data->qacc_warmstart, all_warmstart.data() + idx*nv, nv);

      // run CG forward
      mj_forward(model, data);
      int niter = data->solver_niter[0];
      total_iters += niter;
      for (int j = 0; j < niter && j < mjNSOLVER; j++) {
        total_neval += data->solver[j].neval;
      }

      // compute relative error
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

    mjtNum mean_iters = static_cast<mjtNum>(total_iters) / kNumEval;
    std::printf("  %6d | %11.4e | %11.4e | %10.2f | %8d\n",
                kIterCounts[c], sum_rel_err / kNumEval, max_rel_err,
                mean_iters, total_neval);
  }

  std::printf("  %s\n",
              "-------+-------------+-------------+------------+---------");

  // --- no warmstart (tolerance = 0) ---
  std::printf("\n  No warmstart (tolerance = 0):\n");
  std::printf("  %6s | %11s | %11s | %10s | %8s\n",
              "Iters", "Mean Err", "Max Err", "Mean Iters", "LS evals");
  std::printf("  %s\n",
              "-------+-------------+-------------+------------+---------");

  model->opt.disableflags |= mjDSBL_WARMSTART;

  for (int c = 0; c < kNumIter; c++) {
    model->opt.iterations = kIterCounts[c];

    int total_iters = 0;
    int total_neval = 0;
    mjtNum sum_rel_err = 0;
    mjtNum max_rel_err = 0;

    for (int e = 0; e < kNumEval; e++) {
      int idx = e * kStride;

      mju_copy(data->qpos, all_qpos.data() + idx*nq, nq);
      mju_copy(data->qvel, all_qvel.data() + idx*nv, nv);

      mj_forward(model, data);
      int niter = data->solver_niter[0];
      total_iters += niter;
      for (int j = 0; j < niter && j < mjNSOLVER; j++) {
        total_neval += data->solver[j].neval;
      }

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

    mjtNum mean_iters = static_cast<mjtNum>(total_iters) / kNumEval;
    std::printf("  %6d | %11.4e | %11.4e | %10.2f | %8d\n",
                kIterCounts[c], sum_rel_err / kNumEval, max_rel_err,
                mean_iters, total_neval);
  }

  std::printf("  %s\n",
              "-------+-------------+-------------+------------+---------");

  // --- tolerance sweep (iterations = 100, warmstart) ---
  std::printf("\n  Tolerance sweep (iterations = 100, warmstart):\n");
  std::printf("  %10s | %11s | %11s | %9s | %10s | %11s | %8s\n",
              "Tol", "Mean Err", "Max Err", "Mean Iters", "Max Iters",
              "Solver us", "LS evals");
  std::printf("  %s\n",
              "-----------+-------------+-------------+"
              "------------+------------+-------------+---------");

  model->opt.solver = mjSOL_CG;
  model->opt.iterations = 100;
  model->opt.disableflags &= ~mjDSBL_WARMSTART;

  constexpr mjtNum kTolValues[] = {1e-4, 1e-6, 1e-8, 1e-10, 1e-12, 0};
  constexpr int kNumTol = sizeof(kTolValues) / sizeof(kTolValues[0]);

  mjtNum total_solver_time = 0;
  int grand_total_iters = 0;

  for (int c = 0; c < kNumTol; c++) {
    for (int i = 0; i < mjNTIMER; i++) {
      data->timer[i].duration = 0;
      data->timer[i].number = 0;
    }

    model->opt.tolerance = kTolValues[c];

    int total_iters = 0;
    int max_iters = 0;
    int total_neval = 0;
    mjtNum sum_rel_err = 0;
    mjtNum max_rel_err = 0;

    for (int e = 0; e < kNumEval; e++) {
      int idx = e * kStride;

      mju_copy(data->qpos, all_qpos.data() + idx*nq, nq);
      mju_copy(data->qvel, all_qvel.data() + idx*nv, nv);
      mju_copy(data->qacc_warmstart, all_warmstart.data() + idx*nv, nv);

      mj_forward(model, data);
      int niter = data->solver_niter[0];
      total_iters += niter;
      if (niter > max_iters) {
        max_iters = niter;
      }
      for (int j = 0; j < niter && j < mjNSOLVER; j++) {
        total_neval += data->solver[j].neval;
      }

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
    total_solver_time += solver_time;
    grand_total_iters += total_iters;

    mjtNum mean_iters = static_cast<mjtNum>(total_iters) / kNumEval;
    if (kTolValues[c] > 0) {
      std::printf(
          "  %10.0e | %11.4e | %11.4e | %10.2f | %10d | %11.2f | %8d\n",
          kTolValues[c], sum_rel_err / kNumEval, max_rel_err,
          mean_iters, max_iters, solver_time, total_neval);
    } else {
      std::printf(
          "  %10s | %11.4e | %11.4e | %10.2f | %10d | %11.2f | %8d\n",
          "0", sum_rel_err / kNumEval, max_rel_err,
          mean_iters, max_iters, solver_time, total_neval);
    }
  }

  mjtNum overall_avg = grand_total_iters > 0
      ? total_solver_time / grand_total_iters : 0.0;
  std::printf("  %s\n",
              "-----------+-------------+-------------+"
              "------------+------------+-------------+---------");
  std::printf("  Total solver time: %.2f us, avg time per iter: %.4f us\n",
              total_solver_time, overall_avg);

  // --- third pass: consecutive stepping (mini-testspeed) ---
  // uses model defaults: tolerance = 1e-8, iterations = 100
  std::printf("\n  Pipeline mode (consecutive mj_step, tolerance = 1e-8):\n");

  model->opt.solver = mjSOL_CG;
  model->opt.tolerance = 1e-8;
  model->opt.iterations = 100;
  model->opt.disableflags &= ~mjDSBL_WARMSTART;

  // reset data to initial state
  mj_resetData(model, data);

  // clear timers
  for (int i = 0; i < mjNTIMER; i++) {
    data->timer[i].duration = 0;
    data->timer[i].number = 0;
  }

  // run consecutive steps
  constexpr int kPipeSteps = 1000;
  int pipe_total_iters = 0;
  int pipe_total_neval = 0;
  for (int i = 0; i < kPipeSteps; i++) {
    mj_step(model, data);
    int niter = data->solver_niter[0];
    pipe_total_iters += niter;
    for (int j = 0; j < niter && j < mjNSOLVER; j++) {
      pipe_total_neval += data->solver[j].neval;
    }
  }

  mjtNum pipe_constraint = data->timer[mjTIMER_CONSTRAINT].duration;
  mjtNum pipe_step = data->timer[mjTIMER_STEP].duration;
  int pipe_step_count = data->timer[mjTIMER_STEP].number;
  mjtNum us_per_step = pipe_step_count > 0 ? pipe_step / pipe_step_count : 0;
  mjtNum constraint_per_step = pipe_step_count > 0
      ? pipe_constraint / pipe_step_count : 0;
  mjtNum iters_per_step = pipe_step_count > 0
      ? static_cast<mjtNum>(pipe_total_iters) / pipe_step_count : 0;
  mjtNum steps_per_sec = us_per_step > 0 ? 1e6 / us_per_step : 0;

  std::printf("  %d steps, nv = %d\n", kPipeSteps, nv);
  std::printf("  Steps/s          : %.0f\n", steps_per_sec);
  std::printf("  us/step (total)  : %.1f\n", us_per_step);
  std::printf("  us/step (constr) : %.1f  (%.1f%%)\n",
              constraint_per_step,
              us_per_step > 0 ? 100*constraint_per_step/us_per_step : 0.0);
  std::printf("  CG iters/step    : %.2f\n", iters_per_step);
  std::printf("  LS evals/step    : %.2f\n",
              pipe_step_count > 0
                  ? static_cast<mjtNum>(pipe_total_neval) / pipe_step_count
                  : 0.0);
  std::printf("  us/iter          : %.2f\n",
              pipe_total_iters > 0
                  ? pipe_constraint / pipe_total_iters : 0.0);

  std::printf("\n");
  mjcb_time = nullptr;
  mj_deleteData(data);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
