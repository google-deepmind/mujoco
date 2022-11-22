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

// A benchmark which steps various models without rendering, and measures speed.

#include <array>

#include <benchmark/benchmark.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to roll out before benhmarking
static const int kNumWarmupSteps = 500;

// number of steps to benchmark
static const int kNumBenchmarkSteps = 50;

// copy array into vector
std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

static void run_step_benchmark(const mjModel* model, benchmark::State& state) {
  mjData* data = mj_makeData(model);

  // compute noise
  int nsteps = kNumWarmupSteps+kNumBenchmarkSteps;
  std::vector<mjtNum> ctrl = GetCtrlNoise(model, nsteps);

  // warm-up rollout to get a typcal state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mju_copy(data->ctrl, ctrl.data()+model->nu*i, model->nu);
    mj_step(model, data);
  }
  // save state
  std::vector<mjtNum> qpos = AsVector(data->qpos, model->nq);
  std::vector<mjtNum> qvel = AsVector(data->qvel, model->nv);
  std::vector<mjtNum> act = AsVector(data->act, model->na);
  std::vector<mjtNum> warmstart = AsVector(data->qacc_warmstart, model->nv);

  // reset state, benchmark subsequent kNumBenchmarkSteps steps
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    mju_copy(data->qpos, qpos.data(), model->nq);
    mju_copy(data->qvel, qvel.data(), model->nv);
    mju_copy(data->act, act.data(), model->na);
    mju_copy(data->qacc_warmstart, warmstart.data(), model->nv);

    for (int i=kNumWarmupSteps; i < nsteps; i++) {
      mju_copy(data->ctrl, ctrl.data()+model->nu*i, model->nu);
      mj_step(model, data);
    }
  }

  // finalize
  mj_deleteData(data);
  state.SetItemsProcessed(state.iterations());
}

// Use ABSL_ATTRIBUTE_NO_TAIL_CALL to make sure the benchmark functions appear
// separately in CPU profiles (and don't get replaced with raw calls to
// run_step_benchmark).

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepCloth(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = LoadModelFromPath("composite/cloth.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepCloth);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepFlag(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = LoadModelFromPath("flag/flag.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepFlag);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepHumanoid(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = LoadModelFromPath("humanoid/humanoid.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepHumanoid);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepHumanoid100(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = LoadModelFromPath("humanoid100/humanoid100.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepHumanoid100);

}  // namespace
}  // namespace mujoco
