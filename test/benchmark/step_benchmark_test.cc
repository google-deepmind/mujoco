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

using ::testing::NotNull;

// number of steps to roll out before benhmarking
static const int kNumWarmupSteps = 500;

// number of steps to benchmark
static const int kNumBenchmarkSteps = 50;


// copy array into vector
std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

static void add_ctrl_noise(const mjModel* m, mjData* d, int step) {
  for (int i = 0; i < m->nu; i++) {
    mjtNum center = 0.0;
    mjtNum radius = 1.0;
    mjtNum* range = m->actuator_ctrlrange + 2 * i;
    if (m->actuator_ctrllimited[i]) {
      center = (range[1] + range[0]) / 2;
      radius = (range[1] - range[0]) / 2;
    }
    radius *= 0.01;
    d->ctrl[i] = center + radius * (2 * mju_Halton(step, i + 2) - 1);
  }
}

static void assert_model_not_null(mjModel* model,
                                  const std::array<char, 1024>& error) {
  ASSERT_THAT(model, NotNull()) << "Failed to load model: " << error.data();
}

static mjModel* load_model(const char* model_path) {
  const std::string xml_path = GetModelPath(model_path);
  std::array<char, 1024> error;
  mjModel* model = mj_loadXML(xml_path.c_str(), nullptr, error.data(), error.size());
  assert_model_not_null(model, error);
  return model;
}

static void run_step_benchmark(const mjModel* model, benchmark::State& state) {
  mjData* data = mj_makeData(model);

  // warm-up rollout to get a typcal state
  for (int i=0; i < kNumWarmupSteps; i++) {
    add_ctrl_noise(model, data, i);
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

    for (int i=0; i < kNumBenchmarkSteps; i++) {
      add_ctrl_noise(model, data, i+kNumWarmupSteps);
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
  static mjModel* model = load_model("composite/cloth.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepCloth);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepHumanoid(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = load_model("humanoid/humanoid.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepHumanoid);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_StepHumanoid100(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  static mjModel* model = load_model("humanoid100/humanoid100.xml");
  run_step_benchmark(model, state);
}
BENCHMARK(BM_StepHumanoid100);

}  // namespace
}  // namespace mujoco
