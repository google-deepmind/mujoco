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
#include <array>
#include <vector>

#include <benchmark/benchmark.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjthread.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to roll out before benchmarking
static const int kNumWarmupSteps = 500;

void BM_StepHumanoid200(benchmark::State& state) {
  auto model_path = GetTestDataFilePath("benchmark/testdata/humanoid200.xml");
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());

  model->opt.solver = mjSOL_CG;             // use CG solver
  model->opt.enableflags |= mjENBL_ISLAND;  // enable islands

  mjData* data = mj_makeData(model);
  mjThreadPool* threadpool = mju_threadPoolCreate(10);
  mju_bindThreadPool(data, threadpool);

  // warm-up rollout to get a steady state
  for (int i = 0; i < kNumWarmupSteps; i++) {
    mj_step(model, data);
  }

  // save the initial state and step
  int spec = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, spec);
  std::vector<mjtNum> initial_state(size);
  mj_getState(model, data, initial_state.data(), spec);

  // note: this tests resetting and stepping
  for (int i = 0; i < 10; ++i) {
    // reset to the saved state, step again, get the resulting state
    mj_setState(model, data, initial_state.data(), spec);
    for (int i = 0; i < kNumWarmupSteps; i++) {
      mj_step(model, data);
    }
  }

  state.SetItemsProcessed(state.iterations());
  mj_deleteData(data);
  mju_threadPoolDestroy(threadpool);
}

void BM_Step22Humanoids(benchmark::State& state) {
  auto model_path = GetTestDataFilePath("benchmark/testdata/22_humanoids.xml");
  std::array<char, 1024> error;
  mjModel* model =
      mj_loadXML(model_path.c_str(), nullptr, error.data(), error.size());
  model->opt.solver = mjSOL_CG;             // use CG solver
  model->opt.enableflags |= mjENBL_ISLAND;  // enable islands

  mjData* data = mj_makeData(model);
  mjThreadPool* threadpool = mju_threadPoolCreate(10);
  mju_bindThreadPool(data, threadpool);

  // warm-up rollout to get a steady state
  for (int i = 0; i < kNumWarmupSteps; i++) {
    mj_step(model, data);
  }

  // save the initial state and step
  int spec = mjSTATE_INTEGRATION;
  int size = mj_stateSize(model, spec);
  std::vector<mjtNum> initial_state(size);
  mj_getState(model, data, initial_state.data(), spec);

  std::vector<mjtNum> ctrl = GetCtrlNoise(model, kNumWarmupSteps);

  // note: this tests resetting and stepping
  for (int i = 0; i < 10; ++i) {
    // reset to the saved state, step again, get the resulting state
    mj_setState(model, data, initial_state.data(), spec);
    for (int i = 0; i < kNumWarmupSteps; i++) {
      mju_copy(data->ctrl, ctrl.data()+model->nu*i, model->nu);
      mj_step(model, data);
    }
  }

  state.SetItemsProcessed(state.iterations());
  mj_deleteData(data);
  mju_threadPoolDestroy(threadpool);
}

BENCHMARK(BM_StepHumanoid200);
BENCHMARK(BM_Step22Humanoids);
}  // namespace
}  // namespace mujoco
