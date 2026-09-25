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

#include <cmath>
#include <string>
#include <vector>

#include <benchmark/benchmark.h>
#include <mujoco/mujoco.h>
#include "engine/engine_derivative.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

enum class Operation { kPassive, kMatvec, kAssemble, kStep };

// Fixed, mildly deformed grids isolate elasticity from collision detection.
// Complete steps use the same initial state for each batch, so different
// iteration counts time the same motion.
void FlexStretchBenchmark(benchmark::State& state, Operation operation) {
  int dim = state.range(0);
  int count = state.range(1);
  std::string xml =
      "<mujoco><option integrator='discrete' solver='CG' timestep='0.0001' "
      "gravity='0 0 0'/><worldbody><flexcomp name='soft' type='grid' dim='" +
      std::to_string(dim) + "' count='" + std::to_string(count) + " " +
      std::to_string(count) + " " + std::to_string(dim == 3 ? count : 1) +
      "' spacing='0.05 0.05 0.05' mass='1'>"
      "<contact contype='0' conaffinity='0' selfcollide='none' "
      "internal='false'/>"
      "<elasticity young='1000' poisson='0.3' damping='0.01' " +
      (dim == 2 ? std::string("elastic2d='stretch' thickness='0.01'")
                : std::string()) +
      "/><pin id='0'/></flexcomp></worldbody></mujoco>";
  char error[1024];
  MjModelPtr model = LoadModelFromString(xml, error, sizeof(error));
  if (!model) {
    state.SkipWithError(error);
    return;
  }
  MjDataPtr data = MakeData(model);
  const mjModel* m = model.get();
  mjData* d = data.get();
  for (int i = 0; i < m->nq; i++) {
    d->qpos[i] += 0.001 * std::sin(0.37 * i);
  }
  for (int i = 0; i < m->nv; i++) {
    d->qvel[i] = 0.01 * std::cos(0.23 * i);
  }
  mj_forward(m, d);

  std::vector<mjtNum> direction(m->nv), result(m->nv);
  for (int i = 0; i < m->nv; i++) {
    direction[i] = std::sin(0.17 * i);
  }
  std::vector<int> rownnz(m->nv), rowadr(m->nv);
  mjtNum h = m->opt.timestep;
  int nnz = mjd_flexStiff_assemble(m, d, rownnz.data(), rowadr.data(), nullptr,
                                   nullptr, h * h, h, 0, 1, nullptr);
  std::vector<int> colind(nnz);
  std::vector<mjtNum> values(nnz);
  int state_size = mj_stateSize(m, mjSTATE_INTEGRATION);
  std::vector<mjtNum> initial_state(state_size);
  mj_getState(m, d, initial_state.data(), mjSTATE_INTEGRATION);

  if (operation == Operation::kStep) {
    constexpr int kBatch = 32;
    while (state.KeepRunningBatch(kBatch)) {
      mj_setState(m, d, initial_state.data(), mjSTATE_INTEGRATION);
      for (int i = 0; i < kBatch; i++) {
        mj_step(m, d);
      }
      benchmark::DoNotOptimize(d->qpos);
      benchmark::ClobberMemory();
    }
  } else {
    for (auto _ : state) {
      switch (operation) {
        case Operation::kPassive:
          mj_passive(m, d);
          benchmark::DoNotOptimize(d->qfrc_spring);
          benchmark::DoNotOptimize(d->qfrc_damper);
          break;
        case Operation::kMatvec:
          mju_zero(result.data(), m->nv);
          mjd_flexStretch_mul(m, d, result.data(), direction.data(), h * h, h);
          benchmark::DoNotOptimize(result.data());
          break;
        case Operation::kAssemble:
          mjd_flexStiff_assemble(m, d, rownnz.data(), rowadr.data(),
                                 colind.data(), values.data(), h * h, h, 0, 1,
                                 nullptr);
          benchmark::DoNotOptimize(values.data());
          break;
        case Operation::kStep:
          break;
      }
      benchmark::ClobberMemory();
    }
  }
  for (int i = 0; i < mjNWARNING; i++) {
    if (d->warning[i].number) {
      state.SkipWithError("simulation warning during flex benchmark");
      break;
    }
  }
  state.counters["vertices"] = m->nflexvert;
  state.counters["elements"] = m->nflexelem;
}

void BM_FlexPassive(benchmark::State& state) {
  FlexStretchBenchmark(state, Operation::kPassive);
}
void BM_FlexMatvec(benchmark::State& state) {
  FlexStretchBenchmark(state, Operation::kMatvec);
}
void BM_FlexAssemble(benchmark::State& state) {
  FlexStretchBenchmark(state, Operation::kAssemble);
}
void BM_FlexStep(benchmark::State& state) {
  FlexStretchBenchmark(state, Operation::kStep);
}

BENCHMARK(BM_FlexPassive)
    ->Args({3, 4})
    ->Args({3, 7})
    ->Args({3, 10})
    ->Args({2, 10});
BENCHMARK(BM_FlexMatvec)
    ->Args({3, 4})
    ->Args({3, 7})
    ->Args({3, 10})
    ->Args({2, 10});
BENCHMARK(BM_FlexAssemble)
    ->Args({3, 4})
    ->Args({3, 7})
    ->Args({3, 10})
    ->Args({2, 10});
BENCHMARK(BM_FlexStep)
    ->Args({3, 4})
    ->Args({3, 7})
    ->Args({3, 10})
    ->Args({2, 10});

// Use the checked-in models without changing their solver, integrator,
// contacts, or controls.
void FlexExampleBenchmark(benchmark::State& state, const char* filename,
                          Operation operation) {
  std::string path = GetModelPath((std::string("flex/") + filename).c_str());
  char error[1024];
  MjModelPtr model(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
  if (!model) {
    state.SkipWithError(error);
    return;
  }
  MjDataPtr data = MakeData(model);
  const mjModel* m = model.get();
  mjData* d = data.get();
  for (int i = 0; i < 500; i++) {
    mj_step(m, d);
  }
  mj_forward(m, d);
  int initial_contacts = d->ncon;
  int state_size = mj_stateSize(m, mjSTATE_INTEGRATION);
  std::vector<mjtNum> initial_state(state_size);
  mj_getState(m, d, initial_state.data(), mjSTATE_INTEGRATION);

  if (operation == Operation::kStep) {
    constexpr int kBatch = 50;
    while (state.KeepRunningBatch(kBatch)) {
      mj_setState(m, d, initial_state.data(), mjSTATE_INTEGRATION);
      for (int i = 0; i < kBatch; i++) {
        mj_step(m, d);
      }
      benchmark::DoNotOptimize(d->qpos);
      benchmark::ClobberMemory();
    }
  } else {
    for (auto _ : state) {
      mj_passive(m, d);
      benchmark::DoNotOptimize(d->qfrc_spring);
      benchmark::DoNotOptimize(d->qfrc_damper);
      benchmark::ClobberMemory();
    }
  }
  for (int i = 0; i < mjNWARNING; i++) {
    if (d->warning[i].number) {
      state.SkipWithError("simulation warning during flex example benchmark");
      break;
    }
  }
  state.counters["vertices"] = m->nflexvert;
  state.counters["elements"] = m->nflexelem;
  state.counters["initial_contacts"] = initial_contacts;
  state.counters["integrator"] = m->opt.integrator;
  state.counters["dofs"] = m->nv;
}

BENCHMARK_CAPTURE(FlexExampleBenchmark, press_step, "press.xml",
                  Operation::kStep);
BENCHMARK_CAPTURE(FlexExampleBenchmark, floppy_step, "floppy.xml",
                  Operation::kStep);
BENCHMARK_CAPTURE(FlexExampleBenchmark, jelly_step, "jelly.xml",
                  Operation::kStep);
BENCHMARK_CAPTURE(FlexExampleBenchmark, press_passive, "press.xml",
                  Operation::kPassive);
BENCHMARK_CAPTURE(FlexExampleBenchmark, floppy_passive, "floppy.xml",
                  Operation::kPassive);
BENCHMARK_CAPTURE(FlexExampleBenchmark, jelly_passive, "jelly.xml",
                  Operation::kPassive);
BENCHMARK_CAPTURE(FlexExampleBenchmark, sphere_full_step, "sphere_full.xml",
                  Operation::kStep);
BENCHMARK_CAPTURE(FlexExampleBenchmark, trilinear_step, "trilinear.xml",
                  Operation::kStep);
BENCHMARK_CAPTURE(FlexExampleBenchmark, gripper_step, "gripper.xml",
                  Operation::kStep);

}  // namespace
}  // namespace mujoco
