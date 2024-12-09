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

// A benchmark for comparing different implementations of mj_solveLD.

#include <vector>
#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to roll out before benchmarking
static const int kNumWarmupSteps = 200;

// number of steps to benchmark
static const int kNumBenchmarkSteps = 50;

// ----------------------------- benchmark ------------------------------------

static void BM_solveLD(benchmark::State& state, bool featherstone, bool coil) {
  static mjModel* m;
  if (coil) {
    m = LoadModelFromPath("plugin/elasticity/coil.xml");
  } else {
    m = LoadModelFromPath("humanoid/humanoid100.xml");
  }

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate gradient
  mj_markStack(d);
  mjtNum *grad = mj_stackAllocNum(d, m->nv);
  mjtNum *Ma = mj_stackAllocNum(d, m->nv);
  mjtNum *res = mj_stackAllocNum(d, m->nv);

  // compute gradient
  mj_mulM(m, d, Ma, d->qacc);
  for (int i=0; i < m->nv; i++) {
    grad[i] = Ma[i] - d->qfrc_smooth[i] - d->qfrc_constraint[i];
  }

  // CSR matrix
  mjtNum* LDs = mj_stackAllocNum(d, m->nC);
  for (int i=0; i < m->nC; i++) {
    LDs[i] = d->qLD[d->mapM2C[i]];
  }

  // reset state, benchmark subsequent kNumBenchmarkSteps steps
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    for (int i=0; i < kNumBenchmarkSteps; i++) {
      if (featherstone) {
        mj_solveM(m, d, res, grad, 1);
      } else {
        mju_copy(res, grad, m->nv);
        mj_solveLDs(res, LDs, d->qLDiagInv, m->nv,
                    d->C_rownnz, d->C_rowadr, d->C_diag, d->C_colind);
      }
    }
  }

  // finalize
  mj_freeStack(d);
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_COIL_FS(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, /*featherstone=*/true, /*coil=*/true);
}
BENCHMARK(BM_solveLD_COIL_FS);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_COIL_CSR(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, /*featherstone=*/false, /*coil=*/true);
}
BENCHMARK(BM_solveLD_COIL_CSR);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_H100_FS(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, /*featherstone=*/true, /*coil=*/false);
}
BENCHMARK(BM_solveLD_H100_FS);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_H100_CSR(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, /*featherstone=*/false, /*coil=*/false);
}
BENCHMARK(BM_solveLD_H100_CSR);

}  // namespace
}  // namespace mujoco
