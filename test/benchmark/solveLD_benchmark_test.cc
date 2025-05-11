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

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

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
  mj_forward(m, d);

  // allocate input and output vectors
  mj_markStack(d);
  mjtNum *vec = mj_stackAllocNum(d, m->nv);
  mjtNum *res = mj_stackAllocNum(d, m->nv);

  // arbitrary input vector
  for (int i=0; i < m->nv; i++) {
    vec[i] = 0.2 + 0.3*i;
  }

  // make legacy matrix
  mjtNum* LDlegacy = mj_stackAllocNum(d, m->nM);
  for (int i=0; i < m->nM; i++) {
    LDlegacy[d->mapM2M[i]] = d->qLD[i];
  }

  // benchmark
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    for (int i=0; i < kNumBenchmarkSteps; i++) {
      mju_copy(res, vec, m->nv);
      if (featherstone) {
        mj_solveLD_legacy(m, res, 1, LDlegacy, d->qLDiagInv);
      } else {
        mj_solveLD(res, d->qLD, d->qLDiagInv, m->nv, 1,
                   d->M_rownnz, d->M_rowadr, m->dof_simplenum, d->M_colind);
      }
    }
  }

  // finalize
  mj_freeStack(d);
  mj_deleteData(d);
  mj_deleteModel(m);
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
