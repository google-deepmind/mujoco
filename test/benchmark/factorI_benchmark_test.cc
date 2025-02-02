// Copyright 2025 DeepMind Technologies Limited
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

// A benchmark for comparing different implementations of mj_factorI.

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

static void BM_factorI(benchmark::State& state, bool legacy, bool coil) {
  static mjModel* m;
  if (coil) {
    m = LoadModelFromPath("plugin/elasticity/coil.xml");
  } else {
    m = LoadModelFromPath("humanoid/humanoid100.xml");
  }

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // allocate inputs and outputs
  mj_markStack(d);

  // CSR matrices
  mjtNum* Ms = mj_stackAllocNum(d, m->nC);
  mjtNum* LDs = mj_stackAllocNum(d, m->nC);
  for (int i=0; i < m->nC; i++) {
    Ms[i] = d->qM[d->mapM2C[i]];
  }

  // benchmark
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    for (int i=0; i < kNumBenchmarkSteps; i++) {
      if (legacy) {
        mj_factorI(m, d, d->qM, d->qLD, d->qLDiagInv);
      } else {
        mju_copy(LDs, Ms, m->nC);
        mj_factorIs(LDs, d->qLDiagInv, m->nv,
                    d->C_rownnz, d->C_rowadr, m->dof_simplenum, d->C_colind);
      }
    }
  }

  // finalize
  mj_freeStack(d);
  mj_deleteData(d);
  mj_deleteModel(m);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_factorI_COIL_LEGACY(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_factorI(state, /*legacy=*/true, /*coil=*/true);
}
BENCHMARK(BM_factorI_COIL_LEGACY);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_factorI_COIL_CSR(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_factorI(state, /*legacy=*/false, /*coil=*/true);
}
BENCHMARK(BM_factorI_COIL_CSR);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_factorI_H100_LEGACY(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_factorI(state, /*legacy=*/true, /*coil=*/false);
}
BENCHMARK(BM_factorI_H100_LEGACY);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_factorI_H100_CSR(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_factorI(state, /*legacy=*/false, /*coil=*/false);
}
BENCHMARK(BM_factorI_H100_CSR);

}  // namespace
}  // namespace mujoco
