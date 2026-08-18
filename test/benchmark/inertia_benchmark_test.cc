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

// A benchmark for comparing legacy and two CSR implementations of inertia
// factor and then solve.

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to benchmark
static const int kNumBenchmarkSteps = 50;

// ----------------------------- benchmark ------------------------------------

static void BM_solve(benchmark::State& state) {
  static mjModel* m;
  m = LoadModelFromPath("../test/benchmark/testdata/inertia.xml");

  mjData* d = mj_makeData(m);
  mj_forward(m, d);

  // allocate input and output vectors
  mj_markStack(d);

  // M: mass matrix in CSR format
  mjtNum* M = mj_stackAllocNum(d, m->nC);
  mju_copy(M, d->M, m->nC);

  // arbitrary input vector
  mjtNum *res = mj_stackAllocNum(d, m->nv);
  mjtNum *vec = mj_stackAllocNum(d, m->nv);
  for (int i=0; i < m->nv; i++) {
    vec[i] = 0.2 + 0.3*i;
  }

  // benchmark
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    for (int i=0; i < kNumBenchmarkSteps; i++) {
      mju_copy(res, vec, m->nv);
      mju_copy(d->qLD, M, m->nC);
      mj_factorI(d->qLD, d->qLDiagInv, m->nv,
                 m->M_rownnz, m->M_rowadr, m->M_colind, nullptr);
      mj_solveLD(res, d->qLD, d->qLDiagInv, m->nv, 1,
                 m->M_rownnz, m->M_rowadr, m->M_colind, nullptr);
    }
  }

  // finalize
  mj_freeStack(d);
  mj_deleteData(d);
  mj_deleteModel(m);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solve_CSR(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solve(state);
}
BENCHMARK(BM_solve_CSR);

}  // namespace
}  // namespace mujoco
