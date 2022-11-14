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
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
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

// ----------------------------- old functions --------------------------------

void ABSL_ATTRIBUTE_NOINLINE solveLD_baseline(const mjModel* m, mjtNum* x,
                                              const mjtNum* y,
                                              const mjtNum* qLD,
                                              const mjtNum* qLDiagInv) {
  mjtNum tmp;

  // local copies of key variables
  int* dof_Madr = m->dof_Madr;
  int* dof_parentid = m->dof_parentid;
  int nv = m->nv;

  // x = y
  if (x != y) {
    mju_copy(x, y, nv);
  }

  // x <- inv(L') * x; skip simple, exploit sparsity of input vector
  for (int i=nv-1; i >= 0; i--) {
    if (!m->dof_simplenum[i] && (tmp = x[i])) {
      // init
      int Madr_ij = dof_Madr[i]+1;
      int j = dof_parentid[i];

      // traverse ancestors backwards
      while (j >= 0) {
        x[j] -= qLD[Madr_ij++]*tmp;         // x(j) -= L(i,j) * x(i)

        // advance to parent
        j = dof_parentid[j];
      }
    }
  }

  // x <- inv(D) * x
  for (int i=0; i < nv; i++) {
    x[i] *= qLDiagInv[i];  // x(i) /= L(i,i)
  }

  // x <- inv(L) * x; skip simple
  for (int i=0; i < nv; i++) {
    if (!m->dof_simplenum[i]) {
      // init
      int Madr_ij = dof_Madr[i]+1;
      int j = dof_parentid[i];

      // traverse ancestors backwards
      tmp = x[i];
      while (j>= 0) {
        tmp -= qLD[Madr_ij++]*x[j];             // x(i) -= L(i,j) * x(j)

        // advance to parent
        j = dof_parentid[j];
      }
      x[i] = tmp;
    }
  }
}

void solveM_baseline(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y) {
  solveLD_baseline(m, x, y, d->qLD, d->qLDiagInv);
}

// ----------------------------- benchmark ------------------------------------

static void BM_solveLD(benchmark::State& state, bool new_function) {
  static mjModel* m = LoadModelFromPath("composite/cloth.xml");
  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typcal state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate gadient
  mjMARKSTACK;
  mjtNum *grad = mj_stackAlloc(d, m->nv);
  mjtNum *Ma = mj_stackAlloc(d, m->nv);
  mjtNum *res = mj_stackAlloc(d, m->nv);

  // compute gradient
  mj_mulM(m, d, Ma, d->qacc);
  for (int i=0; i < m->nv; i++) {
    grad[i] = Ma[i] - d->qfrc_smooth[i] - d->qfrc_constraint[i];
  }

  // save state
  std::vector<mjtNum> qpos = AsVector(d->qpos, m->nq);
  std::vector<mjtNum> qvel = AsVector(d->qvel, m->nv);
  std::vector<mjtNum> act = AsVector(d->act, m->na);
  std::vector<mjtNum> warmstart = AsVector(d->qacc_warmstart, m->nv);

  // reset state, benchmark subsequent kNumBenchmarkSteps steps
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    mju_copy(d->qpos, qpos.data(), m->nq);
    mju_copy(d->qvel, qvel.data(), m->nv);
    mju_copy(d->act, act.data(), m->na);
    mju_copy(d->qacc_warmstart, warmstart.data(), m->nv);

    for (int i=0; i < kNumBenchmarkSteps; i++) {
      if (new_function) {
        mj_solveM(m, d, res, grad, 1);
      } else {
        solveM_baseline(m, d, res, grad);
      }
    }
  }

  // finalize
  mjFREESTACK;
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_new(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, true);
}
BENCHMARK(BM_solveLD_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_solveLD_old(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_solveLD(state, false);
}
BENCHMARK(BM_solveLD_old);

}  // namespace
}  // namespace mujoco
