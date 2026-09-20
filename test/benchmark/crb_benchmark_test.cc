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

// A benchmark for comparing different implementations of mj_crb.

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_core_smooth.h"
#include "src/engine/engine_util_spatial.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to benchmark
static const int kNumBenchmarkSteps = 50;

// ----------------------------- previous implementation -----------------------

void ABSL_ATTRIBUTE_NOINLINE mj_crb_baseline(const mjModel* m, mjData* d) {
  int nv = m->nv;
  mjtNum buf[6];
  mjtNum* crb = d->crb;

  // crb = cinert
  mju_copy(crb, d->cinert, 10*m->nbody);

  // backward pass over bodies, accumulate composite inertias
  for (int i=m->nbody - 1; i > 0; i--) {
    if (m->body_parentid[i] > 0) {
      mju_addTo(crb+10*m->body_parentid[i], crb+10*i, 10);
    }
  }

  // clear M
  mju_zero(d->M, m->nC);

  // dense forward pass over dofs
  for (int i=0; i < nv; i++) {
    // process block of diagonals (simple bodies)
    if (m->dof_simplenum[i]) {
      int n = i + m->dof_simplenum[i];
      for (; i < n; i++) {
        d->M[m->M_rowadr[i]] = m->dof_M0[i];
      }

      // finish or else fall through with next row
      if (i == nv) {
        break;
      }
    }

    // init M(i,i) with armature inertia
    int Madr_ij = m->M_rowadr[i] + m->M_rownnz[i] - 1;
    d->M[Madr_ij] = m->dof_armature[i];

    // precompute buf = crb_body_i * cdof_i
    mju_mulInertVec(buf, crb+10*m->dof_bodyid[i], d->cdof+6*i);

    // sparse backward pass over ancestors
    for (int j=i; j >= 0; j = m->dof_parentid[j]) {
      // M(i,j) += cdof_j * (crb_body_i * cdof_i)
      d->M[Madr_ij--] += mju_dot(d->cdof+6*j, buf, 6);
    }
  }
}


// ----------------------------- benchmark ------------------------------------

static void BM_crb(benchmark::State& state, bool baseline) {
  static mjModel* m = nullptr;
  static mjData* d = nullptr;

  if (!m) {
    m = LoadModelFromPath("../test/benchmark/testdata/inertia.xml");
    d = mj_makeData(m);
  }
  mj_forward(m, d);

  // benchmark
  while (state.KeepRunningBatch(kNumBenchmarkSteps)) {
    if (baseline) {
      for (int i=0; i < kNumBenchmarkSteps; i++) {
        mj_crb_baseline(m, d);
      }
    } else {
      for (int i=0; i < kNumBenchmarkSteps; i++) {
        mj_crb(m, d);
      }
    }
  }

  // finalize
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_CRB(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_crb(state, /*baseline=*/false);
}
BENCHMARK(BM_CRB);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_CRB_BASELINE(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_crb(state, /*baseline=*/true);
}
BENCHMARK(BM_CRB_BASELINE);

}  // namespace
}  // namespace mujoco
