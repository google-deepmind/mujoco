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
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

// number of steps to roll out before benhmarking
static const int kNumWarmupSteps = 500;

// copy array into vector
std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

// ----------------------------- old functions --------------------------------

mjtNum ABSL_ATTRIBUTE_NOINLINE dotSparse_1(const mjtNum* vec1,
                                           const mjtNum* vec2,
                                           const int nnz1,
                                           const int* ind1) {
  int i = 0;
  mjtNum res = 0;

  // scalar part
  for (; i < nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
}

mjtNum ABSL_ATTRIBUTE_NOINLINE dotSparse_8(const mjtNum* vec1,
                                           const mjtNum* vec2,
                                           const int nnz1,
                                           const int* ind1) {
  int i = 0;
  mjtNum res = 0;

  int n_8 = nnz1 - 8;

  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;
  mjtNum res4 = 0;
  mjtNum res5 = 0;
  mjtNum res6 = 0;
  mjtNum res7 = 0;

  for (; i <= n_8; i+=8) {
    res0 += vec1[i+0] * vec2[ind1[i+0]];
    res1 += vec1[i+1] * vec2[ind1[i+1]];
    res2 += vec1[i+2] * vec2[ind1[i+2]];
    res3 += vec1[i+3] * vec2[ind1[i+3]];
    res4 += vec1[i+4] * vec2[ind1[i+4]];
    res5 += vec1[i+5] * vec2[ind1[i+5]];
    res6 += vec1[i+6] * vec2[ind1[i+6]];
    res7 += vec1[i+7] * vec2[ind1[i+7]];
  }
  res = ((res0 + res2) + (res1 + res3)) + ((res4 + res6) + (res5 + res7));

  // process remaining
  int n_i = nnz1 - i;
  if (n_i == 7) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]] +
           vec1[i+2]*vec2[ind1[i+2]] + vec1[i+3]*vec2[ind1[i+3]] +
           vec1[i+4]*vec2[ind1[i+4]] + vec1[i+5]*vec2[ind1[i+5]] +
           vec1[i+6]*vec2[ind1[i+6]];
  } else if (n_i == 6) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]] +
           vec1[i+2]*vec2[ind1[i+2]] + vec1[i+3]*vec2[ind1[i+3]] +
           vec1[i+4]*vec2[ind1[i+4]] + vec1[i+5]*vec2[ind1[i+5]];
  } else if (n_i == 5) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]] +
           vec1[i+2]*vec2[ind1[i+2]] + vec1[i+3]*vec2[ind1[i+3]] +
           vec1[i+4]*vec2[ind1[i+4]];
  } else if (n_i == 4) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]] +
           vec1[i+2]*vec2[ind1[i+2]] + vec1[i+1]*vec2[ind1[i+3]];
  } else if (n_i == 3) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]] +
           vec1[i+2]*vec2[ind1[i+2]];
  } else if (n_i == 2) {
    res += vec1[i+0]*vec2[ind1[i+0]] + vec1[i+1]*vec2[ind1[i+1]];
  } else if (n_i == 1) {
    res += vec1[i+0]*vec2[ind1[i+0]];
  }

  return res;
}

void ABSL_ATTRIBUTE_NOINLINE mulMatVecSparse_1(mjtNum* res,
                                               const mjtNum* mat,
                                               const mjtNum* vec,
                                               int nr,
                                               const int* rownnz,
                                               const int* rowadr,
                                               const int* colind,
                                               const int* rowsuper) {
  for (int r=0; r < nr; r++) {
    res[r] = dotSparse_1(
      mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
  }
}

void ABSL_ATTRIBUTE_NOINLINE mulMatVecSparse_8(mjtNum* res,
                                               const mjtNum* mat,
                                               const mjtNum* vec,
                                               int nr,
                                               const int* rownnz,
                                               const int* rowadr,
                                               const int* colind,
                                               const int* rowsuper) {
  for (int r=0; r < nr; r++) {
    res[r] = dotSparse_8(
      mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
  }
}

// ----------------------------- benchmark ------------------------------------

static void BM_MatVecSparse(benchmark::State& state, int unroll) {
  static mjModel* m = LoadModelFromPath("composite/cloth.xml");
  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate gradient
  mjMARKSTACK;
  mjtNum *Ma = mj_stackAlloc(d, m->nv);
  mjtNum *vec = mj_stackAlloc(d, m->nv);
  mjtNum *res = mj_stackAlloc(d, m->nv);
  mjtNum *grad = mj_stackAlloc(d, m->nv);
  mjtNum *Mgrad  = mj_stackAlloc(d, m->nv);

  // compute gradient
  mj_mulM(m, d, Ma, d->qacc);
  for (int i=0; i < m->nv; i++) {
    grad[i] = Ma[i] - d->qfrc_smooth[i] - d->qfrc_constraint[i];
  }

  // compute search direction
  mj_solveM(m, d, Mgrad, grad, 1);
  mju_scl(vec, Mgrad, -1, m->nv);

  // save state
  std::vector<mjtNum> qpos = AsVector(d->qpos, m->nq);
  std::vector<mjtNum> qvel = AsVector(d->qvel, m->nv);
  std::vector<mjtNum> act = AsVector(d->act, m->na);
  std::vector<mjtNum> warmstart = AsVector(d->qacc_warmstart, m->nv);

  // time benchmark
  for (auto s : state) {
    if (unroll == 4) {
      mju_mulMatVecSparse(res, d->efc_J, vec, d->nefc,
                          d->efc_J_rownnz, d->efc_J_rowadr,
                          d->efc_J_colind, d->efc_J_rowsuper);
    } else if (unroll == 1) {
      mulMatVecSparse_1(res, d->efc_J, vec, d->nefc,
                        d->efc_J_rownnz, d->efc_J_rowadr,
                        d->efc_J_colind, d->efc_J_rowsuper);
    } else if (unroll == 8) {
      mulMatVecSparse_8(res, d->efc_J, vec, d->nefc,
                        d->efc_J_rownnz, d->efc_J_rowadr,
                        d->efc_J_colind, d->efc_J_rowsuper);
    }
  }

  // finalize
  mjFREESTACK;
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_MatVecSparse_8(
  benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_MatVecSparse(state, 8);
}
BENCHMARK(BM_MatVecSparse_8);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_MatVecSparse_4(
  benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_MatVecSparse(state, 4);
}
BENCHMARK(BM_MatVecSparse_4);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_MatVecSparse_1(
  benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_MatVecSparse(state, 1);
}
BENCHMARK(BM_MatVecSparse_1);

}  // namespace
}  // namespace mujoco
