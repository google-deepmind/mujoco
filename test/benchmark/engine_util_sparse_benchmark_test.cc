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

#include <cstddef>
#include <benchmark/benchmark.h>
#include <gtest/gtest.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using CombineFuncPtr = decltype(&mju_combineSparse);
using TransposeFuncPtr = decltype(&mju_transposeSparse);

// number of steps to roll out before benchmarking
static const int kNumWarmupSteps = 500;

// copy array into vector
std::vector<mjtNum> AsVector(const mjtNum* array, int n) {
  return std::vector<mjtNum>(array, array + n);
}

// ----------------------------- old functions --------------------------------

// transpose sparse matrix (uncompressed)
void ABSL_ATTRIBUTE_NOINLINE transposeSparse_baseline(
    mjtNum* res, const mjtNum* mat, int nr, int nc, int* res_rownnz,
    int* res_rowadr, int* res_colind, const int* rownnz, const int* rowadr,
    const int* colind) {
  memset(res_rownnz, 0, nc * sizeof(int));
  for (int rt = 0; rt < nc; rt++) {
    res_rowadr[rt] = rt * nr;
  }
  for (int r = 0; r < nr; r++) {
    for (int ci = 0; ci < rownnz[r]; ci++) {
      int rt = colind[rowadr[r] + ci];
      res_colind[rt * nr + res_rownnz[rt]] = r;
      res[rt * nr + res_rownnz[rt]] = mat[rowadr[r] + ci];
      res_rownnz[rt]++;
    }
  }

  mju_compressSparse(res, nc, nr, res_rownnz, res_rowadr, res_colind);
}

int compare_baseline(const int* vec1,
                     const int* vec2,
                     int n) {
  int i = 0;

  for (; i < n; i++) {
    if (vec1[i] != vec2[i]) {
      return 0;
    }
  }

  return 1;
}

void addToSclScl(mjtNum* res,
                 const mjtNum* vec,
                 mjtNum scl1,
                 mjtNum scl2,
                 int n) {
  int i = 0;

  for (; i < n; i++) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }
}

int compare_memcmp(const int* vec1,
                const int* vec2,
                int n) {
  return !memcmp(vec1, vec2, n*sizeof(int));
}

int ABSL_ATTRIBUTE_NOINLINE combineSparse_baseline(mjtNum* dst,
                                                   const mjtNum* src, int n,
                                                   mjtNum a, mjtNum b,
                                                   int dst_nnz, int src_nnz,
                                                   int* dst_ind,
                                                   const int* src_ind,
                                                   mjtNum* buf, int* buf_ind) {
  // check for identical pattern
  if (compare_baseline(dst_ind, src_ind, dst_nnz)) {
    // combine mjtNum data directly
    addToSclScl(dst, src, a, b, dst_nnz);
    return dst_nnz;
  } else {
    return 0;
  }
}

int ABSL_ATTRIBUTE_NOINLINE combineSparse_new(mjtNum* dst,
                                              const mjtNum* src, int n,
                                              mjtNum a, mjtNum b,
                                              int dst_nnz, int src_nnz,
                                              int* dst_ind,
                                              const int* src_ind,
                                              mjtNum* buf, int* buf_ind) {
  // check for identical pattern
  if (compare_memcmp(dst_ind, src_ind, dst_nnz)) {
    // combine mjtNum data directly
    addToSclScl(dst, src, a, b, dst_nnz);
    return dst_nnz;
  } else {
    return 0;
  }
}

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

static void BM_combineSparse(benchmark::State& state, CombineFuncPtr func) {
  static mjModel* m = LoadModelFromPath("humanoid/humanoid.xml");

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate
  mjMARKSTACK;
  mjtNum* H = mj_stackAlloc(d, m->nv*m->nv);
  int* rownnz = (int*)mj_stackAlloc(d, m->nv);
  int* rowadr = (int*)mj_stackAlloc(d, m->nv);
  int* colind = (int*)mj_stackAlloc(d, m->nv*m->nv);

  // compute D corresponding to quad states
  mjtNum* D = mj_stackAlloc(d, d->nefc);
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
      D[i] = d->efc_D[i];
    } else {
      D[i] = 0;
    }
  }

  // compute H = J'*D*J, uncompressed layout
  mju_sqrMatTDSparse(H, d->efc_J, d->efc_JT, D, d->nefc, m->nv,
                      rownnz, rowadr, colind,
                      d->efc_J_rownnz, d->efc_J_rowadr,
                      d->efc_J_colind, d->efc_J_rowsuper,
                      d->efc_JT_rownnz, d->efc_JT_rowadr,
                      d->efc_JT_colind, d->efc_JT_rowsuper, d);

  // compute H = M + J'*D*J
  mj_addM(m, d, H, rownnz, rowadr, colind);

  // time benchmark
  for (auto s : state) {
    for (int r = m->nv-1; r >= 0; r--) {
      for (int i = 0; i < rownnz[r]-1; i++) {
        int adr = rowadr[r];
        int c = colind[adr+i];
        // true arguments should be i+1 and colind+rowadr[r]
        // but instead we repeat rownnz[c] and colind+rowadr[c]
        // in order to trigger all if's in combineSparse
         func(H+rowadr[c], H+rowadr[r], c+1, 1, -H[adr+i],
              rownnz[c], rownnz[c],
              colind+rowadr[c], colind+rowadr[c], NULL, NULL);
      }
    }
  }

  // finalize
  mjFREESTACK;
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_combineSparse_new(
  benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_combineSparse(state, &combineSparse_new);
}
BENCHMARK(BM_combineSparse_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL BM_combineSparse_old(
  benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_combineSparse(state, &combineSparse_baseline);
}
BENCHMARK(BM_combineSparse_old);

static void BM_transposeSparse(benchmark::State& state, TransposeFuncPtr func) {
  static mjModel* m = LoadModelFromPath("humanoid100/humanoid100.xml");

  // force use of sparse matrices
  m->opt.jacobian = mjJAC_SPARSE;

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  while (d->time < 2) {
    mj_step(m, d);
  }

  mjMARKSTACK;

  // need uncompressed layout
  mjtNum* res = mj_stackAlloc(d, m->nv * d->nefc);
  int* res_rownnz = (int*)mj_stackAlloc(d, m->nv);
  int* res_rowadr = (int*)mj_stackAlloc(d, m->nv);
  int* res_colind = (int*)mj_stackAlloc(d, m->nv * d->nefc);

  // time benchmark
  for (auto s : state) {
    func(res, d->efc_J, d->nefc, m->nv, res_rownnz, res_rowadr, res_colind,
         d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
  }

  mjFREESTACK;
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_new(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &mju_transposeSparse);
}
BENCHMARK(BM_transposeSparse_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_old(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &transposeSparse_baseline);
}

BENCHMARK(BM_transposeSparse_old);

}  // namespace
}  // namespace mujoco
