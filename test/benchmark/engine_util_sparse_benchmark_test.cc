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
#include <cstring>
#include <vector>

#include <benchmark/benchmark.h>
#include <absl/base/attributes.h>
#include <mujoco/mjdata.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_support.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using CombineFuncPtr = decltype(&mju_combineSparse);
using TransposeFuncPtr = decltype(&mju_transposeSparse);
using SqrMatTDFuncPtr = decltype(&mju_sqrMatTDSparse);

// number of steps to roll out before benchmarking
static const int kNumWarmupSteps = 500;

// ----------------------------- old functions --------------------------------

void ABSL_ATTRIBUTE_NOINLINE mju_sqrMatTDSparse_baseline(
    mjtNum* res, const mjtNum* mat, const mjtNum* matT, const mjtNum* diag,
    int nr, int nc, int* res_rownnz, int* res_rowadr, int* res_colind,
    const int* rownnz, const int* rowadr, const int* colind,
    const int* rowsuper, const int* rownnzT, const int* rowadrT,
    const int* colindT, const int* rowsuperT, mjData* d, int* unused) {
  mj_markStack(d);
  int* chain = mj_stackAllocInt(d, 2 * nc);
  mjtNum* buffer = mj_stackAllocNum(d, nc);

  for (int r = 0; r < nc; r++) {
    res_rowadr[r] = r * nc;
  }

  for (int r = 0; r < nc; r++) {
    if (rowsuperT && r > 0 && rowsuperT[r - 1] > 0) {
      res_rownnz[r] = res_rownnz[r - 1];
      memcpy(res_colind + res_rowadr[r], res_colind + res_rowadr[r - 1],
             res_rownnz[r] * sizeof(int));

      if (rownnzT[r]) {
        res_colind[res_rowadr[r] + res_rownnz[r]] = r;
        res_rownnz[r]++;
      }
    } else {
      int nchain = 0;
      int inew = 0, iold = nc;
      int lastadded = -1;
      for (int i = 0; i < rownnzT[r]; i++) {
        int c = colindT[rowadrT[r] + i];
        if (rowsuper && lastadded >= 0 &&
            (c - lastadded) <= rowsuper[lastadded]) {
          continue;
        } else {
          lastadded = c;
        }

        int adr = inew;
        inew = iold;
        iold = adr;

        int nnewchain = 0;
        adr = 0;
        int end = rowadr[c] + rownnz[c];
        for (int adr1 = rowadr[c]; adr1 < end; adr1++) {
          int col_mat = colind[adr1];
          while (adr < nchain && chain[iold + adr] < col_mat &&
                 chain[iold + adr] <= r) {
            chain[inew + nnewchain++] = chain[iold + adr++];
          }

          if (col_mat > r) {
            break;
          }

          if (adr < nchain && chain[iold + adr] == col_mat) {
            adr++;
          }
          chain[inew + nnewchain++] = col_mat;
        }

        while (adr < nchain && chain[iold + adr] <= r) {
          chain[inew + nnewchain++] = chain[iold + adr++];
        }
        nchain = nnewchain;
      }
      res_rownnz[r] = nchain;
      if (nchain) {
        memcpy(res_colind + res_rowadr[r], chain + inew, nchain * sizeof(int));
      }
    }
  }

  for (int r = 0; r < nc; r++) {
    int adr = res_rowadr[r];
    for (int i = 0; i < res_rownnz[r]; i++) {
      buffer[res_colind[adr + i]] = 0;
    }
    for (int i = 0; i < rownnzT[r]; i++) {
      int c = colindT[rowadrT[r] + i];
      mjtNum matTrc = matT[rowadrT[r] + i];
      if (diag) {
        matTrc *= diag[c];
      }

      int end = rowadr[c] + rownnz[c];
      for (int adr = rowadr[c]; adr < end; adr++) {
        int adr1;
        if ((adr1 = colind[adr]) > r) {
          break;
        }
        buffer[adr1] += matTrc * mat[adr];
      }
    }
    adr = res_rowadr[r];
    for (int i = 0; i < res_rownnz[r]; i++) {
      res[adr + i] = buffer[res_colind[adr + i]];
    }
  }
  for (int r = 1; r < nc; r++) {
    int end = res_rowadr[r] + res_rownnz[r] - 1;
    for (int adr = res_rowadr[r]; adr < end; adr++) {
      int adr1 =  res_rowadr[res_colind[adr]] + res_rownnz[res_colind[adr]]++;
      res[adr1] = res[adr];
      res_colind[adr1] = r;
    }
  }

  mj_freeStack(d);
}

// transpose sparse matrix (uncompressed)
void ABSL_ATTRIBUTE_NOINLINE transposeSparse_baseline(
    mjtNum* res, const mjtNum* mat, int nr, int nc, int* res_rownnz,
    int* res_rowadr, int* res_colind, int* res_rowsuper,
    const int* rownnz, const int* rowadr, const int* colind) {
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

  mju_compressSparse(res, nc, nr, res_rownnz, res_rowadr, res_colind,
                     /*minval=*/-1);
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
                                                   const mjtNum* src,
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
                                              const mjtNum* src,
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
  static mjModel* m = LoadModelFromPath("flex/flag.xml");
  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate gradient
  mj_markStack(d);
  mjtNum *Ma = mj_stackAllocNum(d, m->nv);
  mjtNum *vec = mj_stackAllocNum(d, m->nv);
  mjtNum *res = mj_stackAllocNum(d, d->nefc);
  mjtNum *grad = mj_stackAllocNum(d, m->nv);
  mjtNum *Mgrad  = mj_stackAllocNum(d, m->nv);

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
  mj_freeStack(d);
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
  m->opt.jacobian = mjJAC_SPARSE;

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  for (int i=0; i < kNumWarmupSteps; i++) {
    mj_step(m, d);
  }

  // allocate
  mj_markStack(d);
  mjtNum* H = mj_stackAllocNum(d, m->nv*m->nv);
  int* rownnz = mj_stackAllocInt(d, m->nv);
  int* rowadr = mj_stackAllocInt(d, m->nv);
  int* colind = mj_stackAllocInt(d, m->nv*m->nv);
  int* diagind = mj_stackAllocInt(d, m->nv);

  // compute D corresponding to quad states
  mjtNum* D = mj_stackAllocNum(d, d->nefc);
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
      D[i] = d->efc_D[i];
    } else {
      D[i] = 0;
    }
  }

  // compute H = J'*D*J, uncompressed layout
  mju_sqrMatTDUncompressedInit(rowadr, m->nv);
  mju_sqrMatTDSparse(H, d->efc_J, d->efc_JT, D, d->nefc, m->nv,
                     rownnz, rowadr, colind,
                     d->efc_J_rownnz, d->efc_J_rowadr,
                     d->efc_J_colind, d->efc_J_rowsuper,
                     d->efc_JT_rownnz, d->efc_JT_rowadr,
                     d->efc_JT_colind, d->efc_JT_rowsuper, d,
                     diagind);

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
         func(H+rowadr[c], H+rowadr[r], 1, -H[adr+i],
              rownnz[c], rownnz[c],
              colind+rowadr[c], colind+rowadr[c], NULL, NULL);
      }
    }
  }

  // finalize
  mj_freeStack(d);
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

enum class Supernode {
  None,
  PostProcess,
  Inline
};

static void BM_transposeSparse(benchmark::State& state, TransposeFuncPtr func,
                               Supernode super) {
  static mjModel* m = LoadModelFromPath("humanoid/humanoid100.xml");

  // force use of sparse matrices
  m->opt.jacobian = mjJAC_SPARSE;

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  while (d->time < 2) {
    mj_step(m, d);
  }

  mj_markStack(d);

  // need uncompressed layout
  mjtNum* res = mj_stackAllocNum(d, m->nv * d->nefc);
  int* res_rownnz = mj_stackAllocInt(d, m->nv);
  int* res_rowadr = mj_stackAllocInt(d, m->nv);
  int* res_rowsuper = mj_stackAllocInt(d, m->nv);
  int* res_colind = mj_stackAllocInt(d, m->nv * d->nefc);

  // time benchmark
  for (auto s : state) {
    int* rowsuper = (super == Supernode::Inline) ? res_rowsuper : nullptr;
    func(res, d->efc_J, d->nefc, m->nv,
         res_rownnz, res_rowadr, res_colind, rowsuper,
         d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);
    if (super == Supernode::PostProcess) {
      mju_superSparse(m->nv, res_rowsuper,
                      res_rownnz, res_rowadr, res_colind);
    }
  }

  mj_freeStack(d);
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_old(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &transposeSparse_baseline, Supernode::None);
}
BENCHMARK(BM_transposeSparse_old);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_new(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &mju_transposeSparse, Supernode::None);
}
BENCHMARK(BM_transposeSparse_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_superpost(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &mju_transposeSparse, Supernode::PostProcess);
}
BENCHMARK(BM_transposeSparse_superpost);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_superinline(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse(state, &mju_transposeSparse, Supernode::Inline);
}
BENCHMARK(BM_transposeSparse_superinline);

static void BM_sqrMatTDSparse(benchmark::State& state, SqrMatTDFuncPtr func) {
  static mjModel* m =
      LoadModelFromPath("../test/benchmark/testdata/2humanoid100.xml");

  // force use of sparse matrices, Newton solver, no islands
  m->opt.jacobian = mjJAC_SPARSE;
  m->opt.solver = mjSOL_NEWTON;
  m->opt.enableflags &= ~mjENBL_ISLAND;

  mjData* d = mj_makeData(m);

  // warm-up rollout to get a typical state
  while (d->time < 2) {
    mj_step(m, d);
  }

  // allocate
  mj_markStack(d);
  mjtNum* H = mj_stackAllocNum(d, m->nv * m->nv);
  int* rownnz = mj_stackAllocInt(d, m->nv);
  int* rowadr = mj_stackAllocInt(d, m->nv);
  int* colind = mj_stackAllocInt(d, m->nv * m->nv);
  int* diagind = mj_stackAllocInt(d, m->nv);

  // compute D corresponding to quad states
  mjtNum* D = mj_stackAllocNum(d, d->nefc);
  for (int i = 0; i < d->nefc; i++) {
    if (d->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
      D[i] = d->efc_D[i];
    } else {
      D[i] = 0;
    }
  }

  // time benchmark
  if (func) {
    mju_sqrMatTDSparseCount(rownnz, rowadr, m->nv,
      d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind,
      d->efc_JT_rownnz, d->efc_JT_rowadr,
      d->efc_JT_colind, nullptr, d, 1);

    for (auto s : state) {
      // compute H = J'*D*J, compressed layout
      func(H, d->efc_J, d->efc_JT, D, d->nefc, m->nv, rownnz, rowadr, colind,
         d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind, NULL,
         d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind,
         d->efc_JT_rowsuper, d, diagind);
    }
  } else {
    for (auto s : state) {
      // baseline depends on efc_J_rowsuper
      mju_superSparse(d->nefc, d->efc_J_rowsuper,
                      d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);

      // compute H = J'*D*J, uncompressed layout
      mju_sqrMatTDSparse_baseline(
          H, d->efc_J, d->efc_JT, D, d->nefc, m->nv, rownnz, rowadr, colind,
          d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind, d->efc_J_rowsuper,
          d->efc_JT_rownnz, d->efc_JT_rowadr, d->efc_JT_colind,
          d->efc_JT_rowsuper, d, /*unused=*/nullptr);
    }
  }

  // finalize
  mj_freeStack(d);
  mj_deleteData(d);
  state.SetItemsProcessed(state.iterations());
}

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_sqrMatTDSparse_col(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_sqrMatTDSparse(state, &mju_sqrMatTDSparse);
}
BENCHMARK(BM_sqrMatTDSparse_col);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_sqrMatTDSparse_row(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_sqrMatTDSparse(state, &mju_sqrMatTDSparse_row);
}
BENCHMARK(BM_sqrMatTDSparse_row);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_sqrMatTDSparse_uncompressed(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_sqrMatTDSparse(state, nullptr);
}
BENCHMARK(BM_sqrMatTDSparse_uncompressed);

}  // namespace
}  // namespace mujoco
