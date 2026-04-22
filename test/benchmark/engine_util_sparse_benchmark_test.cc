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

// ================================ Cached Data ================================

// ---- MatVecSparse data ----
struct MatVecData {
  int nv;
  int nefc;
  int nJ;
  std::vector<mjtNum> efc_J;
  std::vector<int> efc_J_rownnz, efc_J_rowadr, efc_J_colind, efc_J_rowsuper;
  std::vector<mjtNum> vec;
};

MatVecData& GetMatVecData() {
  static MatVecData data = [] {
    MatVecData d;
    mjModel* m = LoadModelFromPath("flex/flag.xml");
    mjData* dat = mj_makeData(m);

    for (int i = 0; i < 500; i++) {
      mj_step(m, dat);
    }

    d.nv = m->nv;
    d.nefc = dat->nefc;
    d.nJ = dat->nJ;
    d.efc_J.assign(dat->efc_J, dat->efc_J + d.nJ);
    d.efc_J_rownnz.assign(dat->efc_J_rownnz, dat->efc_J_rownnz + d.nefc);
    d.efc_J_rowadr.assign(dat->efc_J_rowadr, dat->efc_J_rowadr + d.nefc);
    d.efc_J_colind.assign(dat->efc_J_colind, dat->efc_J_colind + d.nJ);
    d.efc_J_rowsuper.assign(dat->efc_J_rowsuper, dat->efc_J_rowsuper + d.nefc);

    // compute direction: vec = -M^{-1} * (Ma - qfrc_smooth - qfrc_constraint)
    mj_markStack(dat);
    mjtNum* Ma = mj_stackAllocNum(dat, m->nv);
    mjtNum* grad = mj_stackAllocNum(dat, m->nv);
    mjtNum* Mgrad = mj_stackAllocNum(dat, m->nv);
    mj_mulM(m, dat, Ma, dat->qacc);
    for (int i = 0; i < m->nv; i++) {
      grad[i] = Ma[i] - dat->qfrc_smooth[i] - dat->qfrc_constraint[i];
    }
    mj_solveM(m, dat, Mgrad, grad, 1);
    d.vec.resize(m->nv);
    mju_scl(d.vec.data(), Mgrad, -1, m->nv);
    mj_freeStack(dat);

    mj_deleteData(dat);
    mj_deleteModel(m);
    return d;
  }();
  return data;
}

// ---- CombineSparse data ----
struct CombineData {
  int nv;
  std::vector<mjtNum> H;
  std::vector<int> rownnz, rowadr, colind;
};

CombineData& GetCombineData() {
  static CombineData data = [] {
    CombineData cd;
    mjModel* m = LoadModelFromPath("humanoid/humanoid.xml");
    m->opt.jacobian = mjJAC_SPARSE;
    mjData* d = mj_makeData(m);

    for (int i = 0; i < 500; i++) {
      mj_step(m, d);
    }

    cd.nv = m->nv;
    mj_markStack(d);
    mjtNum* H = mj_stackAllocNum(d, m->nv*m->nv);
    int* rownnz = mj_stackAllocInt(d, m->nv);
    int* rowadr = mj_stackAllocInt(d, m->nv);
    int* colind = mj_stackAllocInt(d, m->nv*m->nv);
    int* diagind = mj_stackAllocInt(d, m->nv);

    mjtNum* D = mj_stackAllocNum(d, d->nefc);
    for (int i = 0; i < d->nefc; i++) {
      if (d->efc_state[i] == mjCNSTRSTATE_QUADRATIC) {
        D[i] = d->efc_D[i];
      } else {
        D[i] = 0;
      }
    }

    int* JT_rownnz   = mj_stackAllocInt(d, m->nv);
    int* JT_rowadr   = mj_stackAllocInt(d, m->nv);
    int* JT_rowsuper = mj_stackAllocInt(d, m->nv);
    int* JT_colind   = mj_stackAllocInt(d, d->nJ);
    mjtNum* JT       = mj_stackAllocNum(d, d->nJ);
    mju_transposeSparse(JT, d->efc_J, d->nefc, m->nv,
                        JT_rownnz, JT_rowadr, JT_colind, JT_rowsuper,
                        d->efc_J_rownnz, d->efc_J_rowadr, d->efc_J_colind);

    // compute H = J'*D*J, uncompressed layout
    mju_sqrMatTDUncompressedInit(rowadr, m->nv);
    mju_sqrMatTDSparse(H, d->efc_J, JT, D, d->nefc, m->nv,
                       rownnz, rowadr, colind,
                       d->efc_J_rownnz, d->efc_J_rowadr,
                       d->efc_J_colind, d->efc_J_rowsuper,
                       JT_rownnz, JT_rowadr,
                       JT_colind, JT_rowsuper, d,
                       diagind);

    // compute H = M + J'*D*J
    mj_addM(m, d, H, rownnz, rowadr, colind);

    // copy to persistent storage
    int nH = rowadr[m->nv-1] + m->nv;  // uncompressed: rowadr[r] = r*nv
    cd.H.assign(H, H + nH);
    cd.rownnz.assign(rownnz, rownnz + m->nv);
    cd.rowadr.assign(rowadr, rowadr + m->nv);
    cd.colind.assign(colind, colind + nH);

    mj_freeStack(d);
    mj_deleteData(d);
    mj_deleteModel(m);
    return cd;
  }();
  return data;
}

// ---- TransposeSparse data ----
struct TransposeData {
  int nv;
  int nefc;
  int nJ;
  std::vector<mjtNum> efc_J;
  std::vector<int> efc_J_rownnz, efc_J_rowadr, efc_J_colind;
};

enum class Size { H2_100, H100 };

template <Size S>
const char* ModelPath() {
  if constexpr (S == Size::H2_100) {
    return "../test/benchmark/testdata/2humanoid100_chol.xml";
  } else {
    return "../test/benchmark/testdata/100_humanoids_chol.xml";
  }
}

template <Size S>
TransposeData& GetTransposeData() {
  static TransposeData data = [] {
    TransposeData td;
    mjModel* m = LoadModelFromPath(ModelPath<S>());
    m->opt.jacobian = mjJAC_SPARSE;
    mjData* d = mj_makeData(m);

    while (d->time < 2) {
      mj_step(m, d);
    }

    td.nv = m->nv;
    td.nefc = d->nefc;
    td.nJ = d->nJ;
    td.efc_J.assign(d->efc_J, d->efc_J + d->nJ);
    td.efc_J_rownnz.assign(d->efc_J_rownnz, d->efc_J_rownnz + d->nefc);
    td.efc_J_rowadr.assign(d->efc_J_rowadr, d->efc_J_rowadr + d->nefc);
    td.efc_J_colind.assign(d->efc_J_colind, d->efc_J_colind + d->nJ);

    mj_deleteData(d);
    mj_deleteModel(m);
    return td;
  }();
  return data;
}

// ================================ old functions ==============================

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
                                                   const int* src_ind) {
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
                                              const int* src_ind) {
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

// ----------------------------- benchmark -------------------------------------

static void BM_MatVecSparse(benchmark::State& state, int unroll) {
  MatVecData& data = GetMatVecData();
  std::vector<mjtNum> res(data.nefc);

  for (auto s : state) {
    if (unroll == 4) {
      mju_mulMatVecSparse(res.data(), data.efc_J.data(), data.vec.data(),
                          data.nefc, data.efc_J_rownnz.data(),
                          data.efc_J_rowadr.data(), data.efc_J_colind.data(),
                          data.efc_J_rowsuper.data());
    } else if (unroll == 1) {
      mulMatVecSparse_1(res.data(), data.efc_J.data(), data.vec.data(),
                        data.nefc, data.efc_J_rownnz.data(),
                        data.efc_J_rowadr.data(), data.efc_J_colind.data(),
                        data.efc_J_rowsuper.data());
    } else if (unroll == 8) {
      mulMatVecSparse_8(res.data(), data.efc_J.data(), data.vec.data(),
                        data.nefc, data.efc_J_rownnz.data(),
                        data.efc_J_rowadr.data(), data.efc_J_colind.data(),
                        data.efc_J_rowsuper.data());
    }
  }

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
  CombineData& data = GetCombineData();

  // make working copies that get modified each iteration
  std::vector<mjtNum> H = data.H;
  std::vector<int> rownnz = data.rownnz;
  std::vector<int> rowadr = data.rowadr;
  std::vector<int> colind = data.colind;

  // time benchmark
  for (auto s : state) {
    for (int r = data.nv-1; r >= 0; r--) {
      for (int i = 0; i < rownnz[r]-1; i++) {
        int adr = rowadr[r];
        int c = colind[adr+i];
        // true arguments should be i+1 and colind+rowadr[r]
        // but instead we repeat rownnz[c] and colind+rowadr[c]
        // in order to trigger all if's in combineSparse
         func(H.data()+rowadr[c], H.data()+rowadr[r], 1, -H[adr+i],
              rownnz[c], rownnz[c],
              colind.data()+rowadr[c], colind.data()+rowadr[c]);
      }
    }
  }

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

template <Size S>
static void BM_transposeSparse(benchmark::State& state, TransposeFuncPtr func,
                               Supernode super) {
  TransposeData& data = GetTransposeData<S>();

  // allocate output buffers (uncompressed layout)
  std::vector<mjtNum> res(data.nv * data.nefc);
  std::vector<int> res_rownnz(data.nv);
  std::vector<int> res_rowadr(data.nv);
  std::vector<int> res_rowsuper(data.nv);
  std::vector<int> res_colind(data.nv * data.nefc);

  // time benchmark
  for (auto s : state) {
    int* rowsuper =
        (super == Supernode::Inline) ? res_rowsuper.data() : nullptr;
    func(res.data(), data.efc_J.data(), data.nefc, data.nv,
         res_rownnz.data(), res_rowadr.data(), res_colind.data(), rowsuper,
         data.efc_J_rownnz.data(), data.efc_J_rowadr.data(),
         data.efc_J_colind.data());
    if (super == Supernode::PostProcess) {
      mju_superSparse(data.nv, res_rowsuper.data(),
                      res_rownnz.data(), res_rowadr.data(), res_colind.data());
    }
  }

  state.SetItemsProcessed(state.iterations());
}


void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_2H100_old(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H2_100>(state, &transposeSparse_baseline,
                                   Supernode::None);
}
BENCHMARK(BM_transposeSparse_2H100_old);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_2H100_new(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H2_100>(state, &mju_transposeSparse,
                                   Supernode::None);
}
BENCHMARK(BM_transposeSparse_2H100_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_2H100_superpost(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H2_100>(state, &mju_transposeSparse,
                                   Supernode::PostProcess);
}
BENCHMARK(BM_transposeSparse_2H100_superpost);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_2H100_superinline(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H2_100>(state, &mju_transposeSparse,
                                   Supernode::Inline);
}
BENCHMARK(BM_transposeSparse_2H100_superinline);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_100H_old(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H100>(state, &transposeSparse_baseline,
                                 Supernode::None);
}
BENCHMARK(BM_transposeSparse_100H_old);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_100H_new(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H100>(state, &mju_transposeSparse, Supernode::None);
}
BENCHMARK(BM_transposeSparse_100H_new);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_100H_superpost(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H100>(state, &mju_transposeSparse,
                                 Supernode::PostProcess);
}
BENCHMARK(BM_transposeSparse_100H_superpost);

void ABSL_ATTRIBUTE_NO_TAIL_CALL
BM_transposeSparse_100H_superinline(benchmark::State& state) {
  MujocoErrorTestGuard guard;
  BM_transposeSparse<Size::H100>(state, &mju_transposeSparse,
                                 Supernode::Inline);
}
BENCHMARK(BM_transposeSparse_100H_superinline);

}  // namespace
}  // namespace mujoco
