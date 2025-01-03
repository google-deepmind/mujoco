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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjtnum.h>
#include "engine/engine_util_sparse_avx.h"  // IWYU pragma: keep

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ sparse operations -------------------------------------------------

// dot-product, both vectors are sparse, vec2 can be uncompressed
MJAPI mjtNum mju_dotSparse2(const mjtNum* vec1, const mjtNum* vec2, int nnz1, const int* ind1,
                            int nnz2, const int* ind2, int flg_unc2);

// convert matrix from dense to sparse
//  nnz is size of res and colind, return 1 if too small, 0 otherwise
MJAPI int mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                           int* rownnz, int* rowadr, int* colind, int nnz);

// convert matrix from sparse to dense
MJAPI void mju_sparse2dense(mjtNum* res, const mjtNum* mat, int nr, int nc, const int* rownnz,
                            const int* rowadr, const int* colind);

// multiply sparse matrix and dense vector:  res = mat * vec
MJAPI void mju_mulMatVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                               int nr, const int* rownnz, const int* rowadr,
                               const int* colind, const int* rowsuper);

// multiply transposed sparse matrix and dense vector:  res = mat' * vec
MJAPI void mju_mulMatTVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc,
                                const int* rownnz, const int* rowadr, const int* colind);

// compress layout of sparse matrix
MJAPI void mju_compressSparse(mjtNum* mat, int nr, int nc,
                              int* rownnz, int* rowadr, int* colind);

// count the number of non-zeros in the sum of two sparse vectors
MJAPI int mju_combineSparseCount(int a_nnz, int b_nnz, const int* a_ind, const int* b_ind);

// combine two sparse vectors: dst = a*dst + b*src, return nnz of result
int mju_combineSparse(mjtNum* dst, const mjtNum* src, mjtNum a, mjtNum b,
                      int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                      mjtNum* buf, int* buf_ind);

// incomplete combine sparse: dst = a*dst + b*src at common indices
void mju_combineSparseInc(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                          int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind);

// dst += scl * src, only at common non-zero indices
void mju_addToSclSparseInc(mjtNum* dst, const mjtNum* src,
                           int nnzdst, const int* inddst,
                           int nnzsrc, const int* indsrc, mjtNum scl);

// add to sparse matrix: dst = dst + scl*src, return nnz of result
int mju_addToSparseMat(mjtNum* dst, const mjtNum* src, int n, int nrow, mjtNum scl,
                       int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                       mjtNum* buf, int* buf_ind);

// add(merge) two chains
int mju_addChains(int* res, int n, int NV1, int NV2,
                  const int* chain1, const int* chain2);

// transpose sparse matrix
MJAPI void mju_transposeSparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                               int* res_rownnz, int* res_rowadr, int* res_colind,
                               const int* rownnz, const int* rowadr, const int* colind);

// construct row supernodes
MJAPI void mju_superSparse(int nr, int* rowsuper,
                           const int* rownnz, const int* rowadr, const int* colind);

// compute sparse M'*diag*M (diag=NULL: compute M'*M), res has uncompressed layout
// res_rowadr is required to be precomputed
MJAPI void mju_sqrMatTDSparse(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                              const mjtNum* diag, int nr, int nc,
                              int* res_rownnz, const int* res_rowadr, int* res_colind,
                              const int* rownnz, const int* rowadr,
                              const int* colind, const int* rowsuper,
                              const int* rownnzT, const int* rowadrT,
                              const int* colindT, const int* rowsuperT,
                              mjData* d);

// precount res_rownnz and precompute res_rowadr for mju_sqrMatTDSparse
MJAPI void mju_sqrMatTDSparseInit(int* res_rownnz, int* res_rowadr, int nr,
                                  const int* rownnz, const int* rowadr, const int* colind,
                                  const int* rownnzT, const int* rowadrT, const int* colindT,
                                  const int* rowsuperT, mjData* d);

// precompute res_rowadr for mju_sqrMatTDSparse using uncompressed memory
MJAPI void mju_sqrMatTDUncompressedInit(int* res_rowadr, int nc);

// compute row non-zeros of reverse-Cholesky factor L, return total
MJAPI int mju_cholFactorNNZ(int* L_rownnz, const int* rownnz, const int* rowadr, const int* colind,
                            int n, mjData* d);

// ------------------------------ inlined functions ------------------------------------------------

// dot-product, first vector is sparse
//  flg_unc1: is vec1 memory layout uncompressed
static inline
mjtNum mju_dotSparse(const mjtNum* vec1, const mjtNum* vec2, int nnz1, const int* ind1,
                     int flg_unc1) {
#ifdef mjUSEAVX
  return mju_dotSparse_avx(vec1, vec2, nnz1, ind1, flg_unc1);
#else
  int i = 0;
  mjtNum res = 0;
  int n_4 = nnz1 - 4;
  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;


  if (flg_unc1) {
    for (; i <= n_4; i+=4) {
      res0 += vec1[ind1[i+0]] * vec2[ind1[i+0]];
      res1 += vec1[ind1[i+1]] * vec2[ind1[i+1]];
      res2 += vec1[ind1[i+2]] * vec2[ind1[i+2]];
      res3 += vec1[ind1[i+3]] * vec2[ind1[i+3]];
    }
  } else {
    for (; i <= n_4; i+=4) {
      res0 += vec1[i+0] * vec2[ind1[i+0]];
      res1 += vec1[i+1] * vec2[ind1[i+1]];
      res2 += vec1[i+2] * vec2[ind1[i+2]];
      res3 += vec1[i+3] * vec2[ind1[i+3]];
    }
  }
  res = (res0 + res2) + (res1 + res3);

  // scalar part
  if (flg_unc1) {
    for (; i < nnz1; i++) {
      res += vec1[ind1[i]] * vec2[ind1[i]];
    }
  } else {
    for (; i < nnz1; i++) {
      res += vec1[i] * vec2[ind1[i]];
    }
  }

  return res;
#endif  // mjUSEAVX
}


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_
