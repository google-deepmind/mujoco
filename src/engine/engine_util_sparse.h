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
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ sparse operations -------------------------------------------------

// dot-product, first vector is sparse
mjtNum mju_dotSparse(const mjtNum* vec1, const mjtNum* vec2,
                     const int nnz1, const int* ind1);

// dot-product, both vectors are sparse
mjtNum mju_dotSparse2(const mjtNum* vec1, const mjtNum* vec2,
                      const int nnz1, const int* ind1,
                      const int nnz2, const int* ind2);

// convert matrix from dense to sparse
void mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                      int* rownnz, int* rowadr, int* colind);

// convert matrix from sparse to dense
void mju_sparse2dense(mjtNum* res, const mjtNum* mat, int nr, int nc,
                      const int* rownnz, const int* rowadr, const int* colind);

// multiply sparse matrix and dense vector:  res = mat * vec
void mju_mulMatVecSparse(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                         int nr, const int* rownnz, const int* rowadr,
                         const int* colind, const int* rowsuper);

// compress layout of sparse matrix
void mju_compressSparse(mjtNum* mat, int nr, int nc,
                        int* rownnz, int* rowadr, int* colind);

// combine two sparse vectors: dst = a*dst + b*src, return nnz of result
int mju_combineSparse(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                      int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind,
                      mjtNum* buf, int* buf_ind);

// incomplete combine sparse: dst = a*dst + b*src at common indices
void mju_combineSparseInc(mjtNum* dst, const mjtNum* src, int n, mjtNum a, mjtNum b,
                          int dst_nnz, int src_nnz, int* dst_ind, const int* src_ind);

// transpose sparse matrix
void mju_transposeSparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
                         int* res_rownnz, int* res_rowadr, int* res_colind,
                         const int* rownnz, const int* rowadr, const int* colind);

// construct row supernodes
void mju_superSparse(int nr, int* rowsuper,
                     const int* rownnz, const int* rowadr, const int* colind);

// compute sparse M'*diag*M (diag=NULL: compute M'*M), res has uncompressed layout
void mju_sqrMatTDSparse(mjtNum* res, const mjtNum* mat, const mjtNum* matT,
                        const mjtNum* diag, int nr, int nc,
                        int* res_rownnz, int* res_rowadr, int* res_colind,
                        const int* rownnz, const int* rowadr,
                        const int* colind, const int* rowsuper,
                        const int* rownnzT, const int* rowadrT,
                        const int* colindT, const int* rowsuperT,
                        mjData* d);


#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_H_
