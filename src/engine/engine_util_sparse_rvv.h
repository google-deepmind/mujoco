// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_RVV_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_RVV_H_

#include "engine/engine_util_rvv.h"

#ifdef mjUSERVV

#include <riscv_vector.h>
#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjtype.h>


//------------------------------ sparse operations using rvv ---------------------------------------

// dot-product, first vector is sparse
static inline
mjtNum mju_dotSparse_rvv(const mjtNum* vec1, const mjtNum* vec2, int nnz1, const int* ind1) {
  int i = 0;
  mjtNum res = 0;
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4 >= 0) {
    vfloat64m2_t sum = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec1, 4),
                                              mju_gather4_rvv(vec2, ind1), 4);
    i = 4;
    while (i <= nnz1_4) {
      vfloat64m2_t prod = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec1+i, 4),
                                                 mju_gather4_rvv(vec2, ind1+i), 4);
      sum = __riscv_vfadd_vv_f64m2(sum, prod, 4);
      i += 4;
    }
    res = mju_reduce4_rvv(sum);
  }

  // scalar part
  for (; i < nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
}


// dot-productX3, first vector is sparse; supernode of size 3
static inline
void mju_dotSparseX3_rvv(mjtNum* res0, mjtNum* res1, mjtNum* res2, const mjtNum* vec10,
                         const mjtNum* vec11, const mjtNum* vec12, const mjtNum* vec2,
                         int nnz1, const int* ind1) {
  int i = 0;
  mjtNum RES0 = 0;
  mjtNum RES1 = 0;
  mjtNum RES2 = 0;
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4 >= 0) {
    // gather val2 only once per block
    vfloat64m2_t val2 = mju_gather4_rvv(vec2, ind1);
    vfloat64m2_t sum0 = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec10, 4), val2, 4);
    vfloat64m2_t sum1 = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec11, 4), val2, 4);
    vfloat64m2_t sum2 = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec12, 4), val2, 4);
    i = 4;

    while (i <= nnz1_4) {
      val2 = mju_gather4_rvv(vec2, ind1+i);
      sum0 = __riscv_vfadd_vv_f64m2(
          sum0, __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec10+i, 4), val2, 4), 4);
      sum1 = __riscv_vfadd_vv_f64m2(
          sum1, __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec11+i, 4), val2, 4), 4);
      sum2 = __riscv_vfadd_vv_f64m2(
          sum2, __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec12+i, 4), val2, 4), 4);
      i += 4;
    }

    RES0 = mju_reduce4_rvv(sum0);
    RES1 = mju_reduce4_rvv(sum1);
    RES2 = mju_reduce4_rvv(sum2);
  }

  // scalar part
  for (; i < nnz1; i++) {
    mjtNum v2 = vec2[ind1[i]];
    RES0 += vec10[i] * v2;
    RES1 += vec11[i] * v2;
    RES2 += vec12[i] * v2;
  }

  *res0 = RES0;
  *res1 = RES1;
  *res2 = RES2;
}


// multiply sparse matrix and dense vector:  res = mat * vec.
static inline
void mju_mulMatVecSparse_rvv(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                             int nr, const int* rownnz, const int* rowadr,
                             const int* colind, const int* rowsuper) {
  if (!rowsuper) {
    for (int r=0; r < nr; r++) {
      res[r] = mju_dotSparse_rvv(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }
    return;
  }

  // regular or supernode
  for (int r=0; r < nr; r++) {
    if (rowsuper[r]) {
      int rs = rowsuper[r]+1;

      // handle rows in blocks of 3
      while (rs >= 3) {
        mju_dotSparseX3_rvv(res+r, res+r+1, res+r+2,
                            mat+rowadr[r], mat+rowadr[r+1], mat+rowadr[r+2],
                            vec, rownnz[r], colind+rowadr[r]);
        r += 3;
        rs -= 3;
      }

      // handle remaining rows
      while (rs > 0) {
        res[r] = mju_dotSparse_rvv(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
        r++;
        rs--;
      }

      // go back one, because of outer for loop
      r--;
    } else {
      res[r] = mju_dotSparse_rvv(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }
  }
}


// res = res*scl1 + vec*scl2
static inline
void mju_addToSclScl_rvv(mjtNum* res, const mjtNum* vec, mjtNum scl1, mjtNum scl2, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vfmul_vf_f64m2(__riscv_vle64_v_f64m2(res, vl), scl1, vl);
    vfloat64m2_t b = __riscv_vfmul_vf_f64m2(__riscv_vle64_v_f64m2(vec, vl), scl2, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfadd_vv_f64m2(a, b, vl), vl);
  }
}


// return 1 if vec1==vec2, 0 otherwise
static inline
int mju_compare_rvv(const int* vec1, const int* vec2, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, vec1 += vl, vec2 += vl) {
    vl = __riscv_vsetvl_e32m4(avl);
    vint32m4_t a = __riscv_vle32_v_i32m4(vec1, vl);
    vint32m4_t b = __riscv_vle32_v_i32m4(vec2, vl);
    if (__riscv_vfirst_m_b8(__riscv_vmsne_vv_i32m4_b8(a, b, vl), vl) >= 0) {
      return 0;
    }
  }
  return 1;
}

#endif  // mjUSERVV

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_RVV_H_
