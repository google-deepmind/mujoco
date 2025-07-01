// Copyright 2023 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_AVX_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_AVX_H_

#ifdef mjUSEPLATFORMSIMD
#if defined(__AVX__) && !defined(mjUSESINGLE)

#define mjUSEAVX

#include <immintrin.h>
#include <string.h>

#include <mujoco/mjtnum.h>
#include "engine/engine_macro.h"


//------------------------------ sparse operations using avx ---------------------------------------

// dot-product, first vector is sparse
static inline
mjtNum mju_dotSparse_avx(const mjtNum* vec1, const mjtNum* vec2, int nnz1, const int* ind1) {
  int i = 0;
  mjtNum res = 0;
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4>=0) {
    __m256d sum, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val2 = _mm256_set_pd(vec2[ind1[3]],
                         vec2[ind1[2]],
                         vec2[ind1[1]],
                         vec2[ind1[0]]);

    val1 = _mm256_loadu_pd(vec1);

    sum = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i<=nnz1_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_set_pd(vec2[ind1[i+3]],
                            vec2[ind1[i+2]],
                            vec2[ind1[i+1]],
                            vec2[ind1[i+0]]);
      prod = _mm256_mul_pd(val1, val2);
      sum = _mm256_add_pd(sum, prod);
      i += 4;
    }

    // reduce
    vlow = _mm256_castpd256_pd128(sum);
    vhigh = _mm256_extractf128_pd(sum, 1);
    vlow = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    res = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }

  // scalar part
  for (; i < nnz1; i++) {
    res += vec1[i] * vec2[ind1[i]];
  }

  return res;
}



// dot-productX3, first vector is sparse; supernode of size 3
static inline
void mju_dotSparseX3_avx(mjtNum* res0, mjtNum* res1, mjtNum* res2, const mjtNum* vec10,
                         const mjtNum* vec11, const mjtNum* vec12, const mjtNum* vec2,
                         int nnz1, const int* ind1) {
  int i = 0;

  // clear result
  mjtNum RES0 = 0;
  mjtNum RES1 = 0;
  mjtNum RES2 = 0;
  int nnz1_4 = nnz1 - 4;

  // vector part
  if (nnz1_4>=0) {
    __m256d sum0, sum1, sum2, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val2 = _mm256_set_pd(vec2[ind1[3]],
                         vec2[ind1[2]],
                         vec2[ind1[1]],
                         vec2[ind1[0]]);
    val1 = _mm256_loadu_pd(vec10);
    sum0 = _mm256_mul_pd(val1, val2);
    val1 = _mm256_loadu_pd(vec11);
    sum1 = _mm256_mul_pd(val1, val2);
    val1 = _mm256_loadu_pd(vec12);
    sum2 = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i<=nnz1_4) {
      // get val2 only once
      val2 = _mm256_set_pd(vec2[ind1[i+3]],
                           vec2[ind1[i+2]],
                           vec2[ind1[i+1]],
                           vec2[ind1[i+0]]);

      // process each val1
      val1 = _mm256_loadu_pd(vec10+i);
      prod = _mm256_mul_pd(val1, val2);
      sum0 = _mm256_add_pd(sum0, prod);

      val1 = _mm256_loadu_pd(vec11+i);
      prod = _mm256_mul_pd(val1, val2);
      sum1 = _mm256_add_pd(sum1, prod);

      val1 = _mm256_loadu_pd(vec12+i);
      prod = _mm256_mul_pd(val1, val2);
      sum2 = _mm256_add_pd(sum2, prod);

      i += 4;
    }

    // reduce
    vlow   = _mm256_castpd256_pd128(sum0);
    vhigh  = _mm256_extractf128_pd(sum0, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES0   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));

    vlow   = _mm256_castpd256_pd128(sum1);
    vhigh  = _mm256_extractf128_pd(sum1, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES1   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));

    vlow   = _mm256_castpd256_pd128(sum2);
    vhigh  = _mm256_extractf128_pd(sum2, 1);
    vlow   = _mm_add_pd(vlow, vhigh);
    high64 = _mm_unpackhi_pd(vlow, vlow);
    RES2   = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }

  // scalar part
  for (; i<nnz1; i++) {
    mjtNum v2 = vec2[ind1[i]];

    RES0 += vec10[i] * v2;
    RES1 += vec11[i] * v2;
    RES2 += vec12[i] * v2;
  }

  // copy result
  *res0 = RES0;
  *res1 = RES1;
  *res2 = RES2;
}

// multiply sparse matrix and dense vector:  res = mat * vec.
static inline
void mju_mulMatVecSparse_avx(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                             int nr, const int* rownnz, const int* rowadr,
                             const int* colind, const int* rowsuper) {
  if (!rowsuper) {
    // regular sparse dot-product
    for (int r=0; r<nr; r++) {
      res[r] = mju_dotSparse_avx(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }

    return;
  }

  // regular or supernode
  for (int r=0; r<nr; r++) {
    if (rowsuper[r]) {
      int rs = rowsuper[r]+1;

      // handle rows in blocks of 3
      while (rs>=3) {
        mju_dotSparseX3_avx(res+r, res+r+1, res+r+2,
                            mat+rowadr[r], mat+rowadr[r+1], mat+rowadr[r+2],
                            vec, rownnz[r], colind+rowadr[r]);

        r += 3;
        rs -= 3;
      }

      // handle remaining rows
      while (rs>0) {
        res[r] = mju_dotSparse_avx(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);

        r++;
        rs--;
      }

      // go back one, because of outer for loop
      r--;
    }

    else {
      res[r] = mju_dotSparse_avx(mat+rowadr[r], vec, rownnz[r], colind+rowadr[r]);
    }
  }
}

// res = res*scl1 + vec*scl2
static inline
void mju_addToSclScl_avx(mjtNum* res, const mjtNum* vec, mjtNum scl1, mjtNum scl2, int n) {
  int i = 0;

  int n_4 = n - 4;

  // vector part
  if (n_4>=0) {
    __m256d sclpar1, sclpar2, sum, val1, val2;

    // init
    sclpar1 = _mm256_set1_pd(scl1);
    sclpar2 = _mm256_set1_pd(scl2);

    // parallel computation
    while (i<=n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      val1 = _mm256_mul_pd(val1, sclpar1);
      val2 = _mm256_mul_pd(val2, sclpar2);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i==3) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
    res[i+1] = res[i+1]*scl1 + vec[i+1]*scl2;
    res[i+2] = res[i+2]*scl1 + vec[i+2]*scl2;
  } else if (n_i==2) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
    res[i+1] = res[i+1]*scl1 + vec[i+1]*scl2;
  } else if (n_i==1) {
    res[i] = res[i]*scl1 + vec[i]*scl2;
  }
}

// return 1 if vec1==vec2, 0 otherwise
static inline
int mju_compare_avx(const int* vec1, const int* vec2, int n) {
  int i = 0;
  int n_4 = n - 4;

  // vector part
  if (n_4>=0) {
    __m128i val1, val2, cmp;

    // parallel computation
    while (i<=n_4) {
      val1 = _mm_loadu_si128((const __m128i*)(vec1+i));
      val2 = _mm_loadu_si128((const __m128i*)(vec2+i));
      cmp = _mm_cmpeq_epi32(val1, val2);
      if (_mm_movemask_epi8(cmp)!= 0xFFFF) {
        return 0;
      }
      i += 4;
    }
  }

  // scalar part
  return !memcmp(vec1+i, vec2+i, (n-i)*sizeof(int));
}

#endif  // defined(__AVX__) && !defined(mjUSESINGLE)

#endif  // mjUSEPLATFORMSIMD

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_SPARSE_AVX_H_
