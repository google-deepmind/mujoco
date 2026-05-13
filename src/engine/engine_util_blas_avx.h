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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_AVX_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_AVX_H_

#ifdef mjUSEPLATFORMSIMD
#if defined(__AVX__) && !defined(mjUSESINGLE)

#define mjUSEAVX

#include <immintrin.h>

#include <mujoco/mjtnum.h>

// res = vec*scl
static inline
int mju_scl_avx(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    __m256d sclpar = _mm256_set1_pd(scl);
    while (i <= n_4) {
      _mm256_storeu_pd(res+i, _mm256_mul_pd(_mm256_loadu_pd(vec+i), sclpar));
      i += 4;
    }
  }
  return i;
}

// res = vec1 + vec2
static inline
int mju_add_avx(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    while (i <= n_4) {
      _mm256_storeu_pd(res+i, _mm256_add_pd(_mm256_loadu_pd(vec1+i), _mm256_loadu_pd(vec2+i)));
      i += 4;
    }
  }
  return i;
}

// res = vec1 - vec2
static inline
int mju_sub_avx(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    while (i <= n_4) {
      _mm256_storeu_pd(res+i, _mm256_sub_pd(_mm256_loadu_pd(vec1+i), _mm256_loadu_pd(vec2+i)));
      i += 4;
    }
  }
  return i;
}

// res += vec
static inline
int mju_addTo_avx(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    while (i <= n_4) {
      _mm256_storeu_pd(res+i, _mm256_add_pd(_mm256_loadu_pd(res+i), _mm256_loadu_pd(vec+i)));
      i += 4;
    }
  }
  return i;
}

// res -= vec
static inline
int mju_subFrom_avx(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    while (i <= n_4) {
      _mm256_storeu_pd(res+i, _mm256_sub_pd(_mm256_loadu_pd(res+i), _mm256_loadu_pd(vec+i)));
      i += 4;
    }
  }
  return i;
}

// res += vec*scl
static inline
int mju_addToScl_avx(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    __m256d sclpar = _mm256_set1_pd(scl);
    while (i <= n_4) {
      __m256d val1 = _mm256_loadu_pd(res+i);
      __m256d val2 = _mm256_loadu_pd(vec+i);
      _mm256_storeu_pd(res+i, _mm256_add_pd(val1, _mm256_mul_pd(val2, sclpar)));
      i += 4;
    }
  }
  return i;
}

// res = vec1 + vec2*scl
static inline
int mju_addScl_avx(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n) {
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    __m256d sclpar = _mm256_set1_pd(scl);
    while (i <= n_4) {
      __m256d val1 = _mm256_loadu_pd(vec1+i);
      __m256d val2 = _mm256_loadu_pd(vec2+i);
      _mm256_storeu_pd(res+i, _mm256_add_pd(val1, _mm256_mul_pd(val2, sclpar)));
      i += 4;
    }
  }
  return i;
}

// vector dot-product
static inline
mjtNum mju_dot_avx(const mjtNum* vec1, const mjtNum* vec2, int n, int* processed) {
  mjtNum res = 0;
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    __m256d sum = _mm256_mul_pd(_mm256_loadu_pd(vec1), _mm256_loadu_pd(vec2));
    i = 4;

    while (i <= n_4) {
      sum = _mm256_add_pd(sum, _mm256_mul_pd(_mm256_loadu_pd(vec1+i), _mm256_loadu_pd(vec2+i)));
      i += 4;
    }

    __m128d vlow = _mm256_castpd256_pd128(sum);
    __m128d vhigh = _mm256_extractf128_pd(sum, 1);
    vlow = _mm_add_pd(vlow, vhigh);
    __m128d high64 = _mm_unpackhi_pd(vlow, vlow);
    res = _mm_cvtsd_f64(_mm_add_sd(vlow, high64));
  }
  *processed = i;
  return res;
}

#endif  // defined(__AVX__) && !defined(mjUSESINGLE)
#endif  // mjUSEPLATFORMSIMD

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_AVX_H_
