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

#include "engine/engine_util_blas.h"

#include <string.h>

#include <mujoco/mjtnum.h>

#ifdef mjUSEPLATFORMSIMD
  #if defined(__AVX__) && !defined(mjUSESINGLE)
    #define mjUSEAVX
    #include "immintrin.h"
  #endif
#endif



//------------------------------ 3D vector and matrix-vector operations ----------------------------

// res = 0
void mju_zero3(mjtNum res[3]) {
  res[0] = 0;
  res[1] = 0;
  res[2] = 0;
}



// vec1 == vec2
int mju_equal3(const mjtNum vec1[3], const mjtNum vec2[3]) {
  return mju_abs(vec1[0] - vec2[0]) < mjMINVAL &&
         mju_abs(vec1[1] - vec2[1]) < mjMINVAL &&
         mju_abs(vec1[2] - vec2[2]) < mjMINVAL;
}



// res = vec
void mju_copy3(mjtNum res[3], const mjtNum data[3]) {
  res[0] = data[0];
  res[1] = data[1];
  res[2] = data[2];
}



// res = vec*scl
void mju_scl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl) {
  res[0] = vec[0] * scl;
  res[1] = vec[1] * scl;
  res[2] = vec[2] * scl;
}



// res = vec1 + vec2
void mju_add3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]) {
  res[0] = vec1[0] + vec2[0];
  res[1] = vec1[1] + vec2[1];
  res[2] = vec1[2] + vec2[2];
}



// res = vec1 - vec2
void mju_sub3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]) {
  res[0] = vec1[0] - vec2[0];
  res[1] = vec1[1] - vec2[1];
  res[2] = vec1[2] - vec2[2];
}



// res += vec
void mju_addTo3(mjtNum res[3], const mjtNum vec[3]) {
  res[0] += vec[0];
  res[1] += vec[1];
  res[2] += vec[2];
}



// res -= vec
void mju_subFrom3(mjtNum res[3], const mjtNum vec[3]) {
  res[0] -= vec[0];
  res[1] -= vec[1];
  res[2] -= vec[2];
}



// res += vec*scl
void mju_addToScl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl) {
  res[0] += vec[0] * scl;
  res[1] += vec[1] * scl;
  res[2] += vec[2] * scl;
}



// res = vec1 + vec2*scl
void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl) {
  res[0] = vec1[0] + scl*vec2[0];
  res[1] = vec1[1] + scl*vec2[1];
  res[2] = vec1[2] + scl*vec2[2];
}



// normalize vector, return length before normalization
mjtNum mju_normalize3(mjtNum vec[3]) {
  mjtNum norm = mju_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);

  if (norm < mjMINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
  } else {
    mjtNum normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
  }

  return norm;
}



// compute vector length (without normalizing)
mjtNum mju_norm3(const mjtNum vec[3]) {
  return mju_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
}



// vector dot-product
mjtNum mju_dot3(const mjtNum vec1[3], const mjtNum vec2[3]) {
  return vec1[0]*vec2[0] + vec1[1]*vec2[1] + vec1[2]*vec2[2];
}



// Cartesian distance between 3D vectors
mjtNum mju_dist3(const mjtNum pos1[3], const mjtNum pos2[3]) {
  mjtNum dif[3] = {pos1[0]-pos2[0], pos1[1]-pos2[1], pos1[2]-pos2[2]};
  return mju_sqrt(dif[0]*dif[0] + dif[1]*dif[1] + dif[2]*dif[2]);
}



// multiply 3-by-3 matrix by vector
void mju_mulMatVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]) {
  mjtNum tmp[3] = {
    mat[0]*vec[0] + mat[1]*vec[1] + mat[2]*vec[2],
    mat[3]*vec[0] + mat[4]*vec[1] + mat[5]*vec[2],
    mat[6]*vec[0] + mat[7]*vec[1] + mat[8]*vec[2]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}



// multiply transposed 3-by-3 matrix by vector
void mju_mulMatTVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]) {
  mjtNum tmp[3] = {
    mat[0]*vec[0] + mat[3]*vec[1] + mat[6]*vec[2],
    mat[1]*vec[0] + mat[4]*vec[1] + mat[7]*vec[2],
    mat[2]*vec[0] + mat[5]*vec[1] + mat[8]*vec[2]
  };
  res[0] = tmp[0];
  res[1] = tmp[1];
  res[2] = tmp[2];
}



// multiply 3x3 matrices,
void mju_mulMatMat3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[1]*mat2[3] + mat1[2]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[1]*mat2[4] + mat1[2]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[1]*mat2[5] + mat1[2]*mat2[8];
  res[3] = mat1[3]*mat2[0] + mat1[4]*mat2[3] + mat1[5]*mat2[6];
  res[4] = mat1[3]*mat2[1] + mat1[4]*mat2[4] + mat1[5]*mat2[7];
  res[5] = mat1[3]*mat2[2] + mat1[4]*mat2[5] + mat1[5]*mat2[8];
  res[6] = mat1[6]*mat2[0] + mat1[7]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[6]*mat2[1] + mat1[7]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[6]*mat2[2] + mat1[7]*mat2[5] + mat1[8]*mat2[8];
}



// multiply 3x3 matrices, first argument transposed
void mju_mulMatTMat3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[3]*mat2[3] + mat1[6]*mat2[6];
  res[1] = mat1[0]*mat2[1] + mat1[3]*mat2[4] + mat1[6]*mat2[7];
  res[2] = mat1[0]*mat2[2] + mat1[3]*mat2[5] + mat1[6]*mat2[8];
  res[3] = mat1[1]*mat2[0] + mat1[4]*mat2[3] + mat1[7]*mat2[6];
  res[4] = mat1[1]*mat2[1] + mat1[4]*mat2[4] + mat1[7]*mat2[7];
  res[5] = mat1[1]*mat2[2] + mat1[4]*mat2[5] + mat1[7]*mat2[8];
  res[6] = mat1[2]*mat2[0] + mat1[5]*mat2[3] + mat1[8]*mat2[6];
  res[7] = mat1[2]*mat2[1] + mat1[5]*mat2[4] + mat1[8]*mat2[7];
  res[8] = mat1[2]*mat2[2] + mat1[5]*mat2[5] + mat1[8]*mat2[8];
}



// multiply 3x3 matrices, second argument transposed
void mju_mulMatMatT3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]) {
  res[0] = mat1[0]*mat2[0] + mat1[1]*mat2[1] + mat1[2]*mat2[2];
  res[1] = mat1[0]*mat2[3] + mat1[1]*mat2[4] + mat1[2]*mat2[5];
  res[2] = mat1[0]*mat2[6] + mat1[1]*mat2[7] + mat1[2]*mat2[8];
  res[3] = mat1[3]*mat2[0] + mat1[4]*mat2[1] + mat1[5]*mat2[2];
  res[4] = mat1[3]*mat2[3] + mat1[4]*mat2[4] + mat1[5]*mat2[5];
  res[5] = mat1[3]*mat2[6] + mat1[4]*mat2[7] + mat1[5]*mat2[8];
  res[6] = mat1[6]*mat2[0] + mat1[7]*mat2[1] + mat1[8]*mat2[2];
  res[7] = mat1[6]*mat2[3] + mat1[7]*mat2[4] + mat1[8]*mat2[5];
  res[8] = mat1[6]*mat2[6] + mat1[7]*mat2[7] + mat1[8]*mat2[8];
}



//------------------------------ 4D vector and matrix-vector operations ----------------------------

// res = 0
void mju_zero4(mjtNum res[4]) {
  res[0] = 0;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;
}



// res = (1,0,0,0)
void mju_unit4(mjtNum res[4]) {
  res[0] = 1;
  res[1] = 0;
  res[2] = 0;
  res[3] = 0;
}


// res = vec
void mju_copy4(mjtNum res[4], const mjtNum data[4]) {
  res[0] = data[0];
  res[1] = data[1];
  res[2] = data[2];
  res[3] = data[3];
}



// normalize vector, return length before normalization
mjtNum mju_normalize4(mjtNum vec[4]) {
  mjtNum norm = mju_sqrt(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2] + vec[3]*vec[3]);

  if (norm < mjMINVAL) {
    vec[0] = 1;
    vec[1] = 0;
    vec[2] = 0;
    vec[3] = 0;
  } else if (mju_abs(norm - 1) > mjMINVAL) {
    mjtNum normInv = 1/norm;
    vec[0] *= normInv;
    vec[1] *= normInv;
    vec[2] *= normInv;
    vec[3] *= normInv;
  }

  return norm;
}



//------------------------------ vector operations -------------------------------------------------

// res = 0
void mju_zero(mjtNum* res, int n) {
  memset(res, 0, n*sizeof(mjtNum));
}



// res = val
void mju_fill(mjtNum* res, mjtNum val, int n) {
  for (int i=0; i < n; i++) {
    res[i] = val;
  }
}



// res = vec
void mju_copy(mjtNum* res, const mjtNum* vec, int n) {
  memcpy(res, vec, n*sizeof(mjtNum));
}



// sum(vec)
mjtNum mju_sum(const mjtNum* vec, int n) {
  mjtNum res = 0;

  for (int i=0; i < n; i++) {
    res += vec[i];
  }

  return res;
}



// sum(abs(vec))
mjtNum mju_L1(const mjtNum* vec, int n) {
  mjtNum res = 0;

  for (int i=0; i < n; i++) {
    res += mju_abs(vec[i]);
  }

  return res;
}



// res = vec*scl
void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, val1, val1scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec+i);
      val1scl = _mm256_mul_pd(val1, sclpar);
      _mm256_storeu_pd(res+i, val1scl);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec[i]*scl;
    res[i+1] = vec[i+1]*scl;
    res[i+2] = vec[i+2]*scl;
  } else if (n_i == 2) {
    res[i] = vec[i]*scl;
    res[i+1] = vec[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] = vec[i]*scl;
  }
#endif
}



// res = vec1 + vec2
void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sum, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] + vec2[i];
    res[i+1] = vec1[i+1] + vec2[i+1];
    res[i+2] = vec1[i+2] + vec2[i+2];
  } else if (n_i == 2) {
    res[i] = vec1[i] + vec2[i];
    res[i+1] = vec1[i+1] + vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i];
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] + vec2[i];
  }
#endif
}



// res = vec1 - vec2
void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d dif, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      dif = _mm256_sub_pd(val1, val2);
      _mm256_storeu_pd(res+i, dif);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] - vec2[i];
    res[i+1] = vec1[i+1] - vec2[i+1];
    res[i+2] = vec1[i+2] - vec2[i+2];
  } else if (n_i == 2) {
    res[i] = vec1[i] - vec2[i];
    res[i+1] = vec1[i+1] - vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] - vec2[i];
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] - vec2[i];
  }
#endif
}



// res += vec
void mju_addTo(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sum, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      sum = _mm256_add_pd(val1, val2);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] += vec[i];
    res[i+1] += vec[i+1];
    res[i+2] += vec[i+2];
  } else if (n_i == 2) {
    res[i] += vec[i];
    res[i+1] += vec[i+1];
  } else if (n_i == 1) {
    res[i] += vec[i];
  }

#else
  for (; i < n; i++) {
    res[i] += vec[i];
  }
#endif
}



// res -= vec
void mju_subFrom(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d dif, val1, val2;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      dif = _mm256_sub_pd(val1, val2);
      _mm256_storeu_pd(res+i, dif);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] -= vec[i];
    res[i+1] -= vec[i+1];
    res[i+2] -= vec[i+2];
  } else if (n_i == 2) {
    res[i] -= vec[i];
    res[i+1] -= vec[i+1];
  } else if (n_i == 1) {
    res[i] -= vec[i];
  }

#else
  for (; i < n; i++) {
    res[i] -= vec[i];
  }
#endif
}



// res += vec*scl
void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  int i = 0;

#ifdef mjUSEAVX
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, sum, val1, val2, val2scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(res+i);
      val2 = _mm256_loadu_pd(vec+i);
      val2scl = _mm256_mul_pd(val2, sclpar);
      sum = _mm256_add_pd(val1, val2scl);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] += vec[i]*scl;
    res[i+1] += vec[i+1]*scl;
    res[i+2] += vec[i+2]*scl;
  } else if (n_i == 2) {
    res[i] += vec[i]*scl;
    res[i+1] += vec[i+1]*scl;
  } else if (n_i == 1) {
    res[i] += vec[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] += vec[i]*scl;
  }
#endif
}

// res = vec1 + vec2*scl
void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n) {
  int i = 0;

#if defined(__AVX__) && defined(mjUSEAVX)  && !defined(mjUSESINGLE)
  int n_4 = n - 4;

  // vector part
  if (n_4 >= 0) {
    __m256d sclpar, sum, val1, val2, val2scl;

    // init
    sclpar = _mm256_set1_pd(scl);

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
      val2scl = _mm256_mul_pd(val2, sclpar);
      sum = _mm256_add_pd(val1, val2scl);
      _mm256_storeu_pd(res+i, sum);
      i += 4;
    }
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res[i] = vec1[i] + vec2[i]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
    res[i+2] = vec1[i+2] + vec2[i+2]*scl;
  } else if (n_i == 2) {
    res[i] = vec1[i] + vec2[i]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i]*scl;
  }

#else
  for (; i < n; i++) {
    res[i] = vec1[i] + vec2[i]*scl;
  }
#endif
}



// normalize vector, return length before normalization
mjtNum mju_normalize(mjtNum* res, int n) {
  mjtNum norm = (mjtNum)mju_sqrt(mju_dot(res, res, n));
  mjtNum normInv;

  if (norm < mjMINVAL) {
    res[0] = 1;
    for (int i=1; i < n; i++) {
      res[i] = 0;
    }
  } else {
    normInv = 1/norm;
    for (int i=0; i < n; i++) {
      res[i] *= normInv;
    }
  }

  return norm;
}



// compute vector length (without normalizing)
mjtNum mju_norm(const mjtNum* res, int n) {
  return mju_sqrt(mju_dot(res, res, n));
}



// vector dot-product
mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, int n) {
  mjtNum res = 0;
  int i = 0;
  int n_4 = n - 4;
#ifdef mjUSEAVX

  // vector part
  if (n_4 >= 0) {
    __m256d sum, prod, val1, val2;
    __m128d vlow, vhigh, high64;

    // init
    val1 = _mm256_loadu_pd(vec1);
    val2 = _mm256_loadu_pd(vec2);
    sum = _mm256_mul_pd(val1, val2);
    i = 4;

    // parallel computation
    while (i <= n_4) {
      val1 = _mm256_loadu_pd(vec1+i);
      val2 = _mm256_loadu_pd(vec2+i);
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

#else
  // do the same order of additions as the AVX intrinsics implementation.
  // this is faster than the simple for loop you'd expect for a dot product,
  // and produces exactly the same results.
  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += vec1[i] * vec2[i];
    res1 += vec1[i+1] * vec2[i+1];
    res2 += vec1[i+2] * vec2[i+2];
    res3 += vec1[i+3] * vec2[i+3];
  }
  res = (res0 + res2) + (res1 + res3);
#endif

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res += vec1[i]*vec2[i] + vec1[i+1]*vec2[i+1] + vec1[i+2]*vec2[i+2];
  } else if (n_i == 2) {
    res += vec1[i]*vec2[i] + vec1[i+1]*vec2[i+1];
  } else if (n_i == 1) {
    res += vec1[i]*vec2[i];
  }
  return res;
}

//------------------------------ matrix-vector operations ------------------------------------------

// multiply matrix and vector
void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc) {
  for (int r=0; r < nr; r++) {
    res[r] = mju_dot(mat + r*nc, vec, nc);
  }
}



// multiply transposed matrix and vector
void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc) {
  mjtNum tmp;
  mju_zero(res, nc);

  for (int r=0; r < nr; r++) {
    if ((tmp = vec[r])) {
      mju_addToScl(res, mat+r*nc, tmp, nc);
    }
  }
}



// multiply square matrix with vectors on both sides: return vec1'*mat*vec2
mjtNum mju_mulVecMatVec(const mjtNum* vec1, const mjtNum* mat, const mjtNum* vec2, int n) {
  mjtNum res = 0;
  for (int i=0; i < n; i++) {
    res += vec1[i] * mju_dot(mat + i*n, vec2, n);
  }
  return res;
}



//------------------------------ matrix operations -------------------------------------------------

// transpose matrix
void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc) {
  for (int i=0; i < nr; i++) {
    for (int j=0; j < nc; j++) {
      res[j*nr+i] = mat[i*nc+j];
    }
  }
}



// symmetrize square matrix res = (mat + mat')/2
void mju_symmetrize(mjtNum* res, const mjtNum* mat, int n) {
  for (int i=0; i < n; i++) {
    res[i*(n+1)] = mat[i*(n+1)];
    for (int j=0; j < i; j++) {
      res[i*n+j] = res[j*n+i] = 0.5 * (mat[i*n+j] + mat[j*n+i]);
    }
  }
}



// identity matrix
void mju_eye(mjtNum* mat, int n) {
  mju_zero(mat, n*n);
  for (int i=0; i < n; i++) {
    mat[i*(n + 1)] = 1;
  }
}



//------------------------------ matrix-matrix operations ------------------------------------------

// multiply matrices, exploit sparsity of mat1
void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                   int r1, int c1, int c2) {
  mjtNum tmp;

  mju_zero(res, r1*c2);

  for (int i=0; i < r1; i++) {
    for (int k=0; k < c1; k++) {
      if ((tmp = mat1[i*c1+k])) {
        mju_addToScl(res+i*c2, mat2+k*c2, tmp, c2);
      }
    }
  }
}



// multiply matrices, second argument transposed
void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                    int r1, int c1, int r2) {
  for (int i=0; i < r1; i++) {
    for (int j=0; j < r2; j++) {
      res[i*r2+j] = mju_dot(mat1+i*c1, mat2+j*c1, c1);
    }
  }
}


// compute M'*diag*M (diag=NULL: compute M'*M), upper triangle optional
void mju_sqrMatTD_impl(mjtNum* res, const mjtNum* mat, const mjtNum* diag,
                       int nr, int nc, int flg_upper) {
  mjtNum tmp;

  // half of MatMat routine: only lower triangle
  mju_zero(res, nc*nc);
  if (diag) {
    for (int j=0; j < nr; j++) {
      if (diag[j]) {
        for (int i=0; i < nc; i++) {
          if ((tmp = mat[j*nc+i])) {
            mju_addToScl(res+i*nc, mat+j*nc, tmp*diag[j], i+1);
          }
        }
      }
    }
  } else {
    for (int i=0; i < nc; i++) {
      for (int j=0; j < nr; j++) {
        if ((tmp = mat[j*nc+i])) {
          mju_addToScl(res+i*nc, mat+j*nc, tmp, i+1);
        }
      }
    }
  }

  // flg_upper is set: make symmetric
  if (flg_upper) {
    for (int i=0; i < nc; i++) {
      for (int j=i+1; j < nc; j++) {
        res[i*nc+j] = res[j*nc+i];
      }
    }
  }
}


// compute M'*diag*M (diag=NULL: compute M'*M)
void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc) {
  mju_sqrMatTD_impl(res, mat, diag, nr, nc, /*flg_upper=*/ 1);
}



// multiply matrices, first argument transposed
void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                    int r1, int c1, int c2) {
  mjtNum tmp;

  mju_zero(res, c1*c2);

  for (int i=0; i < r1; i++) {
    for (int j=0; j < c1; j++) {
      if ((tmp = mat1[i*c1+j])) {
        mju_addToScl(res+j*c2, mat2+i*c2, tmp, c2);
      }
    }
  }
}
