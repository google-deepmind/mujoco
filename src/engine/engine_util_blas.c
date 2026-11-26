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

#include <stdint.h>
#include <string.h>

#include <mujoco/mjtype.h>
#include "engine/engine_simd.h"



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
void mju_copy3(mjtNum res[3], const mjtNum vec[3]) {
  res[0] = vec[0];
  res[1] = vec[1];
  res[2] = vec[2];
}

// res = mat
void mju_copy9(mjtNum res[9], const mjtNum mat[9]) {
  res[0] = mat[0];  res[1] = mat[1];  res[2] = mat[2];
  res[3] = mat[3];  res[4] = mat[4];  res[5] = mat[5];
  res[6] = mat[6];  res[7] = mat[7];  res[8] = mat[8];
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
  v4_store(res, v4_load(data));
}


// normalize vector, return length before normalization
mjtNum mju_normalize4(mjtNum vec[4]) {
  mjtNum v0 = vec[0];
  mjtNum v1 = vec[1];
  mjtNum v2 = vec[2];
  mjtNum v3 = vec[3];
  mjtNum norm = mju_sqrt((v0*v0 + v1*v1) + (v2*v2 + v3*v3));

  if (norm < mjMINVAL) {
    v4_store(vec, v4_set(1, 0, 0, 0));
  } else if (mju_abs(norm - 1) > mjMINVAL) {
    v4_store(vec, v4_mul(v4_set(v0, v1, v2, v3), v4_splat(1 / norm)));
  }

  return norm;
}


//------------------------------ vector operations -------------------------------------------------

// res = 0
void mju_zero(mjtNum* res, int n) {
  memset(res, 0, n*sizeof(mjtNum));
}


// res = 0, at given indices
void mju_zeroInd(mjtNum* res, int n, const int* ind) {
  for (int i = 0; i < n; i++) {
    res[ind[i]] = 0;
  }
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


// res = vec, at given indices
void mju_copyInd(mjtNum* res, const mjtNum* vec, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    res[ind[i]] = vec[ind[i]];
  }
}


// sum(vec)
mjtNum mju_sum(const mjtNum* vec, int n) {
  int i = 0;
  int n_4 = n - 4;
  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += vec[i+0];
    res1 += vec[i+1];
    res2 += vec[i+2];
    res3 += vec[i+3];
  }
  mjtNum res = (res0 + res2) + (res1 + res3);

  // process remaining
  for (; i < n; i++) {
    res += vec[i];
  }

  return res;
}


// sum(abs(vec))
mjtNum mju_L1(const mjtNum* vec, int n) {
  int i = 0;
  int n_4 = n - 4;
  mjtNum res0 = 0;
  mjtNum res1 = 0;
  mjtNum res2 = 0;
  mjtNum res3 = 0;

  for (; i <= n_4; i+=4) {
    res0 += mju_abs(vec[i+0]);
    res1 += mju_abs(vec[i+1]);
    res2 += mju_abs(vec[i+2]);
    res3 += mju_abs(vec[i+3]);
  }
  mjtNum res = (res0 + res2) + (res1 + res3);

  // process remaining
  for (; i < n; i++) {
    res += mju_abs(vec[i]);
  }

  return res;
}


// res = vec*scl
void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  int i = 0;
  if (n >= 4) {
    mjtNum4 s = v4_splat(scl);
    for (; i <= n - 4; i += 4) {
      v4_store(res + i, v4_mul(v4_load(vec + i), s));
    }
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] = vec[i+0]*scl;
    res[i+1] = vec[i+1]*scl;
    res[i+2] = vec[i+2]*scl;
  } else if (n_i == 2) {
    res[i+0] = vec[i+0]*scl;
    res[i+1] = vec[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec[i]*scl;
  }
}


// res = vec1 + vec2
void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;
  for (; i <= n - 4; i += 4) {
    v4_store(res + i, v4_add(v4_load(vec1 + i), v4_load(vec2 + i)));
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] = vec1[i+0] + vec2[i+0];
    res[i+1] = vec1[i+1] + vec2[i+1];
    res[i+2] = vec1[i+2] + vec2[i+2];
  } else if (n_i == 2) {
    res[i+0] = vec1[i+0] + vec2[i+0];
    res[i+1] = vec1[i+1] + vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i];
  }
}


// res = vec1 + vec2, at selected indices
void mju_addInd(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] = vec1[j] + vec2[j];
  }
}


// res = vec1 - vec2
void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  int i = 0;
  for (; i <= n - 4; i += 4) {
    v4_store(res + i, v4_sub(v4_load(vec1 + i), v4_load(vec2 + i)));
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] = vec1[i+0] - vec2[i+0];
    res[i+1] = vec1[i+1] - vec2[i+1];
    res[i+2] = vec1[i+2] - vec2[i+2];
  } else if (n_i == 2) {
    res[i+0] = vec1[i+0] - vec2[i+0];
    res[i+1] = vec1[i+1] - vec2[i+1];
  } else if (n_i == 1) {
    res[i] = vec1[i] - vec2[i];
  }
}


// res = vec1 - vec2, at selected indices
void mju_subInd(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] = vec1[j] - vec2[j];
  }
}


// res += vec
void mju_addTo(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;
  for (; i <= n - 4; i += 4) {
    v4_store(res + i, v4_add(v4_load(res + i), v4_load(vec + i)));
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] += vec[i+0];
    res[i+1] += vec[i+1];
    res[i+2] += vec[i+2];
  } else if (n_i == 2) {
    res[i+0] += vec[i+0];
    res[i+1] += vec[i+1];
  } else if (n_i == 1) {
    res[i] += vec[i];
  }
}


// res += vec, at selected indices
void mju_addToInd(mjtNum* res, const mjtNum* vec, const int* ind, int n) {
  for (int i = 0; i < n; i++) {
    int j = ind[i];
    res[j] += vec[j];
  }
}


// res -= vec
void mju_subFrom(mjtNum* res, const mjtNum* vec, int n) {
  int i = 0;
  for (; i <= n - 4; i += 4) {
    v4_store(res + i, v4_sub(v4_load(res + i), v4_load(vec + i)));
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] -= vec[i+0];
    res[i+1] -= vec[i+1];
    res[i+2] -= vec[i+2];
  } else if (n_i == 2) {
    res[i+0] -= vec[i+0];
    res[i+1] -= vec[i+1];
  } else if (n_i == 1) {
    res[i] -= vec[i];
  }
}


// res += vec*scl
void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  mji_addToScl(res, vec, scl, n);
}



// res += vec*scl, at given indices
void mju_addToSclInd(mjtNum* res, const mjtNum* vec, const int* ind, mjtNum scl, int n) {
  for (int i=0; i < n; i++) {
    int k = ind[i];
    res[k] += vec[k]*scl;
  }
}



// res = vec1 + vec2*scl
void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n) {
  int i = 0;
  if (n >= 4) {
    mjtNum4 s = v4_splat(scl);
    for (; i <= n - 4; i += 4) {
      v4_store(res + i, v4_add(v4_load(vec1 + i), v4_mul(v4_load(vec2 + i), s)));
    }
  }

  int n_i = n - i;
  if (n_i == 3) {
    res[i+0] = vec1[i+0] + vec2[i+0]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
    res[i+2] = vec1[i+2] + vec2[i+2]*scl;
  } else if (n_i == 2) {
    res[i+0] = vec1[i+0] + vec2[i+0]*scl;
    res[i+1] = vec1[i+1] + vec2[i+1]*scl;
  } else if (n_i == 1) {
    res[i] = vec1[i] + vec2[i]*scl;
  }
}


// normalize vector, return length before normalization
mjtNum mju_normalize(mjtNum* res, int n) {
  if (n <= 0) {
    return 0;
  }

  mjtNum norm = mju_sqrt(mju_dot(res, res, n));

  if (norm < mjMINVAL) {
    res[0] = 1;
    mju_zero(res + 1, n - 1);
  } else {
    mjtNum normInv = 1 / norm;
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
  int i = 0;
  int n_4 = n - 4;
  mjtNum res = 0;

  if (n_4 >= 0) {
    mjtNum4 sum = v4_mul(v4_load(vec1), v4_load(vec2));
    for (i = 4; i <= n_4; i += 4) {
      sum = v4_add(sum, v4_mul(v4_load(vec1 + i), v4_load(vec2 + i)));
    }
    res = v4_sum(sum);
  }

  // process remaining
  int n_i = n - i;
  if (n_i == 3) {
    res += vec1[i+0]*vec2[i+0] + vec1[i+1]*vec2[i+1] + vec1[i+2]*vec2[i+2];
  } else if (n_i == 2) {
    res += vec1[i+0]*vec2[i+0] + vec1[i+1]*vec2[i+1];
  } else if (n_i == 1) {
    res += vec1[i]*vec2[i];
  }
  return res;
}


//------------------------------ matrix-vector operations ------------------------------------------



// compute 64-bit nonzero column mask for a dense row (nc <= 64)
static inline uint64_t rowNonzeroMask64(const mjtNum* row, int nc) {
  uint64_t mask = 0;
  int k = 0;
  int nc_4 = nc - 4;
  if (nc_4 >= 0) {
    mjtNum4 zero = v4_splat(0);
    for (; k <= nc_4; k += 4) {
      mjtNum4 v = v4_load(row + k);
      uint64_t m4 = (uint64_t)v4_movemask(v4_not(v4_cmpeq(v, zero)));
      mask |= m4 << k;
    }
  }
  for (; k < nc; k++) {
    if (row[k]) {
      mask |= (uint64_t)1 << k;
    }
  }
  return mask;
}


// multiply matrix and vector
void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc) {
  int r = 0;
  int nc_4 = nc - 4;
  if (nc_4 >= 0) {
    int nr_4 = nr - 4;
    for (; r <= nr_4; r += 4) {
      const mjtNum* m0 = mat + (r+0)*nc;
      const mjtNum* m1 = mat + (r+1)*nc;
      const mjtNum* m2 = mat + (r+2)*nc;
      const mjtNum* m3 = mat + (r+3)*nc;
      mjtNum4 v = v4_load(vec);
      mjtNum4 s0 = v4_mul(v4_load(m0), v);
      mjtNum4 s1 = v4_mul(v4_load(m1), v);
      mjtNum4 s2 = v4_mul(v4_load(m2), v);
      mjtNum4 s3 = v4_mul(v4_load(m3), v);
      int i = 4;
      for (; i <= nc_4; i += 4) {
        v = v4_load(vec + i);
        s0 = v4_add(s0, v4_mul(v4_load(m0 + i), v));
        s1 = v4_add(s1, v4_mul(v4_load(m1 + i), v));
        s2 = v4_add(s2, v4_mul(v4_load(m2 + i), v));
        s3 = v4_add(s3, v4_mul(v4_load(m3 + i), v));
      }
      mjtNum r0 = v4_sum(s0);
      mjtNum r1 = v4_sum(s1);
      mjtNum r2 = v4_sum(s2);
      mjtNum r3 = v4_sum(s3);
      int rem = nc - i;
      if (rem == 3) {
        mjtNum v0 = vec[i+0], v1 = vec[i+1], v2 = vec[i+2];
        r0 += m0[i+0]*v0 + m0[i+1]*v1 + m0[i+2]*v2;
        r1 += m1[i+0]*v0 + m1[i+1]*v1 + m1[i+2]*v2;
        r2 += m2[i+0]*v0 + m2[i+1]*v1 + m2[i+2]*v2;
        r3 += m3[i+0]*v0 + m3[i+1]*v1 + m3[i+2]*v2;
      } else if (rem == 2) {
        mjtNum v0 = vec[i+0], v1 = vec[i+1];
        r0 += m0[i+0]*v0 + m0[i+1]*v1;
        r1 += m1[i+0]*v0 + m1[i+1]*v1;
        r2 += m2[i+0]*v0 + m2[i+1]*v1;
        r3 += m3[i+0]*v0 + m3[i+1]*v1;
      } else if (rem == 1) {
        mjtNum v0 = vec[i];
        r0 += m0[i]*v0;
        r1 += m1[i]*v0;
        r2 += m2[i]*v0;
        r3 += m3[i]*v0;
      }
      res[r+0] = r0;
      res[r+1] = r1;
      res[r+2] = r2;
      res[r+3] = r3;
    }
  }
  for (; r < nr; r++) {
    res[r] = mju_dot(mat + r*nc, vec, nc);
  }
}


// multiply transposed matrix and vector
void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc) {
  if (nc <= 0) {
    return;
  }
  if (nr <= 0) {
    mju_zero(res, nc);
    return;
  }

  // collect active rows in chunks and keep output column strips in vector registers
  const mjtNum* rows[128];
  mjtNum vals[128];
  int first = 1;
  for (int r0=0; r0 < nr; ) {
    int nact = 0;
    for (; r0 < nr && nact < 128; r0++) {
      mjtNum v = vec[r0];
      if (v) {
        rows[nact] = mat + r0*nc;
        vals[nact] = v;
        nact++;
      }
    }
    if (nact == 0) {
      if (first) {
        mju_zero(res, nc);
      }
      break;
    }

    int c = 0;
    for (; c <= nc - 24; c += 24) {
      mjtNum4 a0 = first ? v4_splat(0) : v4_load(res + c + 0);
      mjtNum4 a1 = first ? v4_splat(0) : v4_load(res + c + 4);
      mjtNum4 a2 = first ? v4_splat(0) : v4_load(res + c + 8);
      mjtNum4 a3 = first ? v4_splat(0) : v4_load(res + c + 12);
      mjtNum4 a4 = first ? v4_splat(0) : v4_load(res + c + 16);
      mjtNum4 a5 = first ? v4_splat(0) : v4_load(res + c + 20);
      for (int k=0; k < nact; k++) {
        const mjtNum* row = rows[k] + c;
        mjtNum4 s = v4_splat(vals[k]);
        a0 = v4_add(a0, v4_mul(v4_load(row + 0), s));
        a1 = v4_add(a1, v4_mul(v4_load(row + 4), s));
        a2 = v4_add(a2, v4_mul(v4_load(row + 8), s));
        a3 = v4_add(a3, v4_mul(v4_load(row + 12), s));
        a4 = v4_add(a4, v4_mul(v4_load(row + 16), s));
        a5 = v4_add(a5, v4_mul(v4_load(row + 20), s));
      }
      v4_store(res + c + 0, a0);
      v4_store(res + c + 4, a1);
      v4_store(res + c + 8, a2);
      v4_store(res + c + 12, a3);
      v4_store(res + c + 16, a4);
      v4_store(res + c + 20, a5);
    }
    if (c <= nc - 16) {
      mjtNum4 a0 = first ? v4_splat(0) : v4_load(res + c + 0);
      mjtNum4 a1 = first ? v4_splat(0) : v4_load(res + c + 4);
      mjtNum4 a2 = first ? v4_splat(0) : v4_load(res + c + 8);
      mjtNum4 a3 = first ? v4_splat(0) : v4_load(res + c + 12);
      for (int k=0; k < nact; k++) {
        const mjtNum* row = rows[k] + c;
        mjtNum4 s = v4_splat(vals[k]);
        a0 = v4_add(a0, v4_mul(v4_load(row + 0), s));
        a1 = v4_add(a1, v4_mul(v4_load(row + 4), s));
        a2 = v4_add(a2, v4_mul(v4_load(row + 8), s));
        a3 = v4_add(a3, v4_mul(v4_load(row + 12), s));
      }
      v4_store(res + c + 0, a0);
      v4_store(res + c + 4, a1);
      v4_store(res + c + 8, a2);
      v4_store(res + c + 12, a3);
      c += 16;
    }
    if (c <= nc - 8) {
      mjtNum4 a0 = first ? v4_splat(0) : v4_load(res + c + 0);
      mjtNum4 a1 = first ? v4_splat(0) : v4_load(res + c + 4);
      for (int k=0; k < nact; k++) {
        const mjtNum* row = rows[k] + c;
        mjtNum4 s = v4_splat(vals[k]);
        a0 = v4_add(a0, v4_mul(v4_load(row + 0), s));
        a1 = v4_add(a1, v4_mul(v4_load(row + 4), s));
      }
      v4_store(res + c + 0, a0);
      v4_store(res + c + 4, a1);
      c += 8;
    }
    if (c <= nc - 4) {
      mjtNum4 a0 = first ? v4_splat(0) : v4_load(res + c);
      for (int k=0; k < nact; k++) {
        mjtNum4 s = v4_splat(vals[k]);
        a0 = v4_add(a0, v4_mul(v4_load(rows[k] + c), s));
      }
      v4_store(res + c, a0);
      c += 4;
    }
    int rem = nc - c;
    if (rem == 3) {
      mjtNum s0 = first ? 0 : res[c+0];
      mjtNum s1 = first ? 0 : res[c+1];
      mjtNum s2 = first ? 0 : res[c+2];
      for (int k=0; k < nact; k++) {
        const mjtNum* row = rows[k] + c;
        mjtNum v = vals[k];
        s0 += row[0]*v;
        s1 += row[1]*v;
        s2 += row[2]*v;
      }
      res[c+0] = s0;
      res[c+1] = s1;
      res[c+2] = s2;
    } else if (rem == 2) {
      mjtNum s0 = first ? 0 : res[c+0];
      mjtNum s1 = first ? 0 : res[c+1];
      for (int k=0; k < nact; k++) {
        const mjtNum* row = rows[k] + c;
        mjtNum v = vals[k];
        s0 += row[0]*v;
        s1 += row[1]*v;
      }
      res[c+0] = s0;
      res[c+1] = s1;
    } else if (rem == 1) {
      mjtNum s0 = first ? 0 : res[c];
      for (int k=0; k < nact; k++) {
        s0 += rows[k][c]*vals[k];
      }
      res[c] = s0;
    }
    first = 0;
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


// res[ind, :] = mat[ind, :]
void mju_copyRows(mjtNum* res, const mjtNum* mat, const int* ind, int n, int nc) {
  for (int i = 0; i < n; i++) {
    mju_copy(res + nc*ind[i], mat + nc*ind[i], nc);
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
void mju_sqrMatTD_impl(mjtNum* restrict res, const mjtNum* restrict mat,
                       const mjtNum* restrict diag, int nr, int nc, int flg_upper) {
  // half of MatMat routine: only lower triangle
  mju_zero(res, nc*nc);

  // small/medium nc: collect active rows in chunks and compute lower triangle in 4x8/4x4 tiles
  if (nc > 0 && nc <= 64) {
    const mjtNum* act_row[64];
    mjtNum act_dj[64];
    uint64_t act_mask[64];

    int j = 0;
    while (j < nr) {
      int nact = 0;
      uint64_t touched = 0;
      for (; j < nr && nact < 64; j++) {
        mjtNum dj = diag ? diag[j] : 1;
        if (!dj) {
          continue;
        }
        const mjtNum* mat_j = mat + j*nc;
        uint64_t mask = rowNonzeroMask64(mat_j, nc);
        if (!mask) {
          continue;
        }
        act_row[nact] = mat_j;
        act_dj[nact] = dj;
        act_mask[nact] = mask;
        touched |= mask;
        nact++;
      }

      if (nact == 0) {
        break;
      }

      // process 4-row groups g..g+3 in 4x8 and 4x4 register tiles
      int g = 0;
      for (; g <= nc - 4; g += 4) {
        uint64_t gmask = (uint64_t)0xFU << g;
        if ((touched & gmask) == 0) {
          continue;
        }

        const mjtNum* gr[64];
        mjtNum gs0[64], gs1[64], gs2[64], gs3[64];
        int ng = 0;
        uint64_t col_mask = 0;
        for (int a=0; a < nact; a++) {
          if (act_mask[a] & gmask) {
            const mjtNum* row = act_row[a];
            mjtNum dj = act_dj[a];
            gr[ng] = row;
            gs0[ng] = row[g+0] * dj;
            gs1[ng] = row[g+1] * dj;
            gs2[ng] = row[g+2] * dj;
            gs3[ng] = row[g+3] * dj;
            col_mask |= act_mask[a];
            ng++;
          }
        }

        mjtNum* r0 = res + (g+0)*nc;
        mjtNum* r1 = res + (g+1)*nc;
        mjtNum* r2 = res + (g+2)*nc;
        mjtNum* r3 = res + (g+3)*nc;

        int c = 0;
        while (c <= g) {
          if (((col_mask >> c) & 0xFU) == 0) {
            c += 4;
            continue;
          }
          if (c <= g - 4 && ((col_mask >> (c+4)) & 0xFU)) {
            mjtNum4 a00 = v4_load(r0 + c + 0);
            mjtNum4 a01 = v4_load(r0 + c + 4);
            mjtNum4 a10 = v4_load(r1 + c + 0);
            mjtNum4 a11 = v4_load(r1 + c + 4);
            mjtNum4 a20 = v4_load(r2 + c + 0);
            mjtNum4 a21 = v4_load(r2 + c + 4);
            mjtNum4 a30 = v4_load(r3 + c + 0);
            mjtNum4 a31 = v4_load(r3 + c + 4);
            for (int k=0; k < ng; k++) {
              const mjtNum* rk = gr[k] + c;
              mjtNum4 v0 = v4_load(rk + 0);
              mjtNum4 v1 = v4_load(rk + 4);
              mjtNum4 s0 = v4_splat(gs0[k]);
              mjtNum4 s1 = v4_splat(gs1[k]);
              mjtNum4 s2 = v4_splat(gs2[k]);
              mjtNum4 s3 = v4_splat(gs3[k]);
              a00 = v4_add(a00, v4_mul(v0, s0));
              a01 = v4_add(a01, v4_mul(v1, s0));
              a10 = v4_add(a10, v4_mul(v0, s1));
              a11 = v4_add(a11, v4_mul(v1, s1));
              a20 = v4_add(a20, v4_mul(v0, s2));
              a21 = v4_add(a21, v4_mul(v1, s2));
              a30 = v4_add(a30, v4_mul(v0, s3));
              a31 = v4_add(a31, v4_mul(v1, s3));
            }
            v4_store(r0 + c + 0, a00);
            v4_store(r0 + c + 4, a01);
            v4_store(r1 + c + 0, a10);
            v4_store(r1 + c + 4, a11);
            v4_store(r2 + c + 0, a20);
            v4_store(r2 + c + 4, a21);
            v4_store(r3 + c + 0, a30);
            v4_store(r3 + c + 4, a31);
            c += 8;
          } else {
            mjtNum4 a0 = v4_load(r0 + c);
            mjtNum4 a1 = v4_load(r1 + c);
            mjtNum4 a2 = v4_load(r2 + c);
            mjtNum4 a3 = v4_load(r3 + c);
            for (int k=0; k < ng; k++) {
              mjtNum4 v0 = v4_load(gr[k] + c);
              a0 = v4_add(a0, v4_mul(v0, v4_splat(gs0[k])));
              a1 = v4_add(a1, v4_mul(v0, v4_splat(gs1[k])));
              a2 = v4_add(a2, v4_mul(v0, v4_splat(gs2[k])));
              a3 = v4_add(a3, v4_mul(v0, v4_splat(gs3[k])));
            }
            v4_store(r0 + c, a0);
            v4_store(r1 + c, a1);
            v4_store(r2 + c, a2);
            v4_store(r3 + c, a3);
            c += 4;
          }
        }

        // clear strictly-upper entries written by the 4x4 diagonal block
        if (!flg_upper) {
          r0[g+1] = 0; r0[g+2] = 0; r0[g+3] = 0;
          r1[g+2] = 0; r1[g+3] = 0;
          r2[g+3] = 0;
        }
      }

      // process remaining 0..3 rows (use 8-col strips where possible to avoid latency stalls)
      for (int i=g; i < nc; i++) {
        if (((touched >> i) & 1) == 0) {
          continue;
        }
        const mjtNum* r_i[64];
        mjtNum s_i[64];
        int ni = 0;
        uint64_t col_mask = 0;
        for (int a=0; a < nact; a++) {
          if ((act_mask[a] >> i) & 1) {
            r_i[ni] = act_row[a];
            s_i[ni] = act_row[a][i] * act_dj[a];
            col_mask |= act_mask[a];
            ni++;
          }
        }
        mjtNum* res_i = res + i*nc;
        int len = i + 1;
        int c = 0;
        while (c <= len - 4) {
          if (((col_mask >> c) & 0xFU) == 0) {
            c += 4;
            continue;
          }
          if (c <= len - 8 && ((col_mask >> (c+4)) & 0xFU)) {
            mjtNum4 a0 = v4_load(res_i + c + 0);
            mjtNum4 a1 = v4_load(res_i + c + 4);
            for (int k=0; k < ni; k++) {
              const mjtNum* rk = r_i[k] + c;
              mjtNum4 sv = v4_splat(s_i[k]);
              a0 = v4_add(a0, v4_mul(v4_load(rk + 0), sv));
              a1 = v4_add(a1, v4_mul(v4_load(rk + 4), sv));
            }
            v4_store(res_i + c + 0, a0);
            v4_store(res_i + c + 4, a1);
            c += 8;
          } else {
            mjtNum4 a0 = v4_load(res_i + c);
            for (int k=0; k < ni; k++) {
              mjtNum4 sv = v4_splat(s_i[k]);
              a0 = v4_add(a0, v4_mul(v4_load(r_i[k] + c), sv));
            }
            v4_store(res_i + c, a0);
            c += 4;
          }
        }
        for (; c < len; c++) {
          mjtNum a0 = res_i[c];
          for (int k=0; k < ni; k++) {
            a0 += r_i[k][c] * s_i[k];
          }
          res_i[c] = a0;
        }
      }
    }
  } else {
    for (int j=0; j < nr; j++) {
      mjtNum dj = diag ? diag[j] : 1;
      if (dj) {
        const mjtNum* mat_j = mat + j*nc;
        for (int i=0; i < nc; i++) {
          mjtNum tmp = mat_j[i];
          if (tmp) {
            mju_addToScl(res+i*nc, mat_j, tmp*dj, i+1);
          }
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
