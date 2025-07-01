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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_H_

#include <math.h>

#include <mujoco/mjexport.h>
#include <mujoco/mjtnum.h>

#ifdef __cplusplus
extern "C" {
#endif

//------------------------------ standard library functions ----------------------------------------

#if !defined(mjUSESINGLE)
  #define mju_sqrt    sqrt
  #define mju_exp     exp
  #define mju_sin     sin
  #define mju_cos     cos
  #define mju_tan     tan
  #define mju_asin    asin
  #define mju_acos    acos
  #define mju_atan2   atan2
  #define mju_tanh    tanh
  #define mju_pow     pow
  #define mju_abs     fabs
  #define mju_log     log
  #define mju_log10   log10
  #define mju_floor   floor
  #define mju_ceil    ceil

#else
  #define mju_sqrt    sqrtf
  #define mju_exp     expf
  #define mju_sin     sinf
  #define mju_cos     cosf
  #define mju_tan     tanf
  #define mju_asin    asinf
  #define mju_acos    acosf
  #define mju_atan2   atan2f
  #define mju_tanh    tanhf
  #define mju_pow     powf
  #define mju_abs     fabsf
  #define mju_log     logf
  #define mju_log10   log10f
  #define mju_floor   floorf
  #define mju_ceil    ceilf
#endif  // !defined(mjUSESINGLE)


//------------------------------ 3D vector and matrix-vector operations ----------------------------

// res = 0
MJAPI void mju_zero3(mjtNum res[3]);

// vec1 == vec2
MJAPI int mju_equal3(const mjtNum vec1[3], const mjtNum vec2[3]);

// res = vec
MJAPI void mju_copy3(mjtNum res[3], const mjtNum data[3]);

// res = vec*scl
MJAPI void mju_scl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

// res = vec1 + vec2
MJAPI void mju_add3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

// res = vec1 - vec2
MJAPI void mju_sub3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

// res += vec
MJAPI void mju_addTo3(mjtNum res[3], const mjtNum vec[3]);

// res -= vec
MJAPI void mju_subFrom3(mjtNum res[3], const mjtNum vec[3]);

// res += vec*scl
MJAPI void mju_addToScl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

// res = vec1 + vec2*scl
MJAPI void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl);

// normalize vector, return length before normalization, set to [1, 0, 0] if norm is tiny
MJAPI mjtNum mju_normalize3(mjtNum vec[3]);

// compute vector length (without normalizing)
MJAPI mjtNum mju_norm3(const mjtNum vec[3]);

// vector dot-product
MJAPI mjtNum mju_dot3(const mjtNum vec1[3], const mjtNum vec2[3]);

// Cartesian distance between 3D vectors
MJAPI mjtNum mju_dist3(const mjtNum pos1[3], const mjtNum pos2[3]);

// multiply 3-by-3 matrix by vector
MJAPI void mju_mulMatVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]);

// multiply transposed 3-by-3 matrix by vector
MJAPI void mju_mulMatTVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]);

// multiply 3x3 matrices
MJAPI void mju_mulMatMat3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]);

// multiply 3x3 matrices, first argument transposed
MJAPI void mju_mulMatTMat3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]);

// multiply 3x3 matrices, second argument transposed
MJAPI void mju_mulMatMatT3(mjtNum res[9], const mjtNum mat1[9], const mjtNum mat2[9]);

//------------------------------ 4D/quaternion operations ------------------------------------------

// res = 0
MJAPI void mju_zero4(mjtNum res[4]);

// res = (1,0,0,0)
MJAPI void mju_unit4(mjtNum res[4]);

// res = vec
MJAPI void mju_copy4(mjtNum res[4], const mjtNum data[4]);

// normalize vector, return length before normalization
MJAPI mjtNum mju_normalize4(mjtNum vec[4]);


//------------------------------ general vector operations -----------------------------------------

// res = 0
MJAPI void mju_zero(mjtNum* res, int n);

// res = val
MJAPI void mju_fill(mjtNum* res, mjtNum val, int n);

// res = vec
MJAPI void mju_copy(mjtNum* res, const mjtNum* vec, int n);

// sum(vec)
MJAPI mjtNum mju_sum(const mjtNum* vec, int n);

// sum(abs(vec))
MJAPI mjtNum mju_L1(const mjtNum* vec, int n);

// res = vec*scl
MJAPI void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

// res = vec1 + vec2
MJAPI void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

// res = vec1 - vec2
MJAPI void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

// res += vec
MJAPI void mju_addTo(mjtNum* res, const mjtNum* vec, int n);

// res -= vec
MJAPI void mju_subFrom(mjtNum* res, const mjtNum* vec, int n);

// res += vec*scl
MJAPI void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

// res = vec1 + vec2*scl
MJAPI void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n);

// normalize vector, return length before normalization
MJAPI mjtNum mju_normalize(mjtNum* res, int n);

// compute vector length (without normalizing)
MJAPI mjtNum mju_norm(const mjtNum* res, int n);

// vector dot-product
MJAPI mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, int n);


//------------------------------ matrix-vector operations ------------------------------------------

// multiply matrix and vector
MJAPI void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                         int nr, int nc);

// multiply transposed matrix and vector
MJAPI void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
                          int nr, int nc);

// multiply square matrix with vectors on both sides: return vec1'*mat*vec2
MJAPI mjtNum mju_mulVecMatVec(const mjtNum* vec1, const mjtNum* mat, const mjtNum* vec2, int n);


//------------------------------ matrix operations -------------------------------------------------

// transpose matrix
MJAPI void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc);

// symmetrize square matrix res = (mat + mat')/2
MJAPI void mju_symmetrize(mjtNum* res, const mjtNum* mat, int n);

// identity matrix
MJAPI void mju_eye(mjtNum* mat, int n);

//------------------------------ matrix-matrix operations ------------------------------------------

// multiply matrices
MJAPI void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                         int r1, int c1, int c2);

// multiply matrices, second argument transposed
MJAPI void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                          int r1, int c1, int r2);

// multiply matrices, first argument transposed
MJAPI void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
                          int r1, int c1, int c2);

// compute M'*diag*M (diag=NULL: compute M'*M), upper triangle optional
void mju_sqrMatTD_impl(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc,
                       int flg_upper);

// compute M'*diag*M (diag=NULL: compute M'*M)
MJAPI void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc);

#ifdef __cplusplus
}
#endif
#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_H_
