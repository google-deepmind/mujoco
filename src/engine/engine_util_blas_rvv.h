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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_RVV_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_RVV_H_

#include "engine/engine_util_rvv.h"

#ifdef mjUSERVV

#include <riscv_vector.h>
#include <stddef.h>

#include <mujoco/mjtype.h>

// Element-wise operations: vector-length agnostic strip-mining, no scalar tail.
// Multiply and add are kept unfused to match the AVX implementation bit-for-bit.

// res = vec*scl
static inline
void mju_scl_rvv(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t v = __riscv_vle64_v_f64m2(vec, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfmul_vf_f64m2(v, scl, vl), vl);
  }
}


// res = vec1 + vec2
static inline
void mju_add_rvv(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec1 += vl, vec2 += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(vec1, vl);
    vfloat64m2_t b = __riscv_vle64_v_f64m2(vec2, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfadd_vv_f64m2(a, b, vl), vl);
  }
}


// res = vec1 - vec2
static inline
void mju_sub_rvv(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec1 += vl, vec2 += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(vec1, vl);
    vfloat64m2_t b = __riscv_vle64_v_f64m2(vec2, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfsub_vv_f64m2(a, b, vl), vl);
  }
}


// res += vec
static inline
void mju_addTo_rvv(mjtNum* res, const mjtNum* vec, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(res, vl);
    vfloat64m2_t b = __riscv_vle64_v_f64m2(vec, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfadd_vv_f64m2(a, b, vl), vl);
  }
}


// res -= vec
static inline
void mju_subFrom_rvv(mjtNum* res, const mjtNum* vec, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(res, vl);
    vfloat64m2_t b = __riscv_vle64_v_f64m2(vec, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfsub_vv_f64m2(a, b, vl), vl);
  }
}


// res += vec*scl
static inline
void mju_addToScl_rvv(mjtNum* res, const mjtNum* vec, mjtNum scl, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(res, vl);
    vfloat64m2_t b = __riscv_vfmul_vf_f64m2(__riscv_vle64_v_f64m2(vec, vl), scl, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfadd_vv_f64m2(a, b, vl), vl);
  }
}


// res = vec1 + vec2*scl
static inline
void mju_addScl_rvv(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n) {
  for (size_t avl = n > 0 ? (size_t)n : 0, vl; avl > 0; avl -= vl, res += vl, vec1 += vl, vec2 += vl) {
    vl = __riscv_vsetvl_e64m2(avl);
    vfloat64m2_t a = __riscv_vle64_v_f64m2(vec1, vl);
    vfloat64m2_t b = __riscv_vfmul_vf_f64m2(__riscv_vle64_v_f64m2(vec2, vl), scl, vl);
    __riscv_vse64_v_f64m2(res, __riscv_vfadd_vv_f64m2(a, b, vl), vl);
  }
}


// vector dot-product: 4-lane accumulation, same order as AVX and scalar fallback
static inline
mjtNum mju_dot_rvv(const mjtNum* vec1, const mjtNum* vec2, int n) {
  mjtNum res = 0;
  int i = 0;
  int n_4 = n - 4;
  if (n_4 >= 0) {
    vfloat64m2_t sum = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec1, 4),
                                              __riscv_vle64_v_f64m2(vec2, 4), 4);
    i = 4;
    while (i <= n_4) {
      vfloat64m2_t prod = __riscv_vfmul_vv_f64m2(__riscv_vle64_v_f64m2(vec1+i, 4),
                                                 __riscv_vle64_v_f64m2(vec2+i, 4), 4);
      sum = __riscv_vfadd_vv_f64m2(sum, prod, 4);
      i += 4;
    }
    res = mju_reduce4_rvv(sum);
  }

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

#endif  // mjUSERVV

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_BLAS_RVV_H_
