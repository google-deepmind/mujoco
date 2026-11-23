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

// Common configuration for the RISC-V Vector (RVV 1.0) code paths.
//
// mjUSERVV is defined when:
//   - platform SIMD is enabled (mjUSEPLATFORMSIMD),
//   - the compiler targets the V extension with the ratified (v0.12+) intrinsics API,
//   - the vector unit supports 64-bit floating point (zve64d or full V),
//   - mjtNum is double (not mjUSESINGLE).
//
// Reductions use a fixed 4-lane accumulator so that the summation order matches the AVX and
// scalar fallback implementations exactly; this keeps results bit-identical across platforms.
// Element-wise operations are vector-length agnostic.

#ifndef MUJOCO_SRC_ENGINE_ENGINE_UTIL_RVV_H_
#define MUJOCO_SRC_ENGINE_ENGINE_UTIL_RVV_H_

#ifdef mjUSEPLATFORMSIMD
#if defined(__riscv_vector) && defined(__riscv_v_intrinsic) && __riscv_v_intrinsic >= 12000 && \
    defined(__riscv_v_elen_fp) && __riscv_v_elen_fp >= 64 && !defined(mjUSESINGLE)

#ifndef mjUSERVV
#define mjUSERVV
#endif

#include <riscv_vector.h>
#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjtype.h>

// Reduce the 4-lane accumulator as ((s0 + s2) + (s1 + s3)), matching AVX and scalar fallbacks.
// LMUL=2 guarantees 4 x e64 lanes for any VLEN >= 128.
static inline
mjtNum mju_reduce4_rvv(vfloat64m2_t sum) {
  vfloat64m2_t hi = __riscv_vslidedown_vx_f64m2(sum, 2, 4);
  sum = __riscv_vfadd_vv_f64m2(sum, hi, 2);
  hi = __riscv_vslidedown_vx_f64m2(sum, 1, 2);
  sum = __riscv_vfadd_vv_f64m2(sum, hi, 1);
  return __riscv_vfmv_f_s_f64m2_f64(sum);
}


// gather 4 elements: vec[ind[0..3]]
// Scalar loads + one unit-stride vector load (as AVX does with _mm256_set_pd). Indexed vector
// loads (vluxei) are much slower than this on current cores: measured on SpacemiT X100, this
// version is 1.5-2.3x faster than vluxei64 for nnz >= 16.
static inline
vfloat64m2_t mju_gather4_rvv(const mjtNum* vec, const int* ind) {
  mjtNum tmp[4] = {vec[ind[0]], vec[ind[1]], vec[ind[2]], vec[ind[3]]};
  return __riscv_vle64_v_f64m2(tmp, 4);
}

#endif  // __riscv_vector && ...
#endif  // mjUSEPLATFORMSIMD

#endif  // MUJOCO_SRC_ENGINE_ENGINE_UTIL_RVV_H_
