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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
#define MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_

#include <math.h>  // IWYU pragma: keep
#include <stdint.h>
#include <string.h>

#include <mujoco/mjtype.h>


//------------------------------ backend and FMA configuration -------------------------------------

// select SIMD backend when mjUSEPLATFORMSIMD is enabled
#if defined(mjUSEPLATFORMSIMD) && \
    !defined(mjSCALAR) && !defined(mjVECTORIZE) && !defined(mjUSEAVX)
  #if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
    #define mjVECTORIZE
  #endif
  #if defined(__AVX__)
    #define mjUSEAVX
  #endif
#endif

#if defined(mjVECTORIZE) || defined(mjUSEAVX)
  #if defined(__AVX__) || defined(mjUSEAVX)
    #include <immintrin.h>  // IWYU pragma: export
  #elif defined(__SSE2__) || defined(_M_X64)
    #include <emmintrin.h>  // IWYU pragma: export
    #include <xmmintrin.h>  // IWYU pragma: export
  #elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64))
    #include <arm_neon.h>   // IWYU pragma: export
  #elif defined(__wasm_simd128__)
    #include <wasm_simd128.h>  // IWYU pragma: export
  #endif
#endif

// FMA is disabled by default until P4; auto-enabled on hardware FMA when mjENABLEFMA is set
#if defined(mjENABLEFMA) && !defined(mjNOFMA) && !defined(mjUSEFMA)
  #if defined(__aarch64__) || defined(__FMA__) || \
      (defined(_MSC_VER) && (defined(_M_ARM64) || defined(__AVX2__)))
    #define mjUSEFMA
  #endif
#endif
#ifdef mjNOFMA
  #undef mjUSEFMA
#endif


// fused multiply-add: a*b + c
static inline
mjtNum mju_fma(mjtNum a, mjtNum b, mjtNum c) {
#ifdef mjUSEFMA
  #ifdef mjUSESINGLE
    return fmaf(a, b, c);
  #else
    return fma(a, b, c);
  #endif
#else
  #ifdef __clang__
    #pragma STDC FP_CONTRACT OFF
  #endif
  mjtNum prod = a * b;
  return prod + c;
#endif
}


//------------------------------ GCC/Clang vector extensions backend -------------------------------

#if defined(mjVECTORIZE)

#if defined(__clang__) && defined(__AVX__)
  #define mjSIMD128_ATTR static inline __attribute__((__min_vector_width__(128)))
#else
  #define mjSIMD128_ATTR static inline
#endif

typedef float mjtFloat4_vec_ __attribute__((vector_size(16)));
typedef uint32_t mjtFloat4_bits_ __attribute__((vector_size(16)));
typedef struct { mjtFloat4_vec_ v; } mjtFloat4;
typedef struct { mjtFloat4_bits_ m; } mjtFloatMask4;

#ifdef mjUSESINGLE
  #define mjSIMD_ATTR mjSIMD128_ATTR
  typedef float mjtNum2_vec_ __attribute__((vector_size(8)));
  typedef mjtFloat4_vec_ mjtNum4_vec_;
  typedef mjtFloat4_bits_ mjtNum4_bits_;
#else
  #if defined(__clang__) && defined(__AVX__)
    #define mjSIMD_ATTR static inline __attribute__((__min_vector_width__(256)))
  #else
    #define mjSIMD_ATTR static inline
  #endif
  typedef double mjtNum2_vec_ __attribute__((vector_size(16)));
  typedef double mjtNum4_f64 __attribute__((vector_size(32)));
  typedef uint64_t mjtNum4_u64 __attribute__((vector_size(32)));
  typedef mjtNum4_f64 mjtNum4_vec_;
  typedef mjtNum4_u64 mjtNum4_bits_;
#endif
typedef struct { mjtNum4_vec_ v; } mjtNum4;
typedef struct { mjtNum4_bits_ m; } mjtMask4;


// wrap raw vector in mjtNum4
mjSIMD_ATTR
mjtNum4 v4_wrap_(mjtNum4_vec_ v) {
  mjtNum4 r;
  r.v = v;
  return r;
}


// wrap raw mask in mjtMask4
mjSIMD_ATTR
mjtMask4 v4_wrap_mask_(mjtNum4_bits_ m) {
  mjtMask4 r;
  r.m = m;
  return r;
}


// load 4 mjtNums from memory
mjSIMD_ATTR
mjtNum4 v4_load(const mjtNum* p) {
  mjtNum4 r;
  memcpy(&r.v, p, sizeof(r.v));
  return r;
}


// store 4 mjtNums to memory
mjSIMD_ATTR
void v4_store(mjtNum* p, mjtNum4 a) {
  memcpy(p, &a.v, sizeof(a.v));
}


// load 3 mjtNums from memory into lanes 0..2 with 0 in lane 3 (reads 3 mjtNums)
mjSIMD_ATTR
mjtNum4 v4_load3(const mjtNum* p) {
  mjtNum2_vec_ xy;
  memcpy(&xy, p, sizeof(xy));
  mjtNum2_vec_ z0 = {p[2], 0};
  return v4_wrap_(__builtin_shufflevector(xy, z0, 0, 1, 2, 3));
}


// store low 3 lanes to memory (writes 3 mjtNums)
mjSIMD_ATTR
void v4_store3(mjtNum* p, mjtNum4 a) {
  mjtNum2_vec_ xy = __builtin_shufflevector(a.v, a.v, 0, 1);
  memcpy(p, &xy, sizeof(xy));
  p[2] = a.v[2];
}


// set 4 lanes (also used for gathers)
mjSIMD_ATTR
mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
  mjtNum4_vec_ v = {a, b, c, d};
  return v4_wrap_(v);
}


// broadcast scalar to all 4 lanes
mjSIMD_ATTR
mjtNum4 v4_splat(mjtNum a) {
  mjtNum4_vec_ v = {a, a, a, a};
  return v4_wrap_(v);
}


// lane-wise addition
mjSIMD_ATTR
mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(a.v + b.v);
}


// lane-wise subtraction
mjSIMD_ATTR
mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(a.v - b.v);
}


// lane-wise multiplication
mjSIMD_ATTR
mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(a.v * b.v);
}


// lane-wise division
mjSIMD_ATTR
mjtNum4 v4_div(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(a.v / b.v);
}


// lane-wise less-than mask (ordered quiet)
mjSIMD_ATTR
mjtMask4 v4_cmplt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_((mjtNum4_bits_)(a.v < b.v));
}


// lane-wise less-or-equal mask (ordered quiet)
mjSIMD_ATTR
mjtMask4 v4_cmple(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_((mjtNum4_bits_)(a.v <= b.v));
}


// lane-wise greater-than mask (ordered quiet)
mjSIMD_ATTR
mjtMask4 v4_cmpgt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_((mjtNum4_bits_)(a.v > b.v));
}


// lane-wise greater-or-equal mask (ordered quiet)
mjSIMD_ATTR
mjtMask4 v4_cmpge(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_((mjtNum4_bits_)(a.v >= b.v));
}


// lane-wise equality mask (ordered quiet)
mjSIMD_ATTR
mjtMask4 v4_cmpeq(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_((mjtNum4_bits_)(a.v == b.v));
}


// bitwise AND of masks
mjSIMD_ATTR
mjtMask4 v4_and(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(a.m & b.m);
}


// bitwise OR of masks
mjSIMD_ATTR
mjtMask4 v4_or(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(a.m | b.m);
}


// bitwise XOR of masks
mjSIMD_ATTR
mjtMask4 v4_xor(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(a.m ^ b.m);
}


// bitwise (~a & b) of masks
mjSIMD_ATTR
mjtMask4 v4_andnot(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(~a.m & b.m);
}


// bitwise NOT of mask
mjSIMD_ATTR
mjtMask4 v4_not(mjtMask4 a) {
  return v4_wrap_mask_(~a.m);
}


// lane-wise blend: m ? tr : fa
mjSIMD_ATTR
mjtNum4 v4_select(mjtMask4 m, mjtNum4 tr, mjtNum4 fa) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm256_blendv_pd((__m256d)fa.v, (__m256d)tr.v, (__m256d)m.m));
#elif defined(__AVX__) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm_blendv_ps((__m128)fa.v, (__m128)tr.v, (__m128)m.m));
#else
  return v4_wrap_((mjtNum4_vec_)(((mjtNum4_bits_)tr.v & m.m) | ((mjtNum4_bits_)fa.v & ~m.m)));
#endif
}


// extract 4-bit lane mask in [0, 15]
mjSIMD_ATTR
int v4_movemask(mjtMask4 m) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return _mm256_movemask_pd((__m256d)m.m);
#elif (defined(__SSE2__) || defined(_M_X64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = __builtin_shufflevector((mjtNum4_vec_)m.m, (mjtNum4_vec_)m.m, 0, 1);
  mjtNum2_vec_ hi = __builtin_shufflevector((mjtNum4_vec_)m.m, (mjtNum4_vec_)m.m, 2, 3);
  return _mm_movemask_pd((__m128d)lo) | (_mm_movemask_pd((__m128d)hi) << 2);
#elif (defined(__SSE2__) || defined(_M_X64) || defined(__AVX__)) && defined(mjUSESINGLE)
  return _mm_movemask_ps((__m128)m.m);
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64))
  const uint32x4_t bits = {1, 2, 4, 8};
  #ifdef mjUSESINGLE
    return (int)vaddvq_u32(vandq_u32((uint32x4_t)m.m, bits));
  #else
    mjtFloat4_bits_ narrow = __builtin_convertvector(m.m, mjtFloat4_bits_);
    return (int)vaddvq_u32(vandq_u32((uint32x4_t)narrow, bits));
  #endif
#elif defined(__wasm_simd128__)
  #ifdef mjUSESINGLE
    return wasm_i32x4_bitmask((v128_t)m.m);
  #else
    typedef uint64_t mjtMask2_vec_ __attribute__((vector_size(16)));
    mjtMask2_vec_ lo = __builtin_shufflevector(m.m, m.m, 0, 1);
    mjtMask2_vec_ hi = __builtin_shufflevector(m.m, m.m, 2, 3);
    return wasm_i64x2_bitmask((v128_t)lo) | (wasm_i64x2_bitmask((v128_t)hi) << 2);
  #endif
#else
  return (int)((m.m[0] & 1) | (m.m[1] & 2) | (m.m[2] & 4) | (m.m[3] & 8));
#endif
}


// return 1 if any lane in mask is true, 0 otherwise
mjSIMD_ATTR
int v4_any(mjtMask4 m) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return !_mm256_testz_pd((__m256d)m.m, (__m256d)m.m);
#elif defined(__AVX__) && defined(mjUSESINGLE)
  return !_mm_testz_ps((__m128)m.m, (__m128)m.m);
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && defined(mjUSESINGLE)
  return vmaxvq_u32((uint32x4_t)m.m) != 0;
#elif defined(__wasm_simd128__)
  #ifdef mjUSESINGLE
    return wasm_v128_any_true((v128_t)m.m);
  #else
    typedef uint64_t mjtMask2_vec_ __attribute__((vector_size(16)));
    mjtMask2_vec_ lo = __builtin_shufflevector(m.m, m.m, 0, 1);
    mjtMask2_vec_ hi = __builtin_shufflevector(m.m, m.m, 2, 3);
    return wasm_v128_any_true((v128_t)(lo | hi));
  #endif
#else
  return v4_movemask(m) != 0;
#endif
}


// return 1 if all 4 lanes in mask are true, 0 otherwise
mjSIMD_ATTR
int v4_all(mjtMask4 m) {
  return v4_movemask(m) == 15;
}


// lane-wise minimum: a < b ? a : b (returns b on tie or NaN)
mjSIMD_ATTR
mjtNum4 v4_min(mjtNum4 a, mjtNum4 b) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm256_min_pd((__m256d)a.v, (__m256d)b.v));
#elif (defined(__SSE2__) || defined(_M_X64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)_mm_min_pd(
      (__m128d)__builtin_shufflevector(a.v, a.v, 0, 1),
      (__m128d)__builtin_shufflevector(b.v, b.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)_mm_min_pd(
      (__m128d)__builtin_shufflevector(a.v, a.v, 2, 3),
      (__m128d)__builtin_shufflevector(b.v, b.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif (defined(__SSE2__) || defined(_M_X64) || defined(__AVX__)) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm_min_ps((__m128)a.v, (__m128)b.v));
#else
  return v4_select(v4_cmplt(a, b), a, b);
#endif
}


// lane-wise maximum: a > b ? a : b (returns b on tie or NaN)
mjSIMD_ATTR
mjtNum4 v4_max(mjtNum4 a, mjtNum4 b) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm256_max_pd((__m256d)a.v, (__m256d)b.v));
#elif (defined(__SSE2__) || defined(_M_X64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)_mm_max_pd(
      (__m128d)__builtin_shufflevector(a.v, a.v, 0, 1),
      (__m128d)__builtin_shufflevector(b.v, b.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)_mm_max_pd(
      (__m128d)__builtin_shufflevector(a.v, a.v, 2, 3),
      (__m128d)__builtin_shufflevector(b.v, b.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif (defined(__SSE2__) || defined(_M_X64) || defined(__AVX__)) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm_max_ps((__m128)a.v, (__m128)b.v));
#else
  return v4_select(v4_cmpgt(a, b), a, b);
#endif
}


// lane-wise square root
mjSIMD_ATTR
mjtNum4 v4_sqrt(mjtNum4 a) {
#if defined(__AVX__) && !defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm256_sqrt_pd((__m256d)a.v));
#elif (defined(__SSE2__) || defined(_M_X64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)_mm_sqrt_pd((__m128d)__builtin_shufflevector(a.v, a.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)_mm_sqrt_pd((__m128d)__builtin_shufflevector(a.v, a.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif (defined(__SSE2__) || defined(_M_X64) || defined(__AVX__)) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)_mm_sqrt_ps((__m128)a.v));
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)vsqrtq_f64((float64x2_t)__builtin_shufflevector(a.v, a.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)vsqrtq_f64((float64x2_t)__builtin_shufflevector(a.v, a.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)vsqrtq_f32((float32x4_t)a.v));
#elif defined(__wasm_simd128__) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)wasm_f64x2_sqrt((v128_t)__builtin_shufflevector(a.v, a.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)wasm_f64x2_sqrt((v128_t)__builtin_shufflevector(a.v, a.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif defined(__wasm_simd128__) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)wasm_f32x4_sqrt((v128_t)a.v));
#elif defined(mjUSESINGLE)
  return v4_set(sqrtf(a.v[0]), sqrtf(a.v[1]), sqrtf(a.v[2]), sqrtf(a.v[3]));
#else
  return v4_set(sqrt(a.v[0]), sqrt(a.v[1]), sqrt(a.v[2]), sqrt(a.v[3]));
#endif
}


// negate selected lanes by XORing sign bits
mjSIMD_ATTR
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
#ifdef mjUSESINGLE
  const uint32_t sign = (uint32_t)1 << 31;
#else
  const uint64_t sign = (uint64_t)1 << 63;
#endif
  mjtNum4_bits_ m = {s0 ? sign : 0, s1 ? sign : 0, s2 ? sign : 0, s3 ? sign : 0};
  return v4_wrap_((mjtNum4_vec_)((mjtNum4_bits_)a.v ^ m));
}


// lane-wise absolute value (clears sign bits, matching mju_abs)
mjSIMD_ATTR
mjtNum4 v4_abs(mjtNum4 a) {
#if defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)vabsq_f64((float64x2_t)__builtin_shufflevector(a.v, a.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)vabsq_f64((float64x2_t)__builtin_shufflevector(a.v, a.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)vabsq_f32((float32x4_t)a.v));
#elif defined(__wasm_simd128__) && !defined(mjUSESINGLE)
  mjtNum2_vec_ lo = (mjtNum2_vec_)wasm_f64x2_abs((v128_t)__builtin_shufflevector(a.v, a.v, 0, 1));
  mjtNum2_vec_ hi = (mjtNum2_vec_)wasm_f64x2_abs((v128_t)__builtin_shufflevector(a.v, a.v, 2, 3));
  return v4_wrap_(__builtin_shufflevector(lo, hi, 0, 1, 2, 3));
#elif defined(__wasm_simd128__) && defined(mjUSESINGLE)
  return v4_wrap_((mjtNum4_vec_)wasm_f32x4_abs((v128_t)a.v));
#elif defined(mjUSESINGLE)
  const uint32_t mask = ~((uint32_t)1 << 31);
  mjtNum4_bits_ m = {mask, mask, mask, mask};
  return v4_wrap_((mjtNum4_vec_)((mjtNum4_bits_)a.v & m));
#else
  const uint64_t mask = ~((uint64_t)1 << 63);
  mjtNum4_bits_ m = {mask, mask, mask, mask};
  return v4_wrap_((mjtNum4_vec_)((mjtNum4_bits_)a.v & m));
#endif
}


// permute lanes with compile-time indices; a is evaluated multiple times and must be
// side-effect-free
#define v4_shuffle(a, i0, i1, i2, i3) \
  v4_wrap_(__builtin_shufflevector((a).v, (a).v, i0, i1, i2, i3))


// transpose 4x4 matrix of mjtNum4 rows in place
mjSIMD_ATTR
void v4_transpose(mjtNum4* r0, mjtNum4* r1, mjtNum4* r2, mjtNum4* r3) {
  mjtNum4_vec_ t0 = __builtin_shufflevector(r0->v, r1->v, 0, 4, 2, 6);
  mjtNum4_vec_ t1 = __builtin_shufflevector(r0->v, r1->v, 1, 5, 3, 7);
  mjtNum4_vec_ t2 = __builtin_shufflevector(r2->v, r3->v, 0, 4, 2, 6);
  mjtNum4_vec_ t3 = __builtin_shufflevector(r2->v, r3->v, 1, 5, 3, 7);
  r0->v = __builtin_shufflevector(t0, t2, 0, 1, 4, 5);
  r1->v = __builtin_shufflevector(t1, t3, 0, 1, 4, 5);
  r2->v = __builtin_shufflevector(t0, t2, 2, 3, 6, 7);
  r3->v = __builtin_shufflevector(t1, t3, 2, 3, 6, 7);
}


// load and deinterleave 4 contiguous 3D mjtNum triplets into 3 vectors
mjSIMD_ATTR
void v4_load4x3(const mjtNum* p, mjtNum4* c0, mjtNum4* c1, mjtNum4* c2) {
#if defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64)) && defined(mjUSESINGLE)
  float32x4x3_t v = vld3q_f32(p);
  c0->v = (mjtNum4_vec_)v.val[0];
  c1->v = (mjtNum4_vec_)v.val[1];
  c2->v = (mjtNum4_vec_)v.val[2];
#elif !defined(mjUSESINGLE)
  mjtNum2_vec_ r0, r1, r2, r3, r4, r5;
  memcpy(&r0, p + 0, sizeof(r0));
  memcpy(&r1, p + 2, sizeof(r1));
  memcpy(&r2, p + 4, sizeof(r2));
  memcpy(&r3, p + 6, sizeof(r3));
  memcpy(&r4, p + 8, sizeof(r4));
  memcpy(&r5, p + 10, sizeof(r5));
  c0->v = __builtin_shufflevector(__builtin_shufflevector(r0, r1, 0, 3),
                                  __builtin_shufflevector(r3, r4, 0, 3), 0, 1, 2, 3);
  c1->v = __builtin_shufflevector(__builtin_shufflevector(r0, r2, 1, 2),
                                  __builtin_shufflevector(r3, r5, 1, 2), 0, 1, 2, 3);
  c2->v = __builtin_shufflevector(__builtin_shufflevector(r1, r2, 0, 3),
                                  __builtin_shufflevector(r4, r5, 0, 3), 0, 1, 2, 3);
#else
  mjtNum4_vec_ r0 = v4_load(p + 0).v;
  mjtNum4_vec_ r1 = v4_load(p + 4).v;
  mjtNum4_vec_ r2 = v4_load(p + 8).v;
  mjtNum4_vec_ t0 = __builtin_shufflevector(r0, r1, 1, 2, 4, 5);
  mjtNum4_vec_ t1 = __builtin_shufflevector(r1, r2, 2, 3, 5, 6);
  c0->v = __builtin_shufflevector(r0, t1, 0, 3, 4, 6);
  c1->v = __builtin_shufflevector(t0, t1, 0, 2, 5, 7);
  c2->v = __builtin_shufflevector(t0, r2, 1, 3, 4, 7);
#endif
}


// load and deinterleave 4 contiguous 3D float32 vertices into 3 mjtNum4 vectors
mjSIMD_ATTR
void v4_load3_f32(const float* p, mjtNum4* vx, mjtNum4* vy, mjtNum4* vz) {
#if defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64))
  float32x4x3_t v = vld3q_f32(p);
  mjtFloat4_vec_ fx = (mjtFloat4_vec_)v.val[0];
  mjtFloat4_vec_ fy = (mjtFloat4_vec_)v.val[1];
  mjtFloat4_vec_ fz = (mjtFloat4_vec_)v.val[2];
#else
  mjtFloat4_vec_ r0, r1, r2;
  memcpy(&r0, p + 0, sizeof(r0));
  memcpy(&r1, p + 4, sizeof(r1));
  memcpy(&r2, p + 8, sizeof(r2));
  mjtFloat4_vec_ t0 = __builtin_shufflevector(r0, r1, 1, 2, 4, 5);
  mjtFloat4_vec_ t1 = __builtin_shufflevector(r1, r2, 2, 3, 5, 6);
  mjtFloat4_vec_ fx = __builtin_shufflevector(r0, t1, 0, 3, 4, 6);
  mjtFloat4_vec_ fy = __builtin_shufflevector(t0, t1, 0, 2, 5, 7);
  mjtFloat4_vec_ fz = __builtin_shufflevector(t0, r2, 1, 3, 4, 7);
#endif
#ifdef mjUSESINGLE
  vx->v = fx;
  vy->v = fy;
  vz->v = fz;
#else
  vx->v = __builtin_convertvector(fx, mjtNum4_f64);
  vy->v = __builtin_convertvector(fy, mjtNum4_f64);
  vz->v = __builtin_convertvector(fz, mjtNum4_f64);
#endif
}


// horizontal sum with fixed (l0+l2) + (l1+l3) reduction tree
mjSIMD_ATTR
mjtNum v4_sum(mjtNum4 a) {
  mjtNum4_vec_ s = a.v + __builtin_shufflevector(a.v, a.v, 2, 3, 0, 1);
  return s[0] + s[1];
}


// lane-wise fused multiply-add: a*b + c
mjSIMD_ATTR
mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
#ifdef mjUSEFMA
  return v4_set(mju_fma(a.v[0], b.v[0], c.v[0]),
                mju_fma(a.v[1], b.v[1], c.v[1]),
                mju_fma(a.v[2], b.v[2], c.v[2]),
                mju_fma(a.v[3], b.v[3], c.v[3]));
#else
  #ifdef __clang__
    #pragma STDC FP_CONTRACT OFF
  #endif
  mjtNum4_vec_ prod = a.v * b.v;
  return v4_wrap_(prod + c.v);
#endif
}


// load 4 floats from memory
mjSIMD128_ATTR
mjtFloat4 f4_load(const float* p) {
  mjtFloat4 r;
  memcpy(&r.v, p, sizeof(r.v));
  return r;
}


// store 4 floats to memory
mjSIMD128_ATTR
void f4_store(float* p, mjtFloat4 a) {
  memcpy(p, &a.v, sizeof(a.v));
}


// broadcast float scalar to all 4 lanes
mjSIMD128_ATTR
mjtFloat4 f4_splat(float a) {
  mjtFloat4 r = {{a, a, a, a}};
  return r;
}


// lane-wise float32 less-than mask
mjSIMD128_ATTR
mjtFloatMask4 f4_cmplt(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {(mjtFloat4_bits_)(a.v < b.v)};
  return r;
}


// lane-wise float32 less-or-equal mask
mjSIMD128_ATTR
mjtFloatMask4 f4_cmple(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {(mjtFloat4_bits_)(a.v <= b.v)};
  return r;
}


// bitwise AND of float32 masks
mjSIMD128_ATTR
mjtFloatMask4 f4_and(mjtFloatMask4 a, mjtFloatMask4 b) {
  mjtFloatMask4 r = {a.m & b.m};
  return r;
}


// extract 4-bit float32 lane mask in [0, 15]
mjSIMD128_ATTR
int f4_movemask(mjtFloatMask4 m) {
#if defined(__SSE2__) || defined(_M_X64) || defined(__AVX__)
  return _mm_movemask_ps((__m128)m.m);
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64))
  const uint32x4_t bits = {1, 2, 4, 8};
  return (int)vaddvq_u32(vandq_u32((uint32x4_t)m.m, bits));
#elif defined(__wasm_simd128__)
  return wasm_i32x4_bitmask((v128_t)m.m);
#else
  return (int)((m.m[0] & 1) | (m.m[1] & 2) | (m.m[2] & 4) | (m.m[3] & 8));
#endif
}

#undef mjSIMD_ATTR
#undef mjSIMD128_ATTR


//------------------------------ SSE/AVX intrinsics backend (MSVC) ---------------------------------

#elif defined(mjUSEAVX)

typedef struct { __m128 v; } mjtFloat4;
typedef struct { __m128 m; } mjtFloatMask4;


// load 4 floats from memory
static inline
mjtFloat4 f4_load(const float* p) {
  mjtFloat4 r = {_mm_loadu_ps(p)};
  return r;
}


// store 4 floats to memory
static inline
void f4_store(float* p, mjtFloat4 a) {
  _mm_storeu_ps(p, a.v);
}


// broadcast float scalar to all 4 lanes
static inline
mjtFloat4 f4_splat(float a) {
  mjtFloat4 r = {_mm_set1_ps(a)};
  return r;
}


// lane-wise float32 less-than mask
static inline
mjtFloatMask4 f4_cmplt(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {_mm_cmp_ps(a.v, b.v, _CMP_LT_OQ)};
  return r;
}


// lane-wise float32 less-or-equal mask
static inline
mjtFloatMask4 f4_cmple(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {_mm_cmp_ps(a.v, b.v, _CMP_LE_OQ)};
  return r;
}


// bitwise AND of float32 masks
static inline
mjtFloatMask4 f4_and(mjtFloatMask4 a, mjtFloatMask4 b) {
  mjtFloatMask4 r = {_mm_and_ps(a.m, b.m)};
  return r;
}


// extract 4-bit float32 lane mask in [0, 15]
static inline
int f4_movemask(mjtFloatMask4 m) {
  return _mm_movemask_ps(m.m);
}


#ifdef mjUSESINGLE

typedef struct { __m128 v; } mjtNum4;
typedef struct { __m128 m; } mjtMask4;


// wrap __m128 in mjtNum4
static inline
mjtNum4 v4_wrap_(__m128 v) {
  mjtNum4 r;
  r.v = v;
  return r;
}


// wrap __m128 in mjtMask4
static inline
mjtMask4 v4_wrap_mask_(__m128 m) {
  mjtMask4 r;
  r.m = m;
  return r;
}


// load 4 mjtNums from memory
static inline
mjtNum4 v4_load(const mjtNum* p) {
  return v4_wrap_(_mm_loadu_ps(p));
}


// store 4 mjtNums to memory
static inline
void v4_store(mjtNum* p, mjtNum4 a) {
  _mm_storeu_ps(p, a.v);
}


// load 3 mjtNums from memory into lanes 0..2 with 0 in lane 3 (reads 3 mjtNums)
static inline
mjtNum4 v4_load3(const mjtNum* p) {
  __m128 xy = _mm_castsi128_ps(_mm_loadl_epi64((const __m128i*)p));
  return v4_wrap_(_mm_movelh_ps(xy, _mm_load_ss(p + 2)));
}


// store low 3 lanes to memory (writes 3 mjtNums)
static inline
void v4_store3(mjtNum* p, mjtNum4 a) {
  _mm_storel_epi64((__m128i*)p, _mm_castps_si128(a.v));
  _mm_store_ss(p + 2, _mm_movehl_ps(a.v, a.v));
}


// set 4 lanes (also used for gathers)
static inline
mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
  return v4_wrap_(_mm_setr_ps(a, b, c, d));
}


// broadcast scalar to all 4 lanes
static inline
mjtNum4 v4_splat(mjtNum a) {
  return v4_wrap_(_mm_set1_ps(a));
}


// lane-wise addition
static inline
mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_add_ps(a.v, b.v));
}


// lane-wise subtraction
static inline
mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_sub_ps(a.v, b.v));
}


// lane-wise multiplication
static inline
mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_mul_ps(a.v, b.v));
}


// lane-wise division
static inline
mjtNum4 v4_div(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_div_ps(a.v, b.v));
}


// lane-wise less-than mask (ordered quiet)
static inline
mjtMask4 v4_cmplt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm_cmp_ps(a.v, b.v, _CMP_LT_OQ));
}


// lane-wise less-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmple(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm_cmp_ps(a.v, b.v, _CMP_LE_OQ));
}


// lane-wise greater-than mask (ordered quiet)
static inline
mjtMask4 v4_cmpgt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm_cmp_ps(a.v, b.v, _CMP_GT_OQ));
}


// lane-wise greater-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmpge(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm_cmp_ps(a.v, b.v, _CMP_GE_OQ));
}


// lane-wise equality mask (ordered quiet)
static inline
mjtMask4 v4_cmpeq(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm_cmp_ps(a.v, b.v, _CMP_EQ_OQ));
}


// bitwise AND of masks
static inline
mjtMask4 v4_and(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm_and_ps(a.m, b.m));
}


// bitwise OR of masks
static inline
mjtMask4 v4_or(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm_or_ps(a.m, b.m));
}


// bitwise XOR of masks
static inline
mjtMask4 v4_xor(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm_xor_ps(a.m, b.m));
}


// bitwise (~a & b) of masks
static inline
mjtMask4 v4_andnot(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm_andnot_ps(a.m, b.m));
}


// bitwise NOT of mask
static inline
mjtMask4 v4_not(mjtMask4 a) {
  __m128 all = _mm_cmp_ps(_mm_setzero_ps(), _mm_setzero_ps(), _CMP_EQ_OQ);
  return v4_wrap_mask_(_mm_xor_ps(a.m, all));
}


// lane-wise blend: m ? tr : fa
static inline
mjtNum4 v4_select(mjtMask4 m, mjtNum4 tr, mjtNum4 fa) {
  return v4_wrap_(_mm_blendv_ps(fa.v, tr.v, m.m));
}


// extract 4-bit lane mask in [0, 15]
static inline
int v4_movemask(mjtMask4 m) {
  return _mm_movemask_ps(m.m);
}


// return 1 if any lane in mask is true, 0 otherwise
static inline
int v4_any(mjtMask4 m) {
  return !_mm_testz_ps(m.m, m.m);
}


// return 1 if all 4 lanes in mask are true, 0 otherwise
static inline
int v4_all(mjtMask4 m) {
  return _mm_movemask_ps(m.m) == 15;
}


// lane-wise minimum: a < b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_min(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_min_ps(a.v, b.v));
}


// lane-wise maximum: a > b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_max(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm_max_ps(a.v, b.v));
}


// lane-wise square root
static inline
mjtNum4 v4_sqrt(mjtNum4 a) {
  return v4_wrap_(_mm_sqrt_ps(a.v));
}


// negate selected lanes by XORing sign bits
static inline
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  __m128 m = _mm_setr_ps(s0 ? -0.0f : 0.0f, s1 ? -0.0f : 0.0f,
                         s2 ? -0.0f : 0.0f, s3 ? -0.0f : 0.0f);
  return v4_wrap_(_mm_xor_ps(a.v, m));
}


// lane-wise absolute value (clears sign bits, matching mju_abs)
static inline
mjtNum4 v4_abs(mjtNum4 a) {
  return v4_wrap_(_mm_andnot_ps(_mm_set1_ps(-0.0f), a.v));
}


// permute lanes with compile-time indices; a is evaluated multiple times and must be
// side-effect-free
#define v4_shuffle(a, i0, i1, i2, i3) \
  v4_wrap_(_mm_shuffle_ps((a).v, (a).v, _MM_SHUFFLE(i3, i2, i1, i0)))


// transpose 4x4 matrix of mjtNum4 rows in place
static inline
void v4_transpose(mjtNum4* r0, mjtNum4* r1, mjtNum4* r2, mjtNum4* r3) {
  _MM_TRANSPOSE4_PS(r0->v, r1->v, r2->v, r3->v);
}


// load and deinterleave 4 contiguous 3D mjtNum triplets into 3 vectors
static inline
void v4_load4x3(const mjtNum* p, mjtNum4* c0, mjtNum4* c1, mjtNum4* c2) {
  __m128 r0 = _mm_loadu_ps(p + 0);
  __m128 r1 = _mm_loadu_ps(p + 4);
  __m128 r2 = _mm_loadu_ps(p + 8);
  __m128 t0 = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 2, 1));
  __m128 t1 = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(2, 1, 3, 2));
  c0->v = _mm_shuffle_ps(r0, t1, _MM_SHUFFLE(2, 0, 3, 0));
  c1->v = _mm_shuffle_ps(t0, t1, _MM_SHUFFLE(3, 1, 2, 0));
  c2->v = _mm_shuffle_ps(t0, r2, _MM_SHUFFLE(3, 0, 3, 1));
}


// load and deinterleave 4 contiguous 3D float32 vertices into 3 mjtNum4 vectors
static inline
void v4_load3_f32(const float* p, mjtNum4* vx, mjtNum4* vy, mjtNum4* vz) {
  v4_load4x3(p, vx, vy, vz);
}


// horizontal sum with fixed (l0+l2) + (l1+l3) reduction tree
static inline
mjtNum v4_sum(mjtNum4 a) {
  __m128 s = _mm_add_ps(a.v, _mm_shuffle_ps(a.v, a.v, _MM_SHUFFLE(1, 0, 3, 2)));
  return _mm_cvtss_f32(_mm_add_ss(s, _mm_shuffle_ps(s, s, 1)));
}


// lane-wise fused multiply-add: a*b + c
static inline
mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
#if defined(mjUSEFMA) && (defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__)))
  return v4_wrap_(_mm_fmadd_ps(a.v, b.v, c.v));
#elif defined(mjUSEFMA)
  mjtNum av[4], bv[4], cv[4];
  _mm_storeu_ps(av, a.v);
  _mm_storeu_ps(bv, b.v);
  _mm_storeu_ps(cv, c.v);
  return v4_set(mju_fma(av[0], bv[0], cv[0]),
                mju_fma(av[1], bv[1], cv[1]),
                mju_fma(av[2], bv[2], cv[2]),
                mju_fma(av[3], bv[3], cv[3]));
#else
  #ifdef __clang__
    #pragma STDC FP_CONTRACT OFF
  #endif
  __m128 prod = _mm_mul_ps(a.v, b.v);
  return v4_wrap_(_mm_add_ps(prod, c.v));
#endif
}

#else  // !mjUSESINGLE

typedef struct { __m256d v; } mjtNum4;
typedef struct { __m256d m; } mjtMask4;


// wrap __m256d in mjtNum4
static inline
mjtNum4 v4_wrap_(__m256d v) {
  mjtNum4 r;
  r.v = v;
  return r;
}


// wrap __m256d in mjtMask4
static inline
mjtMask4 v4_wrap_mask_(__m256d m) {
  mjtMask4 r;
  r.m = m;
  return r;
}


// extract low or high 128-bit half
static inline
__m128d v4_half_(mjtNum4 a, int hi) {
  return hi ? _mm256_extractf128_pd(a.v, 1) : _mm256_castpd256_pd128(a.v);
}


// combine two 128-bit halves into mjtNum4
static inline
mjtNum4 v4_combine_(__m128d lo, __m128d hi) {
  return v4_wrap_(_mm256_insertf128_pd(_mm256_castpd128_pd256(lo), hi, 1));
}


// load 4 mjtNums from memory
static inline
mjtNum4 v4_load(const mjtNum* p) {
  return v4_wrap_(_mm256_loadu_pd(p));
}


// store 4 mjtNums to memory
static inline
void v4_store(mjtNum* p, mjtNum4 a) {
  _mm256_storeu_pd(p, a.v);
}


// load 3 mjtNums from memory into lanes 0..2 with 0 in lane 3 (reads 3 mjtNums)
static inline
mjtNum4 v4_load3(const mjtNum* p) {
  return v4_combine_(_mm_loadu_pd(p), _mm_load_sd(p + 2));
}


// store low 3 lanes to memory (writes 3 mjtNums)
static inline
void v4_store3(mjtNum* p, mjtNum4 a) {
  _mm_storeu_pd(p, _mm256_castpd256_pd128(a.v));
  _mm_store_sd(p + 2, _mm256_extractf128_pd(a.v, 1));
}


// set 4 lanes (also used for gathers)
static inline
mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
  return v4_wrap_(_mm256_setr_pd(a, b, c, d));
}


// broadcast scalar to all 4 lanes
static inline
mjtNum4 v4_splat(mjtNum a) {
  return v4_wrap_(_mm256_set1_pd(a));
}


// lane-wise addition
static inline
mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_add_pd(a.v, b.v));
}


// lane-wise subtraction
static inline
mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_sub_pd(a.v, b.v));
}


// lane-wise multiplication
static inline
mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_mul_pd(a.v, b.v));
}


// lane-wise division
static inline
mjtNum4 v4_div(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_div_pd(a.v, b.v));
}


// lane-wise less-than mask (ordered quiet)
static inline
mjtMask4 v4_cmplt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm256_cmp_pd(a.v, b.v, _CMP_LT_OQ));
}


// lane-wise less-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmple(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm256_cmp_pd(a.v, b.v, _CMP_LE_OQ));
}


// lane-wise greater-than mask (ordered quiet)
static inline
mjtMask4 v4_cmpgt(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm256_cmp_pd(a.v, b.v, _CMP_GT_OQ));
}


// lane-wise greater-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmpge(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm256_cmp_pd(a.v, b.v, _CMP_GE_OQ));
}


// lane-wise equality mask (ordered quiet)
static inline
mjtMask4 v4_cmpeq(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_mask_(_mm256_cmp_pd(a.v, b.v, _CMP_EQ_OQ));
}


// bitwise AND of masks
static inline
mjtMask4 v4_and(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm256_and_pd(a.m, b.m));
}


// bitwise OR of masks
static inline
mjtMask4 v4_or(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm256_or_pd(a.m, b.m));
}


// bitwise XOR of masks
static inline
mjtMask4 v4_xor(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm256_xor_pd(a.m, b.m));
}


// bitwise (~a & b) of masks
static inline
mjtMask4 v4_andnot(mjtMask4 a, mjtMask4 b) {
  return v4_wrap_mask_(_mm256_andnot_pd(a.m, b.m));
}


// bitwise NOT of mask
static inline
mjtMask4 v4_not(mjtMask4 a) {
  __m256d all = _mm256_cmp_pd(_mm256_setzero_pd(), _mm256_setzero_pd(), _CMP_EQ_OQ);
  return v4_wrap_mask_(_mm256_xor_pd(a.m, all));
}


// lane-wise blend: m ? tr : fa
static inline
mjtNum4 v4_select(mjtMask4 m, mjtNum4 tr, mjtNum4 fa) {
  return v4_wrap_(_mm256_blendv_pd(fa.v, tr.v, m.m));
}


// extract 4-bit lane mask in [0, 15]
static inline
int v4_movemask(mjtMask4 m) {
  return _mm256_movemask_pd(m.m);
}


// return 1 if any lane in mask is true, 0 otherwise
static inline
int v4_any(mjtMask4 m) {
  return !_mm256_testz_pd(m.m, m.m);
}


// return 1 if all 4 lanes in mask are true, 0 otherwise
static inline
int v4_all(mjtMask4 m) {
  return _mm256_movemask_pd(m.m) == 15;
}


// lane-wise minimum: a < b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_min(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_min_pd(a.v, b.v));
}


// lane-wise maximum: a > b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_max(mjtNum4 a, mjtNum4 b) {
  return v4_wrap_(_mm256_max_pd(a.v, b.v));
}


// lane-wise square root
static inline
mjtNum4 v4_sqrt(mjtNum4 a) {
  return v4_wrap_(_mm256_sqrt_pd(a.v));
}


// negate selected lanes by XORing sign bits
static inline
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  __m256d m = _mm256_setr_pd(s0 ? -0.0 : 0.0, s1 ? -0.0 : 0.0,
                             s2 ? -0.0 : 0.0, s3 ? -0.0 : 0.0);
  return v4_wrap_(_mm256_xor_pd(a.v, m));
}


// lane-wise absolute value (clears sign bits, matching mju_abs)
static inline
mjtNum4 v4_abs(mjtNum4 a) {
  return v4_wrap_(_mm256_andnot_pd(_mm256_set1_pd(-0.0), a.v));
}


// permute lanes with compile-time indices (AVX1-compatible); a is evaluated multiple times and
// must be side-effect-free
#define v4_shuffle(a, i0, i1, i2, i3) \
  v4_combine_( \
    _mm_shuffle_pd(v4_half_(a, (i0) >= 2), v4_half_(a, (i1) >= 2), \
                   ((i0) & 1) | (((i1) & 1) << 1)), \
    _mm_shuffle_pd(v4_half_(a, (i2) >= 2), v4_half_(a, (i3) >= 2), \
                   ((i2) & 1) | (((i3) & 1) << 1)))


// transpose 4x4 matrix of mjtNum4 rows in place
static inline
void v4_transpose(mjtNum4* r0, mjtNum4* r1, mjtNum4* r2, mjtNum4* r3) {
  __m256d t0 = _mm256_unpacklo_pd(r0->v, r1->v);
  __m256d t1 = _mm256_unpackhi_pd(r0->v, r1->v);
  __m256d t2 = _mm256_unpacklo_pd(r2->v, r3->v);
  __m256d t3 = _mm256_unpackhi_pd(r2->v, r3->v);
  r0->v = _mm256_permute2f128_pd(t0, t2, 0x20);
  r1->v = _mm256_permute2f128_pd(t1, t3, 0x20);
  r2->v = _mm256_permute2f128_pd(t0, t2, 0x31);
  r3->v = _mm256_permute2f128_pd(t1, t3, 0x31);
}


// load and deinterleave 4 contiguous 3D mjtNum triplets into 3 vectors
static inline
void v4_load4x3(const mjtNum* p, mjtNum4* c0, mjtNum4* c1, mjtNum4* c2) {
  __m128d r0 = _mm_loadu_pd(p + 0);
  __m128d r1 = _mm_loadu_pd(p + 2);
  __m128d r2 = _mm_loadu_pd(p + 4);
  __m128d r3 = _mm_loadu_pd(p + 6);
  __m128d r4 = _mm_loadu_pd(p + 8);
  __m128d r5 = _mm_loadu_pd(p + 10);
  *c0 = v4_combine_(_mm_shuffle_pd(r0, r1, 2), _mm_shuffle_pd(r3, r4, 2));
  *c1 = v4_combine_(_mm_shuffle_pd(r0, r2, 1), _mm_shuffle_pd(r3, r5, 1));
  *c2 = v4_combine_(_mm_shuffle_pd(r1, r2, 2), _mm_shuffle_pd(r4, r5, 2));
}


// load and deinterleave 4 contiguous 3D float32 vertices into 3 mjtNum4 vectors
static inline
void v4_load3_f32(const float* p, mjtNum4* vx, mjtNum4* vy, mjtNum4* vz) {
  __m128 r0 = _mm_loadu_ps(p + 0);
  __m128 r1 = _mm_loadu_ps(p + 4);
  __m128 r2 = _mm_loadu_ps(p + 8);
  __m128 t0 = _mm_shuffle_ps(r0, r1, _MM_SHUFFLE(1, 0, 2, 1));
  __m128 t1 = _mm_shuffle_ps(r1, r2, _MM_SHUFFLE(2, 1, 3, 2));
  vx->v = _mm256_cvtps_pd(_mm_shuffle_ps(r0, t1, _MM_SHUFFLE(2, 0, 3, 0)));
  vy->v = _mm256_cvtps_pd(_mm_shuffle_ps(t0, t1, _MM_SHUFFLE(3, 1, 2, 0)));
  vz->v = _mm256_cvtps_pd(_mm_shuffle_ps(t0, r2, _MM_SHUFFLE(3, 0, 3, 1)));
}


// horizontal sum with fixed (l0+l2) + (l1+l3) reduction tree
static inline
mjtNum v4_sum(mjtNum4 a) {
  __m128d s = _mm_add_pd(_mm256_castpd256_pd128(a.v), _mm256_extractf128_pd(a.v, 1));
  return _mm_cvtsd_f64(_mm_add_sd(s, _mm_unpackhi_pd(s, s)));
}


// lane-wise fused multiply-add: a*b + c
static inline
mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
#if defined(mjUSEFMA) && (defined(__FMA__) || (defined(_MSC_VER) && defined(__AVX2__)))
  return v4_wrap_(_mm256_fmadd_pd(a.v, b.v, c.v));
#elif defined(mjUSEFMA)
  mjtNum av[4], bv[4], cv[4];
  _mm256_storeu_pd(av, a.v);
  _mm256_storeu_pd(bv, b.v);
  _mm256_storeu_pd(cv, c.v);
  return v4_set(mju_fma(av[0], bv[0], cv[0]),
                mju_fma(av[1], bv[1], cv[1]),
                mju_fma(av[2], bv[2], cv[2]),
                mju_fma(av[3], bv[3], cv[3]));
#else
  #ifdef __clang__
    #pragma STDC FP_CONTRACT OFF
  #endif
  __m256d prod = _mm256_mul_pd(a.v, b.v);
  return v4_wrap_(_mm256_add_pd(prod, c.v));
#endif
}

#endif  // !mjUSESINGLE


//------------------------------ plain-C scalar reference backend ----------------------------------

#else

typedef struct { mjtNum v[4]; } mjtNum4;
typedef struct { uint32_t m[4]; } mjtMask4;
typedef struct { float v[4]; } mjtFloat4;
typedef struct { uint32_t m[4]; } mjtFloatMask4;


// load 4 mjtNums from memory
static inline
mjtNum4 v4_load(const mjtNum* p) {
  mjtNum4 r;
  memcpy(r.v, p, sizeof(r.v));
  return r;
}


// store 4 mjtNums to memory
static inline
void v4_store(mjtNum* p, mjtNum4 a) {
  memcpy(p, a.v, sizeof(a.v));
}


// load 3 mjtNums from memory into lanes 0..2 with 0 in lane 3 (reads 3 mjtNums)
static inline
mjtNum4 v4_load3(const mjtNum* p) {
  mjtNum4 r = {{p[0], p[1], p[2], 0}};
  return r;
}


// store low 3 lanes to memory (writes 3 mjtNums)
static inline
void v4_store3(mjtNum* p, mjtNum4 a) {
  memcpy(p, a.v, 3 * sizeof(mjtNum));
}


// set 4 lanes (also used for gathers)
static inline
mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
  mjtNum4 r = {{a, b, c, d}};
  return r;
}


// broadcast scalar to all 4 lanes
static inline
mjtNum4 v4_splat(mjtNum a) {
  mjtNum4 r = {{a, a, a, a}};
  return r;
}


// lane-wise addition
static inline
mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] + b.v[0], a.v[1] + b.v[1], a.v[2] + b.v[2], a.v[3] + b.v[3]);
}


// lane-wise subtraction
static inline
mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] - b.v[0], a.v[1] - b.v[1], a.v[2] - b.v[2], a.v[3] - b.v[3]);
}


// lane-wise multiplication
static inline
mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] * b.v[0], a.v[1] * b.v[1], a.v[2] * b.v[2], a.v[3] * b.v[3]);
}


// lane-wise division
static inline
mjtNum4 v4_div(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] / b.v[0], a.v[1] / b.v[1], a.v[2] / b.v[2], a.v[3] / b.v[3]);
}


// construct scalar mask from 4 boolean conditions
static inline
mjtMask4 v4_mask_(int c0, int c1, int c2, int c3) {
  mjtMask4 r = {{c0 ? ~0u : 0u, c1 ? ~0u : 0u, c2 ? ~0u : 0u, c3 ? ~0u : 0u}};
  return r;
}


// lane-wise less-than mask (ordered quiet)
static inline
mjtMask4 v4_cmplt(mjtNum4 a, mjtNum4 b) {
  return v4_mask_(a.v[0] < b.v[0], a.v[1] < b.v[1], a.v[2] < b.v[2], a.v[3] < b.v[3]);
}


// lane-wise less-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmple(mjtNum4 a, mjtNum4 b) {
  return v4_mask_(a.v[0] <= b.v[0], a.v[1] <= b.v[1], a.v[2] <= b.v[2], a.v[3] <= b.v[3]);
}


// lane-wise greater-than mask (ordered quiet)
static inline
mjtMask4 v4_cmpgt(mjtNum4 a, mjtNum4 b) {
  return v4_mask_(a.v[0] > b.v[0], a.v[1] > b.v[1], a.v[2] > b.v[2], a.v[3] > b.v[3]);
}


// lane-wise greater-or-equal mask (ordered quiet)
static inline
mjtMask4 v4_cmpge(mjtNum4 a, mjtNum4 b) {
  return v4_mask_(a.v[0] >= b.v[0], a.v[1] >= b.v[1], a.v[2] >= b.v[2], a.v[3] >= b.v[3]);
}


// lane-wise equality mask (ordered quiet)
static inline
mjtMask4 v4_cmpeq(mjtNum4 a, mjtNum4 b) {
  return v4_mask_(a.v[0] == b.v[0], a.v[1] == b.v[1], a.v[2] == b.v[2], a.v[3] == b.v[3]);
}


// bitwise AND of masks
static inline
mjtMask4 v4_and(mjtMask4 a, mjtMask4 b) {
  mjtMask4 r = {{a.m[0] & b.m[0], a.m[1] & b.m[1], a.m[2] & b.m[2], a.m[3] & b.m[3]}};
  return r;
}


// bitwise OR of masks
static inline
mjtMask4 v4_or(mjtMask4 a, mjtMask4 b) {
  mjtMask4 r = {{a.m[0] | b.m[0], a.m[1] | b.m[1], a.m[2] | b.m[2], a.m[3] | b.m[3]}};
  return r;
}


// bitwise XOR of masks
static inline
mjtMask4 v4_xor(mjtMask4 a, mjtMask4 b) {
  mjtMask4 r = {{a.m[0] ^ b.m[0], a.m[1] ^ b.m[1], a.m[2] ^ b.m[2], a.m[3] ^ b.m[3]}};
  return r;
}


// bitwise (~a & b) of masks
static inline
mjtMask4 v4_andnot(mjtMask4 a, mjtMask4 b) {
  mjtMask4 r = {{~a.m[0] & b.m[0], ~a.m[1] & b.m[1], ~a.m[2] & b.m[2], ~a.m[3] & b.m[3]}};
  return r;
}


// bitwise NOT of mask
static inline
mjtMask4 v4_not(mjtMask4 a) {
  mjtMask4 r = {{~a.m[0], ~a.m[1], ~a.m[2], ~a.m[3]}};
  return r;
}


// lane-wise blend: m ? tr : fa
static inline
mjtNum4 v4_select(mjtMask4 m, mjtNum4 tr, mjtNum4 fa) {
  return v4_set(m.m[0] ? tr.v[0] : fa.v[0], m.m[1] ? tr.v[1] : fa.v[1],
                m.m[2] ? tr.v[2] : fa.v[2], m.m[3] ? tr.v[3] : fa.v[3]);
}


// extract 4-bit lane mask in [0, 15]
static inline
int v4_movemask(mjtMask4 m) {
  return (int)((m.m[0] & 1u) | (m.m[1] & 2u) | (m.m[2] & 4u) | (m.m[3] & 8u));
}


// return 1 if any lane in mask is true, 0 otherwise
static inline
int v4_any(mjtMask4 m) {
  return v4_movemask(m) != 0;
}


// return 1 if all 4 lanes in mask are true, 0 otherwise
static inline
int v4_all(mjtMask4 m) {
  return v4_movemask(m) == 15;
}


// lane-wise minimum: a < b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_min(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] < b.v[0] ? a.v[0] : b.v[0],
                a.v[1] < b.v[1] ? a.v[1] : b.v[1],
                a.v[2] < b.v[2] ? a.v[2] : b.v[2],
                a.v[3] < b.v[3] ? a.v[3] : b.v[3]);
}


// lane-wise maximum: a > b ? a : b (returns b on tie or NaN)
static inline
mjtNum4 v4_max(mjtNum4 a, mjtNum4 b) {
  return v4_set(a.v[0] > b.v[0] ? a.v[0] : b.v[0],
                a.v[1] > b.v[1] ? a.v[1] : b.v[1],
                a.v[2] > b.v[2] ? a.v[2] : b.v[2],
                a.v[3] > b.v[3] ? a.v[3] : b.v[3]);
}


// lane-wise square root
static inline
mjtNum4 v4_sqrt(mjtNum4 a) {
#ifdef mjUSESINGLE
  return v4_set(sqrtf(a.v[0]), sqrtf(a.v[1]), sqrtf(a.v[2]), sqrtf(a.v[3]));
#else
  return v4_set(sqrt(a.v[0]), sqrt(a.v[1]), sqrt(a.v[2]), sqrt(a.v[3]));
#endif
}


// negate selected lanes by XORing sign bits
static inline
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  return v4_set(s0 ? -a.v[0] : a.v[0], s1 ? -a.v[1] : a.v[1],
                s2 ? -a.v[2] : a.v[2], s3 ? -a.v[3] : a.v[3]);
}


// lane-wise absolute value (clears sign bits, matching mju_abs)
static inline
mjtNum4 v4_abs(mjtNum4 a) {
#ifdef mjUSESINGLE
  return v4_set(fabsf(a.v[0]), fabsf(a.v[1]), fabsf(a.v[2]), fabsf(a.v[3]));
#else
  return v4_set(fabs(a.v[0]), fabs(a.v[1]), fabs(a.v[2]), fabs(a.v[3]));
#endif
}


// permute lanes helper
static inline
mjtNum4 v4_shuffle_(mjtNum4 a, int i0, int i1, int i2, int i3) {
  return v4_set(a.v[i0], a.v[i1], a.v[i2], a.v[i3]);
}


// permute lanes with compile-time indices
#define v4_shuffle(a, i0, i1, i2, i3) v4_shuffle_(a, i0, i1, i2, i3)


// transpose 4x4 matrix of mjtNum4 rows in place
static inline
void v4_transpose(mjtNum4* r0, mjtNum4* r1, mjtNum4* r2, mjtNum4* r3) {
  mjtNum4 a = *r0, b = *r1, c = *r2, d = *r3;
  *r0 = v4_set(a.v[0], b.v[0], c.v[0], d.v[0]);
  *r1 = v4_set(a.v[1], b.v[1], c.v[1], d.v[1]);
  *r2 = v4_set(a.v[2], b.v[2], c.v[2], d.v[2]);
  *r3 = v4_set(a.v[3], b.v[3], c.v[3], d.v[3]);
}


// load and deinterleave 4 contiguous 3D mjtNum triplets into 3 vectors
static inline
void v4_load4x3(const mjtNum* p, mjtNum4* c0, mjtNum4* c1, mjtNum4* c2) {
  *c0 = v4_set(p[0], p[3], p[6], p[9]);
  *c1 = v4_set(p[1], p[4], p[7], p[10]);
  *c2 = v4_set(p[2], p[5], p[8], p[11]);
}


// load and deinterleave 4 contiguous 3D float32 vertices into 3 mjtNum4 vectors
static inline
void v4_load3_f32(const float* p, mjtNum4* vx, mjtNum4* vy, mjtNum4* vz) {
  *vx = v4_set((mjtNum)p[0], (mjtNum)p[3], (mjtNum)p[6], (mjtNum)p[9]);
  *vy = v4_set((mjtNum)p[1], (mjtNum)p[4], (mjtNum)p[7], (mjtNum)p[10]);
  *vz = v4_set((mjtNum)p[2], (mjtNum)p[5], (mjtNum)p[8], (mjtNum)p[11]);
}


// horizontal sum with fixed (l0+l2) + (l1+l3) reduction tree
static inline
mjtNum v4_sum(mjtNum4 a) {
  return (a.v[0] + a.v[2]) + (a.v[1] + a.v[3]);
}


// lane-wise fused multiply-add: a*b + c
static inline
mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
  return v4_set(mju_fma(a.v[0], b.v[0], c.v[0]),
                mju_fma(a.v[1], b.v[1], c.v[1]),
                mju_fma(a.v[2], b.v[2], c.v[2]),
                mju_fma(a.v[3], b.v[3], c.v[3]));
}


// load 4 floats from memory
static inline
mjtFloat4 f4_load(const float* p) {
  mjtFloat4 r;
  memcpy(r.v, p, sizeof(r.v));
  return r;
}


// store 4 floats to memory
static inline
void f4_store(float* p, mjtFloat4 a) {
  memcpy(p, a.v, sizeof(a.v));
}


// broadcast float scalar to all 4 lanes
static inline
mjtFloat4 f4_splat(float a) {
  mjtFloat4 r = {{a, a, a, a}};
  return r;
}


// lane-wise float32 less-than mask
static inline
mjtFloatMask4 f4_cmplt(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {{a.v[0] < b.v[0] ? ~0u : 0u, a.v[1] < b.v[1] ? ~0u : 0u,
                      a.v[2] < b.v[2] ? ~0u : 0u, a.v[3] < b.v[3] ? ~0u : 0u}};
  return r;
}


// lane-wise float32 less-or-equal mask
static inline
mjtFloatMask4 f4_cmple(mjtFloat4 a, mjtFloat4 b) {
  mjtFloatMask4 r = {{a.v[0] <= b.v[0] ? ~0u : 0u, a.v[1] <= b.v[1] ? ~0u : 0u,
                      a.v[2] <= b.v[2] ? ~0u : 0u, a.v[3] <= b.v[3] ? ~0u : 0u}};
  return r;
}


// bitwise AND of float32 masks
static inline
mjtFloatMask4 f4_and(mjtFloatMask4 a, mjtFloatMask4 b) {
  mjtFloatMask4 r = {{a.m[0] & b.m[0], a.m[1] & b.m[1], a.m[2] & b.m[2], a.m[3] & b.m[3]}};
  return r;
}


// extract 4-bit float32 lane mask in [0, 15]
static inline
int f4_movemask(mjtFloatMask4 m) {
  return (int)((m.m[0] & 1u) | (m.m[1] & 2u) | (m.m[2] & 4u) | (m.m[3] & 8u));
}

#endif  // backend selection

//------------------------------ backend-independent helpers ---------------------------------------

// linear combination of three vectors: c0*x0 + c1*x1 + c2*x2
static inline
mjtNum4 v4_lincomb3(mjtNum4 c0, mjtNum4 c1, mjtNum4 c2, mjtNum x0, mjtNum x1, mjtNum x2) {
  return v4_add(v4_add(v4_mul(c0, v4_splat(x0)), v4_mul(c1, v4_splat(x1))),
                v4_mul(c2, v4_splat(x2)));
}

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
