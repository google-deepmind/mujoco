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

#include <stdint.h>
#include <string.h>

#include <mujoco/mjtype.h>


//------------------------------ backend and FMA configuration -------------------------------------

// select SIMD backend when mjUSEPLATFORMSIMD is enabled in double precision
#if defined(mjUSEPLATFORMSIMD) && !defined(mjUSESINGLE) && \
    !defined(mjSCALAR) && !defined(mjVECTORIZE) && !defined(mjUSEAVX)
  #if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
    #define mjVECTORIZE
  #endif
  #if defined(__AVX__)
    #define mjUSEAVX
  #endif
#endif

#if defined(mjUSEAVX) && !defined(mjUSESINGLE)
  #include <immintrin.h>  // IWYU pragma: export
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

#ifdef mjUSEFMA
  #include <math.h>
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

#if defined(mjVECTORIZE) && !defined(mjUSESINGLE)

#if defined(__clang__) && defined(__AVX__)
  #define mjSIMD_ATTR static inline __attribute__((__min_vector_width__(256)))
#else
  #define mjSIMD_ATTR static inline
#endif

typedef double mjtNum4_f64 __attribute__((vector_size(32)));
typedef uint64_t mjtNum4_u64 __attribute__((vector_size(32)));
typedef struct { mjtNum4_f64 v; } mjtNum4;


// wrap raw vector in mjtNum4
mjSIMD_ATTR
mjtNum4 v4_wrap_(mjtNum4_f64 v) {
  mjtNum4 r;
  r.v = v;
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


// set 4 lanes (also used for gathers)
mjSIMD_ATTR
mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
  mjtNum4_f64 v = {a, b, c, d};
  return v4_wrap_(v);
}


// broadcast scalar to all 4 lanes
mjSIMD_ATTR
mjtNum4 v4_splat(mjtNum a) {
  mjtNum4_f64 v = {a, a, a, a};
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


// negate selected lanes by XORing sign bits
mjSIMD_ATTR
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  const uint64_t sign = (uint64_t)1 << 63;
  mjtNum4_u64 m = {s0 ? sign : 0, s1 ? sign : 0, s2 ? sign : 0, s3 ? sign : 0};
  return v4_wrap_((mjtNum4_f64)((mjtNum4_u64)a.v ^ m));
}


// permute lanes with compile-time indices; a is evaluated multiple times and must be
// side-effect-free
#define v4_shuffle(a, i0, i1, i2, i3) \
  v4_wrap_(__builtin_shufflevector((a).v, (a).v, i0, i1, i2, i3))


// horizontal sum with fixed (l0+l2) + (l1+l3) reduction tree
mjSIMD_ATTR
mjtNum v4_sum(mjtNum4 a) {
  mjtNum4_f64 s = a.v + __builtin_shufflevector(a.v, a.v, 2, 3, 0, 1);
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
  mjtNum4_f64 prod = a.v * b.v;
  return v4_wrap_(prod + c.v);
#endif
}

#undef mjSIMD_ATTR


//------------------------------ AVX intrinsics backend (MSVC) -------------------------------------

#elif defined(mjUSEAVX) && !defined(mjUSESINGLE)

typedef struct { __m256d v; } mjtNum4;


// wrap __m256d in mjtNum4
static inline
mjtNum4 v4_wrap_(__m256d v) {
  mjtNum4 r;
  r.v = v;
  return r;
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


// negate selected lanes by XORing sign bits
static inline
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  __m256d m = _mm256_setr_pd(s0 ? -0.0 : 0.0, s1 ? -0.0 : 0.0,
                             s2 ? -0.0 : 0.0, s3 ? -0.0 : 0.0);
  return v4_wrap_(_mm256_xor_pd(a.v, m));
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


// permute lanes with compile-time indices (AVX1-compatible); a is evaluated multiple times and
// must be side-effect-free
#define v4_shuffle(a, i0, i1, i2, i3) \
  v4_combine_( \
    _mm_shuffle_pd(v4_half_(a, (i0) >= 2), v4_half_(a, (i1) >= 2), \
                   ((i0) & 1) | (((i1) & 1) << 1)), \
    _mm_shuffle_pd(v4_half_(a, (i2) >= 2), v4_half_(a, (i3) >= 2), \
                   ((i2) & 1) | (((i3) & 1) << 1)))


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


//------------------------------ plain-C scalar reference backend ----------------------------------

#else

typedef struct { mjtNum v[4]; } mjtNum4;


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


// negate selected lanes by XORing sign bits
static inline
mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
  return v4_set(s0 ? -a.v[0] : a.v[0], s1 ? -a.v[1] : a.v[1],
                s2 ? -a.v[2] : a.v[2], s3 ? -a.v[3] : a.v[3]);
}


// permute lanes helper
static inline
mjtNum4 v4_shuffle_(mjtNum4 a, int i0, int i1, int i2, int i3) {
  return v4_set(a.v[i0], a.v[i1], a.v[i2], a.v[i3]);
}


// permute lanes with compile-time indices
#define v4_shuffle(a, i0, i1, i2, i3) v4_shuffle_(a, i0, i1, i2, i3)


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

#endif  // backend selection

#endif  // MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
