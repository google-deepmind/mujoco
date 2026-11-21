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

// Tests for engine_simd.h: bit-exact parity of all mjtNum4 backends against
// scalar.

#include <math.h>    // IWYU pragma: keep
#include <stdint.h>  // IWYU pragma: keep
#include <string.h>  // IWYU pragma: keep

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#if defined(__AVX__)
  #include <immintrin.h>
#endif

#include <gtest/gtest.h>
#include <mujoco/mjtype.h>
#include "test/fixture.h"

// Include engine_simd.h in isolated namespaces to test every compilable backend
// side-by-side against the plain-C scalar reference.

#define MJ_DEFINE_SHUFFLE_WRAPPERS()           \
  static inline mjtNum4 shuf_0123(mjtNum4 a) { \
    return v4_shuffle(a, 0, 1, 2, 3);          \
  }                                            \
  static inline mjtNum4 shuf_1032(mjtNum4 a) { \
    return v4_shuffle(a, 1, 0, 3, 2);          \
  }                                            \
  static inline mjtNum4 shuf_2301(mjtNum4 a) { \
    return v4_shuffle(a, 2, 3, 0, 1);          \
  }                                            \
  static inline mjtNum4 shuf_3210(mjtNum4 a) { \
    return v4_shuffle(a, 3, 2, 1, 0);          \
  }                                            \
  static inline mjtNum4 shuf_2232(mjtNum4 a) { \
    return v4_shuffle(a, 2, 2, 3, 2);          \
  }                                            \
  static inline mjtNum4 shuf_3323(mjtNum4 a) { \
    return v4_shuffle(a, 3, 3, 2, 3);          \
  }                                            \
  static inline mjtNum4 shuf_1212(mjtNum4 a) { \
    return v4_shuffle(a, 1, 2, 1, 2);          \
  }                                            \
  static inline mjtNum4 shuf_2121(mjtNum4 a) { \
    return v4_shuffle(a, 2, 1, 2, 1);          \
  }                                            \
  static inline mjtNum4 shuf_0000(mjtNum4 a) { \
    return v4_shuffle(a, 0, 0, 0, 0);          \
  }                                            \
  static inline mjtNum4 shuf_3333(mjtNum4 a) { \
    return v4_shuffle(a, 3, 3, 3, 3);          \
  }                                            \
  static inline mjtNum4 shuf_0213(mjtNum4 a) { \
    return v4_shuffle(a, 0, 2, 1, 3);          \
  }                                            \
  static inline mjtNum4 shuf_3021(mjtNum4 a) { \
    return v4_shuffle(a, 3, 0, 2, 1);          \
  }

namespace simd_default {
#include "src/engine/engine_simd.h"
MJ_DEFINE_SHUFFLE_WRAPPERS()
}  // namespace simd_default

// Verify default macro state before undefining for multi-backend instantiation.
#ifdef mjUSEFMA
  #error "mjUSEFMA must be disabled by default until P4"
#endif

#if defined(mjUSEPLATFORMSIMD) && !defined(mjUSESINGLE)
  #if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
    #ifndef mjVECTORIZE
      #error \
          "mjVECTORIZE must be defined by default on Clang/GCC double precision"
    #endif
  #endif
  #if defined(__AVX__)
    #ifndef mjUSEAVX
      #error \
          "mjUSEAVX must be defined when mjUSEPLATFORMSIMD and __AVX__ are active"
    #endif
  #endif
#endif

// Undefine include guard before re-including engine_simd.h.
#undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
#undef v4_shuffle
#undef mjVECTORIZE
#undef mjUSEAVX
#undef mjUSEFMA

namespace simd_scalar {
#define mjSCALAR
#include "src/engine/engine_simd.h"
MJ_DEFINE_SHUFFLE_WRAPPERS()
#undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
#undef v4_shuffle
#undef mjSCALAR
}  // namespace simd_scalar

#if (defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)) && \
    !defined(mjUSESINGLE)
  #define MJ_HAS_VECEXT_BACKEND 1
namespace simd_vecext {
  #define mjVECTORIZE
  #include "src/engine/engine_simd.h"
MJ_DEFINE_SHUFFLE_WRAPPERS()
  #undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
  #undef v4_shuffle
  #undef mjVECTORIZE
}  // namespace simd_vecext
#endif

#if defined(__AVX__) && !defined(mjUSESINGLE)
  #define MJ_HAS_AVX_BACKEND 1
namespace simd_avx {
  #define mjUSEAVX
  #include "src/engine/engine_simd.h"
MJ_DEFINE_SHUFFLE_WRAPPERS()
  #undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
  #undef v4_shuffle
  #undef mjUSEAVX
}  // namespace simd_avx
#endif

// FMA forced-on namespaces
namespace simd_scalar_fma_on {
#define mjSCALAR
#define mjUSEFMA
#include "src/engine/engine_simd.h"
#ifndef mjUSEFMA
  #error "mjUSEFMA must remain defined when forced on without mjNOFMA"
#endif
#undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
#undef v4_shuffle
#undef mjSCALAR
#undef mjUSEFMA
}  // namespace simd_scalar_fma_on

#ifdef MJ_HAS_VECEXT_BACKEND
namespace simd_vecext_fma_on {
  #define mjVECTORIZE
  #define mjUSEFMA
  #include "src/engine/engine_simd.h"
  #undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
  #undef v4_shuffle
  #undef mjVECTORIZE
  #undef mjUSEFMA
}  // namespace simd_vecext_fma_on
#endif

#ifdef MJ_HAS_AVX_BACKEND
namespace simd_avx_fma_on {
  #define mjUSEAVX
  #define mjUSEFMA
  #include "src/engine/engine_simd.h"
  #undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
  #undef v4_shuffle
  #undef mjUSEAVX
  #undef mjUSEFMA
}  // namespace simd_avx_fma_on
#endif

// FMA opt-out namespace (mjENABLEFMA + mjNOFMA -> mjUSEFMA must be undefined)
namespace simd_fma_optout {
#define mjSCALAR
#define mjENABLEFMA
#define mjNOFMA
#include "src/engine/engine_simd.h"
#ifdef mjUSEFMA
  #error "mjNOFMA must override mjENABLEFMA"
#endif
#undef MUJOCO_SRC_ENGINE_ENGINE_SIMD_H_
#undef v4_shuffle
#undef mjSCALAR
#undef mjENABLEFMA
#undef mjNOFMA
}  // namespace simd_fma_optout

namespace mujoco {
namespace {

using EngineSimdTest = MujocoTest;

struct Vec4 {
  mjtNum d[4];
};

std::vector<Vec4> BuildTestVectors() {
  using L = std::numeric_limits<mjtNum>;
  const mjtNum edge_vals[] = {
      mjtNum(0.0),
      mjtNum(-0.0),
      mjtNum(1.0),
      mjtNum(-1.0),
      mjtNum(0.5),
      mjtNum(-0.5),
      mjtNum(2.0),
      mjtNum(-2.0),
      L::min(),
      -L::min(),
      mjtNum(0.5) * L::min(),
      mjtNum(-0.5) * L::min(),
      L::denorm_min(),
      -L::denorm_min(),
      mjtNum(3.0) * L::denorm_min(),
      mjtNum(-7.0) * L::denorm_min(),
      L::max(),
      -L::max(),
      mjtNum(0.5) * L::max(),
      mjtNum(-0.5) * L::max(),
      L::infinity(),
      -L::infinity(),
      L::quiet_NaN(),
      -L::quiet_NaN(),
      mjtNum(1.0) + L::epsilon(),
      mjtNum(-1.0) - L::epsilon(),
      mjtNum(3.14159265358979323846),
      mjtNum(-2.71828182845904523536),
      mjtNum(1e-16),
      mjtNum(-1e-16),
  };
  constexpr int kNumEdge = sizeof(edge_vals) / sizeof(edge_vals[0]);

  std::vector<Vec4> vecs;
  vecs.reserve(kNumEdge * 2 + 256);

  // Strided combinations of edge values
  for (int i = 0; i < kNumEdge; ++i) {
    vecs.push_back(
        {{edge_vals[i], edge_vals[(i + 1) % kNumEdge],
          edge_vals[(i + 3) % kNumEdge], edge_vals[(i + 7) % kNumEdge]}});
    vecs.push_back(
        {{edge_vals[(i * 3) % kNumEdge], edge_vals[(i * 5 + 1) % kNumEdge],
          edge_vals[(i * 7 + 2) % kNumEdge],
          edge_vals[(i * 11 + 3) % kNumEdge]}});
  }

  // Deterministic pseudo-random normal and subnormal values
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<double> unif(-1e6, 1e6);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  for (int i = 0; i < 128; ++i) {
    vecs.push_back(
        {{static_cast<mjtNum>(unif(rng)), static_cast<mjtNum>(unif(rng)),
          static_cast<mjtNum>(unif(rng)), static_cast<mjtNum>(unif(rng))}});
    vecs.push_back(
        {{static_cast<mjtNum>(unit(rng)), static_cast<mjtNum>(unit(rng)),
          static_cast<mjtNum>(unit(rng)), static_cast<mjtNum>(unit(rng))}});
  }
  return vecs;
}

template <typename Backend>
void VerifyBackendParity(const std::vector<Vec4>& vecs) {
  const int n = static_cast<int>(vecs.size());
  for (int i = 0; i < n; ++i) {
    const mjtNum* a = vecs[i].d;
    const mjtNum* b = vecs[(i + 5) % n].d;
    const mjtNum* c = vecs[(i + 13) % n].d;

    auto sa = simd_scalar::v4_load(a);
    auto sb = simd_scalar::v4_load(b);
    auto sc = simd_scalar::v4_load(c);

    auto ba = Backend::v4_load(a);
    auto bb = Backend::v4_load(b);
    auto bc = Backend::v4_load(c);

    auto expect_vec_eq = [](simd_scalar::mjtNum4 sr,
                            typename Backend::mjtNum4 br, const char* op) {
      mjtNum so[4], bo[4];
      simd_scalar::v4_store(so, sr);
      Backend::v4_store(bo, br);
      EXPECT_EQ(std::memcmp(so, bo, sizeof(so)), 0) << "op: " << op;
    };

    // load / store / set / splat
    expect_vec_eq(sa, ba, "load_store");
    expect_vec_eq(simd_scalar::v4_set(a[0], a[1], a[2], a[3]),
                  Backend::v4_set(a[0], a[1], a[2], a[3]), "set");
    expect_vec_eq(simd_scalar::v4_set(b[3], a[2], c[1], b[0]),
                  Backend::v4_set(b[3], a[2], c[1], b[0]), "set_gather");
    for (int lane = 0; lane < 4; ++lane) {
      expect_vec_eq(simd_scalar::v4_splat(a[lane]), Backend::v4_splat(a[lane]),
                    "splat");
    }

    // add / sub / mul / fma
    expect_vec_eq(simd_scalar::v4_add(sa, sb), Backend::v4_add(ba, bb), "add");
    expect_vec_eq(simd_scalar::v4_sub(sa, sb), Backend::v4_sub(ba, bb), "sub");
    expect_vec_eq(simd_scalar::v4_mul(sa, sb), Backend::v4_mul(ba, bb), "mul");
    expect_vec_eq(simd_scalar::v4_fma(sa, sb, sc), Backend::v4_fma(ba, bb, bc),
                  "fma");

    // all 16 sign masks for v4_xorsign
    for (int mask = 0; mask < 16; ++mask) {
      int s0 = (mask >> 0) & 1;
      int s1 = (mask >> 1) & 1;
      int s2 = (mask >> 2) & 1;
      int s3 = (mask >> 3) & 1;
      expect_vec_eq(simd_scalar::v4_xorsign(sa, s0, s1, s2, s3),
                    Backend::v4_xorsign(ba, s0, s1, s2, s3), "xorsign");
    }

    // shuffles
    expect_vec_eq(simd_scalar::shuf_0123(sa), Backend::shuf_0123(ba),
                  "shuf_0123");
    expect_vec_eq(simd_scalar::shuf_1032(sa), Backend::shuf_1032(ba),
                  "shuf_1032");
    expect_vec_eq(simd_scalar::shuf_2301(sa), Backend::shuf_2301(ba),
                  "shuf_2301");
    expect_vec_eq(simd_scalar::shuf_3210(sa), Backend::shuf_3210(ba),
                  "shuf_3210");
    expect_vec_eq(simd_scalar::shuf_2232(sa), Backend::shuf_2232(ba),
                  "shuf_2232");
    expect_vec_eq(simd_scalar::shuf_3323(sa), Backend::shuf_3323(ba),
                  "shuf_3323");
    expect_vec_eq(simd_scalar::shuf_1212(sa), Backend::shuf_1212(ba),
                  "shuf_1212");
    expect_vec_eq(simd_scalar::shuf_2121(sa), Backend::shuf_2121(ba),
                  "shuf_2121");
    expect_vec_eq(simd_scalar::shuf_0000(sa), Backend::shuf_0000(ba),
                  "shuf_0000");
    expect_vec_eq(simd_scalar::shuf_3333(sa), Backend::shuf_3333(ba),
                  "shuf_3333");
    expect_vec_eq(simd_scalar::shuf_0213(sa), Backend::shuf_0213(ba),
                  "shuf_0213");
    expect_vec_eq(simd_scalar::shuf_3021(sa), Backend::shuf_3021(ba),
                  "shuf_3021");

    // horizontal sum: (l0+l2) + (l1+l3)
    mjtNum s_sum = simd_scalar::v4_sum(sa);
    mjtNum b_sum = Backend::v4_sum(ba);
    EXPECT_EQ(std::memcmp(&s_sum, &b_sum, sizeof(mjtNum)), 0) << "op: sum";

    // scalar mju_fma
    for (int lane = 0; lane < 4; ++lane) {
      mjtNum s_fma = simd_scalar::mju_fma(a[lane], b[lane], c[lane]);
      mjtNum b_fma = Backend::mju_fma(a[lane], b[lane], c[lane]);
      EXPECT_EQ(std::memcmp(&s_fma, &b_fma, sizeof(mjtNum)), 0)
          << "op: mju_fma";
    }
  }
}

struct DefaultBackendAdapter {
  using mjtNum4 = simd_default::mjtNum4;
  static mjtNum4 v4_load(const mjtNum* p) { return simd_default::v4_load(p); }
  static void v4_store(mjtNum* p, mjtNum4 a) { simd_default::v4_store(p, a); }
  static mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
    return simd_default::v4_set(a, b, c, d);
  }
  static mjtNum4 v4_splat(mjtNum a) { return simd_default::v4_splat(a); }
  static mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
    return simd_default::v4_add(a, b);
  }
  static mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
    return simd_default::v4_sub(a, b);
  }
  static mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
    return simd_default::v4_mul(a, b);
  }
  static mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
    return simd_default::v4_xorsign(a, s0, s1, s2, s3);
  }
  static mjtNum4 shuf_0123(mjtNum4 a) { return simd_default::shuf_0123(a); }
  static mjtNum4 shuf_1032(mjtNum4 a) { return simd_default::shuf_1032(a); }
  static mjtNum4 shuf_2301(mjtNum4 a) { return simd_default::shuf_2301(a); }
  static mjtNum4 shuf_3210(mjtNum4 a) { return simd_default::shuf_3210(a); }
  static mjtNum4 shuf_2232(mjtNum4 a) { return simd_default::shuf_2232(a); }
  static mjtNum4 shuf_3323(mjtNum4 a) { return simd_default::shuf_3323(a); }
  static mjtNum4 shuf_1212(mjtNum4 a) { return simd_default::shuf_1212(a); }
  static mjtNum4 shuf_2121(mjtNum4 a) { return simd_default::shuf_2121(a); }
  static mjtNum4 shuf_0000(mjtNum4 a) { return simd_default::shuf_0000(a); }
  static mjtNum4 shuf_3333(mjtNum4 a) { return simd_default::shuf_3333(a); }
  static mjtNum4 shuf_0213(mjtNum4 a) { return simd_default::shuf_0213(a); }
  static mjtNum4 shuf_3021(mjtNum4 a) { return simd_default::shuf_3021(a); }
  static mjtNum v4_sum(mjtNum4 a) { return simd_default::v4_sum(a); }
  static mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
    return simd_default::v4_fma(a, b, c);
  }
  static mjtNum mju_fma(mjtNum a, mjtNum b, mjtNum c) {
    return simd_default::mju_fma(a, b, c);
  }
};

#ifdef MJ_HAS_VECEXT_BACKEND
struct VecextBackendAdapter {
  using mjtNum4 = simd_vecext::mjtNum4;
  static mjtNum4 v4_load(const mjtNum* p) { return simd_vecext::v4_load(p); }
  static void v4_store(mjtNum* p, mjtNum4 a) { simd_vecext::v4_store(p, a); }
  static mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
    return simd_vecext::v4_set(a, b, c, d);
  }
  static mjtNum4 v4_splat(mjtNum a) { return simd_vecext::v4_splat(a); }
  static mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) {
    return simd_vecext::v4_add(a, b);
  }
  static mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) {
    return simd_vecext::v4_sub(a, b);
  }
  static mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) {
    return simd_vecext::v4_mul(a, b);
  }
  static mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
    return simd_vecext::v4_xorsign(a, s0, s1, s2, s3);
  }
  static mjtNum4 shuf_0123(mjtNum4 a) { return simd_vecext::shuf_0123(a); }
  static mjtNum4 shuf_1032(mjtNum4 a) { return simd_vecext::shuf_1032(a); }
  static mjtNum4 shuf_2301(mjtNum4 a) { return simd_vecext::shuf_2301(a); }
  static mjtNum4 shuf_3210(mjtNum4 a) { return simd_vecext::shuf_3210(a); }
  static mjtNum4 shuf_2232(mjtNum4 a) { return simd_vecext::shuf_2232(a); }
  static mjtNum4 shuf_3323(mjtNum4 a) { return simd_vecext::shuf_3323(a); }
  static mjtNum4 shuf_1212(mjtNum4 a) { return simd_vecext::shuf_1212(a); }
  static mjtNum4 shuf_2121(mjtNum4 a) { return simd_vecext::shuf_2121(a); }
  static mjtNum4 shuf_0000(mjtNum4 a) { return simd_vecext::shuf_0000(a); }
  static mjtNum4 shuf_3333(mjtNum4 a) { return simd_vecext::shuf_3333(a); }
  static mjtNum4 shuf_0213(mjtNum4 a) { return simd_vecext::shuf_0213(a); }
  static mjtNum4 shuf_3021(mjtNum4 a) { return simd_vecext::shuf_3021(a); }
  static mjtNum v4_sum(mjtNum4 a) { return simd_vecext::v4_sum(a); }
  static mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
    return simd_vecext::v4_fma(a, b, c);
  }
  static mjtNum mju_fma(mjtNum a, mjtNum b, mjtNum c) {
    return simd_vecext::mju_fma(a, b, c);
  }
};
#endif

#ifdef MJ_HAS_AVX_BACKEND
struct AvxBackendAdapter {
  using mjtNum4 = simd_avx::mjtNum4;
  static mjtNum4 v4_load(const mjtNum* p) { return simd_avx::v4_load(p); }
  static void v4_store(mjtNum* p, mjtNum4 a) { simd_avx::v4_store(p, a); }
  static mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {
    return simd_avx::v4_set(a, b, c, d);
  }
  static mjtNum4 v4_splat(mjtNum a) { return simd_avx::v4_splat(a); }
  static mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) { return simd_avx::v4_add(a, b); }
  static mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) { return simd_avx::v4_sub(a, b); }
  static mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) { return simd_avx::v4_mul(a, b); }
  static mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {
    return simd_avx::v4_xorsign(a, s0, s1, s2, s3);
  }
  static mjtNum4 shuf_0123(mjtNum4 a) { return simd_avx::shuf_0123(a); }
  static mjtNum4 shuf_1032(mjtNum4 a) { return simd_avx::shuf_1032(a); }
  static mjtNum4 shuf_2301(mjtNum4 a) { return simd_avx::shuf_2301(a); }
  static mjtNum4 shuf_3210(mjtNum4 a) { return simd_avx::shuf_3210(a); }
  static mjtNum4 shuf_2232(mjtNum4 a) { return simd_avx::shuf_2232(a); }
  static mjtNum4 shuf_3323(mjtNum4 a) { return simd_avx::shuf_3323(a); }
  static mjtNum4 shuf_1212(mjtNum4 a) { return simd_avx::shuf_1212(a); }
  static mjtNum4 shuf_2121(mjtNum4 a) { return simd_avx::shuf_2121(a); }
  static mjtNum4 shuf_0000(mjtNum4 a) { return simd_avx::shuf_0000(a); }
  static mjtNum4 shuf_3333(mjtNum4 a) { return simd_avx::shuf_3333(a); }
  static mjtNum4 shuf_0213(mjtNum4 a) { return simd_avx::shuf_0213(a); }
  static mjtNum4 shuf_3021(mjtNum4 a) { return simd_avx::shuf_3021(a); }
  static mjtNum v4_sum(mjtNum4 a) { return simd_avx::v4_sum(a); }
  static mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {
    return simd_avx::v4_fma(a, b, c);
  }
  static mjtNum mju_fma(mjtNum a, mjtNum b, mjtNum c) {
    return simd_avx::mju_fma(a, b, c);
  }
};
#endif

TEST_F(EngineSimdTest, DefaultBackendMatchesScalar) {
  VerifyBackendParity<DefaultBackendAdapter>(BuildTestVectors());
}

#ifdef MJ_HAS_VECEXT_BACKEND
TEST_F(EngineSimdTest, VectorExtensionsBackendMatchesScalar) {
  VerifyBackendParity<VecextBackendAdapter>(BuildTestVectors());
}
#endif

#ifdef MJ_HAS_AVX_BACKEND
TEST_F(EngineSimdTest, AvxIntrinsicsBackendMatchesScalar) {
  VerifyBackendParity<AvxBackendAdapter>(BuildTestVectors());
}
#endif

TEST_F(EngineSimdTest, SumReductionTreeIsPairwise02Plus13) {
  // Construct a vector where (l0 + l2) + (l1 + l3) differs by 1 ULP from
  // ((l0 + l1) + l2) + l3 and (l0 + l1) + (l2 + l3).
  const mjtNum eps = std::numeric_limits<mjtNum>::epsilon();
  const mjtNum half_eps = mjtNum(0.5) * eps;
  const mjtNum v[4] = {mjtNum(1.0), half_eps, mjtNum(-1.0), half_eps};

  const mjtNum expected = (v[0] + v[2]) + (v[1] + v[3]);  // 0 + eps = eps
  const mjtNum sum_01_23 = (v[0] + v[1]) + (v[2] + v[3]);
  EXPECT_NE(expected, sum_01_23);

  mjtNum s_res = simd_scalar::v4_sum(simd_scalar::v4_load(v));
  mjtNum d_res = simd_default::v4_sum(simd_default::v4_load(v));
  EXPECT_EQ(std::memcmp(&s_res, &expected, sizeof(mjtNum)), 0);
  EXPECT_EQ(std::memcmp(&d_res, &expected, sizeof(mjtNum)), 0);

#ifdef MJ_HAS_VECEXT_BACKEND
  mjtNum v_res = simd_vecext::v4_sum(simd_vecext::v4_load(v));
  EXPECT_EQ(std::memcmp(&v_res, &expected, sizeof(mjtNum)), 0);
#endif

#ifdef MJ_HAS_AVX_BACKEND
  mjtNum a_res = simd_avx::v4_sum(simd_avx::v4_load(v));
  EXPECT_EQ(std::memcmp(&a_res, &expected, sizeof(mjtNum)), 0);
#endif
}

TEST_F(EngineSimdTest, FmaForcedOnAndOffParity) {
  const auto vecs = BuildTestVectors();
  const int n = static_cast<int>(vecs.size());

  for (int i = 0; i < n; ++i) {
    const mjtNum* a = vecs[i].d;
    const mjtNum* b = vecs[(i + 5) % n].d;
    const mjtNum* c = vecs[(i + 13) % n].d;

    mjtNum ref_on[4], ref_off[4];
    simd_scalar_fma_on::v4_store(
        ref_on, simd_scalar_fma_on::v4_fma(simd_scalar_fma_on::v4_load(a),
                                           simd_scalar_fma_on::v4_load(b),
                                           simd_scalar_fma_on::v4_load(c)));
    simd_fma_optout::v4_store(
        ref_off, simd_fma_optout::v4_fma(simd_fma_optout::v4_load(a),
                                         simd_fma_optout::v4_load(b),
                                         simd_fma_optout::v4_load(c)));

    mjtNum def_out[4];
    simd_default::v4_store(
        def_out,
        simd_default::v4_fma(simd_default::v4_load(a), simd_default::v4_load(b),
                             simd_default::v4_load(c)));
    EXPECT_EQ(std::memcmp(def_out, ref_off, sizeof(ref_off)), 0);

#ifdef MJ_HAS_VECEXT_BACKEND
    mjtNum vec_on[4];
    simd_vecext_fma_on::v4_store(
        vec_on, simd_vecext_fma_on::v4_fma(simd_vecext_fma_on::v4_load(a),
                                           simd_vecext_fma_on::v4_load(b),
                                           simd_vecext_fma_on::v4_load(c)));
    EXPECT_EQ(std::memcmp(vec_on, ref_on, sizeof(ref_on)), 0);
#endif

#ifdef MJ_HAS_AVX_BACKEND
    mjtNum avx_on[4];
    simd_avx_fma_on::v4_store(
        avx_on, simd_avx_fma_on::v4_fma(simd_avx_fma_on::v4_load(a),
                                        simd_avx_fma_on::v4_load(b),
                                        simd_avx_fma_on::v4_load(c)));
    EXPECT_EQ(std::memcmp(avx_on, ref_on, sizeof(ref_on)), 0);
#endif
  }

  // Verify single-rounding FMA vs two-rounding mul+add on a classic
  // cancellation
  const mjtNum delta =
      std::ldexp(mjtNum(1.0), -(std::numeric_limits<mjtNum>::digits / 2 + 1));
  const mjtNum x = mjtNum(1.0) + delta;
  const mjtNum neg_c = -(mjtNum(1.0) + mjtNum(2.0) * delta);

  const mjtNum fma_on_val = simd_scalar_fma_on::mju_fma(x, x, neg_c);
  const mjtNum fma_off_val = simd_fma_optout::mju_fma(x, x, neg_c);
  EXPECT_EQ(fma_on_val, delta * delta);
  EXPECT_EQ(fma_off_val, mjtNum(0.0));

  mjtNum cancel_out[4];
  simd_default::v4_store(
      cancel_out,
      simd_default::v4_fma(simd_default::v4_splat(x), simd_default::v4_splat(x),
                           simd_default::v4_splat(neg_c)));
  for (int lane = 0; lane < 4; ++lane) {
    EXPECT_EQ(cancel_out[lane], mjtNum(0.0));
  }

  simd_scalar_fma_on::v4_store(
      cancel_out,
      simd_scalar_fma_on::v4_fma(simd_scalar_fma_on::v4_splat(x),
                                 simd_scalar_fma_on::v4_splat(x),
                                 simd_scalar_fma_on::v4_splat(neg_c)));
  for (int lane = 0; lane < 4; ++lane) {
    EXPECT_EQ(cancel_out[lane], delta * delta);
  }

#ifdef MJ_HAS_VECEXT_BACKEND
  simd_vecext_fma_on::v4_store(
      cancel_out,
      simd_vecext_fma_on::v4_fma(simd_vecext_fma_on::v4_splat(x),
                                 simd_vecext_fma_on::v4_splat(x),
                                 simd_vecext_fma_on::v4_splat(neg_c)));
  for (int lane = 0; lane < 4; ++lane) {
    EXPECT_EQ(cancel_out[lane], delta * delta);
  }
#endif

#ifdef MJ_HAS_AVX_BACKEND
  simd_avx_fma_on::v4_store(
      cancel_out, simd_avx_fma_on::v4_fma(simd_avx_fma_on::v4_splat(x),
                                          simd_avx_fma_on::v4_splat(x),
                                          simd_avx_fma_on::v4_splat(neg_c)));
  for (int lane = 0; lane < 4; ++lane) {
    EXPECT_EQ(cancel_out[lane], delta * delta);
  }
#endif
}

}  // namespace
}  // namespace mujoco
