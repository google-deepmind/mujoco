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

// Tests for engine_simd.h: bit-exact parity of all mjtNum4 and mjtFloat4
// backends against scalar.

#include <math.h>    // IWYU pragma: keep
#include <stdint.h>  // IWYU pragma: keep
#include <string.h>  // IWYU pragma: keep

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#if defined(__AVX__)
  #include <immintrin.h>
#elif defined(__SSE2__) || defined(_M_X64)
  #include <emmintrin.h>
  #include <xmmintrin.h>
#elif defined(__ARM_NEON) && (defined(__aarch64__) || defined(_M_ARM64))
  #include <arm_neon.h>
#elif defined(__wasm_simd128__)
  #include <wasm_simd128.h>
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

#if defined(mjUSEPLATFORMSIMD)
  #if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
    #ifndef mjVECTORIZE
      #error \
          "mjVECTORIZE must be defined by default on Clang/GCC with mjUSEPLATFORMSIMD"
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

#if defined(__clang__) || (defined(__GNUC__) && __GNUC__ >= 12)
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

#if defined(__AVX__)
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

struct FloatVec4 {
  float d[4];
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
      mjtNum(1e-7),
      mjtNum(-1e-7),
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

std::vector<FloatVec4> BuildFloatTestVectors() {
  using L = std::numeric_limits<float>;
  const float edge_vals[] = {
      0.0f,
      -0.0f,
      1.0f,
      -1.0f,
      0.5f,
      -0.5f,
      2.0f,
      -2.0f,
      L::min(),
      -L::min(),
      L::denorm_min(),
      -L::denorm_min(),
      L::max(),
      -L::max(),
      L::infinity(),
      -L::infinity(),
      L::quiet_NaN(),
      -L::quiet_NaN(),
      1.0f + L::epsilon(),
      -1.0f - L::epsilon(),
      3.1415927f,
      -2.7182817f,
      1e-7f,
      -1e-7f,
  };
  constexpr int kNumEdge = sizeof(edge_vals) / sizeof(edge_vals[0]);

  std::vector<FloatVec4> vecs;
  vecs.reserve(kNumEdge * 2 + 64);
  for (int i = 0; i < kNumEdge; ++i) {
    vecs.push_back(
        {{edge_vals[i], edge_vals[(i + 1) % kNumEdge],
          edge_vals[(i + 3) % kNumEdge], edge_vals[(i + 5) % kNumEdge]}});
    vecs.push_back(
        {{edge_vals[(i * 3) % kNumEdge], edge_vals[(i * 5 + 1) % kNumEdge],
          edge_vals[(i * 7 + 2) % kNumEdge],
          edge_vals[(i * 11 + 3) % kNumEdge]}});
  }

  std::mt19937 rng(123);
  std::uniform_real_distribution<float> unif(-1e4f, 1e4f);
  for (int i = 0; i < 64; ++i) {
    vecs.push_back({{unif(rng), unif(rng), unif(rng), unif(rng)}});
  }
  return vecs;
}

template <typename Backend>
void VerifyBackendParity(const std::vector<Vec4>& vecs) {
  const int n = static_cast<int>(vecs.size());
  std::vector<mjtNum> tight3_in(3), tight3_s(3), tight3_b(3);
  std::vector<mjtNum> tight12_num(12);

  for (int i = 0; i < n; ++i) {
    const mjtNum* a = vecs[i].d;
    const mjtNum* b = vecs[(i + 5) % n].d;
    const mjtNum* c = vecs[(i + 13) % n].d;
    const mjtNum* d = vecs[(i + 29) % n].d;

    auto sa = simd_scalar::v4_load(a);
    auto sb = simd_scalar::v4_load(b);
    auto sc = simd_scalar::v4_load(c);
    auto sd = simd_scalar::v4_load(d);

    auto ba = Backend::v4_load(a);
    auto bb = Backend::v4_load(b);
    auto bc = Backend::v4_load(c);
    auto bd = Backend::v4_load(d);

    auto expect_vec_eq = [](simd_scalar::mjtNum4 sr,
                            typename Backend::mjtNum4 br, const char* op) {
      mjtNum so[4], bo[4];
      simd_scalar::v4_store(so, sr);
      Backend::v4_store(bo, br);
      EXPECT_EQ(std::memcmp(so, bo, sizeof(so)), 0) << "op: " << op;
    };

    auto expect_vec_nan_eq = [](simd_scalar::mjtNum4 sr,
                                typename Backend::mjtNum4 br, const char* op) {
      mjtNum so[4], bo[4];
      simd_scalar::v4_store(so, sr);
      Backend::v4_store(bo, br);
      for (int lane = 0; lane < 4; ++lane) {
        if (std::isnan(so[lane])) {
          EXPECT_TRUE(std::isnan(bo[lane])) << "op: " << op;
        } else {
          EXPECT_EQ(std::memcmp(&so[lane], &bo[lane], sizeof(mjtNum)), 0)
              << "op: " << op;
        }
      }
    };

    auto expect_mask_eq = [&](simd_scalar::mjtMask4 sm,
                              typename Backend::mjtMask4 bm, const char* op) {
      int s_bits = simd_scalar::v4_movemask(sm);
      int b_bits = Backend::v4_movemask(bm);
      EXPECT_EQ(s_bits, b_bits) << "op: " << op;
      EXPECT_EQ(simd_scalar::v4_any(sm), Backend::v4_any(bm)) << "op: " << op;
      EXPECT_EQ(simd_scalar::v4_all(sm), Backend::v4_all(bm)) << "op: " << op;
      expect_vec_eq(simd_scalar::v4_select(sm, sa, sb),
                    Backend::v4_select(bm, ba, bb), op);
    };

    // load / store / load3 / store3 / set / splat
    expect_vec_eq(sa, ba, "load_store");
    for (int k = 0; k < 3; ++k) {
      tight3_in[k] = a[k];
    }
    auto sa3 = simd_scalar::v4_load3(tight3_in.data());
    auto ba3 = Backend::v4_load3(tight3_in.data());
    expect_vec_eq(sa3, ba3, "load3");
    simd_scalar::v4_store3(tight3_s.data(), sa);
    Backend::v4_store3(tight3_b.data(), ba);
    EXPECT_EQ(std::memcmp(tight3_s.data(), tight3_b.data(), 3 * sizeof(mjtNum)),
              0)
        << "op: store3";

    expect_vec_eq(simd_scalar::v4_set(a[0], a[1], a[2], a[3]),
                  Backend::v4_set(a[0], a[1], a[2], a[3]), "set");
    expect_vec_eq(simd_scalar::v4_set(b[3], a[2], c[1], b[0]),
                  Backend::v4_set(b[3], a[2], c[1], b[0]), "set_gather");
    for (int lane = 0; lane < 4; ++lane) {
      expect_vec_eq(simd_scalar::v4_splat(a[lane]), Backend::v4_splat(a[lane]),
                    "splat");
    }

    // add / sub / mul / div / min / max / sqrt / fma
    expect_vec_eq(simd_scalar::v4_add(sa, sb), Backend::v4_add(ba, bb), "add");
    expect_vec_eq(simd_scalar::v4_sub(sa, sb), Backend::v4_sub(ba, bb), "sub");
    expect_vec_eq(simd_scalar::v4_mul(sa, sb), Backend::v4_mul(ba, bb), "mul");
    expect_vec_nan_eq(simd_scalar::v4_div(sa, sb), Backend::v4_div(ba, bb),
                      "div");
    expect_vec_eq(simd_scalar::v4_min(sa, sb), Backend::v4_min(ba, bb), "min");
    expect_vec_eq(simd_scalar::v4_max(sa, sb), Backend::v4_max(ba, bb), "max");
    expect_vec_eq(simd_scalar::v4_abs(sa), Backend::v4_abs(ba), "abs");
    expect_vec_nan_eq(simd_scalar::v4_sqrt(sa), Backend::v4_sqrt(ba), "sqrt");
    expect_vec_eq(simd_scalar::v4_fma(sa, sb, sc), Backend::v4_fma(ba, bb, bc),
                  "fma");

    // comparisons, bitwise mask ops, select, movemask, any, all
    auto slt = simd_scalar::v4_cmplt(sa, sb);
    auto sle = simd_scalar::v4_cmple(sa, sb);
    auto sgt = simd_scalar::v4_cmpgt(sa, sc);
    auto sge = simd_scalar::v4_cmpge(sa, sc);
    auto seq = simd_scalar::v4_cmpeq(sa, sb);

    auto blt = Backend::v4_cmplt(ba, bb);
    auto ble = Backend::v4_cmple(ba, bb);
    auto bgt = Backend::v4_cmpgt(ba, bc);
    auto bge = Backend::v4_cmpge(ba, bc);
    auto beq = Backend::v4_cmpeq(ba, bb);

    expect_mask_eq(slt, blt, "cmplt");
    expect_mask_eq(sle, ble, "cmple");
    expect_mask_eq(sgt, bgt, "cmpgt");
    expect_mask_eq(sge, bge, "cmpge");
    expect_mask_eq(seq, beq, "cmpeq");
    expect_mask_eq(simd_scalar::v4_and(sle, sgt), Backend::v4_and(ble, bgt),
                   "and");
    expect_mask_eq(simd_scalar::v4_or(slt, seq), Backend::v4_or(blt, beq),
                   "or");
    expect_mask_eq(simd_scalar::v4_xor(sle, sge), Backend::v4_xor(ble, bge),
                   "xor");
    expect_mask_eq(simd_scalar::v4_andnot(slt, sle),
                   Backend::v4_andnot(blt, ble), "andnot");
    expect_mask_eq(simd_scalar::v4_not(slt), Backend::v4_not(blt), "not");

    // all 16 explicit mask patterns
    const auto zero_s = simd_scalar::v4_splat(0);
    const auto zero_b = Backend::v4_splat(0);
    for (int mask = 0; mask < 16; ++mask) {
      auto mv_s = simd_scalar::v4_set((mask & 1) ? 1 : 0, (mask & 2) ? 1 : 0,
                                      (mask & 4) ? 1 : 0, (mask & 8) ? 1 : 0);
      auto mv_b = Backend::v4_set((mask & 1) ? 1 : 0, (mask & 2) ? 1 : 0,
                                  (mask & 4) ? 1 : 0, (mask & 8) ? 1 : 0);
      auto sm = simd_scalar::v4_cmpgt(mv_s, zero_s);
      auto bm = Backend::v4_cmpgt(mv_b, zero_b);
      EXPECT_EQ(simd_scalar::v4_movemask(sm), mask);
      EXPECT_EQ(Backend::v4_movemask(bm), mask);
      EXPECT_EQ(Backend::v4_any(bm), mask != 0 ? 1 : 0);
      EXPECT_EQ(Backend::v4_all(bm), mask == 15 ? 1 : 0);
      expect_vec_eq(simd_scalar::v4_select(sm, sa, sb),
                    Backend::v4_select(bm, ba, bb), "select_mask16");
    }

    // all 16 sign masks for v4_xorsign
    for (int mask = 0; mask < 16; ++mask) {
      int s0 = (mask >> 0) & 1;
      int s1 = (mask >> 1) & 1;
      int s2 = (mask >> 2) & 1;
      int s3 = (mask >> 3) & 1;
      expect_vec_eq(simd_scalar::v4_xorsign(sa, s0, s1, s2, s3),
                    Backend::v4_xorsign(ba, s0, s1, s2, s3), "xorsign");
    }

    // transpose 4x4
    auto sr0 = sa, sr1 = sb, sr2 = sc, sr3 = sd;
    auto br0 = ba, br1 = bb, br2 = bc, br3 = bd;
    simd_scalar::v4_transpose(&sr0, &sr1, &sr2, &sr3);
    Backend::v4_transpose(&br0, &br1, &br2, &br3);
    expect_vec_eq(sr0, br0, "transpose_r0");
    expect_vec_eq(sr1, br1, "transpose_r1");
    expect_vec_eq(sr2, br2, "transpose_r2");
    expect_vec_eq(sr3, br3, "transpose_r3");

    // load4x3 from exact-sized 12-element buffer
    for (int k = 0; k < 4; ++k) {
      tight12_num[k] = a[k];
      tight12_num[4 + k] = b[k];
      tight12_num[8 + k] = c[k];
    }
    simd_scalar::mjtNum4 sc0, sc1, sc2;
    typename Backend::mjtNum4 bc0, bc1, bc2;
    simd_scalar::v4_load4x3(tight12_num.data(), &sc0, &sc1, &sc2);
    Backend::v4_load4x3(tight12_num.data(), &bc0, &bc1, &bc2);
    expect_vec_eq(sc0, bc0, "load4x3_c0");
    expect_vec_eq(sc1, bc1, "load4x3_c1");
    expect_vec_eq(sc2, bc2, "load4x3_c2");

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

  // Verify v4_load3_f32 and mjtFloat4 / mjtFloatMask4 ops
  const auto fvecs = BuildFloatTestVectors();
  const int fn = static_cast<int>(fvecs.size());
  std::vector<float> tight12_f32(12);
  for (int i = 0; i < fn; ++i) {
    const float* fa = fvecs[i].d;
    const float* fb = fvecs[(i + 3) % fn].d;
    const float* fc = fvecs[(i + 7) % fn].d;

    for (int k = 0; k < 4; ++k) {
      tight12_f32[k] = fa[k];
      tight12_f32[4 + k] = fb[k];
      tight12_f32[8 + k] = fc[k];
    }
    simd_scalar::mjtNum4 svx, svy, svz;
    typename Backend::mjtNum4 bvx, bvy, bvz;
    simd_scalar::v4_load3_f32(tight12_f32.data(), &svx, &svy, &svz);
    Backend::v4_load3_f32(tight12_f32.data(), &bvx, &bvy, &bvz);

    auto check_f32_widen = [](simd_scalar::mjtNum4 sr,
                              typename Backend::mjtNum4 br, const char* op) {
      mjtNum so[4], bo[4];
      simd_scalar::v4_store(so, sr);
      Backend::v4_store(bo, br);
      for (int lane = 0; lane < 4; ++lane) {
        if (std::isnan(so[lane])) {
          EXPECT_TRUE(std::isnan(bo[lane])) << "op: " << op;
        } else {
          EXPECT_EQ(std::memcmp(&so[lane], &bo[lane], sizeof(mjtNum)), 0)
              << "op: " << op;
        }
      }
    };
    check_f32_widen(svx, bvx, "load3_f32_x");
    check_f32_widen(svy, bvy, "load3_f32_y");
    check_f32_widen(svz, bvz, "load3_f32_z");

    auto sfa = simd_scalar::f4_load(fa);
    auto sfb = simd_scalar::f4_load(fb);
    auto bfa = Backend::f4_load(fa);
    auto bfb = Backend::f4_load(fb);

    float sfo[4], bfo[4];
    simd_scalar::f4_store(sfo, sfa);
    Backend::f4_store(bfo, bfa);
    EXPECT_EQ(std::memcmp(sfo, bfo, sizeof(sfo)), 0) << "op: f4_load_store";

    for (int lane = 0; lane < 4; ++lane) {
      simd_scalar::f4_store(sfo, simd_scalar::f4_splat(fa[lane]));
      Backend::f4_store(bfo, Backend::f4_splat(fa[lane]));
      EXPECT_EQ(std::memcmp(sfo, bfo, sizeof(sfo)), 0) << "op: f4_splat";
    }

    auto sflt = simd_scalar::f4_cmplt(sfa, sfb);
    auto bflt = Backend::f4_cmplt(bfa, bfb);
    EXPECT_EQ(simd_scalar::f4_movemask(sflt), Backend::f4_movemask(bflt))
        << "op: f4_cmplt";

    auto sfle = simd_scalar::f4_cmple(sfa, sfb);
    auto bfle = Backend::f4_cmple(bfa, bfb);
    EXPECT_EQ(simd_scalar::f4_movemask(sfle), Backend::f4_movemask(bfle))
        << "op: f4_cmple";

    auto sfc = simd_scalar::f4_load(fc);
    auto bfc = Backend::f4_load(fc);
    auto sfand = simd_scalar::f4_and(sflt, simd_scalar::f4_cmple(sfa, sfc));
    auto bfand = Backend::f4_and(bflt, Backend::f4_cmple(bfa, bfc));
    EXPECT_EQ(simd_scalar::f4_movemask(sfand), Backend::f4_movemask(bfand))
        << "op: f4_and";
  }

  // Exercise misaligned element offsets (+1..+3) for all memory ops.
  std::vector<mjtNum> pad_num(16), out_num_s(8), out_num_b(8);
  std::vector<float> pad_f32(16), out_f32_s(8), out_f32_b(8);
  for (int k = 0; k < 16; ++k) {
    pad_num[k] = static_cast<mjtNum>((k & 1 ? -1 : 1) * (0.25 * (k + 1)));
    pad_f32[k] = static_cast<float>((k & 1 ? -1 : 1) * (0.25f * (k + 1)));
  }
  for (int off = 1; off <= 3; ++off) {
    std::fill(out_num_s.begin(), out_num_s.end(), mjtNum(0));
    std::fill(out_num_b.begin(), out_num_b.end(), mjtNum(0));
    simd_scalar::v4_store(out_num_s.data() + off,
                          simd_scalar::v4_load(pad_num.data() + off));
    Backend::v4_store(out_num_b.data() + off,
                      Backend::v4_load(pad_num.data() + off));
    EXPECT_EQ(std::memcmp(out_num_s.data(), out_num_b.data(),
                          out_num_s.size() * sizeof(mjtNum)),
              0)
        << "op: misaligned_v4_load_store off=" << off;

    std::fill(out_num_s.begin(), out_num_s.end(), mjtNum(0));
    std::fill(out_num_b.begin(), out_num_b.end(), mjtNum(0));
    auto sl3 = simd_scalar::v4_load3(pad_num.data() + off);
    auto bl3 = Backend::v4_load3(pad_num.data() + off);
    mjtNum so[4], bo[4];
    simd_scalar::v4_store(so, sl3);
    Backend::v4_store(bo, bl3);
    EXPECT_EQ(std::memcmp(so, bo, sizeof(so)), 0)
        << "op: misaligned_v4_load3 off=" << off;
    simd_scalar::v4_store3(out_num_s.data() + off, sl3);
    Backend::v4_store3(out_num_b.data() + off, bl3);
    EXPECT_EQ(std::memcmp(out_num_s.data(), out_num_b.data(),
                          out_num_s.size() * sizeof(mjtNum)),
              0)
        << "op: misaligned_v4_store3 off=" << off;

    simd_scalar::mjtNum4 sc0, sc1, sc2;
    typename Backend::mjtNum4 bc0, bc1, bc2;
    simd_scalar::v4_load4x3(pad_num.data() + off, &sc0, &sc1, &sc2);
    Backend::v4_load4x3(pad_num.data() + off, &bc0, &bc1, &bc2);
    for (int lane_vec = 0; lane_vec < 3; ++lane_vec) {
      simd_scalar::v4_store(so,
                            lane_vec == 0 ? sc0 : (lane_vec == 1 ? sc1 : sc2));
      Backend::v4_store(bo, lane_vec == 0 ? bc0 : (lane_vec == 1 ? bc1 : bc2));
      EXPECT_EQ(std::memcmp(so, bo, sizeof(so)), 0)
          << "op: misaligned_v4_load4x3 off=" << off;
    }

    simd_scalar::v4_load3_f32(pad_f32.data() + off, &sc0, &sc1, &sc2);
    Backend::v4_load3_f32(pad_f32.data() + off, &bc0, &bc1, &bc2);
    for (int lane_vec = 0; lane_vec < 3; ++lane_vec) {
      simd_scalar::v4_store(so,
                            lane_vec == 0 ? sc0 : (lane_vec == 1 ? sc1 : sc2));
      Backend::v4_store(bo, lane_vec == 0 ? bc0 : (lane_vec == 1 ? bc1 : bc2));
      EXPECT_EQ(std::memcmp(so, bo, sizeof(so)), 0)
          << "op: misaligned_v4_load3_f32 off=" << off;
    }

    std::fill(out_f32_s.begin(), out_f32_s.end(), 0.0f);
    std::fill(out_f32_b.begin(), out_f32_b.end(), 0.0f);
    simd_scalar::f4_store(out_f32_s.data() + off,
                          simd_scalar::f4_load(pad_f32.data() + off));
    Backend::f4_store(out_f32_b.data() + off,
                      Backend::f4_load(pad_f32.data() + off));
    EXPECT_EQ(std::memcmp(out_f32_s.data(), out_f32_b.data(),
                          out_f32_s.size() * sizeof(float)),
              0)
        << "op: misaligned_f4_load_store off=" << off;
  }
}

#define MJ_DEFINE_BACKEND_ADAPTER(Name, Ns)                                   \
  struct Name {                                                               \
    using mjtNum4 = Ns::mjtNum4;                                              \
    using mjtMask4 = Ns::mjtMask4;                                            \
    using mjtFloat4 = Ns::mjtFloat4;                                          \
    using mjtFloatMask4 = Ns::mjtFloatMask4;                                  \
    static mjtNum4 v4_load(const mjtNum* p) { return Ns::v4_load(p); }        \
    static void v4_store(mjtNum* p, mjtNum4 a) { Ns::v4_store(p, a); }        \
    static mjtNum4 v4_load3(const mjtNum* p) { return Ns::v4_load3(p); }      \
    static void v4_store3(mjtNum* p, mjtNum4 a) { Ns::v4_store3(p, a); }      \
    static mjtNum4 v4_set(mjtNum a, mjtNum b, mjtNum c, mjtNum d) {           \
      return Ns::v4_set(a, b, c, d);                                          \
    }                                                                         \
    static mjtNum4 v4_splat(mjtNum a) { return Ns::v4_splat(a); }             \
    static mjtNum4 v4_add(mjtNum4 a, mjtNum4 b) { return Ns::v4_add(a, b); }  \
    static mjtNum4 v4_sub(mjtNum4 a, mjtNum4 b) { return Ns::v4_sub(a, b); }  \
    static mjtNum4 v4_mul(mjtNum4 a, mjtNum4 b) { return Ns::v4_mul(a, b); }  \
    static mjtNum4 v4_div(mjtNum4 a, mjtNum4 b) { return Ns::v4_div(a, b); }  \
    static mjtMask4 v4_cmplt(mjtNum4 a, mjtNum4 b) {                          \
      return Ns::v4_cmplt(a, b);                                              \
    }                                                                         \
    static mjtMask4 v4_cmple(mjtNum4 a, mjtNum4 b) {                          \
      return Ns::v4_cmple(a, b);                                              \
    }                                                                         \
    static mjtMask4 v4_cmpgt(mjtNum4 a, mjtNum4 b) {                          \
      return Ns::v4_cmpgt(a, b);                                              \
    }                                                                         \
    static mjtMask4 v4_cmpge(mjtNum4 a, mjtNum4 b) {                          \
      return Ns::v4_cmpge(a, b);                                              \
    }                                                                         \
    static mjtMask4 v4_cmpeq(mjtNum4 a, mjtNum4 b) {                          \
      return Ns::v4_cmpeq(a, b);                                              \
    }                                                                         \
    static mjtMask4 v4_and(mjtMask4 a, mjtMask4 b) {                          \
      return Ns::v4_and(a, b);                                                \
    }                                                                         \
    static mjtMask4 v4_or(mjtMask4 a, mjtMask4 b) { return Ns::v4_or(a, b); } \
    static mjtMask4 v4_xor(mjtMask4 a, mjtMask4 b) {                          \
      return Ns::v4_xor(a, b);                                                \
    }                                                                         \
    static mjtMask4 v4_andnot(mjtMask4 a, mjtMask4 b) {                       \
      return Ns::v4_andnot(a, b);                                             \
    }                                                                         \
    static mjtMask4 v4_not(mjtMask4 a) { return Ns::v4_not(a); }              \
    static mjtNum4 v4_select(mjtMask4 m, mjtNum4 tr, mjtNum4 fa) {            \
      return Ns::v4_select(m, tr, fa);                                        \
    }                                                                         \
    static int v4_movemask(mjtMask4 m) { return Ns::v4_movemask(m); }         \
    static int v4_any(mjtMask4 m) { return Ns::v4_any(m); }                   \
    static int v4_all(mjtMask4 m) { return Ns::v4_all(m); }                   \
    static mjtNum4 v4_min(mjtNum4 a, mjtNum4 b) { return Ns::v4_min(a, b); }  \
    static mjtNum4 v4_max(mjtNum4 a, mjtNum4 b) { return Ns::v4_max(a, b); }  \
    static mjtNum4 v4_abs(mjtNum4 a) { return Ns::v4_abs(a); }                \
    static mjtNum4 v4_sqrt(mjtNum4 a) { return Ns::v4_sqrt(a); }              \
    static mjtNum4 v4_xorsign(mjtNum4 a, int s0, int s1, int s2, int s3) {    \
      return Ns::v4_xorsign(a, s0, s1, s2, s3);                               \
    }                                                                         \
    static void v4_transpose(mjtNum4* r0, mjtNum4* r1, mjtNum4* r2,           \
                             mjtNum4* r3) {                                   \
      Ns::v4_transpose(r0, r1, r2, r3);                                       \
    }                                                                         \
    static void v4_load4x3(const mjtNum* p, mjtNum4* c0, mjtNum4* c1,         \
                           mjtNum4* c2) {                                     \
      Ns::v4_load4x3(p, c0, c1, c2);                                          \
    }                                                                         \
    static void v4_load3_f32(const float* p, mjtNum4* vx, mjtNum4* vy,        \
                             mjtNum4* vz) {                                   \
      Ns::v4_load3_f32(p, vx, vy, vz);                                        \
    }                                                                         \
    static mjtNum4 shuf_0123(mjtNum4 a) { return Ns::shuf_0123(a); }          \
    static mjtNum4 shuf_1032(mjtNum4 a) { return Ns::shuf_1032(a); }          \
    static mjtNum4 shuf_2301(mjtNum4 a) { return Ns::shuf_2301(a); }          \
    static mjtNum4 shuf_3210(mjtNum4 a) { return Ns::shuf_3210(a); }          \
    static mjtNum4 shuf_2232(mjtNum4 a) { return Ns::shuf_2232(a); }          \
    static mjtNum4 shuf_3323(mjtNum4 a) { return Ns::shuf_3323(a); }          \
    static mjtNum4 shuf_1212(mjtNum4 a) { return Ns::shuf_1212(a); }          \
    static mjtNum4 shuf_2121(mjtNum4 a) { return Ns::shuf_2121(a); }          \
    static mjtNum4 shuf_0000(mjtNum4 a) { return Ns::shuf_0000(a); }          \
    static mjtNum4 shuf_3333(mjtNum4 a) { return Ns::shuf_3333(a); }          \
    static mjtNum4 shuf_0213(mjtNum4 a) { return Ns::shuf_0213(a); }          \
    static mjtNum4 shuf_3021(mjtNum4 a) { return Ns::shuf_3021(a); }          \
    static mjtNum v4_sum(mjtNum4 a) { return Ns::v4_sum(a); }                 \
    static mjtNum4 v4_fma(mjtNum4 a, mjtNum4 b, mjtNum4 c) {                  \
      return Ns::v4_fma(a, b, c);                                             \
    }                                                                         \
    static mjtNum mju_fma(mjtNum a, mjtNum b, mjtNum c) {                     \
      return Ns::mju_fma(a, b, c);                                            \
    }                                                                         \
    static mjtFloat4 f4_load(const float* p) { return Ns::f4_load(p); }       \
    static void f4_store(float* p, mjtFloat4 a) { Ns::f4_store(p, a); }       \
    static mjtFloat4 f4_splat(float a) { return Ns::f4_splat(a); }            \
    static mjtFloatMask4 f4_cmplt(mjtFloat4 a, mjtFloat4 b) {                 \
      return Ns::f4_cmplt(a, b);                                              \
    }                                                                         \
    static mjtFloatMask4 f4_cmple(mjtFloat4 a, mjtFloat4 b) {                 \
      return Ns::f4_cmple(a, b);                                              \
    }                                                                         \
    static mjtFloatMask4 f4_and(mjtFloatMask4 a, mjtFloatMask4 b) {           \
      return Ns::f4_and(a, b);                                                \
    }                                                                         \
    static int f4_movemask(mjtFloatMask4 m) { return Ns::f4_movemask(m); }    \
  };

MJ_DEFINE_BACKEND_ADAPTER(DefaultBackendAdapter, simd_default)
#ifdef MJ_HAS_VECEXT_BACKEND
MJ_DEFINE_BACKEND_ADAPTER(VecextBackendAdapter, simd_vecext)
#endif
#ifdef MJ_HAS_AVX_BACKEND
MJ_DEFINE_BACKEND_ADAPTER(AvxBackendAdapter, simd_avx)
#endif

TEST_F(EngineSimdTest, AbsMatchesFabs) {
  const mjtNum tiny = static_cast<mjtNum>(1e-30);
  const mjtNum vals[] = {0.0, -0.0, 1.5, -1.5, tiny, -tiny, 3.0, -7.25};
  for (int k = 0; k < 8; k += 4) {
    mjtNum res[4];
    simd_default::v4_store(
        res, simd_default::v4_abs(simd_default::v4_load(vals + k)));
    for (int j = 0; j < 4; ++j) {
      mjtNum expected = std::fabs(vals[k + j]);
      EXPECT_EQ(std::memcmp(&res[j], &expected, sizeof(mjtNum)), 0)
          << "v4_abs(" << vals[k + j] << ")";
    }
  }
}

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

TEST_F(EngineSimdTest, MinMaxSignedZeroAndNanSemantics) {
  // v4_min(a, b) == (a < b ? a : b) and v4_max(a, b) == (a > b ? a : b):
  // second operand b is returned when a == b (+/-0.0) or either operand is NaN.
  const mjtNum nan = std::numeric_limits<mjtNum>::quiet_NaN();
  const mjtNum pos_zero = mjtNum(0.0);
  const mjtNum neg_zero = mjtNum(-0.0);
  const mjtNum a[4] = {pos_zero, neg_zero, nan, mjtNum(2.5)};
  const mjtNum b[4] = {neg_zero, pos_zero, mjtNum(2.5), nan};

  auto check = [&](auto fn) {
    mjtNum min_out[4], max_out[4];
    fn(a, b, min_out, max_out);
    EXPECT_EQ(std::memcmp(&min_out[0], &neg_zero, sizeof(mjtNum)), 0);
    EXPECT_EQ(std::memcmp(&min_out[1], &pos_zero, sizeof(mjtNum)), 0);
    EXPECT_EQ(min_out[2], mjtNum(2.5));
    EXPECT_TRUE(std::isnan(min_out[3]));

    EXPECT_EQ(std::memcmp(&max_out[0], &neg_zero, sizeof(mjtNum)), 0);
    EXPECT_EQ(std::memcmp(&max_out[1], &pos_zero, sizeof(mjtNum)), 0);
    EXPECT_EQ(max_out[2], mjtNum(2.5));
    EXPECT_TRUE(std::isnan(max_out[3]));
  };

  check([](const mjtNum* pa, const mjtNum* pb, mjtNum* mn, mjtNum* mx) {
    simd_scalar::v4_store(mn, simd_scalar::v4_min(simd_scalar::v4_load(pa),
                                                  simd_scalar::v4_load(pb)));
    simd_scalar::v4_store(mx, simd_scalar::v4_max(simd_scalar::v4_load(pa),
                                                  simd_scalar::v4_load(pb)));
  });
  check([](const mjtNum* pa, const mjtNum* pb, mjtNum* mn, mjtNum* mx) {
    simd_default::v4_store(mn, simd_default::v4_min(simd_default::v4_load(pa),
                                                    simd_default::v4_load(pb)));
    simd_default::v4_store(mx, simd_default::v4_max(simd_default::v4_load(pa),
                                                    simd_default::v4_load(pb)));
  });
#ifdef MJ_HAS_VECEXT_BACKEND
  check([](const mjtNum* pa, const mjtNum* pb, mjtNum* mn, mjtNum* mx) {
    simd_vecext::v4_store(mn, simd_vecext::v4_min(simd_vecext::v4_load(pa),
                                                  simd_vecext::v4_load(pb)));
    simd_vecext::v4_store(mx, simd_vecext::v4_max(simd_vecext::v4_load(pa),
                                                  simd_vecext::v4_load(pb)));
  });
#endif
#ifdef MJ_HAS_AVX_BACKEND
  check([](const mjtNum* pa, const mjtNum* pb, mjtNum* mn, mjtNum* mx) {
    simd_avx::v4_store(
        mn, simd_avx::v4_min(simd_avx::v4_load(pa), simd_avx::v4_load(pb)));
    simd_avx::v4_store(
        mx, simd_avx::v4_max(simd_avx::v4_load(pa), simd_avx::v4_load(pb)));
  });
#endif
}

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
