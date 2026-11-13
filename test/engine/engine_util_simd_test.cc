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

// Parity tests for the platform-SIMD (AVX, RVV) code paths of the BLAS and sparse utilities.
//
// Each dispatched function is compared against a scalar reference that follows the evaluation
// order of the SIMD implementations (4-lane accumulation, ((s0+s2) + (s1+s3)) reduction,
// unfused multiply-add). When a SIMD path is compiled in, results must be bit-identical, which
// keeps simulation results identical across platforms. Without a SIMD path, the scalar fallbacks
// may legitimately differ in the last bits (e.g. FMA contraction), so a tolerance is used.

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

#if defined(mjUSEPLATFORMSIMD) && defined(__riscv_vector) && \
    defined(__riscv_v_intrinsic) && __riscv_v_intrinsic >= 12000 && \
    defined(__riscv_v_elen_fp) && __riscv_v_elen_fp >= 64 && !defined(mjUSESINGLE)
  #ifndef mjUSERVV
    #error "mjUSERVV must be defined when mjUSEPLATFORMSIMD and RVV are available"
  #endif
#endif

namespace mujoco {
namespace {

#if defined(mjUSEAVX) || defined(mjUSERVV)
constexpr bool kSimdPath = true;
#else
constexpr bool kSimdPath = false;
#endif

using EngineUtilSimdTest = MujocoTest;

// ------------------------------ scalar references ------------------------------------------------

// Rounded product: the volatile store prevents the compiler from fusing it into an FMA, even
// under -ffp-contract=fast or a vectorizer that ignores -ffp-contract=off.
mjtNum Mul(mjtNum a, mjtNum b) {
  volatile mjtNum r = a * b;
  return r;
}

// Dot product in SIMD order; ind == nullptr means dense.
mjtNum RefDot(const mjtNum* vec1, const mjtNum* vec2, int n, const int* ind = nullptr) {
  auto v2 = [&](int i) { return ind ? vec2[ind[i]] : vec2[i]; };
  mjtNum res = 0;
  int i = 0;
  if (n >= 4) {
    mjtNum s[4];
    for (int k = 0; k < 4; k++) s[k] = Mul(vec1[k], v2(k));
    for (i = 4; i <= n - 4; i += 4) {
      for (int k = 0; k < 4; k++) s[k] = s[k] + Mul(vec1[i+k], v2(i+k));
    }
    res = (s[0] + s[2]) + (s[1] + s[3]);
  }

  // dense tail is summed as a group, sparse tail sequentially (as in the SIMD code)
  int rem = n - i;
  if (!ind && rem == 3) {
    res += (Mul(vec1[i], v2(i)) + Mul(vec1[i+1], v2(i+1))) + Mul(vec1[i+2], v2(i+2));
  } else if (!ind && rem == 2) {
    res += Mul(vec1[i], v2(i)) + Mul(vec1[i+1], v2(i+1));
  } else {
    for (; i < n; i++) res += Mul(vec1[i], v2(i));
  }
  return res;
}

// ------------------------------ helpers ----------------------------------------------------------

constexpr int kGuard = 8;          // sentinel elements after each output
constexpr mjtNum kSentinel = 12345.678;

// sizes covering empty, sub-vector, all tail lengths, and multiple vector widths
std::vector<int> TestSizes() {
  std::vector<int> sizes;
  for (int n = 0; n <= 70; n++) sizes.push_back(n);
  for (int n : {127, 128, 129, 255, 256, 257, 1000, 1023}) sizes.push_back(n);
  return sizes;
}

std::vector<mjtNum> RandomVec(std::mt19937& rng, int n) {
  std::uniform_real_distribution<mjtNum> dist(-2, 2);
  std::vector<mjtNum> v(n);
  for (auto& x : v) x = dist(rng);
  return v;
}

bool SameBits(mjtNum a, mjtNum b) { return std::memcmp(&a, &b, sizeof(mjtNum)) == 0; }

// exact on SIMD paths, close otherwise
void ExpectMatch(mjtNum actual, mjtNum expected, const char* what, int n, int i = -1) {
  if (kSimdPath) {
    EXPECT_TRUE(SameBits(actual, expected))
        << what << " n=" << n << " i=" << i << ": " << actual << " vs " << expected;
  } else {
    EXPECT_NEAR(actual, expected, 1e-12 * (1 + std::abs(expected)))
        << what << " n=" << n << " i=" << i;
  }
}

// checks n outputs against reference and that the guard region is untouched
void ExpectVecMatch(const std::vector<mjtNum>& actual, const std::vector<mjtNum>& expected,
                    int offset, int n, const char* what) {
  for (int i = 0; i < n; i++) {
    ExpectMatch(actual[offset + i], expected[offset + i], what, n, i);
  }
  for (int i = n; i < n + kGuard; i++) {
    EXPECT_EQ(actual[offset + i], kSentinel) << what << ": wrote past end, n=" << n;
  }
}

// ------------------------------ dense ------------------------------------------------------------

TEST_F(EngineUtilSimdTest, DotMatchesReference) {
  std::mt19937 rng(1);
  for (int n : TestSizes()) {
    // +1: also exercise unaligned loads
    for (int offset : {0, 1}) {
      std::vector<mjtNum> a = RandomVec(rng, n + offset), b = RandomVec(rng, n + offset);
      ExpectMatch(mju_dot(a.data() + offset, b.data() + offset, n),
                  RefDot(a.data() + offset, b.data() + offset, n), "mju_dot", n);
    }
  }
}

TEST_F(EngineUtilSimdTest, ElementwiseMatchesReference) {
  std::mt19937 rng(2);
  const mjtNum s1 = 0.73, s2 = -1.91;

  for (int n : TestSizes()) {
    for (int offset : {0, 1}) {
      int len = offset + n + kGuard;
      std::vector<mjtNum> a = RandomVec(rng, len), b = RandomVec(rng, len);
      std::vector<mjtNum> init = RandomVec(rng, len);
      for (int i = offset + n; i < len; i++) init[i] = kSentinel;
      const mjtNum* va = a.data() + offset;
      const mjtNum* vb = b.data() + offset;

      // each case: run the function on `res`, compute the reference into `ref`
      auto check = [&](const char* what, auto fn, auto ref_fn) {
        std::vector<mjtNum> res = init, ref = init;
        fn(res.data() + offset);
        for (int i = 0; i < n; i++) ref[offset + i] = ref_fn(i, init[offset + i]);
        ExpectVecMatch(res, ref, offset, n, what);
      };

      check("mju_scl", [&](mjtNum* r) { mju_scl(r, va, s1, n); },
            [&](int i, mjtNum) { return Mul(va[i], s1); });
      check("mju_add", [&](mjtNum* r) { mju_add(r, va, vb, n); },
            [&](int i, mjtNum) { return va[i] + vb[i]; });
      check("mju_sub", [&](mjtNum* r) { mju_sub(r, va, vb, n); },
            [&](int i, mjtNum) { return va[i] - vb[i]; });
      check("mju_addTo", [&](mjtNum* r) { mju_addTo(r, va, n); },
            [&](int i, mjtNum r0) { return r0 + va[i]; });
      check("mju_subFrom", [&](mjtNum* r) { mju_subFrom(r, va, n); },
            [&](int i, mjtNum r0) { return r0 - va[i]; });
      check("mju_addToScl", [&](mjtNum* r) { mju_addToScl(r, va, s1, n); },
            [&](int i, mjtNum r0) { return r0 + Mul(va[i], s1); });
      check("mju_addScl", [&](mjtNum* r) { mju_addScl(r, va, vb, s1, n); },
            [&](int i, mjtNum) { return va[i] + Mul(vb[i], s1); });
      check("mju_addToSclScl", [&](mjtNum* r) { mju_addToSclScl(r, va, s1, s2, n); },
            [&](int i, mjtNum r0) { return Mul(r0, s1) + Mul(va[i], s2); });
    }
  }
}

// ------------------------------ sparse -----------------------------------------------------------

TEST_F(EngineUtilSimdTest, DotSparseMatchesReference) {
  std::mt19937 rng(3);
  const int nvec = 2048;
  std::vector<mjtNum> dense = RandomVec(rng, nvec);
  std::uniform_int_distribution<int> col(0, nvec - 1);

  for (int nnz : TestSizes()) {
    std::vector<mjtNum> vals = RandomVec(rng, nnz);
    std::vector<int> ind(nnz);
    for (auto& c : ind) c = col(rng);
    ExpectMatch(mju_dotSparse(vals.data(), dense.data(), nnz, ind.data()),
                RefDot(vals.data(), dense.data(), nnz, ind.data()), "mju_dotSparse", nnz);
  }
}

// Exercises the regular and supernode (mju_dotSparseX3) paths of mju_mulMatVecSparse.
TEST_F(EngineUtilSimdTest, MulMatVecSparseMatchesReference) {
  std::mt19937 rng(4);
  const int nc = 300;
  std::vector<mjtNum> vec = RandomVec(rng, nc);
  std::uniform_int_distribution<int> col(0, nc - 1);

  // blocks of rows sharing a sparsity pattern; block sizes 1..7 cover X3 + remainder rows
  std::vector<int> rownnz, rowadr, colind;
  int nr = 0;
  for (int block = 1; block <= 7; block++) {
    for (int nnz : {0, 1, 3, 4, 5, 8, 13, 31, 64}) {
      std::vector<int> pattern(nnz);
      for (auto& c : pattern) c = col(rng);
      for (int r = 0; r < block; r++) {
        rownnz.push_back(nnz);
        rowadr.push_back(static_cast<int>(colind.size()));
        colind.insert(colind.end(), pattern.begin(), pattern.end());
        nr++;
      }
    }
  }
  std::vector<mjtNum> mat = RandomVec(rng, static_cast<int>(colind.size()));
  std::vector<int> rowsuper(nr);
  mju_superSparse(nr, rowsuper.data(), rownnz.data(), rowadr.data(), colind.data());

  std::vector<mjtNum> expected(nr);
  for (int r = 0; r < nr; r++) {
    expected[r] = RefDot(mat.data() + rowadr[r], vec.data(), rownnz[r],
                         colind.data() + rowadr[r]);
  }

  for (const int* super : {static_cast<const int*>(nullptr),
                           static_cast<const int*>(rowsuper.data())}) {
    std::vector<mjtNum> res(nr + kGuard, kSentinel);
    mju_mulMatVecSparse(res.data(), mat.data(), vec.data(), nr, rownnz.data(), rowadr.data(),
                        colind.data(), super);
    for (int r = 0; r < nr; r++) {
      ExpectMatch(res[r], expected[r], super ? "mulMatVecSparse(super)" : "mulMatVecSparse",
                  rownnz[r], r);
    }
    for (int r = nr; r < nr + kGuard; r++) EXPECT_EQ(res[r], kSentinel);
  }
}

TEST_F(EngineUtilSimdTest, CompareDetectsEveryPosition) {
  for (int n : TestSizes()) {
    std::vector<int> a(n), b(n);
    for (int i = 0; i < n; i++) a[i] = b[i] = 7 * i - 3;
    EXPECT_EQ(mju_compare(a.data(), b.data(), n), 1) << "n=" << n;

    // a single difference anywhere must be detected (checks every lane and the tail)
    for (int i = 0; i < n; i++) {
      b[i] ^= 1 << (i % 31);
      EXPECT_EQ(mju_compare(a.data(), b.data(), n), 0) << "n=" << n << " i=" << i;
      b[i] = a[i];
    }
  }
}

}  // namespace
}  // namespace mujoco
