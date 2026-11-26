// Copyright 2022 DeepMind Technologies Limited
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

// Tests for engine/engine_util_blas.c

#include "src/engine/engine_util_blas.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mjtype.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;

using EngineUtilBlasTest = MujocoTest;

TEST_F(EngineUtilBlasTest, MjuDot) {
  mjtNum a[] = {1, 2, 3, 4, 5, 6, 7};
  mjtNum b[] = {7, 6, 5, 4, 3, 2, 1};

  // test various vector lengths because mju_dot adds numbers in groups of four
  EXPECT_EQ(mju_dot(a, b, 0), 0);
  EXPECT_EQ(mju_dot(a, b, 1), 7);
  EXPECT_EQ(mju_dot(a, b, 2), 7 + 2 * 6);
  EXPECT_EQ(mju_dot(a, b, 3), 7 + 2 * 6 + 3 * 5);
  EXPECT_EQ(mju_dot(a, b, 4), 7 + 2 * 6 + 3 * 5 + 4 * 4);
  EXPECT_EQ(mju_dot(a, b, 5), 7 + 2 * 6 + 3 * 5 + 4 * 4 + 5 * 3);
  EXPECT_EQ(mju_dot(a, b, 6), 7 + 2 * 6 + 3 * 5 + 4 * 4 + 5 * 3 + 6 * 2);
  EXPECT_EQ(mju_dot(a, b, 7), 7 + 2 * 6 + 3 * 5 + 4 * 4 + 5 * 3 + 6 * 2 + 7);

  // Bit-exact parity against canonical 4-lane reduction across lengths 0..19
  constexpr int kMaxN = 19;
  mjtNum x[kMaxN], y[kMaxN];
  for (int i = 0; i < kMaxN; ++i) {
    mjtNum sign = (i & 2) ? -1 : 1;
    x[i] = sign * (mjtNum(1) / ((i / 4) + 1)) + mjtNum(1e-7) / (i + 3);
    y[i] = mjtNum(i + 1) / (2 * i + 5) - mjtNum(1) / (i + 7);
  }
  for (int n = 0; n <= kMaxN; ++n) {
    int i = 0;
    mjtNum ref = 0;
    if (n - 4 >= 0) {
      mjtNum r0 = x[0] * y[0];
      mjtNum r1 = x[1] * y[1];
      mjtNum r2 = x[2] * y[2];
      mjtNum r3 = x[3] * y[3];
      for (i = 4; i <= n - 4; i += 4) {
        r0 += x[i + 0] * y[i + 0];
        r1 += x[i + 1] * y[i + 1];
        r2 += x[i + 2] * y[i + 2];
        r3 += x[i + 3] * y[i + 3];
      }
      ref = (r0 + r2) + (r1 + r3);
    }
    int n_i = n - i;
    if (n_i == 3) {
      ref += x[i + 0] * y[i + 0] + x[i + 1] * y[i + 1] + x[i + 2] * y[i + 2];
    } else if (n_i == 2) {
      ref += x[i + 0] * y[i + 0] + x[i + 1] * y[i + 1];
    } else if (n_i == 1) {
      ref += x[i] * y[i];
    }
    mjtNum got = mju_dot(x, y, n);
    EXPECT_EQ(std::memcmp(&got, &ref, sizeof(mjtNum)), 0) << "n=" << n;
  }
}

TEST_F(EngineUtilBlasTest, MjuSumL1) {
  constexpr int kMaxN = 19;
  mjtNum a[kMaxN];

  for (int i = 0; i < kMaxN; i++) {
    mjtNum sign = (i % 3 == 0) ? -1.0 : 1.0;
    a[i] = sign * (1.0 / (i + 1) + 1e-3 * ((i * 7) % 11));
  }

  auto ref_reduce4 = [](const mjtNum* terms, int n) {
    int i = 0;
    mjtNum r0 = 0, r1 = 0, r2 = 0, r3 = 0;
    for (; i <= n - 4; i += 4) {
      r0 += terms[i + 0];
      r1 += terms[i + 1];
      r2 += terms[i + 2];
      r3 += terms[i + 3];
    }
    mjtNum res = (r0 + r2) + (r1 + r3);
    for (; i < n; i++) {
      res += terms[i];
    }
    return res;
  };

  mjtNum abs_a[kMaxN];
  for (int i = 0; i < kMaxN; i++) {
    abs_a[i] = std::abs(a[i]);
  }

  for (int n = 0; n <= kMaxN; n++) {
    EXPECT_EQ(mju_sum(a, n), ref_reduce4(a, n)) << "n=" << n;
    EXPECT_EQ(mju_L1(a, n), ref_reduce4(abs_a, n)) << "n=" << n;
  }

  // Exact ground-truth integer and dyadic sequences.
  mjtNum seq[kMaxN];
  mjtNum alt[kMaxN];
  for (int i = 0; i < kMaxN; i++) {
    seq[i] = static_cast<mjtNum>(i + 1) * static_cast<mjtNum>(0.25);
    alt[i] = (i % 2 == 0) ? seq[i] : -seq[i];
  }
  for (int n = 0; n <= kMaxN; n++) {
    mjtNum expected_l1 =
        static_cast<mjtNum>(n * (n + 1)) * static_cast<mjtNum>(0.125);
    mjtNum expected_alt =
        static_cast<mjtNum>((n % 2 == 0) ? -(n / 2) : (n + 1) / 2) *
        static_cast<mjtNum>(0.25);
    EXPECT_EQ(mju_sum(seq, n), expected_l1) << "n=" << n;
    EXPECT_EQ(mju_sum(alt, n), expected_alt) << "n=" << n;
    EXPECT_EQ(mju_L1(alt, n), expected_l1) << "n=" << n;
  }

  // Reduction-tree check: distinguish (r0 + r2) + (r1 + r3) from both
  // (r0 + r1) + (r2 + r3) and sequential summation.
  const mjtNum eps = std::numeric_limits<mjtNum>::epsilon();
  const mjtNum q = static_cast<mjtNum>(0.25) * eps;
  const mjtNum sum_cancel[4] = {1, q, -1, q};
  // (1 + -1) + (q + q) = 0.5*eps; (1+q)+(-1+q) = 0; sequential = 0.25*eps.
  EXPECT_EQ(mju_sum(sum_cancel, 4), static_cast<mjtNum>(0.5) * eps);

  const mjtNum t = static_cast<mjtNum>(0.375) * eps;
  const mjtNum l1_tree[4] = {static_cast<mjtNum>(0.5), q,
                             static_cast<mjtNum>(-0.5), t};
  // (0.5 + 0.5) + (0.25 + 0.375)*eps = 1 + 0.625*eps -> 1 + eps;
  // (0.5 + 0.25*eps) + (0.5 + 0.375*eps) = 0.5 + (0.5 + 0.5*eps) -> 1;
  // sequential -> 1.
  EXPECT_EQ(mju_L1(l1_tree, 4), static_cast<mjtNum>(1) + eps);

  // Edge cases: negative n, -0.0, Inf, NaN (both 4-wide loop and tail).
  EXPECT_EQ(mju_sum(nullptr, -1), 0);
  EXPECT_EQ(mju_L1(nullptr, -1), 0);
  EXPECT_EQ(mju_sum(a, -100), 0);
  EXPECT_EQ(mju_L1(a, -100), 0);

  const mjtNum nz = static_cast<mjtNum>(-0.0);
  const mjtNum neg_zeros[5] = {nz, nz, nz, nz, nz};
  for (int n : {1, 4, 5}) {
    mjtNum l1 = mju_L1(neg_zeros, n);
    EXPECT_EQ(l1, 0);
    EXPECT_FALSE(std::signbit(l1)) << "n=" << n;
    EXPECT_EQ(mju_sum(neg_zeros, n), 0);
  }

  const mjtNum inf = std::numeric_limits<mjtNum>::infinity();
  const mjtNum qnan = std::numeric_limits<mjtNum>::quiet_NaN();
  const mjtNum pos_inf[5] = {1, 2, inf, 4, 5};
  EXPECT_TRUE(std::isinf(mju_sum(pos_inf, 5)));
  EXPECT_GT(mju_sum(pos_inf, 5), 0);
  EXPECT_TRUE(std::isinf(mju_L1(pos_inf, 5)));

  const mjtNum opp_inf[5] = {inf, 1, -inf, 2, 3};
  EXPECT_TRUE(std::isnan(mju_sum(opp_inf, 5)));
  EXPECT_TRUE(std::isinf(mju_L1(opp_inf, 5)));
  EXPECT_GT(mju_L1(opp_inf, 5), 0);

  const mjtNum with_nan_body[4] = {1, qnan, 2, 3};
  EXPECT_TRUE(std::isnan(mju_sum(with_nan_body, 4)));
  EXPECT_TRUE(std::isnan(mju_L1(with_nan_body, 4)));

  const mjtNum with_nan_tail[5] = {1, 2, 3, 4, qnan};
  EXPECT_TRUE(std::isnan(mju_sum(with_nan_tail, 5)));
  EXPECT_TRUE(std::isnan(mju_L1(with_nan_tail, 5)));
}

TEST_F(EngineUtilBlasTest, MjuMulVecMatVec) {
  mjtNum vec1[] = {1, 2, 3};
  mjtNum vec2[] = {3, 2, 1};
  mjtNum mat[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

  EXPECT_EQ(mju_mulVecMatVec(vec1, mat, vec2, 3), 204);
}

TEST_F(EngineUtilBlasTest, MjuFill) {
  mjtNum vec[] = {0, 1, 4};
  mju_fill(vec, 4.5, 3);

  EXPECT_EQ(vec[0], 4.5);
  EXPECT_EQ(vec[1], 4.5);
  EXPECT_EQ(vec[2], 4.5);
}

TEST_F(EngineUtilBlasTest, MjuEye) {
  mjtNum mat1[1];
  mju_eye(mat1, 1);
  EXPECT_EQ(mat1[0], 1);

  mjtNum mat3[9];
  mju_eye(mat3, 3);
  EXPECT_THAT(mat3, ElementsAre(1, 0, 0, 0, 1, 0, 0, 0, 1));
}

TEST_F(EngineUtilBlasTest, MjuSymmetrize) {
  mjtNum mat[] = {1, 2.5, 3.5, 1.5, 2, 4, 2.5, 3, 3};
  mjtNum res[9] = {0};
  mju_symmetrize(res, mat, 3);
  EXPECT_THAT(res, ElementsAre(1, 2, 3, 2, 2, 3.5, 3, 3.5, 3));

  // test for the case res==mat
  mju_symmetrize(mat, mat, 3);
  EXPECT_THAT(mat, ElementsAre(1, 2, 3, 2, 2, 3.5, 3, 3.5, 3));
}

TEST_F(EngineUtilBlasTest, MjuMulMat3) {
  const mjtNum mat1[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  const mjtNum mat2[9] = {2, 3, 4, 5, 6, 7, 8, 9, 10};
  mjtNum res1[9] = {0};
  mjtNum res2[9] = {0};

  mju_mulMatMat3(res1, mat1, mat2);
  mju_mulMatMat(res2, mat1, mat2, 3, 3, 3);
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(res1[i], res2[i]);
  }

  mju_mulMatTMat3(res1, mat1, mat2);
  mju_mulMatTMat(res2, mat1, mat2, 3, 3, 3);
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(res1[i], res2[i]);
  }

  mju_mulMatMatT3(res1, mat1, mat2);
  mju_mulMatMatT(res2, mat1, mat2, 3, 3, 3);
  for (int i = 0; i < 9; ++i) {
    EXPECT_EQ(res1[i], res2[i]);
  }
}

// utility: random quaternion, normally distributed
void RandomQuat(mjtNum quat[4], int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<mjtNum> dist(0, 1);

  // sample
  for (int i = 0; i < 4; i++) {
    quat[i] = dist(rng);
  }
}

TEST_F(EngineUtilBlasTest, Normalize4IsIdempotent) {
  for (int i = 0; i < 10000; ++i) {
    // make random quaternion and normalize it
    mjtNum quat[4];
    RandomQuat(quat, i);
    mju_normalize4(quat);

    // normalize again
    mjtNum quat_renormalized[4] = {quat[0], quat[1], quat[2], quat[3]};
    mju_normalize4(quat_renormalized);

    // expect equality
    EXPECT_MJTNUM_EQ(quat[0], quat_renormalized[0]);
    EXPECT_MJTNUM_EQ(quat[1], quat_renormalized[1]);
    EXPECT_MJTNUM_EQ(quat[2], quat_renormalized[2]);
    EXPECT_MJTNUM_EQ(quat[3], quat_renormalized[3]);
  }
}

TEST_F(EngineUtilBlasTest, Normalize4EdgeCases) {
  // zero quat normalizes to unit quat
  mjtNum quat[4] = {0};
  mjtNum norm = mju_normalize4(quat);
  EXPECT_EQ(norm, 0);
  EXPECT_EQ(quat[0], 1);
  EXPECT_EQ(quat[1], 0);
  EXPECT_EQ(quat[2], 0);
  EXPECT_EQ(quat[3], 0);

  // small quat normalizes regularly
  quat[0] = 0;
  quat[1] = mjMINVAL;
  norm = mju_normalize4(quat);
  EXPECT_EQ(norm, mjMINVAL);
  EXPECT_EQ(quat[0], 0);
  EXPECT_EQ(quat[1], 1);
  EXPECT_EQ(quat[2], 0);
  EXPECT_EQ(quat[3], 0);

  // tiny quat normalizes to unit quat
  quat[0] = 0;
  quat[1] = mjMINVAL / 2;
  norm = mju_normalize4(quat);
  EXPECT_EQ(norm, mjMINVAL / 2);
  EXPECT_EQ(quat[0], 1);
  EXPECT_EQ(quat[1], 0);
  EXPECT_EQ(quat[2], 0);
  EXPECT_EQ(quat[3], 0);

  // near-unit quat is normalized
  quat[0] = 1 + mjMINVAL;
  quat[1] = 0;
  norm = mju_normalize4(quat);
  EXPECT_EQ(norm, 1 + mjMINVAL);
  EXPECT_EQ(quat[0], 1);
  EXPECT_EQ(quat[1], 0);
  EXPECT_EQ(quat[2], 0);
  EXPECT_EQ(quat[3], 0);

  // very-near-unit quat is untouched
  quat[0] = 1 + mjMINVAL / 2;
  quat[1] = 0;
  norm = mju_normalize4(quat);
  EXPECT_EQ(norm, 1 + mjMINVAL / 2);
  EXPECT_EQ(quat[0], 1 + mjMINVAL / 2);
  EXPECT_EQ(quat[1], 0);
  EXPECT_EQ(quat[2], 0);
  EXPECT_EQ(quat[3], 0);
}

TEST_F(EngineUtilBlasTest, MjuNormalizeZeroLength) {
  mjtNum vec[1] = {42.0};
  EXPECT_MJTNUM_EQ(mju_normalize(vec, 0), 0.0);
  EXPECT_MJTNUM_EQ(vec[0], 42.0);
}

TEST_F(EngineUtilBlasTest, MjuNormalize) {
  // standard normalization
  mjtNum vec[] = {3, 4, 0};
  EXPECT_MJTNUM_EQ(mju_normalize(vec, 3), 5);
  EXPECT_MJTNUM_EQ(vec[0], 0.6);
  EXPECT_MJTNUM_EQ(vec[1], 0.8);
  EXPECT_MJTNUM_EQ(vec[2], 0);

  // zero vector normalizes to x-axis
  mjtNum zero[] = {0, 0, 0};
  EXPECT_MJTNUM_EQ(mju_normalize(zero, 3), 0);
  EXPECT_THAT(zero, ElementsAre(1, 0, 0));

  // tiny vector normalizes to x-axis
  mjtNum tiny[] = {0, mjMINVAL / 2, 0};
  EXPECT_MJTNUM_EQ(mju_normalize(tiny, 3), mjMINVAL / 2);
  EXPECT_THAT(tiny, ElementsAre(1, 0, 0));
}

TEST_F(EngineUtilBlasTest, ElementwiseSimdAndInPlaceParity) {
  constexpr int kMaxN = 19;
  mjtNum x[kMaxN], y[kMaxN], z[kMaxN];
  for (int i = 0; i < kMaxN; ++i) {
    mjtNum sign = (i & 1) ? -1 : 1;
    x[i] = sign * (mjtNum(i + 2) / (2 * i + 3)) + mjtNum(1e-6) / (i + 1);
    y[i] = -sign * (mjtNum(3 * i + 1) / (i + 5)) - mjtNum(1e-5) / (i + 2);
    z[i] = mjtNum(i - 7) / (i + 4) + mjtNum(1e-4) / (i + 3);
  }
  const mjtNum scl = -mjtNum(3) / mjtNum(7);

  for (int n = 0; n <= kMaxN; ++n) {
    mjtNum ref[kMaxN] = {0}, got[kMaxN] = {0}, inplace[kMaxN] = {0};

    // mju_scl (out-of-place and in-place res == vec)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] = x[i] * scl;
    mju_scl(got, x, scl, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "scl n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] = x[i] * scl;
    mju_scl(inplace, inplace, scl, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "scl inplace n=" << n;

    // mju_add (out-of-place, res == vec1, res == vec2)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i];
    mju_add(got, x, y, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "add n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i];
    mju_add(inplace, inplace, y, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "add inplace1 n=" << n;
    std::memcpy(ref, y, sizeof(y));
    std::memcpy(inplace, y, sizeof(y));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i];
    mju_add(inplace, x, inplace, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "add inplace2 n=" << n;

    // mju_sub (out-of-place, res == vec1, res == vec2)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] = x[i] - y[i];
    mju_sub(got, x, y, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "sub n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] = x[i] - y[i];
    mju_sub(inplace, inplace, y, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "sub inplace1 n=" << n;
    std::memcpy(ref, y, sizeof(y));
    std::memcpy(inplace, y, sizeof(y));
    for (int i = 0; i < n; ++i) ref[i] = x[i] - y[i];
    mju_sub(inplace, x, inplace, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "sub inplace2 n=" << n;

    // mju_addTo (out-of-place and res == vec)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] += x[i];
    mju_addTo(got, x, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "addTo n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] += ref[i];
    mju_addTo(inplace, inplace, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "addTo inplace n=" << n;

    // mju_subFrom (out-of-place and res == vec)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] -= x[i];
    mju_subFrom(got, x, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "subFrom n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] -= ref[i];
    mju_subFrom(inplace, inplace, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "subFrom inplace n=" << n;

    // mju_addToScl (out-of-place and res == vec)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] += x[i] * scl;
    mju_addToScl(got, x, scl, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "addToScl n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] += ref[i] * scl;
    mju_addToScl(inplace, inplace, scl, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "addToScl inplace n=" << n;

    // mju_addScl (out-of-place, res == vec1, res == vec2)
    std::memcpy(ref, z, sizeof(z));
    std::memcpy(got, z, sizeof(z));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i] * scl;
    mju_addScl(got, x, y, scl, n);
    EXPECT_EQ(std::memcmp(got, ref, sizeof(ref)), 0) << "addScl n=" << n;
    std::memcpy(ref, x, sizeof(x));
    std::memcpy(inplace, x, sizeof(x));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i] * scl;
    mju_addScl(inplace, inplace, y, scl, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "addScl inplace1 n=" << n;
    std::memcpy(ref, y, sizeof(y));
    std::memcpy(inplace, y, sizeof(y));
    for (int i = 0; i < n; ++i) ref[i] = x[i] + y[i] * scl;
    mju_addScl(inplace, x, inplace, scl, n);
    EXPECT_EQ(std::memcmp(inplace, ref, sizeof(ref)), 0)
        << "addScl inplace2 n=" << n;
  }
}

TEST_F(EngineUtilBlasTest, MjuDotX3MatchesDot) {
  std::mt19937_64 rng(3);
  std::uniform_real_distribution<mjtNum> dist(-2.0, 2.0);
  for (int n = 0; n <= 19; ++n) {
    std::vector<mjtNum> a(n), b(n), c(n), v(n);
    for (int k = 0; k < n; ++k) {
      a[k] = dist(rng);
      b[k] = dist(rng);
      c[k] = dist(rng);
      v[k] = dist(rng);
    }
    mjtNum res[3];
    mju_dotX3(res, res + 1, res + 2, a.data(), b.data(), c.data(), v.data(), n);
    mjtNum ref[3] = {mju_dot(a.data(), v.data(), n),
                     mju_dot(b.data(), v.data(), n),
                     mju_dot(c.data(), v.data(), n)};
    EXPECT_EQ(std::memcmp(res, ref, sizeof(res)), 0) << "n=" << n;
  }
}

TEST_F(EngineUtilBlasTest, MjuMulMatVecAndTVecExact) {
  std::mt19937_64 rng(42);
  std::uniform_real_distribution<mjtNum> dist(-2.0, 2.0);
  std::uniform_real_distribution<mjtNum> prob(0.0, 1.0);

  for (int nc : {1, 2, 3, 4, 5, 7, 9, 16, 18, 24, 27, 31, 32, 49, 64, 67}) {
    for (int nr : {0, 1, 3, 4, 5, 17, 135, 300}) {
      std::vector<mjtNum> mat(nr * nc, 0);
      std::vector<mjtNum> x(nc, 0);
      std::vector<mjtNum> y(nr, 0);
      for (int c = 0; c < nc; ++c) x[c] = dist(rng);
      for (int r = 0; r < nr; ++r) {
        y[r] = prob(rng) < 0.35 ? 0 : dist(rng);
        for (int c = 0; c < nc; ++c) {
          mat[r * nc + c] = prob(rng) < 0.5 ? 0 : dist(rng);
        }
      }

      std::vector<mjtNum> res(nr, 123.0), ref(nr, 0);
      mju_mulMatVec(res.data(), mat.data(), x.data(), nr, nc);
      for (int r = 0; r < nr; ++r) {
        ref[r] = mju_dot(mat.data() + r * nc, x.data(), nc);
      }
      if (nr > 0) {
        EXPECT_EQ(std::memcmp(res.data(), ref.data(), nr * sizeof(mjtNum)), 0)
            << "nr=" << nr << " nc=" << nc;
      }

      std::vector<mjtNum> tres(nc, 123.0), tref(nc, 0);
      mju_mulMatTVec(tres.data(), mat.data(), y.data(), nr, nc);
      for (int r = 0; r < nr; ++r) {
        if (y[r]) {
          mju_addToScl(tref.data(), mat.data() + r * nc, y[r], nc);
        }
      }
      EXPECT_EQ(std::memcmp(tres.data(), tref.data(), nc * sizeof(mjtNum)), 0)
          << "nr=" << nr << " nc=" << nc;

      // Verify all-zero y clears output buffer when nr > 0.
      if (nr > 0) {
        std::vector<mjtNum> yzero(nr, 0);
        std::vector<mjtNum> zres(nc, 123.0), zref(nc, 0);
        mju_mulMatTVec(zres.data(), mat.data(), yzero.data(), nr, nc);
        EXPECT_EQ(std::memcmp(zres.data(), zref.data(), nc * sizeof(mjtNum)), 0)
            << "all-zero y nr=" << nr << " nc=" << nc;
      }
    }
  }
}

TEST_F(EngineUtilBlasTest, MjuSqrMatTDExact) {
  std::mt19937_64 rng(123);
  std::uniform_real_distribution<mjtNum> dist(-2.0, 2.0);
  std::uniform_real_distribution<mjtNum> prob(0.0, 1.0);

  for (int nc : {1, 2, 3, 4, 5, 8, 9, 15, 18, 27, 49, 63, 64, 67}) {
    for (int nr : {0, 1, 7, 135, 300}) {
      std::vector<mjtNum> mat(nr * nc, 0);
      std::vector<mjtNum> diag(nr, 0);
      for (int r = 0; r < nr; ++r) {
        diag[r] = prob(rng) < 0.3 ? 0 : dist(rng);
        if (r < nr / 4) {
          // single-nonzero row before multi-nonzero rows
          mat[r * nc + (r % nc)] = dist(rng);
        } else if (r > (3 * nr) / 4 && (r % 3 == 0)) {
          // single-nonzero row after multi-nonzero rows
          mat[r * nc + (r % nc)] = dist(rng);
        } else {
          for (int c = 0; c < nc; ++c) {
            mat[r * nc + c] = prob(rng) < 0.55 ? 0 : dist(rng);
          }
        }
      }

      for (int use_diag : {0, 1}) {
        const mjtNum* dptr = use_diag ? diag.data() : nullptr;
        for (int flg_upper : {0, 1}) {
          std::vector<mjtNum> res(nc * nc, 123.0), ref(nc * nc, 0);
          mju_sqrMatTD_impl(res.data(), mat.data(), dptr, nr, nc, flg_upper);
          for (int j = 0; j < nr; ++j) {
            mjtNum dj = dptr ? dptr[j] : 1;
            if (!dj) continue;
            const mjtNum* row = mat.data() + j * nc;
            for (int i = 0; i < nc; ++i) {
              if (row[i]) {
                mju_addToScl(ref.data() + i * nc, row, row[i] * dj, i + 1);
              }
            }
          }
          if (flg_upper) {
            for (int i = 0; i < nc; ++i) {
              for (int j = i + 1; j < nc; ++j) {
                ref[i * nc + j] = ref[j * nc + i];
              }
            }
          }
          EXPECT_EQ(
              std::memcmp(res.data(), ref.data(), nc * nc * sizeof(mjtNum)), 0)
              << "nr=" << nr << " nc=" << nc << " use_diag=" << use_diag
              << " flg_upper=" << flg_upper;
        }
      }
    }
  }
}

}  // namespace
}  // namespace mujoco
