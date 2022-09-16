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

// Tests for engine/engine_util_solve.c.

#include "src/engine/engine_util_solve.h"

#include <random>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleEq;
using ::std::string;
using ::std::setw;
using QCQP2Test = MujocoTest;

TEST_F(QCQP2Test, DegenerateAMatrix) {
  // A 2x2 matrix with determinant zero.
  const mjtNum Ain[9] { 6, -15, 2, -5 };

  // Any values will do for these three inputs.
  const mjtNum bin[3] { -12, 49 };
  const mjtNum d[3] { 11, 31 };
  const mjtNum r = 0.01;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[2] { 999, 999 };

  EXPECT_EQ(mju_QCQP2(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
}

using QCQP3Test = MujocoTest;

TEST_F(QCQP3Test, DegenerateAMatrix) {
  // A 3x3 matrix with determinant zero.
  const mjtNum Ain[9] { 1, 4, -2, -3, -7, 5, 2, -9, 0 };

  // Any values will do for these three inputs.
  const mjtNum bin[3] { -12, 49, 8 };
  const mjtNum d[3] { 11, 31, -23 };
  const mjtNum r = 0.1;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[3] { 999, 999, 999 };

  EXPECT_EQ(mju_QCQP3(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
  EXPECT_EQ(res[2], 0);
}

// --------------------------- mju_boxQP ---------------------------------------

using BoxQPTest = MujocoTest;

// utility: compute QP objective = 0.5*x'*H*x + x'*g
mjtNum objective(const mjtNum* x, const mjtNum* H, const mjtNum* g, int n) {
  return 0.5 * mju_mulVecMatVec(x, H, x, n) + mju_dot(x, g, n);
}

// utility: test if res is the minimum of a given box-QP problem
bool isQPminimum(const mjtNum* res, const mjtNum* H, const mjtNum* g, int n,
                 const mjtNum* lower, const mjtNum* upper) {
  static const mjtNum eps = 1e-4;  // epsilon used for nudging
  bool is_minimum = true;
  mjtNum* res_nudge = (mjtNum*) mju_malloc(sizeof(mjtNum)*n);

  // get solution value
  mjtNum value = objective(res, H, g, n);
  mjtNum value_nudge;

  // compare to nudged solution
  mju_copy(res_nudge, res, n);
  int i;
  for (i=0; i < n; i++) {
    // nudge down
    res_nudge[i] = res[i] - eps;
    if (lower) {
      res_nudge[i] = mju_max(lower[i], res_nudge[i]);
    }
    value_nudge = objective(res_nudge, H, g, n);
    if (value_nudge - value < 0) {
      is_minimum = false;
      break;
    }

    // nudge up
    res_nudge[i] = res[i] + eps;
    if (upper) {
      res_nudge[i] = mju_min(upper[i], res_nudge[i]);
    }
    value_nudge = objective(res_nudge, H, g, n);
    if (value_nudge - value < 0) {
      is_minimum = false;
      break;
    }

    // reset
    res_nudge[i] = res[i];
  }

  mju_free(res_nudge);
  return is_minimum;
}

// utility: define QP with pseudorandom values
void randomBoxQP(int n, mjtNum* H, mjtNum* g, mjtNum* lower, mjtNum* upper,
                 int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // square root of H
  mjtNum* sqrtH = (mjtNum*) mju_malloc(sizeof(mjtNum)*n*n);

  for (int i=0; i < n; i++) {
    g[i] = dist(rng);
    lower[i] = 5*dist(rng);
    upper[i] = 5*dist(rng);

    // fix invalid bounds
    if (lower[i] > upper[i]) {
      mjtNum tmp = upper[i];
      upper[i] = lower[i];
      lower[i] = tmp;
    }

    // sample temp
    for (int j=0; j < n; j++) {
      sqrtH[n*i+j] = dist(rng);
    }
  }

  // make SPD matrix H
  mju_mulMatTMat(H, sqrtH, sqrtH, n, n, n);

  mju_free(sqrtH);
}

// test mju_boxQP on a small unbounded QP
TEST_F(BoxQPTest, UnboundedQP) {
  // small arrays, allocate on stack
  static const int n = 2;
  mjtNum H[n*n] = {
    2, 0,
    0, 2
  };
  mjtNum g[n] = {1, 3};
  mjtNum res[n] = {0, 0};
  mjtNum R[n*(n+7)];

  int nfree = mju_boxQP(res, R, /*index=*/nullptr, H, g, n,
                        /*lower=*/nullptr, /*upper=*/nullptr);

  // no bounds, expect Newton point
  EXPECT_EQ(nfree, 2);
  EXPECT_THAT(res[0], DoubleEq(-g[0]/H[0]));
  EXPECT_THAT(res[1], DoubleEq(-g[1]/H[3]));

  // check that solution is actual minimum
  EXPECT_TRUE(isQPminimum(res, H, g, n, /*lower=*/nullptr, /*upper=*/nullptr));

  // perturb solution, expected it no longer be the minimum
  res[0] += 0.001;
  EXPECT_FALSE(isQPminimum(res, H, g, n, /*lower=*/nullptr, /*upper=*/nullptr));

  // negative-definite Hessian, no solution
  H[0] = -1;
  nfree = mju_boxQP(res, R, /*index=*/nullptr, H, g, n,
                    /*lower=*/nullptr, /*upper=*/nullptr);

  EXPECT_EQ(nfree, -1);
}

// test mju_boxQP on a single random bounded QP
TEST_F(BoxQPTest, BoundedQP) {
  int n = 50;                     // problem size

  // allocate on heap
  mjtNum *H, *g, *lower, *upper;  // inputs
  mjtNum *res, *R;                // outputs
  int* index;                     // outputs
  mju_boxQPmalloc(&res, &R, &index, &H, &g, n, &lower, &upper);

  randomBoxQP(n, H, g, lower, upper, /*seed=*/1);

  // initialize res
  mju_zero(res, n);

  // use default options
  int    maxiter    = 100;    // maximum number of iterations
  mjtNum mingrad    = 1E-16;  // minimum squared norm of (unclamped) gradient
  mjtNum backtrack  = 0.5;    // backtrack factor for decreasing stepsize
  mjtNum minstep    = 1E-22;  // minimum stepsize for linesearch
  mjtNum armijo     = 0.1;    // Armijo parameter

  // logging
  static const int logsz = 10000;
  char log[logsz];

  int nfree = mju_boxQPoption(res, R, index, H, g, n, lower, upper,
                              maxiter, mingrad, backtrack,
                              minstep, armijo, log, logsz);

  // EXPECT_TRUE(false) << log;  // uncomment to print `log` to error log

  // check solution
  EXPECT_GT(nfree, -1);
  EXPECT_TRUE(isQPminimum(res, H, g, n, lower, upper));

  // verify clamping
  int j = nfree > 0 ? 0 : -1;
  for (int i=0; i < n; i++) {
    if (j >= 0 && i == index[j]) {  // free dimension
      EXPECT_GT(res[i], lower[i]);
      EXPECT_LT(res[i], upper[i]);
      j++;
    } else {                        // clamped dimension
      EXPECT_TRUE(res[i] == lower[i] || res[i] == upper[i]);
    }
  }
  mju_free(res);
  mju_free(R);
  mju_free(index);
  mju_free(H);
  mju_free(g);
  mju_free(lower);
  mju_free(upper);
}

// test mju_boxQP on a set of random bounded QPs
TEST_F(BoxQPTest, BoundedQPvariations) {
  int nmax = 100;

  // allocate maximum size on heap
  mjtNum *H, *g, *lower, *upper;  // inputs
  mjtNum *res, *R;                // outputs
  int* index;                     // outputs
  mju_boxQPmalloc(&res, &R, &index, &H, &g, nmax, &lower, &upper);

  // logging
  static const int logsz = 10000;
  char log[logsz];

  int seed = 1;
  for (int n : {3, 30, 100}) {
    int count = 0;
    int factorizations = 0;
    for (mjtNum scaleH : {.01, 1.0, 100.0}) {
      for (mjtNum scaleg : {.01, 1.0, 100.0}) {
        for (mjtNum scalebounds : {.01, 1.0, 100.0}) {
          // make random box-QP
          randomBoxQP(n, H, g, lower, upper, seed++);

          mju_scl(H, H, scaleH, n*n);
          mju_scl(g, g, scaleg, n);
          mju_scl(lower, lower, scalebounds, n);
          mju_scl(upper, upper, scalebounds, n);

          // initialize with zeros
          mju_zero(res, n);

          // default algorithm options
          int    maxiter    = 100;
          mjtNum mingrad    = 1E-16;
          mjtNum backtrack  = 0.5;
          mjtNum minstep    = 1E-22;
          mjtNum armijo     = 0.1;

          // solve box-QP with logging
          int nfree = mju_boxQPoption(res, R, index, H, g, n, lower, upper,
                                      maxiter, mingrad, backtrack,
                                      minstep, armijo, log, logsz);

          // check solution
          EXPECT_GT(nfree, -1) << log;
          EXPECT_TRUE(isQPminimum(res, H, g, n, lower, upper))
              << "n " << n << '\n'
              << "scaleH " << scaleH << '\n'
              << "scaleg " << scaleg << '\n'
              << "scalebounds " << scalebounds << '\n';

          // wrap log with string, count factorizations
          string slog(log);
          string factorstr = "factorizations=";
          std::size_t index = slog.find(factorstr) + factorstr.length();
          factorizations += std::stoi(slog.substr(index, 3));
          count++;
        }
      }
    }
    double meanfactor = ((double)factorizations) / count;
    std::cerr << "n=" << setw(3) << n
              << ": average of " << meanfactor << " factorizations\n";
  }
  mju_free(res);
  mju_free(R);
  mju_free(index);
  mju_free(H);
  mju_free(g);
  mju_free(lower);
  mju_free(upper);
}

}  // namespace
}  // namespace mujoco
