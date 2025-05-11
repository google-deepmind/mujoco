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

#include <cstddef>
#include <iostream>
#include <random>
#include <iomanip>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_misc.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleEq;
using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
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
  for (int i=0; i < n; i++) {
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

// small bounded QP with asymmetric Hessian (upper triangle ignored)
TEST_F(BoxQPTest, AsymmetricUpperIgnored) {
  // small arrays, allocate on stack
  static const int n = 2;
  mjtNum H[n*n] = {
    1, -400,
    0, 1
  };
  mjtNum g[n] = {1, 3};
  mjtNum res[n] = {0, 0};
  mjtNum lower[n] = {-2, -2};
  mjtNum upper[n] = {0, 0};
  int index[n];
  mjtNum R[n*(n+7)];

  // solve box-QP
  int nfree = mju_boxQP(res, R, index, H, g, n, lower, upper);

  EXPECT_EQ(nfree, 1);

  EXPECT_THAT(res[0], DoubleEq(-g[0]/H[0]));
  EXPECT_THAT(res[1], DoubleEq(lower[1]));
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
          int num_factor = std::stoi(slog.substr(index, 3));

          // never more than 6 factorizations
          EXPECT_LE(num_factor, 6);

          factorizations += num_factor;
          count++;
        }
      }
    }
    double meanfactor = ((double)factorizations) / count;

    // average of 4.5 factorizations is expected
    EXPECT_LE(meanfactor, 5.0);

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

// ------------------------- band matrices -------------------------------------

using BandMatrixTest = MujocoTest;

// utility: random "arrowhead", banded-then-dense SPD matrix
//   optional random vector and diagonal regularizer
void randomBanded(mjtNum* H, int nTotal, int nBand, int nDense, int seed,
                  mjtNum* vec = nullptr, mjtNum reg = 0) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // allocate square root
  mjtNum* sqrtH = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal*nTotal);

  // sample
  for (int i=0; i < nTotal; i++) {
    if (vec) vec[i] = dist(rng);
    for (int j=0; j < nTotal; j++) {
      sqrtH[nTotal*i+j] = dist(rng);
    }
  }

  // make SPD matrix H
  mju_mulMatTMat(H, sqrtH, sqrtH, nTotal, nTotal, nTotal);

  // set zeros
  int nSparse = nTotal-nDense;
  for (int i=0; i < nSparse; i++) {
    int nzeros = mjMAX(0, i + 1 - nBand);
    for (int j=0; j < nzeros; j++) {
      H[nTotal*i + j] = 0;
      H[nTotal*j + i] = 0;
    }
  }

  // add regularizer to diagonal
  for (int i=0; i < nTotal; i++) {
    H[nTotal*i + i] += reg;
  }

  mju_free(sqrtH);
}


// test banded-vector diagonal values
TEST_F(BandMatrixTest, Diagonal) {
  int seed = 1;
  int nTotal = 8;
  for (int nBand : {1, 3}) {
    for (int nDense : {0, 2}) {
      // allocate
      int nB = (nTotal-nDense)*nBand + nDense*nTotal;
      mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*nB);
      int nH = nTotal*nTotal;
      mjtNum* H = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);

      // make random banded SPD matrix, dense representation
      randomBanded(H, nTotal, nBand, nDense, seed++);

      // convert to banded representation
      mju_dense2Band(B, H, nTotal, nBand, nDense);

      // expect diagonals to be equal
      for (int i=0; i < nTotal; i++) {
        EXPECT_EQ(H[i*nTotal + i], B[mju_bandDiag(i, nTotal, nBand, nDense)]);
      }

      mju_free(H);
      mju_free(B);
    }
  }
}

// test conversion of banded <-> dense
TEST_F(BandMatrixTest, Conversion) {
  int seed = 1;
  int nTotal = 8;
  for (int nBand : {0, 1, 3}) {
    for (int nDense : {0, 2}) {
      // allocate
      int nB = (nTotal-nDense)*nBand + nDense*nTotal;
      mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*nB);
      int nH = nTotal*nTotal;
      mjtNum* H = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
      mjtNum* H1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);

      // make random banded SPD matrix, dense
      randomBanded(H, nTotal, nBand, nDense, seed++);

      // convert to banded
      mju_dense2Band(B, H, nTotal, nBand, nDense);

      // convert back to dense
      mju_band2Dense(H1, B, nTotal, nBand, nDense, /*flg_sym=*/1);

      // expect exact equality
      EXPECT_EQ(AsVector(H, nH), AsVector(H1, nH));

      mju_free(H1);
      mju_free(H);
      mju_free(B);
    }
  }
}

// test banded-vector multiplication
TEST_F(BandMatrixTest, Multiplication) {
  int seed = 1;
  int nTotal = 8;
  for (int nBand : {0, 1, 3}) {
    for (int nDense : {0, 2}) {
      // allocate
      int nB = (nTotal-nDense)*nBand + nDense*nTotal;
      mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*nB);
      int nH = nTotal*nTotal;
      mjtNum* H = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
      mjtNum* vec = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
      mjtNum* res = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
      mjtNum* res1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);

      // make random banded SPD matrix, dense
      randomBanded(H, nTotal, nBand, nDense, seed++, vec);

      // multiply dense
      mju_mulMatVec(res, H, vec, nTotal, nTotal);

      // convert to banded, multiply
      mju_dense2Band(B, H, nTotal, nBand, nDense);
      mju_bandMulMatVec(res1, B, vec, nTotal, nBand, nDense,
                        /*nVec=*/1, /*flg_sym=*/1);

      // expect numerical equality
      mjtNum eps = 1e-12;
      EXPECT_THAT(AsVector(res, nTotal),
                  Pointwise(DoubleNear(eps), AsVector(res1, nTotal)));

      mju_free(res1);
      mju_free(res);
      mju_free(vec);
      mju_free(H);
      mju_free(B);
    }
  }
}

// test banded factorization and vector product with factor
TEST_F(BandMatrixTest, Factorization) {
  int seed = 1;
  int nTotal = 8;
  for (int nBand : {1, 3}) {
    for (int nDense : {0, 2}) {
      for (mjtNum diagadd : {0.0, 1.0}) {
        for (int diagmul : {0.0, 1.3}) {
          // allocate
          int nB = (nTotal-nDense)*nBand + nDense*nTotal;
          mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*nB);
          int nH = nTotal*nTotal;
          mjtNum* H = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
          mjtNum* H1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
          mjtNum* vec = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
          mjtNum* res = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
          mjtNum* res1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);

          // make random banded matrix, dense representation
          //  add regularizer to ensure PD
          randomBanded(H, nTotal, nBand, nDense, seed++, vec, /*reg=*/nTotal);

          // convert to banded
          mju_dense2Band(B, H, nTotal, nBand, nDense);

          // apply diagadd and diagmul
          for (int i=0; i < nTotal; i++) {
            H[nTotal*i + i] += diagadd + diagmul*H[nTotal*i + i];
          }

          // in-place dense factorization
          int rank = mju_cholFactor(H, nTotal, /*mindiag=*/0);

          // expect factorization to have succeeded
          EXPECT_EQ(rank, nTotal);

          // banded factorization
          mjtNum minDiag = mju_cholFactorBand(B, nTotal, nBand, nDense,
                                              diagadd, diagmul);

          // expect factorization to have succeeded
          EXPECT_GT(minDiag, 0);

          // convert back to dense, lower triangle only
          mju_band2Dense(H1, B, nTotal, nBand, nDense, /*flg_sym=*/0);

          // zero upper triangle of H (unused)
          for (int i=0; i < nTotal-1; i++) {
            mju_zero(H + nTotal*i + i + 1, nTotal - i - 1);
          }

          // expect numerical equality
          mjtNum eps = 1e-12;
          EXPECT_THAT(AsVector(H, nH),
                      Pointwise(DoubleNear(eps), AsVector(H1, nH)));

          // multiply dense
          mju_mulMatVec(res, H, vec, nTotal, nTotal);

          // multiply sparse, only lower triangle
          mju_bandMulMatVec(res1, B, vec, nTotal, nBand, nDense,
                            /*nVec=*/1, /*flg_sym=*/0);

          // expect numerical equality
          EXPECT_THAT(AsVector(res, nTotal),
                      Pointwise(DoubleNear(eps), AsVector(res1, nTotal)));

          mju_free(res1);
          mju_free(res);
          mju_free(vec);
          mju_free(H1);
          mju_free(H);
          mju_free(B);
        }
      }
    }
  }
}

// test banded solve
TEST_F(BandMatrixTest, Solve) {
  int seed = 1;
  int nTotal = 8;
  for (int nBand : {1, 3}) {
    for (int nDense : {0, 2}) {
      // allocate
      int nB = (nTotal-nDense)*nBand + nDense*nTotal;
      mjtNum* B = (mjtNum*) mju_malloc(sizeof(mjtNum)*nB);
      int nH = nTotal*nTotal;
      mjtNum* H = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
      mjtNum* H1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nH);
      mjtNum* vec = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
      mjtNum* res = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);
      mjtNum* res1 = (mjtNum*) mju_malloc(sizeof(mjtNum)*nTotal);

      // make random banded matrix, dense representation
      //  add regularizer to ensure PD
      randomBanded(H, nTotal, nBand, nDense, seed++, vec, /*reg=*/nTotal);

      // convert to banded
      mju_dense2Band(B, H, nTotal, nBand, nDense);

      // in-place dense factorization
      int rank = mju_cholFactor(H, nTotal, /*mindiag=*/0);

      // expect factorization to have succeeded
      EXPECT_EQ(rank, nTotal);

      // banded factorization
      mjtNum minDiag = mju_cholFactorBand(B, nTotal, nBand, nDense,
                                          /*diagadd=*/0, /*diagmul=*/0);

      // expect factorization to have succeeded
      EXPECT_GT(minDiag, 0);

      // convert back to dense, lower triangle only
      mju_band2Dense(H1, B, nTotal, nBand, nDense, /*flg_sym=*/0);

      // solve with dense
      mju_cholSolve(res, H, vec, nTotal);

      // solve with banded
      mju_cholSolveBand(res1, B, vec, nTotal, nBand, nDense);

      // expect numerical equality
      mjtNum eps = 1e-12;
      EXPECT_THAT(AsVector(res, nTotal),
                  Pointwise(DoubleNear(eps), AsVector(res1, nTotal)));

      mju_free(res1);
      mju_free(res);
      mju_free(vec);
      mju_free(H1);
      mju_free(H);
      mju_free(B);
    }
  }
}

using EngineUtilSolveTest = MujocoTest;

TEST_F(EngineUtilSolveTest, MjuCholFactorNNZ) {
  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* d = mj_makeData(model);

  int nA = 2;
  mjtNum matA[4] = {1, 0,
                    0, 1};
  mjtNum sparseA[4];
  int rownnzA[2];
  int rowadrA[2];
  int colindA[4];
  int rownnzA_factor[2];
  mju_dense2sparse(sparseA, matA, nA, nA, rownnzA, rowadrA, colindA, 4);
  int nnzA = mju_cholFactorCount(rownnzA_factor,
                                 rownnzA, rowadrA, colindA, nA, d);

  EXPECT_EQ(nnzA, 2);
  EXPECT_THAT(AsVector(rownnzA_factor, 2), ElementsAre(1, 1));

  int nB = 3;
  mjtNum matB[9] = {10, 1, 0,
                    0, 10, 1,
                    0, 0, 10};
  mjtNum sparseB[9];
  int rownnzB[3];
  int rowadrB[3];
  int colindB[9];
  int rownnzB_factor[3];
  mju_dense2sparse(sparseB, matB, nB, nB, rownnzB, rowadrB, colindB, 9);
  int nnzB = mju_cholFactorCount(rownnzB_factor,
                                 rownnzB, rowadrB, colindB, nB, d);

  EXPECT_EQ(nnzB, 5);
  EXPECT_THAT(AsVector(rownnzB_factor, 3), ElementsAre(1, 2, 2));

  int nC = 3;
  mjtNum matC[9] = {10, 1, 0,
                    0, 10, 0,
                    0, 0, 10};
  mjtNum sparseC[9];
  int rownnzC[3];
  int rowadrC[3];
  int colindC[9];
  int rownnzC_factor[3];
  mju_dense2sparse(sparseC, matC, nC, nC, rownnzC, rowadrC, colindC, 9);
  int nnzC = mju_cholFactorCount(rownnzC_factor,
                                 rownnzC, rowadrC, colindC, nC, d);

  EXPECT_EQ(nnzC, 4);
  EXPECT_THAT(AsVector(rownnzC_factor, 3), ElementsAre(1, 2, 1));

  int nD = 4;
  mjtNum matD[16] = {10, 1, 2, 3,
                     0, 10, 0, 0,
                     0, 0, 10, 1,
                     0, 0, 0, 10};
  mjtNum sparseD[16];
  int rownnzD[4];
  int rowadrD[4];
  int colindD[16];
  int rownnzD_factor[4];
  mju_dense2sparse(sparseD, matD, nD, nD, rownnzD, rowadrD, colindD, 16);
  int nnzD = mju_cholFactorCount(rownnzD_factor,
                                 rownnzD, rowadrD, colindD, nD, d);

  EXPECT_EQ(nnzD, 8);
  EXPECT_THAT(AsVector(rownnzD_factor, 4), ElementsAre(1, 2, 2, 3));

  mj_deleteData(d);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
