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
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_misc.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::DoubleEq;
using ::testing::Pointwise;
using ::testing::DoubleNear;
using ::testing::ElementsAre;
using ::std::string;
using ::std::vector;
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

  // ADD_FAILURE() << log;  // uncomment to print `log` to error log

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

TEST_F(EngineUtilSolveTest, MjuCholFactorSymbolic) {
  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* d = mj_makeData(model);

  // Test matrix (upper triangular, representing symmetric matrix):
  //   [10  1  2  3]
  //   [ 0 10  0  0]
  //   [ 0  0 10  1]
  //   [ 0  0  0 10]
  //
  // mju_cholFactorSymbolic reads entries where col >= row

  int n = 4;
  mjtNum H[16] = {10, 1, 2, 3, 0, 10, 0, 0, 0, 0, 10, 1, 0, 0, 0, 10};

  // convert to sparse
  mjtNum sparseH[16];
  int H_rownnz[4], H_rowadr[4], H_colind[16];
  mju_dense2sparse(sparseH, H, n, n, H_rownnz, H_rowadr, H_colind, 16);

  // phase 1: counting
  int L_rownnz[4], L_rowadr[4];
  int LT_rownnz[4], LT_rowadr[4];
  int nnz = mju_cholFactorSymbolic(nullptr, L_rownnz, L_rowadr, nullptr,
                                   LT_rownnz, LT_rowadr, nullptr, H_rownnz,
                                   H_rowadr, H_colind, n, d);

  // verify counting phase outputs
  EXPECT_EQ(nnz, 8);
  // L (CSR) structure for reverse Cholesky (rows filled in reverse order)
  EXPECT_THAT(AsVector(L_rownnz, 4), ElementsAre(1, 2, 2, 3));
  EXPECT_THAT(AsVector(L_rowadr, 4), ElementsAre(0, 1, 3, 5));
  // LT (CSC) structure: transpose of L
  EXPECT_THAT(AsVector(LT_rownnz, 4), ElementsAre(4, 1, 2, 1));
  EXPECT_THAT(AsVector(LT_rowadr, 4), ElementsAre(0, 4, 5, 7));

  // phase 2: filling
  int L_colind[8], LT_colind[8], LT_pos[8];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_pos, H_rownnz, H_rowadr, H_colind, n, d);

  // verify L_colind
  EXPECT_THAT(AsVector(L_colind, 8), ElementsAre(0, 0, 1, 0, 2, 0, 2, 3));
  // Explanation (reverse Cholesky builds from bottom to top):
  //   Row 0 (1 entry): diagonal 0
  //   Row 1 (2 entries): col 0, then diagonal 1
  //   Row 2 (2 entries): col 0, then diagonal 2
  //   Row 3 (3 entries): col 0, col 2, then diagonal 3

  // verify LT_colind: transpose of L
  EXPECT_THAT(AsVector(LT_colind, 8), ElementsAre(0, 1, 2, 3, 1, 2, 3, 3));
  // Explanation:
  //   Column 0 (4 entries): rows 0, 1, 2, 3 (all have L[row, 0] != 0)
  //   Column 1 (1 entry): row 1
  //   Column 2 (2 entries): rows 2, 3
  //   Column 3 (1 entry): row 3

  // verify LT_pos: for each entry in LT, should point to correct position in L
  for (int c = 0; c < n; c++) {
    int adr = LT_rowadr[c];
    for (int k = 0; k < LT_rownnz[c]; k++) {
      int L_idx = LT_pos[adr + k];
      EXPECT_EQ(L_colind[L_idx], c)
          << "LT_pos mismatch at column " << c << ", entry " << k;
    }
  }

  mj_deleteData(d);
  mj_deleteModel(model);
}

// Test for mju_cholUpdate: rank-one Cholesky update L*L' +/- x*x'
// Verifies that after applying the update, reconstructing H = L*L' produces
// the expected result H_original +/- x*x'.
TEST_F(EngineUtilSolveTest, MjuCholUpdate) {
  std::mt19937_64 rng;
  rng.seed(42);
  std::normal_distribution<double> dist(0, 1);

  for (int n : {4, 6, 8}) {
    vector<mjtNum> H(n * n);
    vector<mjtNum> H_expected(n * n);
    vector<mjtNum> L_dense(n * n);
    vector<mjtNum> sqrtH(n * n);
    vector<mjtNum> x(n);
    vector<mjtNum> x_copy(n);
    vector<mjtNum> H_reconstructed(n * n);

    for (int flg_plus : {0, 1}) {
      // generate random lower-triangular matrix for sqrtH
      mju_zero(sqrtH.data(), n * n);
      for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
          sqrtH[n * i + j] = dist(rng);
        }
        sqrtH[n * i + i] = mju_abs(sqrtH[n * i + i]) + n;
      }

      // create SPD matrix H = sqrtH * sqrtH' (forward-order Cholesky: L*L')
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          mjtNum sum = 0;
          for (int k = 0; k <= mju_min(i, j); k++) {
            sum += sqrtH[n * i + k] * sqrtH[n * j + k];
          }
          H[n * i + j] = sum;
        }
      }

      // generate random update vector x
      for (int i = 0; i < n; i++) {
        x[i] = dist(rng) * 0.5;
      }

      // compute expected result: H_expected = H +/- x*x'
      mju_copy(H_expected.data(), H.data(), n * n);
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          if (flg_plus) {
            H_expected[i * n + j] += x[i] * x[j];
          } else {
            H_expected[i * n + j] -= x[i] * x[j];
          }
        }
      }

      // test using dense Cholesky (forward-order: L*L')
      mju_copy(L_dense.data(), H.data(), n * n);

      // dense factorization
      int rank_factor = mju_cholFactor(L_dense.data(), n, 0);
      EXPECT_EQ(rank_factor, n) << "Initial factorization failed";

      // make copy of x for update (it's modified in-place)
      mju_copy(x_copy.data(), x.data(), n);

      // apply dense rank-one update
      int rank_update =
          mju_cholUpdate(L_dense.data(), x_copy.data(), n, flg_plus);
      EXPECT_EQ(rank_update, n)
          << "Dense update rank loss for n=" << n << ", flg_plus=" << flg_plus;

      // zero out upper triangle (Cholesky only uses lower triangle)
      for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
          L_dense[i * n + j] = 0;
        }
      }

      // reconstruct H from L*L' and compare to expected
      mju_mulMatMatT(H_reconstructed.data(), L_dense.data(), L_dense.data(), n,
                     n, n);

      // compare
      mjtNum eps = 1e-8;
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          EXPECT_NEAR(H_reconstructed[i * n + j], H_expected[i * n + j], eps)
              << "Dense mismatch at (" << i << "," << j << ") for n=" << n
              << ", flg_plus=" << flg_plus;
        }
      }
    }
  }
}

// Test for mju_cholUpdateSparse: sparse rank-one Cholesky update L'*L +/- x*x'
// Uses sparse reverse Cholesky factorization and sparse rank-one update.
TEST_F(EngineUtilSolveTest, MjuCholUpdateSparse) {
  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* d = mj_makeData(model);

  std::mt19937_64 rng;
  rng.seed(123);
  std::normal_distribution<double> dist(0, 1);

  for (int n : {4, 6, 8}) {
    int max_nnz = n * (n + 1) / 2;

    vector<mjtNum> H(n * n);
    vector<mjtNum> H_lower(n * n);
    vector<mjtNum> H_expected(n * n);
    vector<mjtNum> sqrtH(n * n);
    vector<mjtNum> L_sparse(max_nnz);
    vector<int> rownnz(n);
    vector<int> rowadr(n);
    vector<int> colind(max_nnz);
    vector<mjtNum> x(n);
    vector<mjtNum> x_sparse(n);
    vector<int> x_ind(n);
    vector<mjtNum> L_sparse_dense(n * n);
    vector<mjtNum> H_sparse_reconstructed(n * n);

    for (int flg_plus : {0, 1}) {
      // generate random lower-triangular matrix for sqrtH
      mju_zero(sqrtH.data(), n * n);
      for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
          sqrtH[n * i + j] = dist(rng);
        }
        sqrtH[n * i + i] = mju_abs(sqrtH[n * i + i]) + n;
      }

      // create SPD matrix H = sqrtH * sqrtH' (forward-order: L*L')
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          mjtNum sum = 0;
          for (int k = 0; k <= mju_min(i, j); k++) {
            sum += sqrtH[n * i + k] * sqrtH[n * j + k];
          }
          H[n * i + j] = sum;
        }
      }

      // generate random update vector x
      for (int i = 0; i < n; i++) {
        x[i] = dist(rng) * 0.5;
      }

      // compute expected result: H_expected = H +/- x*x'
      mju_copy(H_expected.data(), H.data(), n * n);
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          if (flg_plus) {
            H_expected[i * n + j] += x[i] * x[j];
          } else {
            H_expected[i * n + j] -= x[i] * x[j];
          }
        }
      }

      // copy H to H_lower and zero upper triangle (sparse expects lower only)
      mju_copy(H_lower.data(), H.data(), n * n);
      for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
          H_lower[i * n + j] = 0;
        }
      }

      // convert lower-triangular H to sparse format
      mju_dense2sparse(L_sparse.data(), H_lower.data(), n, n, rownnz.data(),
                       rowadr.data(), colind.data(), max_nnz);

      // prepare sparse update vector (all elements, fully dense)
      int x_nnz = n;
      mju_copy(x_sparse.data(), x.data(), n);
      for (int i = 0; i < n; i++) {
        x_ind[i] = i;
      }

      // sparse Cholesky factorization (reverse-order: L'*L)
      mju_cholFactorSparse(L_sparse.data(), n, 0, rownnz.data(), rowadr.data(),
                           colind.data(), d);

      // apply sparse rank-one update
      int rank_sparse = mju_cholUpdateSparse(
          L_sparse.data(), x_sparse.data(), n, flg_plus, rownnz.data(),
          rowadr.data(), colind.data(), x_nnz, x_ind.data(), d);
      EXPECT_EQ(rank_sparse, n)
          << "Sparse update rank loss for n=" << n << ", flg_plus=" << flg_plus;

      // reconstruct H from L'*L and compare to expected
      mju_sparse2dense(L_sparse_dense.data(), L_sparse.data(), n, n,
                       rownnz.data(), rowadr.data(), colind.data());
      mju_mulMatTMat(H_sparse_reconstructed.data(), L_sparse_dense.data(),
                     L_sparse_dense.data(), n, n, n);

      // compare
      mjtNum eps = 1e-8;
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          EXPECT_NEAR(H_sparse_reconstructed[i * n + j], H_expected[i * n + j],
                      eps)
              << "Sparse mismatch at (" << i << "," << j << ") for n=" << n
              << ", flg_plus=" << flg_plus;
        }
      }
    }
  }

  mj_deleteData(d);
  mj_deleteModel(model);
}

// Test that mju_cholFactorSymbolic + mju_cholFactorNumeric produces identical
// results to the reference implementation mju_cholFactorSparse
TEST_F(EngineUtilSolveTest, CholFactorSymbolicNumeric) {
  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* d = mj_makeData(model);

  // test matrix with fill-in: upper triangle structure
  int n = 4;
  mjtNum H[16] = {10, 1, 2, 3, 1, 10, 0, 0, 2, 0, 10, 1, 3, 0, 1, 10};

  // convert to sparse (lower triangle only)
  mjtNum sparseH[16];
  int H_rownnz[4], H_rowadr[4], H_colind[16];
  mju_dense2sparse(sparseH, H, n, n, H_rownnz, H_rowadr, H_colind, 16);

  // transpose for upper triangle (needed by cholFactorSymbolic)
  int HT_rownnz[4], HT_rowadr[4], HT_colind[16];
  mju_transposeSparse(nullptr, nullptr, n, n, HT_rownnz, HT_rowadr, HT_colind,
                      nullptr, H_rownnz, H_rowadr, H_colind);

  // count fill-in (also computes LT structure)
  int L_rownnz[4], L_rowadr[4];
  int LT_rownnz[4], LT_rowadr[4];
  int nnz = mju_cholFactorSymbolic(nullptr, L_rownnz, L_rowadr, nullptr,
                                   LT_rownnz, LT_rowadr, nullptr, HT_rownnz,
                                   HT_rowadr, HT_colind, n, d);

  // filling phase: compute L_colind, LT_colind, and LT_pos
  int L_colind[16], LT_colind[16], LT_pos[16];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_pos, HT_rownnz, HT_rowadr, HT_colind, n,
                         d);

  // verify LT structure matches what we'd get from a separate transpose
  int LT_rownnz_ref[4], LT_rowadr_ref[4], LT_colind_ref[16];
  mju_transposeSparse(nullptr, nullptr, n, n, LT_rownnz_ref, LT_rowadr_ref,
                      LT_colind_ref, nullptr, L_rownnz, L_rowadr, L_colind);

  // LT structure should match
  EXPECT_THAT(AsVector(LT_rownnz, 4),
              ElementsAre(LT_rownnz_ref[0], LT_rownnz_ref[1], LT_rownnz_ref[2],
                          LT_rownnz_ref[3]));
  EXPECT_THAT(AsVector(LT_rowadr, 4),
              ElementsAre(LT_rowadr_ref[0], LT_rowadr_ref[1], LT_rowadr_ref[2],
                          LT_rowadr_ref[3]));

  // verify LT_colind and LT_pos match
  for (int r = 0; r < n; r++) {
    int adr = LT_rowadr[r];
    for (int k = 0; k < LT_rownnz[r]; k++) {
      int L_idx = LT_pos[adr + k];  // index in L array
      // verify L_colind at this position is indeed r
      EXPECT_EQ(L_colind[L_idx], r)
          << "LT_pos mismatch at LT[" << r << ", " << k << "]";
    }
  }

  // numeric factorization using new function
  mjtNum L_new[16];
  int rank_new = mju_cholFactorNumeric(
      L_new, n, 1e-10, L_rownnz, L_rowadr, L_colind, LT_rownnz, LT_rowadr,
      LT_colind, LT_pos, sparseH, H_rownnz, H_rowadr, H_colind, d);

  // reference implementation: copy sparse H into L_ref, then factor in-place
  mjtNum L_ref[16];
  int L_ref_rownnz[4], L_ref_colind[16];
  for (int r = 0; r < n; r++) {
    int nnz_r = H_rownnz[r];
    // count lower triangle elements for this row
    int lower_nnz = 0;
    for (int i = 0; i < nnz_r; i++) {
      if (H_colind[H_rowadr[r] + i] <= r) {
        L_ref[L_rowadr[r] + lower_nnz] = sparseH[H_rowadr[r] + i];
        L_ref_colind[L_rowadr[r] + lower_nnz] = H_colind[H_rowadr[r] + i];
        lower_nnz++;
      }
    }
    L_ref_rownnz[r] = lower_nnz;
  }
  int rank_ref = mju_cholFactorSparse(L_ref, n, 1e-10, L_ref_rownnz, L_rowadr,
                                      L_ref_colind, d);

  // compare results
  EXPECT_EQ(rank_new, rank_ref);
  EXPECT_EQ(rank_new, n);

  // compare L values
  mjtNum eps = 1e-10;
  for (int i = 0; i < nnz; i++) {
    EXPECT_NEAR(L_new[i], L_ref[i], eps) << "mismatch at index " << i;
  }

  mj_deleteData(d);
  mj_deleteModel(model);
}

}  // namespace
}  // namespace mujoco
