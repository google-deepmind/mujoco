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

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "src/engine/engine_util_blas.h"
#include "src/engine/engine_util_misc.h"
#include "src/engine/engine_util_spatial.h"
#include "src/engine/engine_util_sparse.h"
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::Pointwise;

using ::std::setw;
using ::std::string;
using ::std::vector;
using ::testing::ElementsAre;
using QCQP2Test = MujocoTest;

TEST_F(QCQP2Test, DegenerateAMatrix) {
  // A 2x2 matrix with determinant zero.
  const mjtNum Ain[9]{6, -15, 2, -5};

  // Any values will do for these three inputs.
  const mjtNum bin[3]{-12, 49};
  const mjtNum d[3]{11, 31};
  const mjtNum r = 0.01;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[2]{999, 999};

  EXPECT_EQ(mju_QCQP2(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
}

using QCQP3Test = MujocoTest;

TEST_F(QCQP3Test, DegenerateAMatrix) {
  // A 3x3 matrix with determinant zero.
  const mjtNum Ain[9]{1, 4, -2, -3, -7, 5, 2, -9, 0};

  // Any values will do for these three inputs.
  const mjtNum bin[3]{-12, 49, 8};
  const mjtNum d[3]{11, 31, -23};
  const mjtNum r = 0.1;

  // Make output array explicitly nonzero to simulate uninitialized memory.
  mjtNum res[3]{999, 999, 999};

  EXPECT_EQ(mju_QCQP3(res, Ain, bin, d, r), 0);
  EXPECT_EQ(res[0], 0);
  EXPECT_EQ(res[1], 0);
  EXPECT_EQ(res[2], 0);
}

// --------------------------- mju_boxQP ---------------------------------------

using BoxQPTest = MujocoTest;

// utility: test if res is the minimum of a given box-QP problem
bool isQPminimum(const mjtNum* res, const mjtNum* H, const mjtNum* g, int n,
                 const mjtNum* lower, const mjtNum* upper) {
  const mjtNum eps = MjEps(1e-4, 5e-2);      // epsilon used for nudging
  const mjtNum threshold = MjTol(0, -2e-3);  // comparison threshold
  for (int i = 0; i < n; i++) {
    mjtNum grad_i = mju_dot(H + i * n, res, n) + g[i];
    mjtNum Hii = H[i * n + i];

    // nudge down
    mjtNum lo = res[i] - eps;
    if (lower) {
      lo = mju_max(lower[i], lo);
    }
    mjtNum dlo = lo - res[i];
    if (dlo * grad_i + 0.5 * Hii * dlo * dlo < threshold) {
      return false;
    }

    // nudge up
    mjtNum hi = res[i] + eps;
    if (upper) {
      hi = mju_min(upper[i], hi);
    }
    mjtNum dhi = hi - res[i];
    if (dhi * grad_i + 0.5 * Hii * dhi * dhi < threshold) {
      return false;
    }
  }
  return true;
}

// utility: define QP with pseudorandom values
void randomBoxQP(int n, mjtNum* H, mjtNum* g, mjtNum* lower, mjtNum* upper,
                 int seed) {
  // make distribution using seed
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);

  // square root of H
  mjtNum* sqrtH = (mjtNum*)mju_malloc(sizeof(mjtNum) * n * n);

  for (int i = 0; i < n; i++) {
    g[i] = dist(rng);
    lower[i] = 5 * dist(rng);
    upper[i] = 5 * dist(rng);

    // fix invalid bounds
    if (lower[i] > upper[i]) {
      mjtNum tmp = upper[i];
      upper[i] = lower[i];
      lower[i] = tmp;
    }

    // sample temp
    for (int j = 0; j < n; j++) {
      sqrtH[n * i + j] = dist(rng);
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
  mjtNum H[n * n] = {2, 0, 0, 2};
  mjtNum g[n] = {1, 3};
  mjtNum res[n] = {0, 0};
  mjtNum R[n * (n + 7)];

  int nfree = mju_boxQP(res, R, /*index=*/nullptr, H, g, n,
                        /*lower=*/nullptr, /*upper=*/nullptr);

  // no bounds, expect Newton point
  EXPECT_EQ(nfree, 2);
  EXPECT_MJTNUM_EQ(res[0], -g[0] / H[0]);
  EXPECT_MJTNUM_EQ(res[1], -g[1] / H[3]);

  // check that solution is actual minimum
  EXPECT_TRUE(isQPminimum(res, H, g, n, /*lower=*/nullptr, /*upper=*/nullptr));

  // perturb solution, expected it no longer be the minimum
  res[0] += MjTol(0.001, 0.1);
  EXPECT_FALSE(isQPminimum(res, H, g, n, /*lower=*/nullptr, /*upper=*/nullptr));

  // negative-definite Hessian, no solution
  H[0] = -1;
  nfree = mju_boxQP(res, R, /*index=*/nullptr, H, g, n,
                    /*lower=*/nullptr, /*upper=*/nullptr);

  EXPECT_EQ(nfree, -1);
}

TEST_F(BoxQPTest, LineSearchFailureRejectsCandidateAndStops) {
  static const int n = 2;
  mjtNum H[n * n] = {100, 90, 90, 100};
  mjtNum g[n] = {-190, 0};
  mjtNum lower[n] = {-100, 0};
  mjtNum upper[n] = {100, 100};
  mjtNum res[n] = {0, 0};
  int index[n];
  mjtNum R[n * (n + 7)];
  char log[1024] = {0};

  // Clipped step=1 increases objective (+3100 > 0); backtrack=0.5 < minstep=0.6
  // fails line search.
  mju_boxQPoption(res, R, index, H, g, n, lower, upper, /*maxiter=*/5,
                  /*mingrad=*/MjTol(1e-12, 1e-5), /*backtrack=*/0.5,
                  /*minstep=*/0.6, /*armijo=*/0.1, log, sizeof(log));

  EXPECT_MJTNUM_EQ(res[0], 0);
  EXPECT_MJTNUM_EQ(res[1], 0);
  EXPECT_THAT(std::string(log),
              ::testing::HasSubstr("Maximum line-search iterations exceeded"));
}

TEST_F(BoxQPTest, LineSearchEvaluatesArmijoBeforeBacktrack) {
  static constexpr int n = 2;
  mjtNum H[n * n] = {100.0, -90.0, -90.0, 100.0};
  mjtNum g[n] = {-10.0, -10.0};
  mjtNum lower[n] = {-10.0, -10.0};
  mjtNum upper[n] = {0.58, 10.0};
  mjtNum res[n] = {0.0, 0.0};
  mjtNum R[n * (n + 7)] = {0.0};
  int index[n] = {-1, -1};

  // At step=1.0, clipped candidate (0.58, 1.0) has Armijo ratio 0.059 < 0.1;
  // backtracks to 0.5.
  int nfree = mju_boxQPoption(res, R, index, H, g, n, lower, upper,
                              /*maxiter=*/1, /*mingrad=*/MjTol(1e-12, 1e-5),
                              /*backtrack=*/0.5, /*minstep=*/MjTol(1e-10, 1e-6),
                              /*armijo=*/0.1, nullptr, 0);
  EXPECT_EQ(nfree, 2);
  EXPECT_MJTNUM_EQ(res[0], 0.5);
  EXPECT_MJTNUM_EQ(res[1], 0.5);
}

TEST_F(BoxQPTest, UnboundedUpperTrianglePoisoned) {
  static constexpr int n = 2;
  mjtNum H[n * n] = {4.0, -999.0, 2.0, 5.0};
  mjtNum g[n] = {-4.0, -2.0};
  mjtNum res[n] = {0.0, 0.0};
  mjtNum R[n * (n + 7)] = {77.0, 77.0, 77.0, 77.0};
  int index[n] = {-1, -1};

  int nfree = mju_boxQP(res, R, index, H, g, n, nullptr, nullptr);
  EXPECT_EQ(nfree, n);
  EXPECT_MJTNUM_EQ(R[1], 77.0);
}

// small bounded QP with asymmetric Hessian (upper triangle ignored)
TEST_F(BoxQPTest, AsymmetricUpperIgnored) {
  // small arrays, allocate on stack
  static const int n = 2;
  mjtNum H[n * n] = {1, -400, 0, 1};
  mjtNum g[n] = {1, 3};
  mjtNum res[n] = {0, 0};
  mjtNum lower[n] = {-2, -2};
  mjtNum upper[n] = {0, 0};
  int index[n];
  mjtNum R[n * (n + 7)];

  // solve box-QP
  int nfree = mju_boxQP(res, R, index, H, g, n, lower, upper);

  EXPECT_EQ(nfree, 1);

  EXPECT_MJTNUM_EQ(res[0], -g[0] / H[0]);
  EXPECT_MJTNUM_EQ(res[1], lower[1]);
}

// verify mju_boxQP reads only the lower triangle of H by poisoning the upper
// triangle with NaN and comparing to a clean symmetric solve
TEST_F(BoxQPTest, UpperTrianglePoisoned) {
  int n = 30;
  const mjtNum nan = std::numeric_limits<mjtNum>::quiet_NaN();

  // allocate on heap
  mjtNum *H, *g, *lower, *upper;  // inputs
  mjtNum *res, *R;                // outputs
  int* index;                     // outputs
  mju_boxQPmalloc(&res, &R, &index, &H, &g, n, &lower, &upper);

  struct Cleanup {
    mjtNum *res, *R, *H, *g, *lower, *upper;
    int* index;
    ~Cleanup() {
      mju_free(res);
      mju_free(R);
      mju_free(index);
      mju_free(H);
      mju_free(g);
      mju_free(lower);
      mju_free(upper);
    }
  } cleanup{res, R, H, g, lower, upper, index};

  // generate a symmetric SPD Hessian and bounded QP problem
  randomBoxQP(n, H, g, lower, upper, /*seed=*/1);

  int maxiter = 100;
  mjtNum mingrad = MjTol(1E-16, 1E-5);
  mjtNum backtrack = 0.5;
  mjtNum minstep = MjTol(1E-22, 1E-10);
  mjtNum armijo = 0.1;

  // solve with symmetric H to get the reference result
  mju_zero(res, n);
  int nfree_ref =
      mju_boxQPoption(res, R, index, H, g, n, lower, upper, maxiter, mingrad,
                      backtrack, minstep, armijo, nullptr, 0);
  ASSERT_GT(nfree_ref, -1);

  // save reference
  std::vector<mjtNum> res_ref(res, res + n);
  std::vector<int> index_ref(index, index + n);
  std::vector<mjtNum> R_ref(R, R + nfree_ref * nfree_ref);

  // poison the strict upper triangle of H with NaN
  for (int i = 0; i < n; i++) {
    for (int j = i + 1; j < n; j++) {
      H[n * i + j] = nan;
    }
  }

  // solve again; result must match because only lower triangle should be read
  mju_zero(res, n);
  int nfree_poisoned =
      mju_boxQPoption(res, R, index, H, g, n, lower, upper, maxiter, mingrad,
                      backtrack, minstep, armijo, nullptr, 0);

  EXPECT_EQ(nfree_poisoned, nfree_ref);
  for (int i = 0; i < n; i++) {
    EXPECT_EQ(res[i], res_ref[i]) << "mismatch at index " << i;
  }
  for (int i = 0; i < nfree_ref; i++) {
    EXPECT_EQ(index[i], index_ref[i]) << "index mismatch at " << i;
    for (int j = 0; j <= i; j++) {
      int k = i * nfree_ref + j;
      EXPECT_EQ(R[k], R_ref[k]) << "R mismatch at row " << i << ", col " << j;
    }
  }
}

// test mju_boxQP on a single random bounded QP
TEST_F(BoxQPTest, BoundedQP) {
  int n = 50;  // problem size

  // allocate on heap
  mjtNum *H, *g, *lower, *upper;  // inputs
  mjtNum *res, *R;                // outputs
  int* index;                     // outputs
  mju_boxQPmalloc(&res, &R, &index, &H, &g, n, &lower, &upper);

  randomBoxQP(n, H, g, lower, upper, /*seed=*/1);

  // initialize res
  mju_zero(res, n);

  // use default options
  int maxiter = 100;  // maximum number of iterations
  mjtNum mingrad =
      MjTol(1E-16, 1E-5);  // minimum squared norm of (unclamped) gradient
  mjtNum backtrack = 0.5;  // backtrack factor for decreasing stepsize
  mjtNum minstep = MjTol(1E-22, 1E-10);  // minimum stepsize for linesearch
  mjtNum armijo = 0.1;                   // Armijo parameter

  // logging
  static const int logsz = 10000;
  char log[logsz];

  int nfree = mju_boxQPoption(res, R, index, H, g, n, lower, upper, maxiter,
                              mingrad, backtrack, minstep, armijo, log, logsz);

  // ADD_FAILURE() << log;  // uncomment to print `log` to error log

  // check solution
  EXPECT_GT(nfree, -1);
  EXPECT_TRUE(isQPminimum(res, H, g, n, lower, upper));

  // verify clamping
  int j = nfree > 0 ? 0 : -1;
  for (int i = 0; i < n; i++) {
    if (j >= 0 && i == index[j]) {  // free dimension
      EXPECT_GT(res[i], lower[i]);
      EXPECT_LT(res[i], upper[i]);
      j++;
    } else {  // clamped dimension
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

          mju_scl(H, H, scaleH, n * n);
          mju_scl(g, g, scaleg, n);
          mju_scl(lower, lower, scalebounds, n);
          mju_scl(upper, upper, scalebounds, n);

          // initialize with zeros
          mju_zero(res, n);

          // default algorithm options
          int maxiter = 100;
          mjtNum mingrad = MjTol(1E-16, 1E-5);
          mjtNum backtrack = 0.5;
          mjtNum minstep = MjTol(1E-22, 1E-10);
          mjtNum armijo = 0.1;

          // solve box-QP with logging
          int nfree =
              mju_boxQPoption(res, R, index, H, g, n, lower, upper, maxiter,
                              mingrad, backtrack, minstep, armijo, log, logsz);

          // check solution
          EXPECT_GT(nfree, -1) << log;
          EXPECT_TRUE(isQPminimum(res, H, g, n, lower, upper))
              << "n " << n << '\n'
              << "scaleH " << scaleH << '\n'
              << "scaleg " << scaleg << '\n'
              << "scalebounds " << scalebounds << '\n'
              << log;

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

    std::cerr << "n=" << setw(3) << n << ": average of " << meanfactor
              << " factorizations\n";
  }
  mju_free(res);
  mju_free(R);
  mju_free(index);
  mju_free(H);
  mju_free(g);
  mju_free(lower);
  mju_free(upper);
}

// ----------------------------- mju_QP ----------------------------------------

using QPTest = MujocoTest;

// a quadratic program and the outputs of mju_QP
struct QP {
  int n;
  int m;
  vector<mjtNum> H, g, lower, upper, A, alower, aupper, threshold, compliance;
  vector<mjtNum> res, dual;
  vector<int> active;

  QP(int n, int m)
      : n(n),
        m(m),
        H(n * n),
        g(n),
        lower(n, -mjMAXVAL),
        upper(n, mjMAXVAL),
        A(n * m),
        alower(m, -mjMAXVAL),
        aupper(m, mjMAXVAL),
        threshold(n + m, 0),
        compliance(n + m, 0),
        res(n),
        dual(n + m),
        active(n + m, 0) {}

  int Solve(mjtNum* work = nullptr) {
    return mju_QP(res.data(), dual.data(), active.data(), work, H.data(),
                  g.data(), n, lower.data(), upper.data(), A.data(),
                  alower.data(), aupper.data(), m, threshold.data(),
                  compliance.data());
  }

  // largest violation of the optimality conditions, relative to problem scale
  mjtNum Residual() const {
    // stationarity: H*x + g + A'*dual = 0
    vector<mjtNum> grad(n);
    mju_mulMatVec(grad.data(), H.data(), res.data(), n, n);
    mjtNum scale = 1 + mju_norm(g.data(), n);
    for (int i = 0; i < n; i++) {
      grad[i] += g[i] + dual[i];
      for (int j = 0; j < m; j++) {
        grad[i] += A[j * n + i] * dual[n + j];
      }
    }
    mjtNum residual = mju_norm(grad.data(), n) / scale;

    for (int c = 0; c < n + m; c++) {
      mjtNum lo = c < n ? lower[c] : alower[c - n];
      mjtNum hi = c < n ? upper[c] : aupper[c - n];
      mjtNum val = c < n ? res[c] : mju_dot(&A[(c - n) * n], res.data(), n);
      mjtNum norm = c < n ? 1 : mju_norm(&A[(c - n) * n], n);
      mjtNum lam = dual[c];

      // violation permitted by the multiplier, and its precision
      mjtNum slack = compliance[c] * mju_max(0, mju_abs(lam) - threshold[c]);
      mjtNum eps = std::numeric_limits<mjtNum>::epsilon();
      mjtNum precision = 4 * eps * compliance[c] * mju_abs(lam);

      // primal feasibility
      mjtNum viol = mju_max(val - hi - (lam > 0 ? slack : 0),
                            lo - val - (lam < 0 ? slack : 0));
      viol = mju_max(0, viol - precision);
      residual = mju_max(residual, viol / (norm + mju_abs(val)));

      // complementarity
      if (lam > 0) {
        viol = mju_max(0, mju_abs(val - hi - slack) - precision);
        residual = mju_max(residual, viol / (norm + mju_abs(hi) + slack));
      } else if (lam < 0) {
        viol = mju_max(0, mju_abs(val - lo + slack) - precision);
        residual = mju_max(residual, viol / (norm + mju_abs(lo) + slack));
      }

      // only soft constraints yield
      if (std::abs(active[c]) == 2 && compliance[c] == 0) {
        residual = 1;
      }
    }
    return residual;
  }

  // number of constraints in a given state, regardless of sign
  int Count(int state) const {
    int count = 0;
    for (int a : active) {
      count += std::abs(a) == state;
    }
    return count;
  }
};

// make a random QP with neq equalities, feasible if all constraints are hard
QP RandomQP(int n, int m, int neq, int seed) {
  std::mt19937_64 rng;
  rng.seed(seed);
  std::normal_distribution<double> dist(0, 1);
  std::uniform_real_distribution<double> uniform(0, 1);

  QP qp(n, m);

  // positive definite H
  vector<mjtNum> sqrtH(n * n);
  for (int i = 0; i < n * n; i++) {
    sqrtH[i] = dist(rng);
  }
  mju_mulMatTMat(qp.H.data(), sqrtH.data(), sqrtH.data(), n, n, n);
  for (int i = 0; i < n; i++) {
    qp.H[i * (n + 1)] += 0.1;
    qp.g[i] = 3 * dist(rng);
  }

  // feasible point
  vector<mjtNum> x(n);
  for (int i = 0; i < n; i++) {
    x[i] = dist(rng);
  }

  // bounds: half are absent on each side
  for (int i = 0; i < n; i++) {
    if (uniform(rng) < 0.5) qp.lower[i] = x[i] - uniform(rng);
    if (uniform(rng) < 0.5) qp.upper[i] = x[i] + uniform(rng);
  }

  // rows: equalities, then inequalities with a quarter absent on each side
  for (int j = 0; j < m; j++) {
    for (int i = 0; i < n; i++) {
      qp.A[j * n + i] = dist(rng);
    }
    mjtNum val = mju_dot(&qp.A[j * n], x.data(), n);
    if (j < neq) {
      qp.alower[j] = qp.aupper[j] = val;
    } else {
      if (uniform(rng) < 0.75) qp.alower[j] = val - uniform(rng);
      if (uniform(rng) < 0.75) qp.aupper[j] = val + uniform(rng);
    }
  }

  return qp;
}

// without constraints the solution is the Newton point
TEST_F(QPTest, Unconstrained) {
  QP qp = RandomQP(5, 0, 0, 1);
  int n = qp.n;
  EXPECT_EQ(mju_QP(qp.res.data(), nullptr, nullptr, nullptr, qp.H.data(),
                   qp.g.data(), n, nullptr, nullptr, nullptr, nullptr, nullptr,
                   0, nullptr, nullptr),
            0);

  // expect H*res + g = 0
  vector<mjtNum> grad(n);
  mju_mulMatVec(grad.data(), qp.H.data(), qp.res.data(), n, n);
  mju_addTo(grad.data(), qp.g.data(), n);
  EXPECT_LT(mju_norm(grad.data(), n), MjTol(1e-14, 1e-5));
}

// a small problem with a known solution
TEST_F(QPTest, KnownSolution) {
  // min 0.5*|x|^2 - x0 - x1  s.t.  x0 + x1 <= 1,  x0 - x1 = 0.2
  mjtNum H[4] = {1, 0, 0, 1};
  mjtNum g[2] = {-1, -1};
  mjtNum A[4] = {1, 1, 1, -1};
  mjtNum alower[2] = {-mjMAXVAL, 0.2};
  mjtNum aupper[2] = {1, 0.2};

  mjtNum res[2], dual[4];
  int active[4] = {0, 0, 0, 0};
  EXPECT_EQ(mju_QP(res, dual, active, nullptr, H, g, 2, nullptr, nullptr, A,
                   alower, aupper, 2, nullptr, nullptr),
            2);
  EXPECT_THAT(res, Pointwise(MjNear(1e-15, 1e-6), {0.6, 0.4}));
  EXPECT_THAT(dual, Pointwise(MjNear(1e-15, 1e-6), {0.0, 0.0, 0.5, -0.1}));
  EXPECT_THAT(active, ElementsAre(0, 0, 1, -1));
}

// with bounds only, the solution matches mju_boxQP
TEST_F(QPTest, MatchesBoxQP) {
  for (int n : {3, 10, 40}) {
    mjtNum *H, *g, *lower, *upper, *res, *R;
    int* index;
    mju_boxQPmalloc(&res, &R, &index, &H, &g, n, &lower, &upper);
    for (int seed = 0; seed < 10; seed++) {
      randomBoxQP(n, H, g, lower, upper, seed);

      // improve conditioning, so that solutions are comparable
      for (int i = 0; i < n; i++) {
        H[i * (n + 1)] += 1;
      }
      mju_zero(res, n);
      ASSERT_GT(mju_boxQP(res, R, index, H, g, n, lower, upper), -1);

      vector<mjtNum> x(n);
      int nactive =
          mju_QP(x.data(), nullptr, nullptr, nullptr, H, g, n, lower, upper,
                 nullptr, nullptr, nullptr, 0, nullptr, nullptr);
      EXPECT_GE(nactive, 0);
      for (int i = 0; i < n; i++) {
        EXPECT_NEAR(x[i], res[i], MjTol(1e-13, 1e-4));
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
}

// optimality conditions hold on random feasible problems
TEST_F(QPTest, RandomFeasible) {
  int seed = 0;
  int nactive_total = 0;
  for (int n : {2, 7, 30}) {
    for (int m : {0, 3, 2 * n, 6 * n}) {
      for (int neq : {0, 1, n / 2}) {
        if (neq > m) continue;
        for (int rep = 0; rep < 5; rep++) {
          QP qp = RandomQP(n, m, neq, seed++);
          int nactive = qp.Solve();
          EXPECT_GE(nactive, neq) << "n " << n << " m " << m << " neq " << neq;
          EXPECT_LT(qp.Residual(), MjTol(1e-11, 5e-3))
              << "n " << n << " m " << m << " neq " << neq;
          nactive_total += nactive - neq;
        }
      }
    }
  }

  // inequalities are active
  EXPECT_GT(nactive_total, 0);
}

// only the lower triangle of H is read
TEST_F(QPTest, LowerTriangle) {
  QP qp = RandomQP(6, 8, 2, 3);
  ASSERT_GE(qp.Solve(), 0);
  vector<mjtNum> expected = qp.res;

  for (int i = 0; i < qp.n; i++) {
    for (int j = i + 1; j < qp.n; j++) {
      qp.H[i * qp.n + j] = std::numeric_limits<mjtNum>::quiet_NaN();
    }
  }
  mju_zeroInt(qp.active.data(), qp.n + qp.m);
  ASSERT_GE(qp.Solve(), 0);
  EXPECT_EQ(qp.res, expected);
}

// a user-allocated workspace gives the same solution
TEST_F(QPTest, Workspace) {
  QP qp = RandomQP(6, 8, 2, 4);
  ASSERT_GE(qp.Solve(), 0);
  vector<mjtNum> expected = qp.res;

  int n = qp.n, m = qp.m;
  vector<mjtNum> work(n * n + (n + m) * (2 * n + m + 13));
  mju_zeroInt(qp.active.data(), n + m);
  ASSERT_GE(qp.Solve(work.data()), 0);
  EXPECT_EQ(qp.res, expected);
}

// a warmstart from any working set gives the same solution
TEST_F(QPTest, Warmstart) {
  for (int seed = 0; seed < 10; seed++) {
    QP qp = RandomQP(10, 30, 3, seed);
    int nactive = qp.Solve();
    ASSERT_GE(nactive, 0);
    vector<mjtNum> expected = qp.res;

    // from the solution
    EXPECT_EQ(qp.Solve(), nactive);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-12, 1e-3), expected));

    // from all upper bounds
    std::fill(qp.active.begin(), qp.active.end(), 1);
    EXPECT_EQ(qp.Solve(), nactive);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-12, 1e-3), expected));

    // from all lower bounds
    std::fill(qp.active.begin(), qp.active.end(), -1);
    EXPECT_EQ(qp.Solve(), nactive);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-12, 1e-3), expected));
  }
}

// repeated and redundant constraints are handled
TEST_F(QPTest, Degenerate) {
  // min 0.5*|x - (2, 2)|^2, three constraints meet at (1, 1), one is repeated
  mjtNum H[4] = {1, 0, 0, 1};
  mjtNum g[2] = {-2, -2};
  mjtNum A[8] = {1, 0, 0, 1, 1, 1, 1, 1};
  mjtNum aupper[4] = {1, 1, 2, 2};

  mjtNum res[2], dual[6];
  EXPECT_GE(mju_QP(res, dual, nullptr, nullptr, H, g, 2, nullptr, nullptr, A,
                   nullptr, aupper, 4, nullptr, nullptr),
            0);
  EXPECT_THAT(res, Pointwise(MjNear(1e-12, 1e-5), {1.0, 1.0}));
}

// problems with known solutions from the test suites of quadprog and qpsolvers
TEST_F(QPTest, ReferenceProblems) {
  // Goldfarb and Idnani (1983): min 0.5*|x|^2 - 5*x1 with three inequalities
  {
    QP qp(3, 3);
    qp.H = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    qp.g = {0, -5, 0};
    qp.A = {-4, -3, 0, 2, 1, 0, 0, -2, 1};
    qp.alower = {-8, 2, 0};
    EXPECT_EQ(qp.Solve(), 2);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-14, 1e-5),
                                  {10.0 / 21, 22.0 / 21, 44.0 / 21}));
    EXPECT_THAT(qp.dual,
                Pointwise(MjNear(1e-14, 1e-5),
                          {0.0, 0.0, 0.0, 0.0, -5.0 / 21, -44.0 / 21}));
  }

  // three equalities, two variables have identical columns
  {
    QP qp(5, 3);
    for (int i = 0; i < 5; i++) qp.H[6 * i] = 1;
    qp.g = {-0.73727161, -0.75526241, -0.04741426, 0.11260887, 0.11260887};
    qp.A = {3.6, -3.4, -3.8,  1.6,   1.6, 0, -1.9, -1.7,
            -4,  -4,   -9.72, -8.67, 0,   0, 0};
    qp.alower = qp.aupper = {1.02, 0.03, 0.081};
    EXPECT_EQ(qp.Solve(), 3);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-8, 1e-5),
                                  {0.07313507, -0.09133482, -0.08677699,
                                   0.03638213, 0.03638213}));
    EXPECT_LT(qp.Residual(), MjTol(1e-14, 1e-5));
  }

  // ill-conditioned H, bounds and two rows
  {
    QP qp(3, 2);
    qp.H = {224.60560028, 181.38561347, 299.23703769,
            181.38561347, 148.05984179, 238.58321617,
            299.23703769, 238.58321617, 406.34542188};
    qp.g = {-239.91135277, -196.08680183, -313.40206452};
    qp.lower = {0, 0, 0};
    qp.A = {-1, -1, -1, 1, 1, 1};
    qp.alower = {-1.19, 0.7};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_LT(qp.Residual(), MjTol(1e-13, 1e-5));
  }

  // bounds, an equality, an inequality and an infinite bound
  {
    QP qp(3, 3);
    mjtNum M[9] = {1, 2, 0, -8, 3, 2, 0, 1, 1};
    mju_mulMatTMat(qp.H.data(), M, M, 3, 3, 3);
    qp.g = {-13, 15, 7};
    qp.lower = {-0.5, -0.4, -0.5};
    qp.upper = {1, 1, 1};
    qp.A = {4, 2, 0, -1, 2, -1, 1, 1, 1};
    qp.alower = {-mjMAXVAL, -mjMAXVAL, 1};
    qp.aupper = {mjMAXVAL, -2, 1};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-13, 1e-4), {0.4, -0.4, 1.0}));
    EXPECT_LT(qp.Residual(), MjTol(1e-13, 1e-4));
  }

  // an equality that holds at the unconstrained minimum
  {
    QP qp(2, 1);
    qp.H = {1, 0, 0, 1};
    qp.g = {-1, 1};
    qp.A = {1, -1};
    qp.alower = qp.aupper = {2};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-15, 1e-6), {1.0, -1.0}));
    EXPECT_THAT(qp.dual, Pointwise(MjNear(1e-15, 1e-6), {0.0, 0.0, 0.0}));
  }
}

// the working set fills up before the solution is found, then mostly empties
TEST_F(QPTest, FullWorkingSet) {
  // min 0.5*|x|^2  s.t.  x >= 1,  z'*x >= 1.01, where z alternates between 1
  // and -1
  int n = 20;
  QP qp(n, 1);
  for (int i = 0; i < n; i++) {
    qp.H[i * (n + 1)] = 1;
    qp.lower[i] = 1;
    qp.A[i] = (i % 2 ? -1 : 1) + 1e-6 * mju_sin(i);
  }
  qp.alower[0] = 1.01;
  EXPECT_GE(qp.Solve(), 0);
  EXPECT_LT(qp.Residual(), MjTol(1e-14, 1e-5));
  EXPECT_EQ(qp.active[n], -1);
}

// more constraints than variables are active at the solution
TEST_F(QPTest, DegenerateVertex) {
  // each triplet of variables is in a pyramid |x0| <= x2, |x1| <= x2
  // the bias pushes all triplets to the apex, where four constraints are active
  int npyramid = 4;
  int n = 3 * npyramid, m = 4 * npyramid;
  std::mt19937_64 rng(11);
  std::uniform_real_distribution<double> uniform(0.5, 1.5);
  for (int trial = 0; trial < 50; trial++) {
    QP qp = RandomQP(n, m, 0, trial);
    std::fill(qp.lower.begin(), qp.lower.end(), -mjMAXVAL);
    std::fill(qp.upper.begin(), qp.upper.end(), mjMAXVAL);
    std::fill(qp.A.begin(), qp.A.end(), 0);
    std::fill(qp.aupper.begin(), qp.aupper.end(), mjMAXVAL);
    std::fill(qp.alower.begin(), qp.alower.end(), 0);
    for (int p = 0; p < npyramid; p++) {
      for (int k = 0; k < 4; k++) {
        mjtNum* row = &qp.A[(4 * p + k) * n + 3 * p];
        row[k / 2] = k % 2 ? -1 : 1;
        row[2] = 1;
      }
    }

    // g = A'*lam with positive lam: the origin is optimal
    vector<mjtNum> lam(m);
    for (mjtNum& v : lam) v = uniform(rng);
    mju_mulMatTVec(qp.g.data(), qp.A.data(), lam.data(), m, n);

    EXPECT_GE(qp.Solve(), 0);
    EXPECT_LT(mju_norm(qp.res.data(), n), MjTol(1e-12, 1e-3));
  }
}

// equalities that determine all variables
TEST_F(QPTest, DeterminedByEqualities) {
  int n = 4;
  QP qp = RandomQP(n, n + 1, n, 5);
  std::fill(qp.lower.begin(), qp.lower.end(), -mjMAXVAL);
  std::fill(qp.upper.begin(), qp.upper.end(), mjMAXVAL);
  qp.alower[n] = -mjMAXVAL;
  qp.aupper[n] = mjMAXVAL;
  ASSERT_EQ(qp.Solve(), n);
  vector<mjtNum> x = qp.res;
  mjtNum val = mju_dot(&qp.A[n * n], x.data(), n);

  // an inequality that holds at the determined point
  qp.aupper[n] = val + 1;
  EXPECT_EQ(qp.Solve(), n);
  EXPECT_THAT(qp.res, Pointwise(MjNear(1e-12, 1e-3), x));

  // an inequality that is violated at the determined point
  qp.aupper[n] = val - 1;
  EXPECT_EQ(qp.Solve(), -2);

  // a consistent additional equality
  qp.alower[n] = qp.aupper[n] = val;
  EXPECT_GE(qp.Solve(), n);
  EXPECT_THAT(qp.res, Pointwise(MjNear(1e-12, 1e-3), x));

  // an inconsistent additional equality
  qp.alower[n] = qp.aupper[n] = val + 1;
  EXPECT_EQ(qp.Solve(), -2);

  // a bound that is violated at the determined point
  qp.alower[n] = -mjMAXVAL;
  qp.aupper[n] = mjMAXVAL;
  qp.upper[0] = x[0] - 1;
  EXPECT_EQ(qp.Solve(), -2);
}

// a warmstart with dependent or too many constraints gives the cold solution
TEST_F(QPTest, WarmstartDependent) {
  // an equality that depends on the bounds of the warmstart
  {
    QP qp(2, 1);
    qp.H = {1, 0, 0, 1};
    qp.g = {-3, -1};
    qp.upper = {0.5, 0.5};
    qp.A = {1, 1};
    qp.alower = qp.aupper = {1};
    qp.active = {1, 1, 0};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-15, 1e-6), {0.5, 0.5}));

    // the equality still holds when a bound is relaxed
    qp.upper = {2, 0.5};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-15, 1e-6), {1.5, -0.5}));
  }

  // more constraints in the warmstart than the working set can hold
  {
    QP qp(2, 5);
    qp.H = {1, 0, 0, 1};
    qp.g = {-3, -1};
    qp.A = {1, 0, 0, 1, 1, 1, 1, 0, 1, -1};
    qp.aupper = {1, 1, 1.5, 5, 0};
    qp.alower[4] = 0;
    ASSERT_GE(qp.Solve(), 0);
    vector<mjtNum> expected = qp.res;
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-15, 1e-6), {0.75, 0.75}));

    std::fill(qp.active.begin(), qp.active.end(), 1);
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-14, 1e-5), expected));
  }
}

// nearly parallel constraints are not reported as infeasible
TEST_F(QPTest, NearlyParallel) {
  std::mt19937_64 rng(3);
  std::normal_distribution<double> normal(0, 1);
  for (mjtNum eps : {1e-3, 1e-6, 1e-9, 1e-12}) {
    for (int trial = 0; trial < 10; trial++) {
      // rows 0 and 1 differ by eps, the bias pushes against both
      int n = 4;
      QP qp(n, 3);
      for (int i = 0; i < n; i++) {
        qp.H[i * (n + 1)] = 1;
        qp.A[i] = normal(rng);
        qp.A[n + i] = qp.A[i] + eps * normal(rng);
        qp.A[2 * n + i] = normal(rng);
        qp.g[i] = -10 * qp.A[i];
      }
      qp.aupper = {1, 1, 1};
      EXPECT_GE(qp.Solve(), 0) << "eps " << eps;
      EXPECT_LT(qp.Residual(), MjTol(1e-12, 1e-4)) << "eps " << eps;
    }
  }
}

// failure is reported for infeasible constraints and indefinite H
TEST_F(QPTest, Failure) {
  mjtNum H[4] = {1, 0, 0, 1};
  mjtNum g[2] = {0, 0};
  mjtNum res[2];

  // inconsistent inequalities: x0 + x1 <= 0, x0 + x1 >= 1
  mjtNum A[4] = {1, 1, 2, 2};
  mjtNum alower[2] = {-mjMAXVAL, 2};
  mjtNum aupper[2] = {0, mjMAXVAL};
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, H, g, 2, nullptr, nullptr, A,
                   alower, aupper, 2, nullptr, nullptr),
            -2);

  // inconsistent equalities
  mjtNum equal[2] = {0, 2};
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, H, g, 2, nullptr, nullptr, A,
                   equal, equal, 2, nullptr, nullptr),
            -2);

  // crossed bounds
  mjtNum lower[2] = {1, 0};
  mjtNum upper[2] = {0, 1};
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, H, g, 2, lower, upper,
                   nullptr, nullptr, nullptr, 0, nullptr, nullptr),
            -2);

  // indefinite H
  mjtNum Hbad[4] = {1, 0, 0, -1};
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, Hbad, g, 2, nullptr, nullptr,
                   nullptr, nullptr, nullptr, 0, nullptr, nullptr),
            -1);

  // required arguments are missing
  auto error = MjuErrorMessageFrom(mju_QP);
  EXPECT_THAT(error(nullptr, nullptr, nullptr, nullptr, H, g, 2, nullptr,
                    nullptr, nullptr, nullptr, nullptr, 0, nullptr, nullptr),
              ::testing::HasSubstr("must be given"));
  EXPECT_THAT(error(res, nullptr, nullptr, nullptr, nullptr, g, 2, nullptr,
                    nullptr, nullptr, nullptr, nullptr, 0, nullptr, nullptr),
              ::testing::HasSubstr("must be given"));
  EXPECT_THAT(error(res, nullptr, nullptr, nullptr, H, nullptr, 2, nullptr,
                    nullptr, nullptr, nullptr, nullptr, 0, nullptr, nullptr),
              ::testing::HasSubstr("must be given"));
}

// a singular H is solved if the constraints bound the problem
TEST_F(QPTest, SingularHessian) {
  // min 0.5*(x0 + x1 - 1)^2 + x2  s.t.  -1 <= x <= 1,  x0 - x1 = 0.5
  mjtNum H[9] = {1, 1, 0, 1, 1, 0, 0, 0, 0};
  mjtNum g[3] = {-1, -1, 1};
  mjtNum lower[3] = {-1, -1, -1};
  mjtNum upper[3] = {1, 1, 1};
  mjtNum A[3] = {1, -1, 0};
  mjtNum b[1] = {0.5};
  mjtNum res[3], dual[4];
  EXPECT_GE(mju_QP(res, dual, nullptr, nullptr, H, g, 3, lower, upper, A, b, b,
                   1, nullptr, nullptr),
            0);
  mjtNum expected_res[3] = {0.75, 0.25, -1};
  mjtNum expected_dual[4] = {0, 0, -1, 0};
  EXPECT_THAT(res, Pointwise(MjNear(1e-11, 1e-3), expected_res));
  EXPECT_THAT(dual, Pointwise(MjNear(1e-13, 1e-5), expected_dual));

  // random low-rank H with bounds: optimality conditions hold
  std::mt19937_64 rng(7);
  std::normal_distribution<double> normal(0, 1);
  for (int trial = 0; trial < 20; trial++) {
    int n = 12, rank = 4, m = 6;
    QP qp(n, m);
    vector<mjtNum> J(rank * n);
    for (mjtNum& v : J) v = normal(rng);
    mju_mulMatTMat(qp.H.data(), J.data(), J.data(), rank, n, n);
    for (int i = 0; i < n; i++) {
      qp.g[i] = normal(rng);
      qp.lower[i] = -1;
      qp.upper[i] = 1;
    }
    for (mjtNum& v : qp.A) v = normal(rng);
    for (int i = 0; i < m; i++) {
      qp.alower[i] = -1;
      qp.aupper[i] = 1;
    }
    ASSERT_GE(qp.Solve(), 0);
    EXPECT_LT(qp.Residual(), MjTol(1e-8, 1e-3));
  }

  // unbounded along the null space of H
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, H, g, 3, nullptr, nullptr,
                   nullptr, nullptr, nullptr, 0, nullptr, nullptr),
            -3);

  // bounded by the curvature of H: g is in its range
  mjtNum grange[3] = {-1, -1, 0};
  EXPECT_EQ(mju_QP(res, nullptr, nullptr, nullptr, H, grange, 3, nullptr,
                   nullptr, nullptr, nullptr, nullptr, 0, nullptr, nullptr),
            0);
  EXPECT_NEAR(res[0] + res[1], 1, MjTol(3e-12, 3e-6));

  // curvature that is negative within rounding error is not indefinite
  mjtNum eps = std::numeric_limits<mjtNum>::epsilon();
  mjtNum Hround[4] = {1, 0, 0, -eps / 10};
  mjtNum ground[2] = {-0.5, 0};
  mjtNum lower2[2] = {-1, -1};
  mjtNum upper2[2] = {1, 1};
  EXPECT_GE(mju_QP(res, nullptr, nullptr, nullptr, Hround, ground, 2, lower2,
                   upper2, nullptr, nullptr, nullptr, 0, nullptr, nullptr),
            0);
  EXPECT_NEAR(res[0], 0.5, MjTol(1e-9, 1e-3));
}

// a linear program, H = 0, is solved if the constraints bound the problem
TEST_F(QPTest, LinearProgram) {
  // min x0 - 2*x1 + 0.5*x2  s.t.  -1 <= x <= 1
  {
    QP qp(3, 0);
    qp.g = {1, -2, 0.5};
    qp.lower = {-1, -1, -1};
    qp.upper = {1, 1, 1};
    EXPECT_EQ(qp.Solve(), 3);
    mjtNum expected_res[3] = {-1, 1, -1};
    mjtNum expected_dual[3] = {-1, 2, -0.5};
    EXPECT_THAT(qp.res, Pointwise(MjNear(5e-15, 3e-6), expected_res));
    EXPECT_THAT(qp.dual, Pointwise(MjNear(5e-15, 5e-6), expected_dual));
    EXPECT_THAT(qp.active, ElementsAre(-1, 1, -1));
  }

  // min -x0 - x1  s.t.  x >= 0,  x0 + 2*x1 <= 4,  3*x0 + x1 <= 6
  {
    QP qp(2, 2);
    qp.g = {-1, -1};
    qp.lower = {0, 0};
    qp.A = {1, 2, 3, 1};
    qp.aupper = {4, 6};
    EXPECT_EQ(qp.Solve(), 2);
    mjtNum expected_res[2] = {1.6, 1.2};
    mjtNum expected_dual[4] = {0, 0, 0.4, 0.2};
    EXPECT_THAT(qp.res, Pointwise(MjNear(5e-15, 3e-6), expected_res));
    EXPECT_THAT(qp.dual, Pointwise(MjNear(5e-15, 5e-6), expected_dual));
  }

  // min x0 + 2*x1  s.t.  0 <= x <= 2,  x0 + x1 = 1
  {
    QP qp(2, 1);
    qp.g = {1, 2};
    qp.lower = {0, 0};
    qp.upper = {2, 2};
    qp.A = {1, 1};
    qp.alower = qp.aupper = {1};
    EXPECT_EQ(qp.Solve(), 2);
    mjtNum expected_res[2] = {1, 0};
    mjtNum expected_dual[3] = {0, -1, -1};
    EXPECT_THAT(qp.res, Pointwise(MjNear(5e-15, 3e-6), expected_res));
    EXPECT_THAT(qp.dual, Pointwise(MjNear(5e-15, 5e-6), expected_dual));

    // degenerate vertex: the upper bound of x0 also holds at the solution
    qp.upper = {1, 1};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_THAT(qp.res, Pointwise(MjNear(5e-15, 3e-6), expected_res));
    EXPECT_LT(qp.Residual(), MjTol(1e-12, 2e-4));
  }

  // no objective: any feasible point is a solution
  {
    QP qp(2, 1);
    qp.lower = {1, -2};
    qp.upper = {3, -1};
    qp.A = {1, 1};
    qp.aupper = {1};
    EXPECT_GE(qp.Solve(), 0);
    EXPECT_LT(qp.Residual(), MjTol(1e-12, 2e-4));
  }

  // random objectives over a bounded polytope: optimality conditions hold
  std::mt19937_64 rng(11);
  std::normal_distribution<double> normal(0, 1);
  for (int trial = 0; trial < 20; trial++) {
    int n = 8, m = 6;
    QP qp(n, m);
    for (int i = 0; i < n; i++) {
      qp.g[i] = normal(rng);
      qp.lower[i] = -1;
      qp.upper[i] = 1;
    }
    for (mjtNum& v : qp.A) v = normal(rng);
    for (int i = 0; i < m; i++) {
      qp.alower[i] = -1;
      qp.aupper[i] = 1;
    }
    ASSERT_GE(qp.Solve(), 0);
    EXPECT_LT(qp.Residual(), MjTol(1e-12, 2e-4));
  }

  // unbounded: x0 decreases freely
  {
    QP qp(2, 0);
    qp.g = {1, 0};
    qp.upper = {1, 1};
    EXPECT_EQ(qp.Solve(), -3);
  }

  // unbounded along a ray of the feasible cone  x0 >= x1 >= 0
  {
    QP qp(2, 1);
    qp.g = {-1, 0.5};
    qp.lower = {0, 0};
    qp.A = {1, -1};
    qp.alower = {0};
    EXPECT_EQ(qp.Solve(), -3);

    // a bound on x0 makes it bounded
    qp.upper = {3, mjMAXVAL};
    EXPECT_EQ(qp.Solve(), 2);
    mjtNum expected_res[2] = {3, 0};
    EXPECT_THAT(qp.res, Pointwise(MjNear(5e-15, 3e-6), expected_res));

    // so does a soft bound: min -x0 + (x0 - 3)^2 / 2 for x0 > 3
    qp.compliance[0] = 1;
    EXPECT_GE(qp.Solve(), 0);
    mjtNum expected_soft[2] = {4, 0};
    EXPECT_THAT(qp.res, Pointwise(MjNear(2e-10, 2e-4), expected_soft));
  }

  // indefinite H with a zero diagonal
  {
    QP qp(2, 0);
    qp.H = {0, 1, 1, 0};
    EXPECT_EQ(qp.Solve(), -1);
  }

  // a long face along which the objective decreases slowly
  //  min -x1  s.t.  0 <= x0 <= 1000,  x1 <= 0.001*x0
  {
    QP qp(2, 1);
    qp.g = {0, -1};
    qp.lower = {0, -mjMAXVAL};
    qp.upper = {1000, mjMAXVAL};
    qp.A = {-0.001, 1};
    qp.aupper = {0};
    EXPECT_EQ(qp.Solve(), 2);
    mjtNum expected_res[2] = {1000, 1};
    EXPECT_THAT(qp.res, Pointwise(MjNear(1e-11, 2e-3), expected_res));
  }
}

// a Hessian with weak curvature along some directions, as in damped least
// squares
//  min 0.5*|J*x - e|^2 + 0.5*damping*|x|^2, with fewer rows in J than variables
TEST_F(QPTest, WeakCurvature) {
  int n = 8, rank = 4, m = 6;
  for (double damping : {1e-4, 1e-5, 1e-7, 1e-10, 0.0}) {
    for (int trial = 0; trial < 20; trial++) {
      std::mt19937_64 rng(trial);
      std::normal_distribution<double> normal(0, 1);
      std::uniform_real_distribution<double> uniform(0, 1);
      QP qp(n, m);

      // H = J'*J + damping*I, relative to the largest diagonal, g = -J'*e
      vector<mjtNum> J(rank * n), e(rank);
      for (mjtNum& v : J) v = normal(rng);
      for (mjtNum& v : e) v = normal(rng);
      mju_mulMatTMat(qp.H.data(), J.data(), J.data(), rank, n, n);
      mjtNum hmax = 0;
      for (int i = 0; i < n; i++) hmax = mju_max(hmax, qp.H[i * (n + 1)]);
      for (int i = 0; i < n; i++) qp.H[i * (n + 1)] += damping * hmax;
      mju_mulMatTVec(qp.g.data(), J.data(), e.data(), rank, n);
      for (mjtNum& v : qp.g) v = -v;

      // the origin is feasible, some constraints are tight there
      for (mjtNum& v : qp.A) v = normal(rng);
      for (int c = 0; c < n + m; c++) {
        mjtNum lo = -0.1 - uniform(rng), hi = 0.1 + uniform(rng);
        double u = uniform(rng);
        if (u < 0.15) {
          hi = 0;
        } else if (u < 0.3) {
          lo = 0;
        }
        (c < n ? qp.lower[c] : qp.alower[c - n]) = lo;
        (c < n ? qp.upper[c] : qp.aupper[c - n]) = hi;
      }

      ASSERT_GE(qp.Solve(), 0) << "damping " << damping << " trial " << trial;

      // constraints hold relative to the norms of their rows
      for (int c = 0; c < n + m; c++) {
        mjtNum val =
            c < n ? qp.res[c] : mju_dot(&qp.A[(c - n) * n], qp.res.data(), n);
        mjtNum norm = c < n ? 1 : mju_norm(&qp.A[(c - n) * n], n);
        mjtNum lo = c < n ? qp.lower[c] : qp.alower[c - n];
        mjtNum hi = c < n ? qp.upper[c] : qp.aupper[c - n];
        EXPECT_LT(mju_max(val - hi, lo - val), norm * MjTol(3e-7, 2e-3))
            << "damping " << damping << " trial " << trial;
      }
      EXPECT_LT(qp.Residual(), MjTol(8e-8, 7e-4))
          << "damping " << damping << " trial " << trial;
    }
  }
}

// the solution scales with the variables and is independent of the scale of the
// objective, for a strictly convex problem and for a linear program
TEST_F(QPTest, ScaleCovariance) {
  int n = 8, m = 6;
  std::mt19937_64 rng(13);
  std::normal_distribution<double> normal(0, 1);
  for (int trial = 0; trial < 10; trial++) {
    // variables and rows are within [-1, 1]
    QP base(n, m);
    for (int i = 0; i < n; i++) {
      base.g[i] = normal(rng);
      base.lower[i] = -1;
      base.upper[i] = 1;
    }
    for (mjtNum& v : base.A) v = normal(rng);
    for (int i = 0; i < m; i++) {
      base.alower[i] = -1;
      base.aupper[i] = 1;
    }

    for (bool linear : {false, true}) {
      for (int i = 0; i < n; i++) base.H[i * (n + 1)] = linear ? 0 : 1;
      int nactive = base.Solve();
      ASSERT_GE(nactive, 0);

      for (double sx : {1e-6, 1e-3, 1e3, 1e6}) {
        for (double sg : {1e-6, 1.0, 1e6}) {
          // x scales by sx, the objective by sg*sx
          QP qp = base;
          std::fill(qp.active.begin(), qp.active.end(), 0);
          for (int i = 0; i < n; i++) {
            qp.H[i * (n + 1)] *= sg / sx;
            qp.g[i] *= sg;
            qp.lower[i] *= sx;
            qp.upper[i] *= sx;
          }
          for (int i = 0; i < m; i++) {
            qp.alower[i] *= sx;
            qp.aupper[i] *= sx;
          }
          ASSERT_EQ(qp.Solve(), nactive) << "sx " << sx << " sg " << sg;

          // same solution, and constraints hold relative to the scale of x
          for (int i = 0; i < n; i++) {
            EXPECT_NEAR(qp.res[i] / sx, base.res[i], MjTol(6e-14, 3e-5))
                << "sx " << sx << " sg " << sg;
          }
          for (int j = 0; j < m; j++) {
            mjtNum val = mju_dot(&qp.A[j * n], qp.res.data(), n);
            mjtNum norm = mju_norm(&qp.A[j * n], n);
            EXPECT_LT(mju_abs(val) / sx - 1, norm * MjTol(3e-14, 2e-5))
                << "sx " << sx << " sg " << sg;
          }
        }
      }
    }
  }
}

// a soft constraint holds up to its threshold, then yields with its compliance
TEST_F(QPTest, SoftKnownSolution) {
  // min 0.5*x^2 - x  s.t.  x <= 0: holding the constraint takes a multiplier 1
  mjtNum H[1] = {1};
  mjtNum g[1] = {-1};
  mjtNum upper[1] = {0};
  mjtNum res[1], dual[1];
  int active[1] = {0};

  // threshold above 1: holds
  mjtNum threshold[1] = {2};
  mjtNum compliance[1] = {1};
  EXPECT_EQ(mju_QP(res, dual, active, nullptr, H, g, 1, nullptr, upper, nullptr,
                   nullptr, nullptr, 0, threshold, compliance),
            1);
  EXPECT_NEAR(res[0], 0, MjTol(1e-14, 1e-6));
  EXPECT_NEAR(dual[0], 1, MjTol(1e-14, 1e-6));
  EXPECT_EQ(active[0], 1);

  // threshold below 1: yields, dual = (1 + rho*w) / (1 + rho)
  threshold[0] = 0.5;
  compliance[0] = 3;
  active[0] = 0;
  EXPECT_EQ(mju_QP(res, dual, active, nullptr, H, g, 1, nullptr, upper, nullptr,
                   nullptr, nullptr, 0, threshold, compliance),
            1);
  EXPECT_NEAR(dual[0], 2.5 / 4, MjTol(1e-14, 1e-6));
  EXPECT_NEAR(res[0], 3 * (2.5 / 4 - 0.5), MjTol(1e-14, 1e-6));
  EXPECT_EQ(active[0], 2);

  // no threshold: quadratic penalty, dual = 1 / (1 + rho)
  active[0] = 0;
  EXPECT_EQ(mju_QP(res, dual, active, nullptr, H, g, 1, nullptr, upper, nullptr,
                   nullptr, nullptr, 0, nullptr, compliance),
            1);
  EXPECT_NEAR(dual[0], 0.25, MjTol(1e-14, 1e-6));
  EXPECT_NEAR(res[0], 0.75, MjTol(1e-14, 1e-6));
  EXPECT_EQ(active[0], 2);
}

// repeated soft constraints share the multiplier that holds them
TEST_F(QPTest, SoftDependent) {
  // min 0.5*x^2 - x  s.t.  x = 0 (three times): holding takes a multiplier 1
  int n = 1, m = 3;
  for (mjtNum threshold : {0.2, 0.4, 2.0}) {
    QP qp(n, m);
    qp.H[0] = 1;
    qp.g[0] = -1;
    for (int j = 0; j < m; j++) {
      qp.A[j] = 1;
      qp.alower[j] = qp.aupper[j] = 0;
      qp.threshold[n + j] = threshold;
      qp.compliance[n + j] = 0.5;
    }
    EXPECT_EQ(qp.Solve(), threshold < 2 ? 3 : 1);
    EXPECT_LT(qp.Residual(), MjTol(1e-14, 1e-6));

    // thresholds add up to more than 1: constraints hold, otherwise all yield
    if (3 * threshold > 1) {
      EXPECT_NEAR(qp.res[0], 0, MjTol(1e-14, 1e-6));
    } else {
      EXPECT_NEAR(qp.res[0], 0.4 / 7, MjTol(1e-14, 1e-6));
      EXPECT_EQ(qp.Count(2), 3);
    }
  }
}

// repeated soft constraints with thresholds that add up to more than the
// multiplier of the hard constraint are an exact penalty
TEST_F(QPTest, SoftDuplicates) {
  int ntested = 0;
  for (mjtNum scale : {1.0, 1e3}) {
    for (int seed = 0; seed < 20; seed++) {
      // rows 0 and 1 are the same upper bound, violated at the unconstrained
      // minimum
      int n = 8, m = 10;
      QP qp = RandomQP(n, m, 0, seed);
      mju_scl(qp.H.data(), qp.H.data(), scale, n * n);
      mju_scl(qp.g.data(), qp.g.data(), scale, n);
      mju_copy(&qp.A[n], &qp.A[0], n);
      qp.alower[0] = qp.alower[1] = -mjMAXVAL;
      qp.aupper[0] = qp.aupper[1] = qp.aupper[0] < mjMAXVAL ? qp.aupper[0] : 0;

      // hard solution: the multiplier is carried by one of the two rows
      if (qp.Solve() < 0) continue;
      mjtNum lam = qp.dual[n] + qp.dual[n + 1];
      if (lam < 1e-3 * scale) continue;
      vector<mjtNum> expected = qp.res;

      // soft, each threshold holds 3/4 of the multiplier
      qp.threshold[n] = qp.threshold[n + 1] = 0.75 * lam;
      qp.compliance[n] = qp.compliance[n + 1] = 1 / scale;
      for (int warm = 0; warm < 2; warm++) {
        if (!warm) mju_zeroInt(qp.active.data(), n + m);
        ASSERT_GE(qp.Solve(), 0);
        EXPECT_THAT(qp.res, Pointwise(MjNear(1e-11, 1e-3), expected));
        EXPECT_NEAR(qp.dual[n] + qp.dual[n + 1], lam, MjTol(1e-9, 1e-2) * lam);
      }
      ntested++;
    }
  }
  EXPECT_GT(ntested, 10);
}

// soft constraints make an infeasible problem feasible
TEST_F(QPTest, SoftInfeasible) {
  int seed = 0;
  for (int n : {2, 7, 30}) {
    for (int m : {n, 3 * n}) {
      for (mjtNum threshold : {0.0, 0.3, 3.0}) {
        for (mjtNum compliance : {1e-3, 1.0, 1e6}) {
          QP qp = RandomQP(n, m, n / 2, seed++);

          // shift bounds
          std::mt19937_64 rng;
          rng.seed(seed);
          std::normal_distribution<double> dist(0, 1);
          for (int j = 0; j < m; j++) {
            mjtNum shift = dist(rng);
            if (qp.alower[j] > -mjMAXVAL) qp.alower[j] += shift;
            if (qp.aupper[j] < mjMAXVAL) qp.aupper[j] += shift;
          }

          // make the last two rows inconsistent: a*x <= 0, a*x >= 1
          mju_copy(&qp.A[(m - 1) * n], &qp.A[(m - 2) * n], n);
          qp.alower[m - 2] = -mjMAXVAL;
          qp.aupper[m - 2] = 0;
          qp.alower[m - 1] = 1;
          qp.aupper[m - 1] = mjMAXVAL;

          // rows are soft, bounds are hard
          for (int j = 0; j < m; j++) {
            qp.threshold[n + j] = threshold;
            qp.compliance[n + j] = compliance;
          }

          // without soft constraints the problem is infeasible
          QP hard = qp;
          std::fill(hard.compliance.begin(), hard.compliance.end(), 0);
          EXPECT_EQ(hard.Solve(), -2);

          EXPECT_GE(qp.Solve(), 0) << "n " << n << " m " << m << " threshold "
                                   << threshold << " compliance " << compliance;
          EXPECT_LT(qp.Residual(), MjTol(1e-10, 0.1))
              << "n " << n << " m " << m << " threshold " << threshold
              << " compliance " << compliance;
          EXPECT_GT(qp.Count(2), 0);
        }
      }
    }
  }
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
  mjtNum* sqrtH = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal * nTotal);

  // sample
  for (int i = 0; i < nTotal; i++) {
    if (vec) vec[i] = dist(rng);
    for (int j = 0; j < nTotal; j++) {
      sqrtH[nTotal * i + j] = dist(rng);
    }
  }

  // make SPD matrix H
  mju_mulMatTMat(H, sqrtH, sqrtH, nTotal, nTotal, nTotal);

  // set zeros
  int nSparse = nTotal - nDense;
  for (int i = 0; i < nSparse; i++) {
    int nzeros = mjMAX(0, i + 1 - nBand);
    for (int j = 0; j < nzeros; j++) {
      H[nTotal * i + j] = 0;
      H[nTotal * j + i] = 0;
    }
  }

  // add regularizer to diagonal
  for (int i = 0; i < nTotal; i++) {
    H[nTotal * i + i] += reg;
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
      int nB = (nTotal - nDense) * nBand + nDense * nTotal;
      mjtNum* B = (mjtNum*)mju_malloc(sizeof(mjtNum) * nB);
      int nH = nTotal * nTotal;
      mjtNum* H = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);

      // make random banded SPD matrix, dense representation
      randomBanded(H, nTotal, nBand, nDense, seed++);

      // convert to banded representation
      mju_dense2Band(B, H, nTotal, nBand, nDense);

      // expect diagonals to be equal
      for (int i = 0; i < nTotal; i++) {
        EXPECT_EQ(H[i * nTotal + i], B[mju_bandDiag(i, nTotal, nBand, nDense)]);
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
      int nB = (nTotal - nDense) * nBand + nDense * nTotal;
      mjtNum* B = (mjtNum*)mju_malloc(sizeof(mjtNum) * nB);
      int nH = nTotal * nTotal;
      mjtNum* H = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
      mjtNum* H1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);

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
      int nB = (nTotal - nDense) * nBand + nDense * nTotal;
      mjtNum* B = (mjtNum*)mju_malloc(sizeof(mjtNum) * nB);
      int nH = nTotal * nTotal;
      mjtNum* H = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
      mjtNum* vec = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
      mjtNum* res = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
      mjtNum* res1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);

      // make random banded SPD matrix, dense
      randomBanded(H, nTotal, nBand, nDense, seed++, vec);

      // multiply dense
      mju_mulMatVec(res, H, vec, nTotal, nTotal);

      // convert to banded, multiply
      mju_dense2Band(B, H, nTotal, nBand, nDense);
      mju_bandMulMatVec(res1, B, vec, nTotal, nBand, nDense,
                        /*nVec=*/1, /*flg_sym=*/1);

      // expect numerical equality
      mjtNum eps = MjTol(1e-12, 5e-5);
      EXPECT_THAT(AsVector(res, nTotal),
                  Pointwise(MjNear(eps, eps), AsVector(res1, nTotal)));

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
          int nB = (nTotal - nDense) * nBand + nDense * nTotal;
          mjtNum* B = (mjtNum*)mju_malloc(sizeof(mjtNum) * nB);
          int nH = nTotal * nTotal;
          mjtNum* H = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
          mjtNum* H1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
          mjtNum* vec = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
          mjtNum* res = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
          mjtNum* res1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);

          // make random banded matrix, dense representation
          //  add regularizer to ensure PD
          randomBanded(H, nTotal, nBand, nDense, seed++, vec, /*reg=*/nTotal);

          // convert to banded
          mju_dense2Band(B, H, nTotal, nBand, nDense);

          // apply diagadd and diagmul
          for (int i = 0; i < nTotal; i++) {
            H[nTotal * i + i] += diagadd + diagmul * H[nTotal * i + i];
          }

          // in-place dense factorization
          int rank = mju_cholFactor(H, nTotal, /*mindiag=*/0);

          // expect factorization to have succeeded
          EXPECT_EQ(rank, nTotal);

          // banded factorization
          mjtNum minDiag =
              mju_cholFactorBand(B, nTotal, nBand, nDense, diagadd, diagmul);

          // expect factorization to have succeeded
          EXPECT_GT(minDiag, 0);

          // convert back to dense, lower triangle only
          mju_band2Dense(H1, B, nTotal, nBand, nDense, /*flg_sym=*/0);

          // zero upper triangle of H (unused)
          for (int i = 0; i < nTotal - 1; i++) {
            mju_zero(H + nTotal * i + i + 1, nTotal - i - 1);
          }

          // expect numerical equality
          mjtNum eps = MjTol(1e-12, 5e-5);
          EXPECT_THAT(AsVector(H, nH),
                      Pointwise(MjNear(eps, eps), AsVector(H1, nH)));

          // multiply dense
          mju_mulMatVec(res, H, vec, nTotal, nTotal);

          // multiply sparse, only lower triangle
          mju_bandMulMatVec(res1, B, vec, nTotal, nBand, nDense,
                            /*nVec=*/1, /*flg_sym=*/0);

          // expect numerical equality
          EXPECT_THAT(AsVector(res, nTotal),
                      Pointwise(MjNear(eps, eps), AsVector(res1, nTotal)));

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
      int nB = (nTotal - nDense) * nBand + nDense * nTotal;
      mjtNum* B = (mjtNum*)mju_malloc(sizeof(mjtNum) * nB);
      int nH = nTotal * nTotal;
      mjtNum* H = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
      mjtNum* H1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nH);
      mjtNum* vec = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
      mjtNum* res = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);
      mjtNum* res1 = (mjtNum*)mju_malloc(sizeof(mjtNum) * nTotal);

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
      mjtNum eps = MjTol(1e-12, 5e-5);
      EXPECT_THAT(AsVector(res, nTotal),
                  Pointwise(MjNear(eps, eps), AsVector(res1, nTotal)));

      mju_free(res1);
      mju_free(res);
      mju_free(vec);
      mju_free(H1);
      mju_free(H);
      mju_free(B);
    }
  }
}

TEST_F(BandMatrixTest, CholFactorBandReturnsMinFactorizedDiag) {
  mjtNum B[3] = {9.0, 0.0, 4.0};
  mjtNum minDiag =
      mju_cholFactorBand(B, /*ntotal=*/2, /*nband=*/1, /*ndense=*/1,
                         /*diagadd=*/0, /*diagmul=*/0);
  EXPECT_MJTNUM_EQ(B[mju_bandDiag(0, 2, 1, 1)], 3.0);
  EXPECT_MJTNUM_EQ(B[mju_bandDiag(1, 2, 1, 1)], 2.0);
  EXPECT_MJTNUM_EQ(minDiag, 2.0);
}

using EngineUtilSolveTest = MujocoTest;

TEST_F(EngineUtilSolveTest, MjuCholFactorSymbolic) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

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
                                   LT_rownnz, LT_rowadr, nullptr, nullptr,
                                   H_rownnz, H_rowadr, H_colind, n, d.get());

  // verify counting phase outputs
  EXPECT_EQ(nnz, 8);
  // L (CSR) structure for reverse Cholesky (rows filled in reverse order)
  EXPECT_THAT(AsVector(L_rownnz, 4), ElementsAre(1, 2, 2, 3));
  EXPECT_THAT(AsVector(L_rowadr, 4), ElementsAre(0, 1, 3, 5));
  // LT (CSC) structure: transpose of L
  EXPECT_THAT(AsVector(LT_rownnz, 4), ElementsAre(4, 1, 2, 1));
  EXPECT_THAT(AsVector(LT_rowadr, 4), ElementsAre(0, 4, 5, 7));

  // phase 2: filling
  int L_colind[8], LT_colind[8], LT_pos[8], L_super[4];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_pos, L_super, H_rownnz, H_rowadr,
                         H_colind, n, d.get());

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
      mjtNum eps = MjTol(1e-8, 5e-4);
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

TEST_F(EngineUtilSolveTest, MjuCholUpdateParity) {
  // Frozen verbatim copy of HEAD mju_cholUpdate
  auto ref_chol_update = [](mjtNum* mat, mjtNum* x, int n, int flg_plus) {
    int rank = n;
    mjtNum r, c, cinv, s, Lkk, tmp;

    for (int k = 0; k < n; k++) {
      if (x[k]) {
        Lkk = mat[k * (n + 1)];
        tmp = Lkk * Lkk + (flg_plus ? x[k] * x[k] : -x[k] * x[k]);
        if (!(tmp >= mjMINVAL)) {
          tmp = mjMINVAL;
          rank--;
        }
        r = mju_sqrt(tmp);
        c = r / Lkk;
        cinv = 1 / c;
        s = x[k] / Lkk;

        mat[k * (n + 1)] = r;

        mjtNum s_signed = flg_plus ? s : -s;
        for (int i = k + 1; i < n; i++) {
          mjtNum m = (mat[i * n + k] + s_signed * x[i]) * cinv;
          mat[i * n + k] = m;
          x[i] = c * x[i] - s * m;
        }
      }
    }
    return rank;
  };

  for (int n = 1; n <= 35; n++) {
    for (int flg_plus : {0, 1}) {
      for (int pattern = 0; pattern < 6; pattern++) {
        vector<mjtNum> mat(n * n);
        for (int i = 0; i < n; i++) {
          for (int j = 0; j < n; j++) {
            if (j < i) {
              bool zero_row =
                  (pattern == 4 && i % 2 == 1) || (pattern == 5 && i == 1);
              mat[i * n + j] =
                  zero_row ? 0.0 : 0.2 * (mju_Halton(i * n + j + 1, 2) - 0.5);
            } else if (j == i) {
              mat[i * n + j] = 1.5 + mju_Halton(i + 1, 3);
            } else {
              // upper triangle sentinel
              mat[i * n + j] = -77.0 - (i * n + j);
            }
          }
        }

        vector<mjtNum> x(n, 0.0);
        for (int i = 0; i < n; i++) {
          mjtNum val = mju_Halton(i + 1, 5) - 0.5;
          if (pattern == 0) {
            // dense
            x[i] = val;
          } else if (pattern == 1) {
            // alternating zeros and non-zeros (tests xk == 0 and fill-in)
            x[i] = (i % 2 == 0) ? val : 0.0;
          } else if (pattern == 2) {
            // leading and trailing zeros with sparse interior
            x[i] = (i >= n / 4 && i < (3 * n) / 4 && (i % 3 != 1)) ? val : 0.0;
          } else if (pattern == 3) {
            // large magnitude to trigger rank deficiency when flg_plus == 0
            x[i] = 4.0 * val;
          } else if (pattern == 4) {
            // alternating zeros with zero odd-row lower triangle (tests xk1==0)
            x[i] = (i % 2 == 0) ? val : 0.0;
          } else {
            // single zero row k=1 (tests xk1==0 with all 4 unrolled lanes != 0)
            x[i] = (i == 1) ? 0.0 : val;
          }
        }

        vector<mjtNum> mat_ref = mat;
        vector<mjtNum> x_ref = x;

        int rank = mju_cholUpdate(mat.data(), x.data(), n, flg_plus);
        int rank_ref =
            ref_chol_update(mat_ref.data(), x_ref.data(), n, flg_plus);

        EXPECT_EQ(rank, rank_ref)
            << "n=" << n << " flg_plus=" << flg_plus << " pattern=" << pattern;
        for (int i = 0; i < n * n; i++) {
          EXPECT_EQ(mat[i], mat_ref[i])
              << "mat mismatch at " << i << " n=" << n
              << " flg_plus=" << flg_plus << " pattern=" << pattern;
        }
        for (int i = 0; i < n; i++) {
          EXPECT_EQ(x[i], x_ref[i])
              << "x mismatch at " << i << " n=" << n << " flg_plus=" << flg_plus
              << " pattern=" << pattern;
        }
      }
    }
  }
}

// Test for mju_cholUpdateSparse: sparse rank-one Cholesky update L'*L +/- x*x'
// Uses sparse reverse Cholesky factorization and sparse rank-one update.
TEST_F(EngineUtilSolveTest, MjuCholUpdateSparse) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

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
    vector<mjtNum> scratch(n);
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
                           colind.data(), d.get());

      // apply sparse rank-one update
      int rank_sparse = mju_cholUpdateSparse(
          L_sparse.data(), x_sparse.data(), n, flg_plus, rownnz.data(),
          rowadr.data(), colind.data(), x_nnz, x_ind.data(), scratch.data());
      EXPECT_EQ(rank_sparse, n)
          << "Sparse update rank loss for n=" << n << ", flg_plus=" << flg_plus;

      // reconstruct H from L'*L and compare to expected
      mju_sparse2dense(L_sparse_dense.data(), L_sparse.data(), n, n,
                       rownnz.data(), rowadr.data(), colind.data());
      mju_mulMatTMat(H_sparse_reconstructed.data(), L_sparse_dense.data(),
                     L_sparse_dense.data(), n, n, n);

      // compare
      mjtNum eps = MjTol(1e-8, 5e-4);
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
}

// Test that mju_cholFactorSymbolic + mju_cholFactorNumeric produces identical
// results to the reference implementation mju_cholFactorSparse
TEST_F(EngineUtilSolveTest, CholFactorSymbolicNumeric) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

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
                                   LT_rownnz, LT_rowadr, nullptr, nullptr,
                                   HT_rownnz, HT_rowadr, HT_colind, n, d.get());

  // filling phase: compute L_colind, LT_colind, and LT_pos
  int L_colind[16], LT_colind[16], LT_pos[16], L_super[4];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_pos, L_super, HT_rownnz, HT_rowadr,
                         HT_colind, n, d.get());

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
  int numeric_scratch[4];
  int rank_new = mju_cholFactorNumeric(
      L_new, n, 1e-10, L_rownnz, L_rowadr, L_colind, LT_rownnz, LT_rowadr,
      LT_colind, LT_pos, L_super, sparseH, H_rownnz, H_rowadr, H_colind,
      numeric_scratch);

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
                                      L_ref_colind, d.get());

  // compare results
  EXPECT_EQ(rank_new, rank_ref);
  EXPECT_EQ(rank_new, n);

  // compare L values
  mjtNum eps = MjTol(1e-10, 5e-5);
  for (int i = 0; i < nnz; i++) {
    EXPECT_NEAR(L_new[i], L_ref[i], eps) << "mismatch at index " << i;
  }
}

// A pivot lost to rounding leaves its row's coupling in place. The clamp must
// decouple the row, as mju_cholFactor does: dividing the coupling by
// sqrt(mindiag) would drive every later pivot of the factorization negative.
TEST_F(EngineUtilSolveTest, CholFactorNumericClampDecouples) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

  // reverse elimination: row 2 has pivot 1, leaving row 1 with pivot 0 and
  // coupling 0.5 to row 0, whose pivot is 1 once row 1 is decoupled
  constexpr int n = 3;
  mjtNum H[n * n] = {1, .5, 0, .5, 1, 1, 0, 1, 1};
  mjtNum sparseH[n * n];
  int H_rownnz[n], H_rowadr[n], H_colind[n * n];
  mju_dense2sparse(sparseH, H, n, n, H_rownnz, H_rowadr, H_colind, n * n);

  // symbolic factorization
  int L_rownnz[n], L_rowadr[n], LT_rownnz[n], LT_rowadr[n];
  int nnz = mju_cholFactorSymbolic(nullptr, L_rownnz, L_rowadr, nullptr,
                                   LT_rownnz, LT_rowadr, nullptr, nullptr,
                                   H_rownnz, H_rowadr, H_colind, n, d.get());
  int L_colind[n * n], LT_colind[n * n], LT_map[n * n], L_super[n];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_map, L_super, H_rownnz, H_rowadr,
                         H_colind, n, d.get());

  // numeric factorization: only row 1 is clamped
  mjtNum L[n * n];
  int scratch[n];
  int rank =
      mju_cholFactorNumeric(L, n, mjMINVAL, L_rownnz, L_rowadr, L_colind,
                            LT_rownnz, LT_rowadr, LT_colind, LT_map, L_super,
                            sparseH, H_rownnz, H_rowadr, H_colind, scratch);
  EXPECT_EQ(rank, n - 1);

  // row 1 is decoupled: clamped diagonal, zero off-diagonals
  int adr1 = L_rowadr[1], nnz1 = L_rownnz[1];
  EXPECT_EQ(L[adr1 + nnz1 - 1], mju_sqrt(mjMINVAL));
  for (int i = 0; i < nnz1 - 1; i++) {
    EXPECT_EQ(L[adr1 + i], 0);
  }

  // row 0 keeps its pivot, every entry is bounded
  EXPECT_EQ(L[L_rowadr[0] + L_rownnz[0] - 1], 1);
  for (int i = 0; i < nnz; i++) {
    EXPECT_LE(mju_abs(L[i]), 1) << "entry " << i;
  }
}

// NaN pivots must trigger rank-deficiency clamping and row decoupling rather
// than bypassing `diag < mindiag` and poisoning the factor with NaN while
// reporting full rank.
TEST_F(EngineUtilSolveTest, CholFactorNaNPivotClamps) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);
  const mjtNum nan = std::numeric_limits<mjtNum>::quiet_NaN();

  // rows {0, 1, 2} form a 3-row supernode; row 3 is an isolated leaf
  constexpr int n = 4;
  mjtNum H[n * n] = {
      4, 1, 1, 0, 1, 4, 1, 0, 1, 1, 4, 0, 0, 0, 0, 4,
  };
  mjtNum sparseH[n * n];
  int H_rownnz[n], H_rowadr[n], H_colind[n * n];
  mju_dense2sparse(sparseH, H, n, n, H_rownnz, H_rowadr, H_colind, n * n);

  int L_rownnz[n], L_rowadr[n], LT_rownnz[n], LT_rowadr[n];
  int nnz = mju_cholFactorSymbolic(nullptr, L_rownnz, L_rowadr, nullptr,
                                   LT_rownnz, LT_rowadr, nullptr, nullptr,
                                   H_rownnz, H_rowadr, H_colind, n, d.get());
  int L_colind[n * n], LT_colind[n * n], LT_map[n * n], L_super[n];
  mju_cholFactorSymbolic(L_colind, L_rownnz, L_rowadr, LT_colind, LT_rownnz,
                         LT_rowadr, LT_map, L_super, H_rownnz, H_rowadr,
                         H_colind, n, d.get());

  // poison supernode pivot (1, 1) and leaf pivot (3, 3) with NaN
  H[1 * n + 1] = nan;
  H[3 * n + 3] = nan;
  for (int r : {1, 3}) {
    for (int k = H_rowadr[r]; k < H_rowadr[r] + H_rownnz[r]; k++) {
      if (H_colind[k] == r) {
        sparseH[k] = nan;
      }
    }
  }

  // dense mju_cholFactor
  mjtNum L_dense[n * n];
  mju_copy(L_dense, H, n * n);
  EXPECT_EQ(mju_cholFactor(L_dense, n, mjMINVAL), n - 2);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j <= i; j++) {
      EXPECT_FALSE(mju_isBad(L_dense[i * n + j]))
          << "dense (" << i << ", " << j << ")";
    }
  }
  EXPECT_EQ(L_dense[1 * n + 1], mju_sqrt(mjMINVAL));
  EXPECT_EQ(L_dense[3 * n + 3], mju_sqrt(mjMINVAL));

  // sparse mju_cholFactorNumeric (hits both leaf fast path and supernode path)
  mjtNum L_num[n * n];
  int scratch[n];
  EXPECT_EQ(
      mju_cholFactorNumeric(L_num, n, mjMINVAL, L_rownnz, L_rowadr, L_colind,
                            LT_rownnz, LT_rowadr, LT_colind, LT_map, L_super,
                            sparseH, H_rownnz, H_rowadr, H_colind, scratch),
      n - 2);
  for (int i = 0; i < nnz; i++) {
    EXPECT_FALSE(mju_isBad(L_num[i])) << "numeric entry " << i;
  }
  EXPECT_EQ(L_num[L_rowadr[1] + L_rownnz[1] - 1], mju_sqrt(mjMINVAL));
  EXPECT_EQ(L_num[L_rowadr[1]], 0);
  EXPECT_EQ(L_num[L_rowadr[3] + L_rownnz[3] - 1], mju_sqrt(mjMINVAL));

  // sparse reference mju_cholFactorSparse
  mjtNum L_ref[n * n];
  int L_ref_rownnz[n], L_ref_colind[n * n];
  for (int r = 0; r < n; r++) {
    int lower_nnz = 0;
    for (int k = 0; k < H_rownnz[r]; k++) {
      int c = H_colind[H_rowadr[r] + k];
      if (c <= r) {
        L_ref[L_rowadr[r] + lower_nnz] = sparseH[H_rowadr[r] + k];
        L_ref_colind[L_rowadr[r] + lower_nnz] = c;
        lower_nnz++;
      }
    }
    L_ref_rownnz[r] = lower_nnz;
  }
  EXPECT_EQ(mju_cholFactorSparse(L_ref, n, mjMINVAL, L_ref_rownnz, L_rowadr,
                                 L_ref_colind, d.get()),
            n - 2);
  for (int r = 0; r < n; r++) {
    for (int k = 0; k < L_ref_rownnz[r]; k++) {
      EXPECT_FALSE(mju_isBad(L_ref[L_rowadr[r] + k]))
          << "sparse (" << r << ", " << k << ")";
    }
  }

  // banded mju_cholFactorBand returns 0 on NaN pivot
  mjtNum B[n] = {2, nan, 2, 2};
  EXPECT_EQ(mju_cholFactorBand(B, n, /*nband=*/1, /*ndense=*/0, 0, 0), 0);
}

// Sparse factor and solve on random elimination-tree patterns, with rank-1
// couplings across branches so that adjacent rows can have the same number of
// descendants but different descendant rows.
TEST_F(EngineUtilSolveTest, CholFactorNumericRandomTree) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);
  std::mt19937 rng(1);
  std::uniform_real_distribution<double> unif(-1, 1);

  for (int trial = 0; trial < 1000; trial++) {
    const int n = 4 + rng() % 30;

    // random forest, parent[i] < i
    vector<int> parent(n, -1);
    for (int i = 1; i < n; i++) {
      parent[i] = (rng() % 5 == 0) ? -1 : rng() % i;
    }

    // mass-matrix pattern: H(i, j) != 0 iff j is an ancestor of i
    vector<mjtNum> H(n * n, 0);
    for (int i = 0; i < n; i++) {
      for (int j = parent[i]; j >= 0; j = parent[j]) {
        H[i * n + j] = H[j * n + i] = unif(rng);
      }
    }

    // rank-1 terms J'J with J supported on the ancestor chains of two dofs
    const int ncoupling = rng() % 4;
    for (int k = 0; k < ncoupling; k++) {
      vector<mjtNum> J(n, 0);
      for (int leaf :
           {static_cast<int>(rng() % n), static_cast<int>(rng() % n)}) {
        for (int j = leaf; j >= 0; j = parent[j]) {
          J[j] = unif(rng);
        }
      }
      for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
          H[i * n + j] += J[i] * J[j];
        }
      }
    }

    // make diagonally dominant
    for (int i = 0; i < n; i++) {
      mjtNum sum = 1;
      for (int j = 0; j < n; j++) {
        sum += (j != i) ? mju_abs(H[i * n + j]) : 0;
      }
      H[i * n + i] += sum;
    }

    // sparse H, symbolic and numeric factorization
    vector<mjtNum> sparseH(n * n);
    vector<int> H_rownnz(n), H_rowadr(n), H_colind(n * n);
    mju_dense2sparse(sparseH.data(), H.data(), n, n, H_rownnz.data(),
                     H_rowadr.data(), H_colind.data(), n * n);
    vector<int> L_rownnz(n), L_rowadr(n), LT_rownnz(n), LT_rowadr(n);
    int nnz = mju_cholFactorSymbolic(
        nullptr, L_rownnz.data(), L_rowadr.data(), nullptr, LT_rownnz.data(),
        LT_rowadr.data(), nullptr, nullptr, H_rownnz.data(), H_rowadr.data(),
        H_colind.data(), n, d.get());
    vector<int> L_colind(nnz), LT_colind(nnz), LT_map(nnz), L_super(n);
    mju_cholFactorSymbolic(L_colind.data(), L_rownnz.data(), L_rowadr.data(),
                           LT_colind.data(), LT_rownnz.data(), LT_rowadr.data(),
                           LT_map.data(), L_super.data(), H_rownnz.data(),
                           H_rowadr.data(), H_colind.data(), n, d.get());
    vector<mjtNum> L(nnz);
    vector<int> scratch(n, 0x7f7f7f7f);
    int rank = mju_cholFactorNumeric(
        L.data(), n, mjMINVAL, L_rownnz.data(), L_rowadr.data(),
        L_colind.data(), LT_rownnz.data(), LT_rowadr.data(), LT_colind.data(),
        LT_map.data(), L_super.data(), sparseH.data(), H_rownnz.data(),
        H_rowadr.data(), H_colind.data(), scratch.data());
    ASSERT_EQ(rank, n) << "trial " << trial;

    // H == L'L (reverse Cholesky)
    vector<mjtNum> L_dense(n * n), LTL(n * n);
    mju_sparse2dense(L_dense.data(), L.data(), n, n, L_rownnz.data(),
                     L_rowadr.data(), L_colind.data());
    mju_mulMatTMat(LTL.data(), L_dense.data(), L_dense.data(), n, n, n);
    mjtNum eps = MjTol(1e-12, 1e-4);
    for (int i = 0; i < n * n; i++) {
      ASSERT_THAT(LTL[i], MjNear(H[i], eps, eps))
          << "trial " << trial << ", n " << n << ", entry (" << i / n << ", "
          << i % n << ")";
    }

    // H * solve(rhs) == rhs
    vector<mjtNum> rhs(n), x(n), Hx(n);
    for (int i = 0; i < n; i++) {
      rhs[i] = unif(rng);
    }
    mju_cholSolveSparse(x.data(), L.data(), rhs.data(), n, L_rownnz.data(),
                        L_rowadr.data(), L_colind.data());
    mju_mulMatVec(Hx.data(), H.data(), x.data(), n, n);
    EXPECT_THAT(Hx, Pointwise(MjNear(eps, eps), rhs)) << "trial " << trial;
  }
}

// Verify mju_cholFactorNumeric on multi-DOF supernodes (ns = 1, 2, 3) with
// fill-in and batched non-suffix descendant updates (covering all 11 (ns,
// batch) specializations) against mju_cholFactorSparse.
TEST_F(EngineUtilSolveTest, CholFactorNumericSupernodeNonSuffix) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

  std::mt19937_64 rng(17);
  std::normal_distribution<double> normal(0.0, 1.0);
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  auto verify_blocks = [&](const vector<int>& bsize, double density) {
    int nb = bsize.size();
    vector<int> badr(nb + 1, 0), blk_of(0);
    for (int p = 0; p < nb; p++) {
      badr[p + 1] = badr[p] + bsize[p];
      for (int a = 0; a < bsize[p]; a++) {
        blk_of.push_back(p);
      }
    }
    int n = badr[nb];

    vector<vector<bool>> adj(nb, vector<bool>(nb, false));
    for (int p = 0; p < nb; p++) {
      adj[p][p] = true;
      for (int q = p + 1; q < nb; q++) {
        if (unif(rng) < density) {
          adj[p][q] = adj[q][p] = true;
        }
      }
    }

    // build SPD block-sparse matrix H = A * A' + n * I
    vector<mjtNum> A(n * n, 0);
    for (int p = 0; p < nb; p++) {
      for (int q = 0; q <= p; q++) {
        if (!adj[p][q]) continue;
        for (int a = 0; a < bsize[p]; a++) {
          for (int b = 0; b < bsize[q]; b++) {
            A[(badr[p] + a) * n + (badr[q] + b)] = normal(rng);
          }
        }
      }
    }
    vector<mjtNum> H_dense(n * n, 0);
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        int p = blk_of[i], q = blk_of[j];
        bool coupled = false;
        for (int k = 0; k < nb; k++) {
          if (adj[p][k] && adj[q][k]) {
            coupled = true;
            break;
          }
        }
        if (!coupled) continue;
        mjtNum sum = (i == j) ? static_cast<mjtNum>(n) : 0;
        for (int k = 0; k < n; k++) {
          sum += A[i * n + k] * A[j * n + k];
        }
        H_dense[i * n + j] = sum;
      }
    }

    // convert H_dense to CSR
    int max_nnz = n * n;
    vector<mjtNum> H_val(max_nnz);
    vector<int> H_rownnz(n), H_rowadr(n), H_colind(max_nnz);
    ASSERT_EQ(
        mju_dense2sparse(H_val.data(), H_dense.data(), n, n, H_rownnz.data(),
                         H_rowadr.data(), H_colind.data(), max_nnz),
        0);
    int h_nnz = H_rowadr[n - 1] + H_rownnz[n - 1];
    H_val.resize(h_nnz);
    H_colind.resize(h_nnz);

    // symbolic factorization
    vector<int> L_rownnz(n), L_rowadr(n), LT_rownnz(n), LT_rowadr(n);
    int l_nnz = mju_cholFactorSymbolic(
        nullptr, L_rownnz.data(), L_rowadr.data(), nullptr, LT_rownnz.data(),
        LT_rowadr.data(), nullptr, nullptr, H_rownnz.data(), H_rowadr.data(),
        H_colind.data(), n, d.get());
    vector<int> L_colind(l_nnz), LT_colind(l_nnz), LT_map(l_nnz), L_super(n);
    mju_cholFactorSymbolic(L_colind.data(), L_rownnz.data(), L_rowadr.data(),
                           LT_colind.data(), LT_rownnz.data(), LT_rowadr.data(),
                           LT_map.data(), L_super.data(), H_rownnz.data(),
                           H_rowadr.data(), H_colind.data(), n, d.get());

    // numeric factorization with poison sentinel in scratch to catch unmapped
    // reads under ASAN
    vector<mjtNum> L_num(l_nnz);
    vector<int> scratch(n, 0x7f7f7f7f);
    int rank_num = mju_cholFactorNumeric(
        L_num.data(), n, mjMINVAL, L_rownnz.data(), L_rowadr.data(),
        L_colind.data(), LT_rownnz.data(), LT_rowadr.data(), LT_colind.data(),
        LT_map.data(), L_super.data(), H_val.data(), H_rownnz.data(),
        H_rowadr.data(), H_colind.data(), scratch.data());
    EXPECT_EQ(rank_num, n);

    // reference mju_cholFactorSparse
    vector<mjtNum> L_ref(l_nnz);
    vector<int> L_ref_rownnz(n), L_ref_colind(l_nnz);
    for (int r = 0; r < n; r++) {
      int lower_nnz = 0;
      for (int i = 0; i < H_rownnz[r]; i++) {
        int c = H_colind[H_rowadr[r] + i];
        if (c <= r) {
          L_ref[L_rowadr[r] + lower_nnz] = H_val[H_rowadr[r] + i];
          L_ref_colind[L_rowadr[r] + lower_nnz] = c;
          lower_nnz++;
        }
      }
      L_ref_rownnz[r] = lower_nnz;
    }
    int rank_ref =
        mju_cholFactorSparse(L_ref.data(), n, mjMINVAL, L_ref_rownnz.data(),
                             L_rowadr.data(), L_ref_colind.data(), d.get());
    EXPECT_EQ(rank_ref, n);

    mjtNum eps = MjTol(1e-11, 5e-4);
    for (int i = 0; i < l_nnz; i++) {
      EXPECT_NEAR(L_num[i], L_ref[i], eps)
          << "nb=" << nb << " n=" << n << " density=" << density
          << " idx=" << i;
    }

    // verify L' * L * x = rhs
    vector<mjtNum> rhs(n), sol(n);
    for (int i = 0; i < n; i++) rhs[i] = normal(rng);
    mju_cholSolveSparse(sol.data(), L_num.data(), rhs.data(), n,
                        L_rownnz.data(), L_rowadr.data(), L_colind.data());
    for (int i = 0; i < n; i++) {
      mjtNum ax = 0;
      for (int j = 0; j < n; j++) ax += H_dense[i * n + j] * sol[j];
      EXPECT_NEAR(ax, rhs[i], eps);
    }
  };

  // uniform block sizes
  for (int bs : {1, 2, 3, 4}) {
    for (int nb : {6, 12}) {
      for (double density : {0.25, 0.5}) {
        verify_blocks(vector<int>(nb, bs), density);
      }
    }
  }

  // mixed per-block sizes in {1, 2, 3, 4} to cover all (ns, batch) pairs
  for (int trial = 0; trial < 40; trial++) {
    int nb = 8 + (trial % 7);
    double density = (trial % 2 == 0) ? 0.25 : 0.4;
    vector<int> bsize(nb);
    for (int p = 0; p < nb; p++) {
      bsize[p] = 1 + static_cast<int>(rng() % 4);
    }
    verify_blocks(bsize, density);
  }
}

// ----------------------------- dense LU --------------------------------------

// mju_cholFactorSymbolicBlocked on a block pattern computes the same L arrays,
// entry for entry, as mju_cholFactorSymbolic on the expanded dof pattern, and
// the block transpose of its LT arrays
TEST_F(EngineUtilSolveTest, CholFactorSymbolicBlocked) {
  std::mt19937 rng(7);
  for (int bs = 1; bs <= 3; bs++) {
    for (int trial = 0; trial < 20; trial++) {
      int nb = 1 + trial;
      std::uniform_real_distribution<double> unif(0, 1);
      double density = trial % 2 ? 0.15 : 0.4;

      // a random symmetric block pattern, columns ascending, diagonal listed
      // on odd trials only
      vector<vector<int>> rows(nb);
      vector<vector<bool>> adj(nb, vector<bool>(nb, false));
      for (int p = 0; p < nb; p++) {
        for (int q = p + 1; q < nb; q++) {
          if (unif(rng) < density) adj[p][q] = adj[q][p] = true;
        }
      }
      vector<int> rownnz(nb), rowadr(nb), colind;
      for (int p = 0; p < nb; p++) {
        rowadr[p] = colind.size();
        for (int q = 0; q < nb; q++) {
          if (adj[p][q] || (q == p && trial % 2)) colind.push_back(q);
        }
        rownnz[p] = colind.size() - rowadr[p];
      }

      // the expanded dof pattern: dense blocks, dense diagonal blocks
      int n = bs * nb;
      vector<int> drownnz(n), drowadr(n), dcolind;
      for (int r = 0; r < n; r++) {
        drowadr[r] = dcolind.size();
        for (int c = 0; c < n; c++) {
          if (r / bs == c / bs || adj[r / bs][c / bs]) dcolind.push_back(c);
        }
        drownnz[r] = dcolind.size() - drowadr[r];
      }

      // counting
      vector<int> L_rownnz(n), L_rowadr(n), LT_rownnz(n), LT_rowadr(n);
      vector<int> bL_rownnz(n), bL_rowadr(n), BT_adr(nb + 1);
      vector<int> scratch(3 * nb);
      int nnz = mju_cholFactorSymbolic(
          nullptr, L_rownnz.data(), L_rowadr.data(), nullptr, LT_rownnz.data(),
          LT_rowadr.data(), nullptr, nullptr, drownnz.data(), drowadr.data(),
          dcolind.data(), n, nullptr);
      int bnnz = mju_cholFactorSymbolicBlocked(
          nullptr, bL_rownnz.data(), bL_rowadr.data(), BT_adr.data(), nullptr,
          nullptr, rownnz.data(), rowadr.data(), colind.data(), nb, bs,
          scratch.data());
      ASSERT_EQ(bnnz, nnz) << "bs " << bs << " trial " << trial;
      EXPECT_EQ(bL_rownnz, L_rownnz);
      EXPECT_EQ(bL_rowadr, L_rowadr);

      // filling
      vector<int> L_colind(nnz), LT_colind(nnz), LT_map(nnz), L_super(n);
      vector<int> bL_colind(nnz), BT_row(BT_adr[nb]), BT_off(BT_adr[nb]);
      mju_cholFactorSymbolic(
          L_colind.data(), L_rownnz.data(), L_rowadr.data(), LT_colind.data(),
          LT_rownnz.data(), LT_rowadr.data(), LT_map.data(), L_super.data(),
          drownnz.data(), drowadr.data(), dcolind.data(), n, nullptr);
      mju_cholFactorSymbolicBlocked(
          bL_colind.data(), L_rownnz.data(), L_rowadr.data(), BT_adr.data(),
          BT_row.data(), BT_off.data(), rownnz.data(), rowadr.data(),
          colind.data(), nb, bs, scratch.data());
      EXPECT_EQ(bL_colind, L_colind) << "bs " << bs << " trial " << trial;

      // the block transpose: the first dof of each block row below the
      // diagonal block in the transpose row of the first dof of each column
      vector<int> eBT_adr(nb + 1, 0), eBT_row, eBT_off;
      for (int q = 0; q < nb; q++) {
        eBT_adr[q] = eBT_row.size();
        for (int e = LT_rowadr[bs * q];
             e < LT_rowadr[bs * q] + LT_rownnz[bs * q]; e++) {
          int r = LT_colind[e];
          if (r >= bs * (q + 1) && r % bs == 0) {
            eBT_row.push_back(r / bs);
            eBT_off.push_back(LT_map[e] - L_rowadr[r]);
          }
        }
      }
      eBT_adr[nb] = eBT_row.size();
      EXPECT_EQ(BT_adr, eBT_adr) << "bs " << bs << " trial " << trial;
      EXPECT_EQ(BT_row, eBT_row) << "bs " << bs << " trial " << trial;
      EXPECT_EQ(BT_off, eBT_off) << "bs " << bs << " trial " << trial;
    }
  }
}

using DenseLUTest = MujocoTest;

// factor identity, solve recovers b exactly
TEST_F(DenseLUTest, Identity) {
  constexpr int n = 4;
  mjtNum A[n * n] = {
      1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
  };
  int pivot[n];
  mjtNum b[n] = {1, 2, 3, 4};
  mjtNum x[n];

  EXPECT_EQ(mju_factorLU(A, n, pivot), 1);
  mju_solveLU(x, A, b, pivot, n);

  for (int i = 0; i < n; i++) {
    EXPECT_MJTNUM_EQ(x[i], b[i]);
  }
}

// 3x3 with known solution
TEST_F(DenseLUTest, SmallKnown) {
  constexpr int n = 3;
  // A = [2 1 1; 4 3 3; 8 7 9], b = [1; 1; 1]
  // solution: x = [1; -1; 0]  (verified: A*x = [2-1; 4-3; 8-7] = [1;1;1])
  mjtNum A[n * n] = {
      2, 1, 1, 4, 3, 3, 8, 7, 9,
  };
  int pivot[n];
  mjtNum b[n] = {1, 1, 1};
  mjtNum x[n];

  EXPECT_EQ(mju_factorLU(A, n, pivot), 1);
  mju_solveLU(x, A, b, pivot, n);

  mjtNum eps = MjTol(1e-14, 1e-6);
  EXPECT_NEAR(x[0], 1, eps);
  EXPECT_NEAR(x[1], -1, eps);
  EXPECT_NEAR(x[2], 0, eps);
}

// random SPD matrices: compare LU solve against Cholesky solve
TEST_F(DenseLUTest, RandomSPD) {
  std::mt19937_64 rng;
  rng.seed(7);
  std::normal_distribution<double> dist(0, 1);

  for (int n : {4, 8, 16}) {
    vector<mjtNum> sqrtH(n * n);
    vector<mjtNum> A(n * n);
    vector<mjtNum> A_chol(n * n);
    vector<mjtNum> b(n);
    vector<mjtNum> x_lu(n);
    vector<mjtNum> x_chol(n);
    vector<int> pivot(n);

    // generate random SPD matrix
    for (int i = 0; i < n * n; i++) sqrtH[i] = dist(rng);
    mju_mulMatTMat(A.data(), sqrtH.data(), sqrtH.data(), n, n, n);

    // add diagonal regularizer
    for (int i = 0; i < n; i++) A[i * n + i] += n;

    // generate random rhs
    for (int i = 0; i < n; i++) b[i] = dist(rng);

    // solve with Cholesky
    mju_copy(A_chol.data(), A.data(), n * n);
    int rank = mju_cholFactor(A_chol.data(), n, 0);
    EXPECT_EQ(rank, n);
    mju_cholSolve(x_chol.data(), A_chol.data(), b.data(), n);

    // solve with LU
    int ok = mju_factorLU(A.data(), n, pivot.data());
    EXPECT_EQ(ok, 1);
    mju_solveLU(x_lu.data(), A.data(), b.data(), pivot.data(), n);

    // compare
    mjtNum eps = MjTol(1e-15, 1e-7);
    EXPECT_THAT(AsVector(x_lu.data(), n),
                Pointwise(MjNear(eps, eps), AsVector(x_chol.data(), n)));
  }
}

// random non-symmetric matrices: verify A*x == b
TEST_F(DenseLUTest, RandomGeneral) {
  std::mt19937_64 rng;
  rng.seed(42);
  std::normal_distribution<double> dist(0, 1);

  for (int n : {3, 5, 10, 20}) {
    vector<mjtNum> A(n * n);
    vector<mjtNum> A_orig(n * n);
    vector<mjtNum> b(n);
    vector<mjtNum> x(n);
    vector<mjtNum> Ax(n);
    vector<int> pivot(n);

    // random non-symmetric matrix with diagonal dominance
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        A[i * n + j] = dist(rng);
      }
      A[i * n + i] += 2 * n;
    }
    mju_copy(A_orig.data(), A.data(), n * n);

    // random rhs
    for (int i = 0; i < n; i++) b[i] = dist(rng);

    // factor and solve
    int ok = mju_factorLU(A.data(), n, pivot.data());
    EXPECT_EQ(ok, 1);
    mju_solveLU(x.data(), A.data(), b.data(), pivot.data(), n);

    // verify: A_orig * x == b
    mju_mulMatVec(Ax.data(), A_orig.data(), x.data(), n, n);

    mjtNum eps = MjTol(1e-14, 1e-5);
    EXPECT_THAT(AsVector(Ax.data(), n),
                Pointwise(MjNear(eps, eps), AsVector(b.data(), n)));
  }
}

// near-singular matrix returns 0
TEST_F(DenseLUTest, Singular) {
  constexpr int n = 3;
  // all zeros: maximally singular
  mjtNum A[n * n] = {0};
  int pivot[n];

  EXPECT_EQ(mju_factorLU(A, n, pivot), 0);
}

// 6x6 specialization: identical results to the generic version
TEST_F(DenseLUTest, Fixed6MatchesGeneric) {
  constexpr int n = 6;
  std::mt19937_64 rng;
  rng.seed(3);
  std::normal_distribution<double> dist(0, 1);

  for (int trial = 0; trial < 100; trial++) {
    mjtNum A[n * n], A6[n * n], b[n], x[n], x6[n];
    int pivot[n], pivot6[n];

    // random asymmetric matrix; first trial zeroes the diagonal to force
    // pivoting
    for (int i = 0; i < n * n; i++) {
      A[i] = dist(rng);
    }
    if (trial == 0) {
      for (int i = 0; i < n; i++) {
        A[i * n + i] = 0;
      }
    }
    for (int i = 0; i < n; i++) {
      b[i] = dist(rng);
    }
    mju_copy(A6, A, n * n);

    // same algorithm; results agree to rounding (not bitwise: the compiler may
    // contract the unrolled fixed-size version differently)
    int rank = mju_factorLU(A, n, pivot);
    int rank6 = mju_factorLU6(A6, pivot6);
    ASSERT_EQ(rank, rank6);
    if (!rank) {
      continue;
    }
    for (int i = 0; i < n; i++) {
      EXPECT_EQ(pivot[i], pivot6[i]);
    }
    for (int i = 0; i < n * n; i++) {
      EXPECT_NEAR(A[i], A6[i], MjTol(1e-13, 1e-4));
    }

    mju_solveLU(x, A, b, pivot, n);
    mju_solveLU6(x6, A6, b, pivot6);
    for (int i = 0; i < n; i++) {
      EXPECT_NEAR(x[i], x6[i], MjTol(1e-12, 1e-3));
    }
  }
}

// 6x6 specialization: singularity at each step k = 0..5 and in-place solve
TEST_F(DenseLUTest, Fixed6SingularAndInPlace) {
  constexpr int n = 6;
  for (int zero_row = 0; zero_row < n; zero_row++) {
    mjtNum A[n * n] = {0};
    for (int i = 0; i < n; i++) {
      if (i != zero_row) {
        A[i * n + i] = i + 1;
      }
    }
    int pivot[n];
    EXPECT_EQ(mju_factorLU6(A, pivot), 0) << "zero_row=" << zero_row;
  }

  mjtNum A[n * n] = {
      10, 1, 2, 0, -1, 3, 2, 9, -1, 1, 0, 2, -1, 3, 8, 2,  1, 0,
      0,  2, 1, 7, -2, 1, 1, 0, -2, 3, 9, 1, -2, 1, 0, -1, 2, 11,
  };
  mjtNum A_ref[n * n];
  mju_copy(A_ref, A, n * n);
  int pivot[n], pivot_ref[n];
  ASSERT_EQ(mju_factorLU6(A, pivot), 1);
  ASSERT_EQ(mju_factorLU(A_ref, n, pivot_ref), 1);

  mjtNum x[n] = {1, -2, 3, -4, 5, -6};
  mjtNum b_copy[n], x_ref[n];
  mju_copy(b_copy, x, n);
  mju_solveLU6(x, A, x, pivot);
  mju_solveLU(x_ref, A_ref, b_copy, pivot_ref, n);
  mjtNum eps = MjTol(1e-14, 1e-5);
  EXPECT_THAT(AsVector(x, n), Pointwise(MjNear(eps, eps), AsVector(x_ref, n)));
}

// nonsingular matrix with all-zero diagonal: requires pivoting
TEST_F(DenseLUTest, ZeroDiagonal) {
  constexpr int n = 3;
  // A = [0 2 1; 1 0 3; 4 1 0], det(A) = 25
  mjtNum A[n * n] = {
      0, 2, 1, 1, 0, 3, 4, 1, 0,
  };
  mjtNum A_orig[n * n];
  mju_copy(A_orig, A, n * n);
  int pivot[n];
  mjtNum b[n] = {1, 2, 3};
  mjtNum x[n], Ax[n];

  EXPECT_EQ(mju_factorLU(A, n, pivot), 1);
  mju_solveLU(x, A, b, pivot, n);

  // verify A_orig * x == b
  mju_mulMatVec(Ax, A_orig, x, n, n);
  mjtNum eps = MjTol(1e-14, 1e-6);
  EXPECT_THAT(AsVector(Ax, n), Pointwise(MjNear(eps, eps), AsVector(b, n)));
}

// --------------------------------- mju_eig3 ----------------------------------

using Eig3Test = MujocoTest;

// residuals of the decomposition mat = eigvec * diag(eigval) * eigvec':
// reconstruction error relative to the largest element of mat, and deviation of
// eigvec from orthonormality
static void Eig3Residual(mjtNum* recon, mjtNum* orth, const mjtNum mat[9],
                         const mjtNum eigval[3], const mjtNum eigvec[9]) {
  mjtNum scale = 0;
  for (int i = 0; i < 9; i++) scale = mju_max(scale, mju_abs(mat[i]));

  *recon = *orth = 0;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mjtNum A = 0, I = 0;
      for (int k = 0; k < 3; k++) {
        A += eigvec[3 * i + k] * eigval[k] * eigvec[3 * j + k];
        I += eigvec[3 * k + i] * eigvec[3 * k + j];
      }
      *recon = mju_max(*recon, mju_abs(A - mat[3 * i + j]) / scale);
      *orth = mju_max(*orth, mju_abs(I - (i == j)));
    }
  }
}

// the decomposition converges to roundoff, whatever the scale of the matrix
TEST_F(Eig3Test, ConvergesAtAnyScale) {
  for (mjtNum scale : {1e-15, 1.0, 1e15}) {
    mjtNum mat[9] = {2, .1, .2, .1, 3, .3, .2, .3, 4};
    mju_scl(mat, mat, scale, 9);

    mjtNum eigval[3], eigvec[9], quat[4];
    EXPECT_LT(mju_eig3(eigval, eigvec, quat, mat), 20);

    // eigenvalues are decreasing
    EXPECT_GT(eigval[0], eigval[1]);
    EXPECT_GT(eigval[1], eigval[2]);

    mjtNum recon, orth;
    Eig3Residual(&recon, &orth, mat, eigval, eigvec);
    EXPECT_LE(recon, MjTol(1e-13, 3e-5)) << "scale " << scale;
    EXPECT_LE(orth, MjTol(5e-14, 5e-6)) << "scale " << scale;
  }
}

// repeated and nearly repeated eigenvalues, where the Jacobi rotation is
// ill-defined, converge in a few iterations
TEST_F(Eig3Test, RepeatedEigenvalues) {
  std::mt19937_64 rng;
  rng.seed(3);
  std::normal_distribution<double> dist(0, 1);

  const mjtNum eps = std::numeric_limits<mjtNum>::epsilon();
  const mjtNum spectrum[4][3] = {
      {3, 1, 1}, {3, 3, 1}, {1, 1, 1}, {1 + 100 * eps, 1, 1 - 100 * eps}};

  for (const mjtNum* w : spectrum) {
    int max_iter = 0;
    mjtNum max_recon = 0;
    for (int n = 0; n < 1000; n++) {
      // random rotation
      mjtNum R[9], q[4] = {(mjtNum)dist(rng), (mjtNum)dist(rng),
                           (mjtNum)dist(rng), (mjtNum)dist(rng)};
      mju_normalize4(q);
      mju_quat2Mat(R, q);

      // mat = R * diag(w) * R', exactly symmetric
      mjtNum mat[9];
      for (int i = 0; i < 3; i++) {
        for (int j = i; j < 3; j++) {
          mat[3 * i + j] = mat[3 * j + i] = R[3 * i] * w[0] * R[3 * j] +
                                            R[3 * i + 1] * w[1] * R[3 * j + 1] +
                                            R[3 * i + 2] * w[2] * R[3 * j + 2];
        }
      }

      mjtNum eigval[3], eigvec[9], quat[4], recon, orth;
      max_iter = mjMAX(max_iter, mju_eig3(eigval, eigvec, quat, mat));
      Eig3Residual(&recon, &orth, mat, eigval, eigvec);
      max_recon = mju_max(max_recon, recon);
    }
    EXPECT_LT(max_iter, 20)
        << "spectrum " << w[0] << " " << w[1] << " " << w[2];
    EXPECT_LE(max_recon, MjTol(1e-13, 3e-5))
        << "spectrum " << w[0] << " " << w[1] << " " << w[2];
  }
}

// a loose tolerance stops early: off-diagonal elements are below the tolerance
// and eigvec is orthonormal, as the broadphase requires of its frame
TEST_F(Eig3Test, Tolerance) {
  const mjtNum mat[9] = {2, .1, .2, .1, 3, .3, .2, .3, 4};
  const mjtNum reltol = 1e-3;

  mjtNum eigval[3], eigvec[9], quat[4];
  int tight = mju_eig3(eigval, eigvec, quat, mat);
  int loose = mju_eig3Tol(eigval, eigvec, quat, mat, reltol);
  EXPECT_LT(loose, tight);

  // D = eigvec' * mat * eigvec
  mjtNum tmp[9], D[9];
  mju_mulMatTMat(tmp, eigvec, mat, 3, 3, 3);
  mju_mulMatMat(D, tmp, eigvec, 3, 3, 3);
  EXPECT_LE(mju_abs(D[1]), reltol * 4);
  EXPECT_LE(mju_abs(D[2]), reltol * 4);
  EXPECT_LE(mju_abs(D[5]), reltol * 4);

  mjtNum recon, orth;
  Eig3Residual(&recon, &orth, mat, eigval, eigvec);
  EXPECT_LE(orth, MjTol(5e-14, 5e-6));
}

// Verify mju_cholFactorSymbolic matches a scalar reference bit-for-bit
// (including exact LT_colind traversal order and L_super) on mixed supernodes,
// partial lower-triangle coupling, empty rows, and n = 0/1.
TEST_F(EngineUtilSolveTest, CholFactorSymbolicExactMatchesScalar) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr d = MakeData(model);

  auto scalar_ref = [](int n, const vector<int>& rownnz,
                       const vector<int>& rowadr, const vector<int>& colind,
                       vector<int>& L_rownnz, vector<int>& L_rowadr,
                       vector<int>& L_colind, vector<int>& LT_rownnz,
                       vector<int>& LT_rowadr, vector<int>& LT_colind,
                       vector<int>& LT_map, vector<int>& L_super) {
    if (n <= 0) return 0;
    vector<int> parent(n), flag(n);
    L_rownnz.assign(n, 0);
    LT_rownnz.assign(n, 0);
    auto ustart = [&](int r) {
      int s = rowadr[r], e = s + rownnz[r];
      while (s < e && colind[s] <= r) s++;
      return s;
    };
    for (int r = n - 1; r >= 0; r--) {
      parent[r] = -1;
      flag[r] = r;
      L_rownnz[r] = 1;
      int lt_nnz = 1;
      int end = rowadr[r] + rownnz[r];
      for (int c = ustart(r); c < end; c++) {
        int i = colind[c];
        while (flag[i] != r) {
          L_rownnz[i]++;
          lt_nnz++;
          flag[i] = r;
          int p = parent[i];
          if (p == -1) {
            parent[i] = r;
            break;
          }
          i = p;
        }
      }
      LT_rownnz[r] = lt_nnz;
    }
    L_rowadr.resize(n);
    LT_rowadr.resize(n);
    L_rowadr[0] = LT_rowadr[0] = 0;
    int nnz = L_rownnz[0];
    for (int r = 1; r < n; r++) {
      L_rowadr[r] = L_rowadr[r - 1] + L_rownnz[r - 1];
      LT_rowadr[r] = LT_rowadr[r - 1] + LT_rownnz[r - 1];
      nnz += L_rownnz[r];
    }
    L_colind.assign(nnz, -1);
    LT_colind.assign(nnz, -1);
    LT_map.assign(nnz, -1);
    L_super.assign(n, 0);
    vector<int> cursor(n);
    for (int r = n - 1; r >= 0; r--) {
      parent[r] = -1;
      flag[r] = r;
      int diag_idx = L_rowadr[r] + L_rownnz[r] - 1;
      cursor[r] = diag_idx - 1;
      L_colind[diag_idx] = r;
      int lt_write = LT_rowadr[r];
      LT_colind[lt_write] = r;
      LT_map[lt_write] = diag_idx;
      lt_write++;
      int end = rowadr[r] + rownnz[r];
      int overlap = 0;
      for (int c = ustart(r); c < end; c++) {
        int i = colind[c];
        while (flag[i] != r) {
          overlap += (flag[i] == r + 1);
          int L_idx = cursor[i]--;
          L_colind[L_idx] = r;
          LT_colind[lt_write] = i;
          LT_map[lt_write] = L_idx;
          lt_write++;
          flag[i] = r;
          int p = parent[i];
          if (p == -1) {
            parent[i] = r;
            break;
          }
          i = p;
        }
      }
      if (r < n - 1) {
        L_super[r + 1] =
            (parent[r + 1] == r && L_rownnz[r] == L_rownnz[r + 1] - 1 &&
             LT_rownnz[r] == LT_rownnz[r + 1] + 1 &&
             overlap == LT_rownnz[r + 1]);
      }
    }
    L_super[0] = 0;
    for (int r = n - 1; r >= 0; r--) {
      int merge = L_super[r];
      L_super[r] = 1 + merge + (merge && r > 0 && L_super[r - 1]);
    }
    return nnz;
  };

  // n = 0 edge case
  EXPECT_EQ(mju_cholFactorSymbolic(nullptr, nullptr, nullptr, nullptr, nullptr,
                                   nullptr, nullptr, nullptr, nullptr, nullptr,
                                   nullptr, 0, d.get()),
            0);

  std::mt19937 rng(42);
  auto check_graph = [&](int n, const vector<vector<int>>& adj, bool upper_only,
                         mjData* data_ptr) {
    vector<int> rownnz(n), rowadr(n), colind;
    for (int r = 0; r < n; r++) {
      rowadr[r] = colind.size();
      for (int c : adj[r]) {
        if (!upper_only || c >= r) {
          colind.push_back(c);
        }
      }
      rownnz[r] = static_cast<int>(colind.size()) - rowadr[r];
    }

    vector<int> r_L_nnz, r_L_adr, r_L_col, r_LT_nnz, r_LT_adr, r_LT_col,
        r_LT_map, r_L_sup;
    int ref_nnz =
        scalar_ref(n, rownnz, rowadr, colind, r_L_nnz, r_L_adr, r_L_col,
                   r_LT_nnz, r_LT_adr, r_LT_col, r_LT_map, r_L_sup);

    vector<int> L_nnz(n), L_adr(n), LT_nnz(n), LT_adr(n);
    int nnz = mju_cholFactorSymbolic(nullptr, L_nnz.data(), L_adr.data(),
                                     nullptr, LT_nnz.data(), LT_adr.data(),
                                     nullptr, nullptr, rownnz.data(),
                                     rowadr.data(), colind.data(), n, data_ptr);
    ASSERT_EQ(nnz, ref_nnz);
    EXPECT_EQ(L_nnz, r_L_nnz);
    EXPECT_EQ(L_adr, r_L_adr);
    EXPECT_EQ(LT_nnz, r_LT_nnz);
    EXPECT_EQ(LT_adr, r_LT_adr);

    vector<int> L_col(nnz), LT_col(nnz), LT_map(nnz), L_sup(n);
    mju_cholFactorSymbolic(L_col.data(), L_nnz.data(), L_adr.data(),
                           LT_col.data(), LT_nnz.data(), LT_adr.data(),
                           LT_map.data(), L_sup.data(), rownnz.data(),
                           rowadr.data(), colind.data(), n, data_ptr);
    EXPECT_EQ(L_col, r_L_col);
    EXPECT_EQ(LT_col, r_LT_col);
    EXPECT_EQ(LT_map, r_LT_map);
    EXPECT_EQ(L_sup, r_L_sup);
  };

  // n = 1
  check_graph(1, {{0}}, /*upper_only=*/true, d.get());
  check_graph(1, {{}}, /*upper_only=*/true, nullptr);

  // mixed supernode blocks (1, 2, 3, 6) with partial lower-triangle coupling
  for (int trial = 0; trial < 40; trial++) {
    vector<int> bsizes;
    int n = 0;
    while (n < 36) {
      int choices[] = {1, 2, 3, 6};
      int bs = choices[rng() % 4];
      bsizes.push_back(bs);
      n += bs;
    }
    int nb = bsizes.size();
    vector<int> badr(nb + 1, 0);
    for (int b = 0; b < nb; b++) badr[b + 1] = badr[b] + bsizes[b];

    vector<vector<bool>> mat(n, vector<bool>(n, false));
    for (int b = 0; b < nb; b++) {
      // block diagonal clique (occasionally omit diagonal to test empty rows)
      if (trial % 7 != 0 || b != nb - 1) {
        for (int i = badr[b]; i < badr[b + 1]; i++) {
          for (int j = badr[b]; j < badr[b + 1]; j++) {
            mat[i][j] = true;
          }
        }
      }
      // block-level couplings
      for (int b2 = b + 1; b2 < nb; b2++) {
        if (rng() % 4 == 0) {
          for (int i = badr[b]; i < badr[b + 1]; i++) {
            for (int j = badr[b2]; j < badr[b2 + 1]; j++) {
              mat[i][j] = mat[j][i] = true;
            }
          }
        }
      }
    }
    // add sparse single-DOF partial couplings so rows sharing an upper pattern
    // can differ in lower triangle (exercises partial target-block hits)
    for (int k = 0; k < 6; k++) {
      int u = rng() % n, v = rng() % n;
      if (u != v) mat[u][v] = mat[v][u] = true;
    }
    vector<vector<int>> adj(n);
    for (int r = 0; r < n; r++) {
      for (int c = 0; c < n; c++) {
        if (mat[r][c]) adj[r].push_back(c);
      }
    }
    check_graph(n, adj, /*upper_only=*/(trial % 2 == 0),
                (trial % 5 == 0) ? nullptr : d.get());
  }
}

}  // namespace
}  // namespace mujoco
