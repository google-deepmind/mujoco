// Copyright 2024 DeepMind Technologies Limited
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

// Tests for user/user_util.cc

#include "src/user/user_util.h"

#include <cerrno>
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using user::FilePath;
using user::StringToVector;
using user::VectorToString;
using ::testing::ElementsAre;
using ::testing::IsNan;

using UserUtilTest = MujocoTest;

TEST_F(UserUtilTest, PathReduce) {
  FilePath path = FilePath("/hello/.././world/");
  EXPECT_EQ(path.Str(), "/world/");
}

TEST_F(UserUtilTest, PathReduce2) {
  FilePath path = FilePath("../hello/./world/");
  EXPECT_EQ(path.Str(), "../hello/world/");
}

TEST_F(UserUtilTest, PathReduce3) {
  FilePath path = FilePath("../../hello/world.txt");
  EXPECT_EQ(path.Str(), "../../hello/world.txt");
}

TEST_F(UserUtilTest, PathReduceWin) {
  FilePath path = FilePath("C:\\hello\\..\\world");
  EXPECT_EQ(path.Str(), "C:\\world");
}

TEST_F(UserUtilTest, IsAbs) {
  EXPECT_TRUE(FilePath("/hello").IsAbs());
  EXPECT_TRUE(FilePath("C:\\hello").IsAbs());
  EXPECT_FALSE(FilePath("hello").IsAbs());
}

TEST_F(UserUtilTest, Combine) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "/hello/world");
}

TEST_F(UserUtilTest, Combine2) {
  FilePath path1 = FilePath("hello/");
  FilePath path2 = FilePath("world");
  EXPECT_EQ((path1 + path2).Str(), "hello/world");
}

TEST_F(UserUtilTest, Combine3) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("../world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST_F(UserUtilTest, CombineAbs) {
  FilePath path1 = FilePath("/hello");
  FilePath path2 = FilePath("/world");
  EXPECT_EQ((path1 + path2).Str(), "/world");
}

TEST_F(UserUtilTest, Ext) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.Ext(), ".txt");
}

TEST_F(UserUtilTest, ExtEmpty) {
  FilePath path = FilePath("/hello/world");
  EXPECT_EQ(path.Ext(), "");
}

TEST_F(UserUtilTest, StripExt) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripExt().Str(), "/hello/world");
}

TEST_F(UserUtilTest, StripPath) {
  FilePath path = FilePath("/hello/world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST_F(UserUtilTest, StripPathEmpty) {
  FilePath path = FilePath("world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}

TEST_F(UserUtilTest, StripPathWin) {
  FilePath path = FilePath("\\world.txt");
  EXPECT_EQ(path.StripPath().Str(), "world.txt");
}


TEST_F(UserUtilTest, StrLower) {
  FilePath path = FilePath("/HELLO/worlD.txt");
  EXPECT_EQ(path.StrLower(), "/hello/world.txt");
}

TEST_F(UserUtilTest, StringToVectorFloat) {
  std::vector<float> v = StringToVector<float>(" 1.2 3.2     5.3 6 ");
  EXPECT_THAT(v, ElementsAre(1.2, 3.2, 5.3, 6));
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorEmpty) {
  std::vector<float> v = StringToVector<float>("   ");
  EXPECT_THAT(v, ElementsAre());
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorError) {
  std::vector<float> v = StringToVector<float>("2.1 3ABCD. /123/122/113");
  EXPECT_THAT(v, ElementsAre(2.1));
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UserUtilTest, StringToVectorInt) {
  std::vector<int> v = StringToVector<int>("  -1 3  5 6  ");
  EXPECT_THAT(v, ElementsAre(-1, 3, 5, 6));
  EXPECT_EQ(errno, 0);
}

TEST_F(UserUtilTest, StringToVectorString) {
  auto v = StringToVector<std::string>(" abc  def ");
  EXPECT_THAT(v, ElementsAre("abc", "def"));
}

TEST_F(UserUtilTest, StringToVectorInvalidNumber) {
  auto v = StringToVector<double>("1 0.1.2.3");
  EXPECT_THAT(v, ElementsAre(1));
  EXPECT_EQ(errno, EINVAL);
}

TEST_F(UserUtilTest, StringToVectorNan) {
  mju_user_warning = nullptr;
  auto v = StringToVector<double>("1 2 nan 3.21");
  EXPECT_THAT(v[2], IsNan());
  EXPECT_EQ(v[3], 3.21);
  EXPECT_EQ(errno, EDOM);
}

TEST_F(UserUtilTest, StringToVectorRange) {
  auto v = StringToVector<unsigned char>("-10");
  EXPECT_EQ(errno, ERANGE);
}

TEST_F(UserUtilTest, VectorToString) {
  std::vector<double> v = {1.2, 3.2, 5.3, 6};
  EXPECT_EQ(VectorToString(v), "1.2 3.2 5.3 6");
}

TEST_F(UserUtilTest, VectorToStringEmpty) {
  std::vector<double> v;
  EXPECT_EQ(VectorToString(v), "");
}

// utility: modified Gram-Schmidt to orthogonalize columns of Q (n x n)
static void gramSchmidt(double* Q, int n) {
  for (int j = 0; j < n; j++) {
    // subtract projections onto previous columns
    for (int k = 0; k < j; k++) {
      double dot = 0;
      for (int i = 0; i < n; i++) {
        dot += Q[i * n + j] * Q[i * n + k];
      }
      for (int i = 0; i < n; i++) {
        Q[i * n + j] -= dot * Q[i * n + k];
      }
    }
    // normalize
    double norm = 0;
    for (int i = 0; i < n; i++) {
      norm += Q[i * n + j] * Q[i * n + j];
    }
    norm = std::sqrt(norm);
    for (int i = 0; i < n; i++) {
      Q[i * n + j] /= norm;
    }
  }
}

// utility: compose SPD matrix A = Q * diag(eigvals) * Q^T
static void composeMatrix(double* A, const double* Q,
                          const double* eigvals, int n) {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j <= i; j++) {
      double sum = 0;
      for (int k = 0; k < n; k++) {
        sum += Q[i * n + k] * eigvals[k] * Q[j * n + k];
      }
      A[i * n + j] = sum;
      A[j * n + i] = sum;
    }
  }
}

TEST_F(UserUtilTest, EigendecomposeConvergence) {
  // seeded RNG for reproducibility
  std::mt19937_64 rng;
  rng.seed(42);
  std::normal_distribution<double> dist(0, 1);

  // sweep over matrix sizes used by flex stiffness
  //   order=1: 8 nodes * 3 dof = 24
  //   order=2: 27 nodes * 3 dof = 81
  for (int n : {24, 81}) {
    int total_sweeps = 0;
    int max_sweeps = 0;
    int count = 0;

    // generate random orthogonal matrix Q via Gram-Schmidt
    std::vector<double> Q(n * n);
    for (int i = 0; i < n * n; i++) {
      Q[i] = dist(rng);
    }
    gramSchmidt(Q.data(), n);

    // sweep eigenvalue spectra of varying difficulty
    //   well-separated, clustered, wide condition number
    for (double condition : {1e1, 1e3, 1e6}) {
      for (double cluster : {0.0, 0.5, 0.9}) {
        // construct eigenvalues
        std::vector<double> eigvals(n);
        for (int i = 0; i < n; i++) {
          // base: logarithmically spaced from 1 to condition
          double t = (double)i / (n - 1);
          double base = std::exp(t * std::log(condition));

          // cluster: push eigenvalues toward geometric mean
          double mean = std::sqrt(condition);
          eigvals[i] = (1 - cluster) * base + cluster * mean;
        }

        // compose A = Q * diag(eigvals) * Q^T
        std::vector<double> A(n * n);
        composeMatrix(A.data(), Q.data(), eigvals.data(), n);

        // save copy for verification
        std::vector<double> A_copy(A);

        // decompose
        std::vector<double> found_eigval(n);
        std::vector<double> found_eigvec(n * n);
        int sweeps = mjuu_eigendecompose(
            A.data(), found_eigval.data(),
            found_eigvec.data(), n);

        total_sweeps += sweeps;
        if (sweeps > max_sweeps) max_sweeps = sweeps;
        count++;

        // verify convergence
        EXPECT_LT(sweeps, 200)
            << "n=" << n
            << " condition=" << condition
            << " cluster=" << cluster;

        // verify A*v = lambda*v for each eigenpair
        for (int i = 0; i < n; i++) {
          for (int r = 0; r < n; r++) {
            double Av = 0;
            for (int c = 0; c < n; c++) {
              Av += A_copy[r * n + c] * found_eigvec[c * n + i];
            }
            double lv = found_eigval[i] * found_eigvec[r * n + i];
            EXPECT_NEAR(Av, lv,
                        1e-6 * std::abs(found_eigval[i]))
                << "n=" << n << " condition=" << condition
                << " cluster=" << cluster
                << " eigpair=" << i << " row=" << r;
          }
        }

        // verify all eigenvalues are positive
        for (int i = 0; i < n; i++) {
          EXPECT_GT(found_eigval[i], 0)
              << "n=" << n << " eigenvalue " << i;
        }
      }
    }

    double mean_sweeps = (double)total_sweeps / count;

    // assert reasonable average convergence
    EXPECT_LE(mean_sweeps, 20.0)
        << "n=" << n << ": mean sweeps too high";

    // assert max sweeps within budget
    EXPECT_LT(max_sweeps, 200)
        << "n=" << n << ": max sweeps exceeded 200";
  }
}

}  // namespace
}  // namespace mujoco
