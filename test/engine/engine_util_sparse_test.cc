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

// Tests for engine/engine_util_sparse.c

#include <array>
#include <vector>

#include "src/engine/engine_util_sparse.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using EngineUtilSparseTest = MujocoTest;

template <typename T>
std::vector<T> AsVector(const T* array, int n) {
  return std::vector<T>(array, array + n);
}

TEST_F(EngineUtilSparseTest, MjuDot) {
  mjtNum a[] = {2,    3,       4,          5,          6,       7,    8};
  mjtNum u[] = {2, 1, 3, 1, 1, 4, 1, 1, 1, 5, 1, 1, 1, 6, 1, 1, 7, 1, 8};
  mjtNum b[] = {8, 1, 7, 1, 1, 6, 1, 1, 1, 5, 1, 1, 1, 4, 1, 1, 3, 1, 2};
  int i[] = {0, 2, 5, 9, 13, 16, 18};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four

  // a is compressed
  int flg_unc1 = 0;
  EXPECT_EQ(mju_dotSparse(a, b, 0, i, flg_unc1), 0);
  EXPECT_EQ(mju_dotSparse(a, b, 1, i, flg_unc1), 2*8);
  EXPECT_EQ(mju_dotSparse(a, b, 2, i, flg_unc1), 2*8 + 3*7);
  EXPECT_EQ(mju_dotSparse(a, b, 3, i, flg_unc1), 2*8 + 3*7 + 4*6);
  EXPECT_EQ(mju_dotSparse(a, b, 4, i, flg_unc1), 2*8 + 3*7 + 4*6 + 5*5);
  EXPECT_EQ(mju_dotSparse(a, b, 5, i, flg_unc1), 2*8 + 3*7 + 4*6 + 5*5 + 6*4);
  EXPECT_EQ(mju_dotSparse(a, b, 6, i, flg_unc1),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3);
  EXPECT_EQ(mju_dotSparse(a, b, 7, i, flg_unc1),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3 + 8*2);

  // u is compressed
  flg_unc1 = 1;
  EXPECT_EQ(mju_dotSparse(u, b, 0, i, flg_unc1), 0);
  EXPECT_EQ(mju_dotSparse(u, b, 1, i, flg_unc1), 2*8);
  EXPECT_EQ(mju_dotSparse(u, b, 2, i, flg_unc1), 2*8 + 3*7);
  EXPECT_EQ(mju_dotSparse(u, b, 3, i, flg_unc1), 2*8 + 3*7 + 4*6);
  EXPECT_EQ(mju_dotSparse(u, b, 4, i, flg_unc1), 2*8 + 3*7 + 4*6 + 5*5);
  EXPECT_EQ(mju_dotSparse(u, b, 5, i, flg_unc1), 2*8 + 3*7 + 4*6 + 5*5 + 6*4);
  EXPECT_EQ(mju_dotSparse(u, b, 6, i, flg_unc1),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3);
  EXPECT_EQ(mju_dotSparse(u, b, 7, i, flg_unc1),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3 + 8*2);
}

TEST_F(EngineUtilSparseTest, MjuDot2) {
  constexpr int annz = 6;
  constexpr int bnnz = 5;
  int ia[annz]   = {0,    2,       5, 6, 7};
  mjtNum a[annz] = {2,    3,       4, 5, 6};
  int ib[bnnz]   = {   1, 2, 3,    5,    7};
  mjtNum b[bnnz] = {   8, 7, 6,    5,    4};
  mjtNum u[]     = {1, 8, 7, 6, 1, 5, 1, 4};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four

  // a is compressed
  int flg_unc2 = 0;
  EXPECT_EQ(mju_dotSparse2(a, b, annz, ia, bnnz, ib, flg_unc2), 3*7+4*5+6*4);

  // u is uncompressed
  flg_unc2 = 1;
  EXPECT_EQ(mju_dotSparse2(a, u, annz, ia, bnnz, ib, flg_unc2), 3*7+4*5+6*4);
}

TEST_F(EngineUtilSparseTest, CombineSparseCount) {
  {
    std::array a_ind{0, 1};
    std::array b_ind{2};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 3);
  }

  {
    std::array a_ind{2};
    std::array b_ind{0, 1};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 3);
  }

  {
    std::array a_ind{0, 1};
    std::array b_ind{2, 3, 4};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 5);
  }

  {
    std::array a_ind{5, 6};
    std::array b_ind{1, 3, 8};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 5);
  }

  {
    std::array a_ind{1, 2, 3};
    std::array b_ind{0, 4};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 5);
  }

  {
    std::array a_ind{1, 4};
    std::array b_ind{2, 3};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 4);
  }

  {
    std::array a_ind{0, 1, 3};
    std::array b_ind{0, 3, 4};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 4);
  }

  {
    std::array a_ind{1, 3, 5, 6};
    std::array b_ind{1, 3, 5, 6};
    EXPECT_EQ(mju_combineSparseCount(
        a_ind.size(), b_ind.size(), a_ind.data(), b_ind.data()), 4);
  }

  EXPECT_EQ(mju_combineSparseCount(0, 0, nullptr, nullptr), 0);

  {
    std::array b_ind{1, 2};
    EXPECT_EQ(
        mju_combineSparseCount(0, b_ind.size(), nullptr, b_ind.data()), 2);
  }

  {
    std::array a_ind{0};
    EXPECT_EQ(
        mju_combineSparseCount(a_ind.size(), 0, a_ind.data(), nullptr), 1);
  }
}

TEST_F(EngineUtilSparseTest, MjuTranspose3by3) {
  // 1 2 0      1 0 0
  // 0 1 0 -->  2 1 3
  // 0 3 0      0 0 0

  mjtNum mat[] = {1, 2, 1, 3};
  int colind[] = {0, 1, 1, 1};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {0, 0, 0, 0};
  int colindT[] = {0, 0, 0, 0};
  int rownnzT[] = {0, 0, 0};
  int rowadrT[] = {0, 0, 0};

  mju_transposeSparse(matT, mat, 3, 3, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1, 2, 1, 3));
  EXPECT_THAT(colindT, ElementsAre(0, 0, 1, 2));
  EXPECT_THAT(rownnzT, ElementsAre(1, 3, 0));
  EXPECT_THAT(rowadrT, ElementsAre(0, 1, 4));
}

TEST_F(EngineUtilSparseTest, MjuTranspose1by3) {
  // 1 0 3      1
  //       -->  0
  //            3

  mjtNum mat[] = {1, 3};
  int colind[] = {0, 2};
  int rownnz[] = {2};
  int rowadr[] = {0};

  mjtNum matT[] = {0, 0};
  int colindT[] = {0, 0};
  int rownnzT[] = {0, 0, 0};
  int rowadrT[] = {0, 0, 0};

  mju_transposeSparse(matT, mat, 1, 3, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1, 3));
  EXPECT_THAT(colindT, ElementsAre(0, 0));
  EXPECT_THAT(rownnzT, ElementsAre(1, 0, 1));
  EXPECT_THAT(rowadrT, ElementsAre(0, 1, 1));
}

TEST_F(EngineUtilSparseTest, MjuTranspose3by1) {
  // 1       1 0 3
  // 0  -->
  // 3

  mjtNum mat[] = {1, 3};
  int colind[] = {0, 0};
  int rownnz[] = {1, 0, 1};
  int rowadr[] = {0, 1, 1};

  mjtNum matT[] = {0, 0};
  int colindT[] = {0, 0};
  int rownnzT[] = {0};
  int rowadrT[] = {0};

  mju_transposeSparse(matT, mat, 3, 1, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1, 3));
  EXPECT_THAT(colindT, ElementsAre(0, 2));
  EXPECT_THAT(rownnzT, ElementsAre(2));
  EXPECT_THAT(rowadrT, ElementsAre(0));
}

TEST_F(EngineUtilSparseTest, MjuTransposeDense) {
  // 1 2 3      1 4 7
  // 4 5 6 -->  2 5 8
  // 7 8 9      3 6 9

  mjtNum mat[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzT[] = {0, 0, 0};
  int rowadrT[] = {0, 0, 0};

  mju_transposeSparse(matT, mat, 3, 3, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1, 4, 7, 2, 5, 8, 3, 6, 9));
  EXPECT_THAT(colindT, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzT, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrT, ElementsAre(0, 3, 6));
}

TEST_F(EngineUtilSparseTest, MjuTranspose1by1) {
  // 1 -> 1

  mjtNum mat[] = {1};
  int colind[] = {0};
  int rownnz[] = {1};
  int rowadr[] = {0};

  mjtNum matT[] = {0};
  int colindT[] = {0};
  int rownnzT[] = {0};
  int rowadrT[] = {0};

  mju_transposeSparse(matT, mat, 1, 1, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1));
  EXPECT_THAT(colindT, ElementsAre(0));
  EXPECT_THAT(rownnzT, ElementsAre(1));
  EXPECT_THAT(rowadrT, ElementsAre(0));
}

TEST_F(EngineUtilSparseTest, MjuTransposeNullMatrix) {
  // 0 -> 0

  mjtNum mat[] = {};
  int colind[] = {};
  int rownnz[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowadr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  mjtNum matT[] = {};
  int colindT[] = {};
  int rownnzT[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  int rowadrT[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  mju_transposeSparse(matT, mat, 10, 10, rownnzT, rowadrT, colindT, rownnz,
                      rowadr, colind);

  EXPECT_THAT(rownnzT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(rowadrT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
}

static constexpr char modelStr[] = R"(<mujoco/>)";

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse1) {
  //       0 0 0
  // M  =  0 0 0
  //       0 0 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse2) {
  //     2 -1  1
  // M = 1  2 -1
  //     2  2  3

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {2, -1, 1, 2, -1, 2, 2, 2, 3};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {2, 2, 2, -1, -1, 2, 1, 2, 3};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));


  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(12, 0, 12, 0, 6, 3, 12, 3, 14));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse3) {
  //     1 2 0
  // M = 0 3 0
  //     4 0 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 1, 1, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 2, 0};
  int rowadrT[] = {0, 2, 4};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 4));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(66, 4, 0, 4, 35, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 0, 1, 0, 0, 0, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse4) {
  //     1 0 2
  // M = 0 0 3
  //     4 0 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 2, 2, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 0, 2};
  int rowadrT[] = {0, 2, 2};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};


  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(2, 0, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 2));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(66, 4, 0, 0, 0, 0, 4, 35, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 2, 0, 0, 0, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 0, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse5) {
  //     1 0 4
  // M = 0 0 0
  //     2 3 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 4, 2, 3};
  int colind[] = {0, 2, 0, 1};
  int rownnz[] = {2, 0, 2};
  int rowadr[] = {0, 2, 2};

  mjtNum matT[] = {1, 2, 3, 4};
  int colindT[] = {0, 2, 2, 0};
  int rownnzT[] = {2, 1, 1};
  int rowadrT[] = {0, 2, 3};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};


  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 5));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(5, 6, 4, 6, 9, 0, 4, 16, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse6) {
  //     1 0 2
  // M = 0 2 0
  //     0 0 3

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 2, 3};
  int colind[] = {0, 2, 1, 2};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 2, 2, 3};
  int colindT[] = {0, 1, 0, 2};
  int rownnzT[] = {1, 1, 2};
  int rowadrT[] = {0, 1, 2};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(2, 1, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 3));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(1, 2, 0, 4, 0, 0, 2, 13, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 2, 0, 1, 0, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 1, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse7) {
  //     1 2
  // M = 0 3
  //     4 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 1, 1, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 2};
  int rowadrT[] = {0, 2};

  mjtNum matH[] = {0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0};
  int rownnzH[] = {0, 0};
  int rowadrH[] = {0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 2, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 2);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 2, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(66, 4, 4, 35));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 1));
  EXPECT_THAT(rownnzH, ElementsAre(2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse8) {
  // M = 1 0 4
  //     2 3 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 4, 2, 3};
  int colind[] = {0, 2, 0, 1};
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 2};

  mjtNum matT[] = {1, 2, 3, 4};
  int colindT[] = {0, 1, 1, 0};
  int rownnzT[] = {2, 1, 1};
  int rowadrT[] = {0, 2, 3};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 5));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 2, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(14, 18, 8, 18, 27, 0, 8, 32, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse9) {
  //     1 2 2
  // M = 1 3 4
  //     4 4 4

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 2, 1, 3, 4, 4, 4, 4};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {1, 1, 4, 2, 3, 4, 2, 4, 4};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, nullptr, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data);

  EXPECT_THAT(matH, ElementsAre(69, 77, 80, 77, 99, 108, 80, 108, 120));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse10) {
  //     1 1 1
  // M = 2 2 2
  //     3 3 3

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 1, 1, 2, 2, 2, 3, 3, 3};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {1, 2, 3, 1, 2, 3, 1, 2, 3};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};
  int rowsuperT[] = {2, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, rowsuperT, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data);

  EXPECT_THAT(matH, ElementsAre(14, 14, 14, 14, 14, 14, 14, 14, 14));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse11) {
  //     1 1 1
  // M = 0 0 0
  //     0 3 3

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 1, 1, 3, 3};
  int colind[] = {0, 1, 2, 1, 2};
  int rownnz[] = {3, 0, 2};
  int rowadr[] = {0, 3, 3};

  mjtNum matT[] = {1, 1, 3, 1, 3};
  int colindT[] = {0, 0, 2, 0, 2};
  int rownnzT[] = {1, 2, 2};
  int rowadrT[] = {0, 1, 3};
  int rowsuperT[] = {0, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, rowsuperT, data);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data);

  EXPECT_THAT(matH, ElementsAre(1, 1, 1, 1, 10, 10, 1, 10, 10));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse12) {
  //     1 1 1 1
  // M = 0 0 0 0
  //     0 0 3 3

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 3, 3};
  int colind[] = {0, 1, 2, 3, 2, 3};
  int rownnz[] = {4, 0, 2};
  int rowadr[] = {0, 4, 4};

  mjtNum matT[] = {1, 1, 1, 3, 1, 3};
  int colindT[] = {0, 0, 0, 2, 0, 2};
  int rownnzT[] = {1, 1, 2, 2};
  int rowadrT[] = {0, 1, 2, 4};
  int rowsuperT[] = {1, 0, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0, 0};
  int rowadrH[] = {0, 0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 4, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, rowsuperT, data);

  EXPECT_THAT(rownnzH, ElementsAre(4, 4, 4, 4));
  EXPECT_THAT(rowadrH, ElementsAre(0, 4, 8, 12));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 4);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 4, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data);

  EXPECT_THAT(matH,
              ElementsAre(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10, 10, 1, 1, 10, 10));
  EXPECT_THAT(colindH,
              ElementsAre(0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3));
  EXPECT_THAT(rownnzH, ElementsAre(4, 4, 4, 4));
  EXPECT_THAT(rowadrH, ElementsAre(0, 4, 8, 12));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse13) {
  //     1 1 0 0 0
  // M = 1 1 0 0 0
  //     1 1 0 0 0

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 1, 1};
  int colind[] = {0, 1, 0, 1, 0, 1};
  int rownnz[] = {2, 2, 2};
  int rowadr[] = {0, 2, 4};

  mjtNum matT[] = {1, 1, 1, 1, 1, 1};
  int colindT[] = {0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 0, 0, 0};
  int rowadrT[] = {0, 3, 6, 6, 6};
  int rowsuperT[] = {1, 0, 2, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0, 0, 0};
  int rowadrH[] = {0, 0, 0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 5, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, rowsuperT, data);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0, 0, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 4, 4, 4));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 5);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 5, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data);

  EXPECT_THAT(matH, ElementsAre(3, 3, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0, 0, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 5, 10, 15, 20));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse14) {
  // M = 1 1 1 1 2 2 2

  mjModel* model = LoadModelFromString(modelStr);
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 2, 2, 2};
  int colind[] = {0, 1, 2, 3, 4, 5, 6};
  int rownnz[] = {7};
  int rowadr[] = {0};

  mjtNum matT[] = {1, 1, 1, 1, 2, 2, 2};
  int colindT[] = {0, 0, 0, 0, 0, 0, 0};
  int rownnzT[] = {1, 1, 1, 1, 1, 1, 1};
  int rowadrT[] = {0, 1, 2, 3, 4, 5, 6};
  int rowsuperT[] = {3, 2, 1, 0, 2, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0, 0, 0, 0, 0};
  int rowadrH[] = {0, 0, 0, 0, 0, 0, 0};

  // test precount
  mju_sqrMatTDSparseInit(rownnzH, rowadrH, 7, rownnz, rowadr, colind,
                         rownnzT, rowadrT, colindT, rowsuperT, data);

  EXPECT_THAT(rownnzH, ElementsAre(7, 7, 7, 7, 7, 7, 7));
  EXPECT_THAT(rowadrH, ElementsAre(0, 7, 14, 21, 28, 35, 42));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 7);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 1, 7, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data);

  EXPECT_THAT(
      matH, ElementsAre(1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 2,
                        2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2, 2, 2,
                        2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4));
  EXPECT_THAT(colindH,
              ElementsAre(0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3,
                          4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0,
                          1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6));

  EXPECT_THAT(rownnzH, ElementsAre(7, 7, 7, 7, 7, 7, 7));
  EXPECT_THAT(rowadrH, ElementsAre(0, 7, 14, 21, 28, 35, 42));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuCholFactorNNZ) {
  mjModel* model = LoadModelFromString(modelStr);
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
  int nnzA = mju_cholFactorNNZ(rownnzA_factor,
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
  int nnzB = mju_cholFactorNNZ(rownnzB_factor,
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
  int nnzC = mju_cholFactorNNZ(rownnzC_factor,
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
  int nnzD = mju_cholFactorNNZ(rownnzD_factor,
                               rownnzD, rowadrD, colindD, nD, d);

  EXPECT_EQ(nnzD, 8);
  EXPECT_THAT(AsVector(rownnzD_factor, 4), ElementsAre(1, 2, 2, 3));

  mj_deleteData(d);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuMulMatTVec) {
  int nr = 2;
  int nc = 3;
  mjtNum mat[] = {1, 2, 0,
                  0, 3, 4};

  mjtNum mat_sparse[4];
  int rownnz[2];
  int rowadr[2];
  int colind[4];
  mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, 4);

  // multiply: res = mat' * vec
  mjtNum vec[] = {5, 6};
  mjtNum res[3];
  mju_mulMatTVecSparse(res, mat_sparse, vec, nr, nc, rownnz, rowadr, colind);

  EXPECT_THAT(AsVector(res, 3), ElementsAre(5, 28, 24));
}

TEST_F(EngineUtilSparseTest, MjuDenseToSparse) {
  int nr = 2;
  int nc = 2;
  mjtNum mat[] = {1, 2,
                  0, 3};

  mjtNum mat_sparse[4];
  int rownnz[2];
  int rowadr[2];
  int colind[4];

  // nnz == number of non-zeros
  int status3 =
      mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, 3);

  EXPECT_EQ(status3, 0);

  // nnz > number of non-zeros
  int status4 =
      mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, 4);

  EXPECT_EQ(status4, 0);

  // nnz < number of non-zeros
  int status2 =
      mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, 2);

  EXPECT_EQ(status2, 1);

  // nnz == 0
  int status0 =
      mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, 0);

  EXPECT_EQ(status0, 1);
}

}  // namespace
}  // namespace mujoco
