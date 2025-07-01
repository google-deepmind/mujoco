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

#include "src/engine/engine_util_sparse.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// permute the rows and columns of a dense matrix
inline void PermuteMat(mjtNum* res, const mjtNum* mat, int nr, int nc,
                       const int* perm_r, const int* perm_c,
                       bool scatter_r, bool scatter_c) {
  for (int r = 0; r < nr; r++) {
    for (int c = 0; c < nc; c++) {
      if (scatter_r && scatter_c) {
        // scatter both
        res[perm_r[r] * nc + perm_c[c]] = mat[r * nc + c];
      } else if (scatter_r && !scatter_c) {
        // scatter rows, gather columns
        res[perm_r[r] * nc + c] = mat[r * nc + perm_c[c]];
      } else if (!scatter_r && scatter_c) {
        // gather rows, scatter columns
        res[r * nc + perm_c[c]] = mat[perm_r[r] * nc + c];
      } else {
        // gather both
        res[r * nc + c] = mat[perm_r[r] * nc + perm_c[c]];
      }
    }
  }
}

using ::testing::ElementsAre;
using EngineUtilSparseTest = MujocoTest;

TEST_F(EngineUtilSparseTest, MjuDot) {
  mjtNum a[] = {2,    3,       4,          5,          6,       7,    8};
  mjtNum b[] = {8, 1, 7, 1, 1, 6, 1, 1, 1, 5, 1, 1, 1, 4, 1, 1, 3, 1, 2};
  int i[] = {0, 2, 5, 9, 13, 16, 18};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four

  EXPECT_EQ(mju_dotSparse(a, b, 0, i), 0);
  EXPECT_EQ(mju_dotSparse(a, b, 1, i), 2*8);
  EXPECT_EQ(mju_dotSparse(a, b, 2, i), 2*8 + 3*7);
  EXPECT_EQ(mju_dotSparse(a, b, 3, i), 2*8 + 3*7 + 4*6);
  EXPECT_EQ(mju_dotSparse(a, b, 4, i), 2*8 + 3*7 + 4*6 + 5*5);
  EXPECT_EQ(mju_dotSparse(a, b, 5, i), 2*8 + 3*7 + 4*6 + 5*5 + 6*4);
  EXPECT_EQ(mju_dotSparse(a, b, 6, i),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3);
  EXPECT_EQ(mju_dotSparse(a, b, 7, i),
            2*8 + 3*7 + 4*6 + 5*5 + 6*4 + 7*3 + 8*2);
}

TEST_F(EngineUtilSparseTest, MjuDot2) {
  constexpr int annz = 6;
  constexpr int bnnz = 5;

  // values
  mjtNum a[annz] = {2,    3,       4, 5, 6};
  mjtNum b[bnnz] = {   8, 7, 6,    5,    4};

  // indices
  int ia[annz]   = {0,    2,       5, 6, 7};
  int ib[bnnz]   = {   1, 2, 3,    5,    7};

  EXPECT_EQ(mju_dotSparse2(a, ia, annz, b, ib, bnnz), 3 * 7 + 4 * 5 + 6 * 4);
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

  mju_transposeSparse(matT, mat, 3, 3, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind);

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

  mju_transposeSparse(matT, mat, 1, 3, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind);

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

  mju_transposeSparse(matT, mat, 3, 1, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind);

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
  int rowsuperT[] = {0, 0, 0};

  mju_transposeSparse(matT, mat, 3, 3, rownnzT, rowadrT, colindT, rowsuperT,
                      rownnz, rowadr, colind);

  EXPECT_THAT(matT, ElementsAre(1, 4, 7, 2, 5, 8, 3, 6, 9));
  EXPECT_THAT(colindT, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzT, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrT, ElementsAre(0, 3, 6));
  EXPECT_THAT(rowsuperT, ElementsAre(2, 1, 0));
}

TEST_F(EngineUtilSparseTest, MjuTransposeSuper) {
  //  mat:   0,  1,  2,  0,  3,  4,  5,  0,  0,  0,  0,  0,  0
  //         0,  0,  0,  6,  7,  8,  9, 10, 11, 12,  0,  0,  0
  //
  //  super: 0,  1,  0,  0,  2,  1,  0,  2,  1,  0,  2,  1,  0

  mjtNum mat[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  int colind[] = {1, 2, 4, 5, 6, 3, 4, 5, 6, 7,  8,  9};
  int rownnz[] = {5, 7};
  int rowadr[] = {0, 5};

  mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowadrT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowsuperT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  mju_transposeSparse(matT, mat, 2, 13, rownnzT, rowadrT, colindT, rowsuperT,
                      rownnz, rowadr, colind);

  EXPECT_THAT(matT,    ElementsAre(1, 2, 6, 3, 7, 4, 8, 5, 9, 10, 11, 12));
  EXPECT_THAT(colindT, ElementsAre(0, 0, 1, 0, 1, 0, 1, 0, 1, 1,  1,  1));
  EXPECT_THAT(rownnzT, ElementsAre(0, 1, 1, 1, 2, 2, 2, 1, 1,  1,  0,  0,  0));
  EXPECT_THAT(rowadrT, ElementsAre(0, 0, 1, 2, 3, 5, 7, 9, 10, 11, 12, 12, 12));
  EXPECT_THAT(rowsuperT, ElementsAre(0, 1, 0, 0, 2, 1, 0, 2, 1, 0, 2,  1,  0));
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

  mju_transposeSparse(matT, mat, 1, 1, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind);

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

  mju_transposeSparse(matT, mat, 10, 10, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind);

  EXPECT_THAT(rownnzT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(rowadrT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, MjuCompressSparse) {
  // sparse matrix (uncompressed with spurious values between the rows):
  // [[1, 0, 2]
  //  [0, O, 3]  (second zero represented)
  mjtNum mat[] = {1, 2,  9, 0, 3};  // spurious 9 value
  int colind[] = {0, 2, -1, 1, 2};  // spurious -1 index
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 3};

  mjtNum dense_expected[] = {1, 0, 2, 0, 0, 3};
  mjtNum dense[6];
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that spurious values are removed
  int nnz  = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind,
                                /*minval=*/-1);
  EXPECT_EQ(nnz, 4);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that represented zero gets compressed aways with minval=0
  nnz  = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind, /*minval=*/0);
  EXPECT_EQ(nnz, 3);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that 1 gets compressed aways with minval=1
  nnz  = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind, /*minval=*/1);
  EXPECT_EQ(nnz, 2);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  mjtNum dense_expected_minval1[] = {0, 0, 2, 0, 0, 3};
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected_minval1, 6));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse1) {
  //       0 0 0
  // M  =  0 0 0
  //       0 0 0

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseLower) {
  //     2 -1  1
  // M = 2 -1  2
  //     2  2  3

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 0);
  EXPECT_THAT(rownnzH, ElementsAre(1, 2, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 1, 3));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, nullptr);

  EXPECT_THAT(matH, ElementsAre(12, 0, 0, 0, 6, 0, 12, 3, 14));
  EXPECT_THAT(colindH, ElementsAre(0, 0, 0, 0, 1, 0, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(1, 2, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse2) {
  //     2 -1  1
  // M = 2 -1  2
  //     2  2  3

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));


  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(12, 0, 12, 0, 6, 3, 12, 3, 14));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));
  EXPECT_THAT(diagindH, ElementsAre(0, 4, 8));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse3) {
  //     1 2 0
  // M = 0 3 0
  //     4 0 0

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 4));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(66, 4, 0, 4, 35, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 0, 1, 0, 2, 0, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 1));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse3b) {
  //     1 2 0
  // M = 0 3 4
  //     5 0 0

  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 3, 4, 5};
  int colind[] = {0, 1, 1, 2, 0};
  int rownnz[] = {2, 2, 1};
  int rowadr[] = {0, 2, 4};

  mjtNum matT[] = {1, 5, 2, 3, 4};
  int colindT[] = {0, 2, 0, 1, 1};
  int rownnzT[] = {2, 2, 1};
  int rowadrT[] = {0, 2, 4};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 3, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 5));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(26, 2, 0, 2, 13, 12, 12, 16, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 0, 1, 2, 1, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 3, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));
  EXPECT_THAT(diagindH, ElementsAre(0, 4, 7));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse4) {
  //     1 0 2
  // M = 0 0 3
  //     4 0 0

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};


  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 0, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 2));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(66, 4, 0, 0, 0, 0, 4, 35, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 2, 0, 1, 0, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 1, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse5) {
  //     1 0 4
  // M = 0 0 0
  //     2 3 0

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};


  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 5));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 1, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 3));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(1, 2, 0, 4, 0, 0, 2, 13, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 2, 0, 1, 0, 0, 0, 2, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 1, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));
  EXPECT_THAT(diagindH, ElementsAre(0, 3, 7));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse7) {
  //     1 2
  // M = 0 3
  //     4 0

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 2, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 2);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 2, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 2, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 5));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 2, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {2, 3, 4};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, nullptr, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     nullptr, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(69, 77, 80, 77, 99, 108, 80, 108, 120));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse10) {
  //     1 2 3
  // M = 2 3 2
  //     3 1 1

  mjModel* model = LoadModelFromString("<mujoco/>");
  mjData* data = mj_makeData(model);

  mjtNum mat[] = {1, 2, 3, 2, 3, 2, 3, 1, 1};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {1, 2, 3, 2, 3, 1, 3, 2, 1};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};
  int rowsuperT[] = {2, 1, 0};

  mjtNum matH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindH[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {1, 2, 1};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, rowsuperT, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(18, 17, 14, 17, 23, 19, 14, 19, 18));
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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 3, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, rowsuperT, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(3, 3, 3));
  EXPECT_THAT(rowadrH, ElementsAre(0, 3, 6));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 3);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 3, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data, diagindH);

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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 4, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, rowsuperT, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(4, 4, 4, 4));
  EXPECT_THAT(rowadrH, ElementsAre(0, 4, 8, 12));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 4);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 4, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data, diagindH);

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

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0, 0, 0};

  mjtNum diag[] = {1, 1, 1};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 5, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, rowsuperT, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 0, 0, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 4, 4, 4));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 5);
  mju_sqrMatTDSparse(matH, mat, matT, diag, 3, 5, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data, diagindH);

  EXPECT_THAT(matH, ElementsAre(3, 3, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0));
  EXPECT_THAT(colindH, ElementsAre(0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 0,
                                   3, 0, 0, 0, 0, 4, 0, 0, 0, 0));
  EXPECT_THAT(rownnzH, ElementsAre(2, 2, 1, 1, 1));
  EXPECT_THAT(rowadrH, ElementsAre(0, 5, 10, 15, 20));

  mj_deleteData(data);
  mj_deleteModel(model);
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse14) {
  // M = 1 1 1 1 2 2 2

  mjModel* model = LoadModelFromString("<mujoco/>");
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
  int diagindH[] = {0, 0, 0, 0, 0, 0, 0};

  // test precount
  mju_sqrMatTDSparseCount(rownnzH, rowadrH, 7, rownnz, rowadr, colind,
                          rownnzT, rowadrT, colindT, rowsuperT, data, 1);

  EXPECT_THAT(rownnzH, ElementsAre(7, 7, 7, 7, 7, 7, 7));
  EXPECT_THAT(rowadrH, ElementsAre(0, 7, 14, 21, 28, 35, 42));

  // test computation
  mju_sqrMatTDUncompressedInit(rowadrH, 7);
  mju_sqrMatTDSparse(matH, mat, matT, nullptr, 1, 7, rownnzH, rowadrH, colindH,
                     rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT,
                     rowsuperT, data, diagindH);

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

TEST_F(EngineUtilSparseTest, MjuAddToSymSparse) {
  //     1 2 4
  // M = 2 3 0
  //     4 0 5

  // only lower triangle represented
  mjtNum mat[] = {1, 2, 3, 4, 5};
  int colind[] = {0, 0, 1, 0, 2};
  int rownnz[] = {1, 2, 2};
  int rowadr[] = {0, 1, 3};

  //     0 0 0
  // A = 5 4 2
  //     4 3 2
  mjtNum A[] = {0, 0, 0,
                5, 4, 2,
                4, 3, 2};

  mju_addToSymSparse(A, mat, 3, rownnz, rowadr, colind, /*flg_upper=*/1);
  EXPECT_THAT(AsVector(A, 9), ElementsAre(1, 2, 4,
                                          7, 7, 2,
                                          8, 3, 7));

  // same as A
  mjtNum B[] = {0, 0, 0,
                5, 4, 2,
                4, 3, 2};

  mju_addToSymSparse(B, mat, 3, rownnz, rowadr, colind, /*flg_upper=*/0);
  EXPECT_THAT(AsVector(B, 9), ElementsAre(1, 0, 0,
                                          7, 7, 2,
                                          8, 3, 7));
}

TEST_F(EngineUtilSparseTest, MjuMulSymVecSparse) {
  constexpr int n = 4;
  constexpr int nnz = 9;

  mjtNum mat[n*n] = {1,  0,  0,  0,
                     0,  2,  0,  0,
                     3,  0,  4,  0,
                     5,  6,  7,  8};

  // dense, full matrix
  mjtNum sym[n*n] = {1,  0,  3,  5,
                     0,  2,  0,  6,
                     3,  0,  4,  7,
                     5,  6,  7,  8};

  mjtNum mat_sparse[nnz];
  int rownnz[n];
  int rowadr[n];
  int colind[nnz];
  mju_dense2sparse(mat_sparse, mat, n, n, rownnz, rowadr, colind, nnz);

  // multiply: res = (mat + strict_upper(mat')) * vec
  mjtNum vec[n] = {4, 3, 2, 1};
  mjtNum res[n];
  mju_mulSymVecSparse(res, mat_sparse, vec, n, rownnz, rowadr, colind);

  // dense multiply
  mjtNum res2[n];
  mju_mulMatVec(res2, sym, vec, n, n);

  for (int i=0; i < n; i++) {
    EXPECT_EQ(res[i], res2[i]);
  }
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

TEST_F(EngineUtilSparseTest, MergeSorted) {
  const int chain1_a[] = {1, 2, 3};
  const int chain2_a[] = {};
  int merged_a[3];
  int n1 = 3;
  int n2 = 0;
  EXPECT_EQ(mj_mergeSorted(merged_a, chain1_a, n1, chain2_a, n2), 3);
  EXPECT_THAT(merged_a, ElementsAre(1, 2, 3));

  const int chain1_b[] = {1, 3, 5, 7, 8};
  const int chain2_b[] = {2, 4, 5, 6, 8};
  int merged_b[8];
  n1 = 5;
  n2 = 5;
  EXPECT_EQ(mj_mergeSorted(merged_b, chain1_b, n1, chain2_b, n2), 8);
  EXPECT_THAT(merged_b, ElementsAre(1, 2, 3, 4, 5, 6, 7, 8));
}

TEST_F(EngineUtilSparseTest, BlockDiag) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {1, 2, 0, 0, 0,
                               0, 0, 3, 4, 0,
                               0, 0, 5, 6, 0,
                               0, 0, 0, 0, 7};

  // block structure
  constexpr int nb = 3;
  const int block_nr[nb] = {1, 2, 1};
  const int block_nc[nb] = {2, 2, 1};
  const int block_r[nb] = {0, 1, 3};
  const int block_c[nb] = {0, 2, 4};

  // test with identity permutations
  const int perm_r[nr] = {0, 1, 2, 3};
  const int perm_c[nc] = {0, 1, 2, 3, 4};
  mjtNum res[nr * nc] = {0};
  mju_blockDiag(res, mat, nc, nc, nb,
                perm_r, perm_c,
                block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res, ElementsAre(1, 2, 0, 0, 0,
                               3, 4, 5, 6, 0,
                               0, 0, 0, 0, 0,
                               7, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, BlockDiagPerm) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {
    1, 2, 0, 0, 0,
    0, 0, 3, 4, 0,
    0, 0, 5, 6, 0,
    0, 0, 0, 0, 7
  };

  // block structure
  constexpr int nb = 3;
  const int block_nr[nb] = {1, 2, 1};
  const int block_nc[nb] = {2, 2, 1};
  const int block_r[nb] = {0, 1, 3};
  const int block_c[nb] = {0, 2, 4};

  // scatter mat into mat_p
  const int perm_r[nr] = {1, 3, 2, 0};
  const int perm_c[nc] = {2, 0, 4, 3, 1};
  mjtNum mat_p[nr * nc];
  PermuteMat(mat_p, mat, nr, nc, perm_r, perm_c, true, true);

  // test with permutation
  mjtNum res[nr * nc] = {0};
  mju_blockDiag(res, mat_p, nc, nc, nb,
                perm_r, perm_c,
                block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res, ElementsAre(1, 2, 0, 0, 0,
                               3, 4, 5, 6, 0,
                               0, 0, 0, 0, 0,
                               7, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, BlockDiagLessCols) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {
    1, 2, 0, 0, 0,
    0, 0, 3, 4, 0,
    0, 0, 5, 6, 0,
    0, 0, 0, 0, 7
  };

  // block structure (ignore middle block)
  constexpr int nb = 2;
  const int block_nr[nb] = {1, 1};
  const int block_nc[nb] = {2, 1};
  const int block_r[nb] = {0, 3};
  const int block_c[nb] = {0, 4};

  // scatter mat into mat_p
  const int perm_r[nr] = {1, 3, 2, 0};
  const int perm_c[nc] = {2, 0, 4, 3, 1};
  mjtNum mat_p[nr * nc];
  PermuteMat(mat_p, mat, nr, nc, perm_r, perm_c, true, true);

  // test with permutation and less columns (ignore middle block)
  constexpr int nc_res = 3;
  mjtNum res2[nr * nc_res] = {0};
  mju_blockDiag(res2, mat_p, nc, nc_res, nb,
                perm_r, perm_c,
                block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res2, ElementsAre(1, 2, 0,
                                0, 0, 0,
                                0, 0, 0,
                                7, 0, 0));
}

TEST_F(EngineUtilSparseTest, BlockDiagSparse) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {
    1, 2, 0, 0, 0,
    0, 0, 3, 4, 0,
    0, 0, 5, 6, 0,
    0, 0, 0, 0, 7
  };
  constexpr int nnz = 7;

  // block structure
  constexpr int nb = 3;
  const int block_r[nb] = {0, 1, 3};
  const int block_c[nb] = {0, 2, 4};

  // convert to sparse
  int rownnz[nr];
  int rowadr[nr];
  int colind[nnz];
  mjtNum mat_sparse[nnz];
  mju_dense2sparse(mat_sparse, mat, nr, nc, rownnz, rowadr, colind, nnz);

  // test with identity permutations
  const int perm_r[nr] = {0, 1, 2, 3};
  const int perm_c[nc] = {0, 1, 2, 3, 4};
  int res_rownnz[nr];
  int res_rowadr[nr];
  int res_colind[nnz];
  mjtNum res[nnz];
  mju_blockDiagSparse(res, res_rownnz, res_rowadr, res_colind,
                      mat_sparse, rownnz, rowadr, colind, nr, nb,
                      perm_r, perm_c,
                      block_r, block_c, nullptr, nullptr);
  mjtNum dense_res[nr * nc];
  mju_sparse2dense(dense_res, res, nr, nc, res_rownnz, res_rowadr, res_colind);
  EXPECT_THAT(dense_res, ElementsAre(1, 2, 0, 0, 0,
                                     3, 4, 0, 0, 0,
                                     5, 6, 0, 0, 0,
                                     7, 0, 0, 0, 0));

  // permute mat into mat_p (scatter rows, gather columns)
  const int perm_r2[nr] = {3, 1, 0, 2};
  const int perm_c2[nc] = {4, 0, 2, 1, 3};
  mjtNum mat_p[nr*nc];
  PermuteMat(mat_p, mat, nr, nc, perm_r2, perm_c2, true, false);
  mju_dense2sparse(mat_sparse, mat_p, nr, nc, rownnz, rowadr, colind, nnz);

  // test with permutation
  mju_blockDiagSparse(res, res_rownnz, res_rowadr, res_colind,
                      mat_sparse, rownnz, rowadr, colind, nr, nb,
                      perm_r2, perm_c2,
                      block_r, block_c, nullptr, nullptr);
  mju_sparse2dense(dense_res, res, nr, nc, res_rownnz, res_rowadr, res_colind);
  EXPECT_THAT(dense_res, ElementsAre(1, 2, 0, 0, 0,
                                     3, 4, 0, 0, 0,
                                     5, 6, 0, 0, 0,
                                     7, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, PermuteMat) {
  const mjtNum mat[] = {1, 2, 0, 0,
                        0, 0, 3, 4,
                        0, 0, 5, 6};
  const int perm_r[] = {2, 0, 1};
  const int perm_c[] = {3, 2, 0, 1};
  mjtNum gather[3 * 4];
  PermuteMat(gather, mat, 3, 4, perm_r, perm_c, false, false);
  EXPECT_THAT(gather, ElementsAre(6, 5, 0, 0,
                                  0, 0, 1, 2,
                                  4, 3, 0, 0));
  mjtNum scatter[3 * 4];
  PermuteMat(scatter, gather, 3, 4, perm_r, perm_c, true, true);
  EXPECT_THAT(scatter, ElementsAre(1, 2, 0, 0,
                                   0, 0, 3, 4,
                                   0, 0, 5, 6));
  mjtNum mixed[3 * 4];
  PermuteMat(mixed, mat, 3, 4, perm_r, perm_c, true, false);
  EXPECT_THAT(mixed, ElementsAre(4, 3, 0, 0,
                                 6, 5, 0, 0,
                                 0, 0, 1, 2));
  mjtNum mixed_back[3 * 4];
  PermuteMat(mixed_back, mixed, 3, 4, perm_r, perm_c, false, true);
  EXPECT_THAT(mixed_back, ElementsAre(1, 2, 0, 0,
                                      0, 0, 3, 4,
                                      0, 0, 5, 6));
}

}  // namespace
}  // namespace mujoco
