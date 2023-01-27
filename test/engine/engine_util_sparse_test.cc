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

#include "src/engine/engine_util_sparse.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

using ::testing::ElementsAre;
using EngineUtilSparseTest = MujocoTest;

TEST_F(EngineUtilSparseTest, MjuDot) {
  mjtNum a[] = {1, 2, 3, 4, 5, 6, 7};
  mjtNum b[] = {7, 0, 6, 0, 0, 5, 0, 0, 0, 4, 0, 0, 0, 3, 0, 0, 2, 0, 1};
  int i[] = {0, 2, 5, 9, 13, 16, 18};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four
  EXPECT_EQ(mju_dotSparse(a, b, 0, i), 0);
  EXPECT_EQ(mju_dotSparse(a, b, 1, i), 7);
  EXPECT_EQ(mju_dotSparse(a, b, 2, i), 7 + 2*6);
  EXPECT_EQ(mju_dotSparse(a, b, 3, i), 7 + 2*6 + 3*5);
  EXPECT_EQ(mju_dotSparse(a, b, 4, i), 7 + 2*6 + 3*5 + 4*4);
  EXPECT_EQ(mju_dotSparse(a, b, 5, i), 7 + 2*6 + 3*5 + 4*4 + 5*3);
  EXPECT_EQ(mju_dotSparse(a, b, 6, i), 7 + 2*6 + 3*5 + 4*4 + 5*3 + 6*2);
  EXPECT_EQ(mju_dotSparse(a, b, 7, i), 7 + 2*6 + 3*5 + 4*4 + 5*3 + 6*2 + 7);
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

}  // namespace
}  // namespace mujoco
