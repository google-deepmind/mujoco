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

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <mujoco/mujoco.h>
#include "test/fixture.h"

namespace mujoco {
namespace {

// permute the rows and columns of a dense matrix
inline void PermuteMat(mjtNum* res, const mjtNum* mat, int nr, int nc,
                       const int* perm_r, const int* perm_c, bool scatter_r,
                       bool scatter_c) {
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
  mjtNum a[] = {2, 3, 4, 5, 6, 7, 8};
  mjtNum b[] = {8, 1, 7, 1, 1, 6, 1, 1, 1, 5, 1, 1, 1, 4, 1, 1, 3, 1, 2};
  int i[] = {0, 2, 5, 9, 13, 16, 18};

  // test various vector lengths as mju_dotSparse adds numbers in groups of four

  EXPECT_EQ(mju_dotSparse(a, b, 0, i), 0);
  EXPECT_EQ(mju_dotSparse(a, b, 1, i), 2 * 8);
  EXPECT_EQ(mju_dotSparse(a, b, 2, i), 2 * 8 + 3 * 7);
  EXPECT_EQ(mju_dotSparse(a, b, 3, i), 2 * 8 + 3 * 7 + 4 * 6);
  EXPECT_EQ(mju_dotSparse(a, b, 4, i), 2 * 8 + 3 * 7 + 4 * 6 + 5 * 5);
  EXPECT_EQ(mju_dotSparse(a, b, 5, i), 2 * 8 + 3 * 7 + 4 * 6 + 5 * 5 + 6 * 4);
  EXPECT_EQ(mju_dotSparse(a, b, 6, i),
            2 * 8 + 3 * 7 + 4 * 6 + 5 * 5 + 6 * 4 + 7 * 3);
  EXPECT_EQ(mju_dotSparse(a, b, 7, i),
            2 * 8 + 3 * 7 + 4 * 6 + 5 * 5 + 6 * 4 + 7 * 3 + 8 * 2);
}

TEST_F(EngineUtilSparseTest, MjuDotX3) {
  constexpr int kMaxNnz = 19;
  std::vector<mjtNum> v10(kMaxNnz), v11(kMaxNnz), v12(kMaxNnz);
  std::vector<mjtNum> v2(3 * kMaxNnz);
  std::vector<int> ind(kMaxNnz);

  for (int k = 0; k < 3 * kMaxNnz; ++k) {
    v2[k] = mjtNum(1) / (k + 2) - mjtNum(1) / (2 * k + 3);
  }
  for (int i = 0; i < kMaxNnz; ++i) {
    ind[i] = 3 * i + (i % 2);
    // include terms with cancellation sensitive to reduction association order
    mjtNum sign = (i & 2) ? -1 : 1;
    v10[i] = sign * (mjtNum(1) / ((i / 4) + 1)) + mjtNum(1e-7) / (i + 3);
    v11[i] = mjtNum(i + 1) / (2 * i + 5) - mjtNum(1) / (i + 7);
    v12[i] = -sign * mjtNum(3) / (i + 2) + mjtNum(1e-6) * (i + 1);
  }

  for (int nnz = 0; nnz <= kMaxNnz; ++nnz) {
    mjtNum res0 = -1, res1 = -1, res2 = -1;
    mju_dotSparseX3(&res0, &res1, &res2, v10.data(), v11.data(), v12.data(),
                    v2.data(), nnz, ind.data());
    EXPECT_EQ(res0, mju_dotSparse(v10.data(), v2.data(), nnz, ind.data()))
        << "nnz=" << nnz;
    EXPECT_EQ(res1, mju_dotSparse(v11.data(), v2.data(), nnz, ind.data()))
        << "nnz=" << nnz;
    EXPECT_EQ(res2, mju_dotSparse(v12.data(), v2.data(), nnz, ind.data()))
        << "nnz=" << nnz;
  }
}

TEST_F(EngineUtilSparseTest, MjuDotX3AndSparseSimdParity) {
  constexpr int kMaxNnz = 19;
  std::vector<mjtNum> v10(kMaxNnz), v11(kMaxNnz), v12(kMaxNnz);
  std::vector<mjtNum> v2(3 * kMaxNnz);
  std::vector<int> ind(kMaxNnz);

  for (int k = 0; k < 3 * kMaxNnz; ++k) {
    v2[k] = mjtNum(1) / (k + 2) - mjtNum(1) / (2 * k + 3);
  }
  for (int i = 0; i < kMaxNnz; ++i) {
    ind[i] = 3 * i + (i % 2);
    mjtNum sign = (i & 2) ? -1 : 1;
    v10[i] = sign * (mjtNum(1) / ((i / 4) + 1)) + mjtNum(1e-7) / (i + 3);
    v11[i] = mjtNum(i + 1) / (2 * i + 5) - mjtNum(1) / (i + 7);
    v12[i] = -sign * mjtNum(3) / (i + 2) + mjtNum(1e-6) * (i + 1);
  }

  auto scalar_dot_sparse = [](const mjtNum* vec1, const mjtNum* vec2, int nnz1,
                              const int* ind1) {
    int i = 0;
    mjtNum res = 0;
    int n_4 = nnz1 - 4;
    if (n_4 >= 0) {
      mjtNum r0 = vec1[0] * vec2[ind1[0]];
      mjtNum r1 = vec1[1] * vec2[ind1[1]];
      mjtNum r2 = vec1[2] * vec2[ind1[2]];
      mjtNum r3 = vec1[3] * vec2[ind1[3]];
      for (i = 4; i <= n_4; i += 4) {
        r0 += vec1[i + 0] * vec2[ind1[i + 0]];
        r1 += vec1[i + 1] * vec2[ind1[i + 1]];
        r2 += vec1[i + 2] * vec2[ind1[i + 2]];
        r3 += vec1[i + 3] * vec2[ind1[i + 3]];
      }
      res = (r0 + r2) + (r1 + r3);
    }
    for (; i < nnz1; ++i) {
      res += vec1[i] * vec2[ind1[i]];
    }
    return res;
  };

  for (int nnz = 0; nnz <= kMaxNnz; ++nnz) {
    mjtNum ref0 = scalar_dot_sparse(v10.data(), v2.data(), nnz, ind.data());
    mjtNum ref1 = scalar_dot_sparse(v11.data(), v2.data(), nnz, ind.data());
    mjtNum ref2 = scalar_dot_sparse(v12.data(), v2.data(), nnz, ind.data());

    mjtNum ds0 = mju_dotSparse(v10.data(), v2.data(), nnz, ind.data());
    mjtNum ds1 = mju_dotSparse(v11.data(), v2.data(), nnz, ind.data());
    mjtNum ds2 = mju_dotSparse(v12.data(), v2.data(), nnz, ind.data());
    EXPECT_EQ(std::memcmp(&ds0, &ref0, sizeof(mjtNum)), 0) << "nnz=" << nnz;
    EXPECT_EQ(std::memcmp(&ds1, &ref1, sizeof(mjtNum)), 0) << "nnz=" << nnz;
    EXPECT_EQ(std::memcmp(&ds2, &ref2, sizeof(mjtNum)), 0) << "nnz=" << nnz;

    mjtNum res0 = -1, res1 = -1, res2 = -1;
    mju_dotSparseX3(&res0, &res1, &res2, v10.data(), v11.data(), v12.data(),
                    v2.data(), nnz, ind.data());
    EXPECT_EQ(std::memcmp(&res0, &ref0, sizeof(mjtNum)), 0) << "nnz=" << nnz;
    EXPECT_EQ(std::memcmp(&res1, &ref1, sizeof(mjtNum)), 0) << "nnz=" << nnz;
    EXPECT_EQ(std::memcmp(&res2, &ref2, sizeof(mjtNum)), 0) << "nnz=" << nnz;

    // Also test mju_addToSclScl parity for length nnz
    std::vector<mjtNum> dst_simd(v10.begin(), v10.end());
    std::vector<mjtNum> dst_ref(v10.begin(), v10.end());
    mjtNum scl1 = mjtNum(1.25), scl2 = mjtNum(-0.75);
    mju_addToSclScl(dst_simd.data(), v11.data(), scl1, scl2, nnz);
    for (int j = 0; j < nnz; ++j) {
      dst_ref[j] = dst_ref[j] * scl1 + v11[j] * scl2;
    }
    EXPECT_EQ(
        std::memcmp(dst_simd.data(), dst_ref.data(), kMaxNnz * sizeof(mjtNum)),
        0)
        << "nnz=" << nnz;
  }

  // Test mju_mulMatVecSparse with and without rowsuper (supernodes of size 4 ->
  // 3+1 and size 2)
  constexpr int kNr = 6;
  std::vector<mjtNum> mat(kNr * kMaxNnz);
  int rownnz[kNr];
  int rowadr[kNr];
  std::vector<int> colind(kNr * kMaxNnz);
  int rowsuper[kNr] = {3, 2, 1, 0, 1, 0};
  for (int r = 0; r < kNr; ++r) {
    rownnz[r] = (r < 4) ? 11 : 7;
    rowadr[r] = r * kMaxNnz;
    for (int j = 0; j < rownnz[r]; ++j) {
      colind[rowadr[r] + j] = ind[j];
      mat[rowadr[r] + j] = (r % 2 == 0 ? v10[j] : v11[j]) * (r + 1);
    }
  }
  mjtNum mv_nosuper[kNr] = {0}, mv_super[kNr] = {0};
  mju_mulMatVecSparse(mv_nosuper, mat.data(), v2.data(), kNr, rownnz, rowadr,
                      colind.data(), nullptr);
  mju_mulMatVecSparse(mv_super, mat.data(), v2.data(), kNr, rownnz, rowadr,
                      colind.data(), rowsuper);
  EXPECT_EQ(std::memcmp(mv_nosuper, mv_super, sizeof(mv_nosuper)), 0);
  EXPECT_EQ(mju_compare(nullptr, nullptr, 0), 1);
  EXPECT_EQ(mju_compare(nullptr, nullptr, -1), 1);
}

TEST_F(EngineUtilSparseTest, MjuDot2) {
  constexpr int annz = 6;
  constexpr int bnnz = 5;

  // values
  mjtNum a[annz] = {2, 3, 4, 5, 6};
  mjtNum b[bnnz] = {8, 7, 6, 5, 4};

  // indices
  int ia[annz] = {0, 2, 5, 6, 7};
  int ib[bnnz] = {1, 2, 3, 5, 7};

  EXPECT_EQ(mju_dotSparse2(a, ia, annz, b, ib, bnnz), 3 * 7 + 4 * 5 + 6 * 4);
}

TEST_F(EngineUtilSparseTest, CombineSparseCount) {
  {
    std::array a_ind{0, 1};
    std::array b_ind{2};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              3);
  }

  {
    std::array a_ind{2};
    std::array b_ind{0, 1};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              3);
  }

  {
    std::array a_ind{0, 1};
    std::array b_ind{2, 3, 4};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              5);
  }

  {
    std::array a_ind{5, 6};
    std::array b_ind{1, 3, 8};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              5);
  }

  {
    std::array a_ind{1, 2, 3};
    std::array b_ind{0, 4};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              5);
  }

  {
    std::array a_ind{1, 4};
    std::array b_ind{2, 3};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              4);
  }

  {
    std::array a_ind{0, 1, 3};
    std::array b_ind{0, 3, 4};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              4);
  }

  {
    std::array a_ind{1, 3, 5, 6};
    std::array b_ind{1, 3, 5, 6};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), b_ind.size(), a_ind.data(),
                                     b_ind.data()),
              4);
  }

  EXPECT_EQ(mju_combineSparseCount(0, 0, nullptr, nullptr), 0);

  {
    std::array b_ind{1, 2};
    EXPECT_EQ(mju_combineSparseCount(0, b_ind.size(), nullptr, b_ind.data()),
              2);
  }

  {
    std::array a_ind{0};
    EXPECT_EQ(mju_combineSparseCount(a_ind.size(), 0, a_ind.data(), nullptr),
              1);
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
                      rownnz, rowadr, colind, nullptr);

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
                      rownnz, rowadr, colind, nullptr);

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
                      rownnz, rowadr, colind, nullptr);

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
  int rowsuper[] = {2, 1, 0};

  for (const int* in_super :
       {static_cast<const int*>(nullptr), static_cast<const int*>(rowsuper)}) {
    mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    int colindT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    int rownnzT[] = {0, 0, 0};
    int rowadrT[] = {0, 0, 0};
    int rowsuperT[] = {0, 0, 0};

    mju_transposeSparse(matT, mat, 3, 3, rownnzT, rowadrT, colindT, rowsuperT,
                        rownnz, rowadr, colind, in_super);

    EXPECT_THAT(matT, ElementsAre(1, 4, 7, 2, 5, 8, 3, 6, 9));
    EXPECT_THAT(colindT, ElementsAre(0, 1, 2, 0, 1, 2, 0, 1, 2));
    EXPECT_THAT(rownnzT, ElementsAre(3, 3, 3));
    EXPECT_THAT(rowadrT, ElementsAre(0, 3, 6));
    EXPECT_THAT(rowsuperT, ElementsAre(2, 1, 0));
  }
}

TEST_F(EngineUtilSparseTest, MjuTransposeSuper) {
  //  mat:   0,  1,  2,  0,  3,  4,  5,  0,  0,  0,  0,  0,  0
  //         0,  0,  0,  6,  7,  8,  9, 10, 11, 12,  0,  0,  0
  //
  //  super: 0,  1,  0,  0,  2,  1,  0,  2,  1,  0,  2,  1,  0

  mjtNum mat[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  int colind[] = {1, 2, 4, 5, 6, 3, 4, 5, 6, 7, 8, 9};
  int rownnz[] = {5, 7};
  int rowadr[] = {0, 5};

  mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rownnzT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowadrT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowsuperT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  mju_transposeSparse(matT, mat, 2, 13, rownnzT, rowadrT, colindT, rowsuperT,
                      rownnz, rowadr, colind, nullptr);

  EXPECT_THAT(matT, ElementsAre(1, 2, 6, 3, 7, 4, 8, 5, 9, 10, 11, 12));
  EXPECT_THAT(colindT, ElementsAre(0, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1));
  EXPECT_THAT(rownnzT, ElementsAre(0, 1, 1, 1, 2, 2, 2, 1, 1, 1, 0, 0, 0));
  EXPECT_THAT(rowadrT, ElementsAre(0, 0, 1, 2, 3, 5, 7, 9, 10, 11, 12, 12, 12));
  EXPECT_THAT(rowsuperT, ElementsAre(0, 1, 0, 0, 2, 1, 0, 2, 1, 0, 2, 1, 0));

  // Verify non-NULL input rowsuper matches nullptr across mixed supernode sizes
  // (1, 2, 3, 4, 5, 6, 9 plus empty rows), values, pattern-only, output
  // rowsuper, non-zero rowadr[0], and padded (non-contiguous) rowadr.
  constexpr int nc = 15;
  struct RowGroup {
    int count;
    std::vector<int> cols;
  };
  const std::vector<RowGroup> groups = {
      {1, {0, 1, 2}},
      {3, {1, 2, 3, 7, 8, 9}},
      {2, {}},  // empty rows
      {4, {4, 5, 6, 10, 11, 12}},
      {6, {1, 2, 3, 7, 8, 9}},
      {5, {0, 5, 14}},
      {9, {2, 3, 4, 11, 12, 13}},
      {1, {14}},
  };

  for (int pad : {0, 1}) {
    for (int base_offset : {0, 7}) {
      std::vector<int> g_rownnz, g_rowadr, g_colind;
      std::vector<mjtNum> g_mat;
      int val_counter = 1;
      for (const auto& g : groups) {
        for (int s = 0; s < g.count; ++s) {
          g_rowadr.push_back(base_offset + static_cast<int>(g_colind.size()));
          g_rownnz.push_back(static_cast<int>(g.cols.size()));
          for (int c : g.cols) {
            g_colind.push_back(c);
            g_mat.push_back(static_cast<mjtNum>(val_counter++));
          }
          if (pad) {
            g_colind.push_back(-1);
            g_mat.push_back(static_cast<mjtNum>(-999));
          }
        }
      }

      const int nr = static_cast<int>(g_rownnz.size());
      int total_nnz = 0;
      for (int nnz : g_rownnz) total_nnz += nnz;

      std::vector<int> rel_rowadr(nr);
      for (int r = 0; r < nr; ++r) rel_rowadr[r] = g_rowadr[r] - base_offset;
      std::vector<int> rowsuper(nr, 0);
      mju_superSparse(nr, rowsuper.data(), g_rownnz.data(), rel_rowadr.data(),
                      g_colind.data());

      // reference with rowsuper=nullptr
      std::vector<mjtNum> ref_val(total_nnz, 0);
      std::vector<int> ref_rownnz(nc, 0), ref_rowadr(nc, 0),
          ref_colind(total_nnz, 0), ref_rowsuper(nc, 0);
      mju_transposeSparse(
          ref_val.data(), g_mat.data(), nr, nc, ref_rownnz.data(),
          ref_rowadr.data(), ref_colind.data(), ref_rowsuper.data(),
          g_rownnz.data(), g_rowadr.data(), g_colind.data(), nullptr);

      // values + output supernodes with input rowsuper
      std::vector<mjtNum> got_val(total_nnz, 0);
      std::vector<int> got_rownnz(nc, 0), got_rowadr(nc, 0),
          got_colind(total_nnz, 0), got_rowsuper(nc, 0);
      mju_transposeSparse(
          got_val.data(), g_mat.data(), nr, nc, got_rownnz.data(),
          got_rowadr.data(), got_colind.data(), got_rowsuper.data(),
          g_rownnz.data(), g_rowadr.data(), g_colind.data(), rowsuper.data());
      EXPECT_EQ(got_val, ref_val);
      EXPECT_EQ(got_rownnz, ref_rownnz);
      EXPECT_EQ(got_rowadr, ref_rowadr);
      EXPECT_EQ(got_colind, ref_colind);
      EXPECT_EQ(got_rowsuper, ref_rowsuper);

      // pattern-only + no output supernodes with input rowsuper
      std::vector<int> pat_rownnz(nc, 0), pat_rowadr(nc, 0),
          pat_colind(total_nnz, 0);
      mju_transposeSparse(nullptr, nullptr, nr, nc, pat_rownnz.data(),
                          pat_rowadr.data(), pat_colind.data(), nullptr,
                          g_rownnz.data(), g_rowadr.data(), g_colind.data(),
                          rowsuper.data());
      EXPECT_EQ(pat_rownnz, ref_rownnz);
      EXPECT_EQ(pat_rowadr, ref_rowadr);
      EXPECT_EQ(pat_colind, ref_colind);

      // precomputed res_rowadr (res_rownnz=nullptr): fills values & colind and
      // restores res_rowadr
      std::vector<mjtNum> pre_val(total_nnz, 0);
      std::vector<int> pre_rowadr = ref_rowadr;
      std::vector<int> pre_colind(total_nnz, 0);
      mju_transposeSparse(pre_val.data(), g_mat.data(), nr, nc, nullptr,
                          pre_rowadr.data(), pre_colind.data(), nullptr,
                          g_rownnz.data(), g_rowadr.data(), g_colind.data(),
                          rowsuper.data());
      EXPECT_EQ(pre_val, ref_val);
      EXPECT_EQ(pre_rowadr, ref_rowadr);
      EXPECT_EQ(pre_colind, ref_colind);
    }
  }
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
                      rownnz, rowadr, colind, nullptr);

  EXPECT_THAT(matT, ElementsAre(1));
  EXPECT_THAT(colindT, ElementsAre(0));
  EXPECT_THAT(rownnzT, ElementsAre(1));
  EXPECT_THAT(rowadrT, ElementsAre(0));
}

TEST_F(EngineUtilSparseTest, MjuTransposeNullMatrix) {
  // 0 -> 0

  mjtNum* mat = nullptr;
  int* colind = nullptr;
  int rownnz[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowadr[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int rowsuper[10];
  mju_superSparse(10, rowsuper, rownnz, rowadr, colind);

  for (const int* in_super :
       {static_cast<const int*>(nullptr), static_cast<const int*>(rowsuper)}) {
    mjtNum* matT = nullptr;
    int* colindT = nullptr;
    int rownnzT[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    int rowadrT[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    mju_transposeSparse(matT, mat, 10, 10, rownnzT, rowadrT, colindT, nullptr,
                        rownnz, rowadr, colind, in_super);

    EXPECT_THAT(rownnzT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
    EXPECT_THAT(rowadrT, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0, 0));
  }
}

TEST_F(EngineUtilSparseTest, MjuCompressSparse) {
  // sparse matrix (uncompressed with spurious values between the rows):
  // [[1, 0, 2]
  //  [0, O, 3]  (second zero represented)
  mjtNum mat[] = {1, 2, 9, 0, 3};   // spurious 9 value
  int colind[] = {0, 2, -1, 1, 2};  // spurious -1 index
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 3};

  mjtNum dense_expected[] = {1, 0, 2, 0, 0, 3};
  mjtNum dense[6];
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that spurious values are removed
  int nnz = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind,
                               /*minval=*/-1);
  EXPECT_EQ(nnz, 4);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that represented zero gets compressed aways with minval=0
  nnz = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind, /*minval=*/0);
  EXPECT_EQ(nnz, 3);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected, 6));

  // check that 1 gets compressed aways with minval=1
  nnz = mju_compressSparse(mat, 2, 3, rownnz, rowadr, colind, /*minval=*/1);
  EXPECT_EQ(nnz, 2);
  mju_sparse2dense(dense, mat, 2, 3, rownnz, rowadr, colind);
  mjtNum dense_expected_minval1[] = {0, 0, 2, 0, 0, 3};
  EXPECT_EQ(AsVector(dense, 6), AsVector(dense_expected_minval1, 6));
}

TEST_F(EngineUtilSparseTest, MjuSym2Dense) {
  // lower-triangular CSR for a 3x3 symmetric matrix:
  //     1 2 0
  //     2 3 4
  //     0 4 5
  // stored as lower triangle:
  //   row 0: [1]           (col 0)
  //   row 1: [2, 3]        (cols 0, 1)
  //   row 2: [4, 5]        (cols 1, 2)
  mjtNum mat[] = {1, 2, 3, 4, 5};
  int rownnz[] = {1, 2, 2};
  int rowadr[] = {0, 1, 3};
  int colind[] = {0, 0, 1, 1, 2};

  mjtNum dense[9];
  mju_sym2dense(dense, mat, 3, rownnz, rowadr, colind);

  mjtNum expected[] = {1, 2, 0, 2, 3, 4, 0, 4, 5};
  EXPECT_EQ(AsVector(dense, 9), AsVector(expected, 9));
}

TEST_F(EngineUtilSparseTest, MjuSym2DenseWithUpper) {
  mjtNum mat[] = {1, 999, 2, 3, 4, 5};
  int rownnz[] = {2, 2, 2};
  int rowadr[] = {0, 2, 4};
  int colind[] = {0, 1, 0, 1, 1, 2};

  mjtNum dense[9];
  mju_sym2dense(dense, mat, 3, rownnz, rowadr, colind);

  mjtNum expected[] = {1, 2, 0, 2, 3, 4, 0, 4, 5};
  EXPECT_EQ(AsVector(dense, 9), AsVector(expected, 9));
}

// helper: run split-col approach and return dense result
static void SqrMatTDSplitCol(std::vector<mjtNum>& dense_result, int nr, int nc,
                             const mjtNum* mat, const int* rownnz,
                             const int* rowadr, const int* colind,
                             const mjtNum* matT, const int* rownnzT,
                             const int* rowadrT, const int* colindT,
                             const int* rowsuperT, const mjtNum* diag,
                             int* out_diagind, mjData* d) {
  // count mode
  std::vector<int> H_rownnz(nc, 0);
  std::vector<int> H_rowadr(nc, 0);
  int nnz = mju_sqrMatTDSparseSymbolic(
      H_rownnz.data(), H_rowadr.data(), nullptr, out_diagind, nr, nc, rownnz,
      rowadr, colind, nullptr, rownnzT, rowadrT, colindT, rowsuperT, d);

  // fill mode
  std::vector<int> H_colind(nnz);
  mju_sqrMatTDSparseSymbolic(H_rownnz.data(), H_rowadr.data(), H_colind.data(),
                             out_diagind, nr, nc, rownnz, rowadr, colind,
                             nullptr, rownnzT, rowadrT, colindT, rowsuperT, d);

  // numeric phase
  std::vector<mjtNum> H(nnz, 0);
  mju_sqrMatTDSparseNumeric(H.data(), nc, H_rownnz.data(), H_rowadr.data(),
                            H_colind.data(), out_diagind, mat, rownnz, rowadr,
                            colind, matT, rownnzT, rowadrT, colindT, rowsuperT,
                            diag, d);

  // densify
  dense_result.assign(nc * nc, 0);
  for (int r = 0; r < nc; r++) {
    for (int j = 0; j < H_rownnz[r]; j++) {
      int c = H_colind[H_rowadr[r] + j];
      dense_result[r * nc + c] = H[H_rowadr[r] + j];
    }
  }
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse1) {
  //       0 0 0
  // M  =  0 0 0
  //       0 0 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, nullptr, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(0, 0, 0, 0, 0, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseLower) {
  //     2 -1  1
  // M = 2 -1  2
  //     2  2  3

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {2, -1, 1, 2, -1, 2, 2, 2, 3};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {2, 2, 2, -1, -1, 2, 1, 2, 3};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, nullptr, nullptr, data.get());

  EXPECT_THAT(dense, ElementsAre(12, 0, 0, 0, 6, 0, 12, 3, 14));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse2) {
  //     2 -1  1
  // M = 2 -1  2
  //     2  2  3

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {2, -1, 1, 2, -1, 2, 2, 2, 3};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {2, 2, 2, -1, -1, 2, 1, 2, 3};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, nullptr, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(12, 0, 12, 0, 6, 3, 12, 3, 14));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse3) {
  //     1 2 0
  // M = 0 3 0
  //     4 0 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 1, 1, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 2, 0};
  int rowadrT[] = {0, 2, 4};

  mjtNum diag[] = {2, 3, 4};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(66, 4, 0, 4, 35, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse3b) {
  //     1 2 0
  // M = 0 3 4
  //     5 0 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 3, 4, 5};
  int colind[] = {0, 1, 1, 2, 0};
  int rownnz[] = {2, 2, 1};
  int rowadr[] = {0, 2, 4};

  mjtNum matT[] = {1, 5, 2, 3, 4};
  int colindT[] = {0, 2, 0, 1, 1};
  int rownnzT[] = {2, 2, 1};
  int rowadrT[] = {0, 2, 4};

  mjtNum diag[] = {1, 1, 1};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(26, 2, 0, 2, 13, 12, 0, 12, 16));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse4) {
  //     1 0 2
  // M = 0 0 3
  //     4 0 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 2, 2, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 0, 2};
  int rowadrT[] = {0, 2, 2};

  mjtNum diag[] = {2, 3, 4};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(66, 0, 4, 0, 0, 0, 4, 0, 35));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse5) {
  //     1 0 4
  // M = 0 0 0
  //     2 3 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 4, 2, 3};
  int colind[] = {0, 2, 0, 1};
  int rownnz[] = {2, 0, 2};
  int rowadr[] = {0, 2, 2};

  mjtNum matT[] = {1, 2, 3, 4};
  int colindT[] = {0, 2, 2, 0};
  int rownnzT[] = {2, 1, 1};
  int rowadrT[] = {0, 2, 3};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, nullptr, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(5, 6, 4, 6, 9, 0, 4, 0, 16));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse6) {
  //     1 0 2
  // M = 0 2 0
  //     0 0 3

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 2, 3};
  int colind[] = {0, 2, 1, 2};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 2, 2, 3};
  int colindT[] = {0, 1, 0, 2};
  int rownnzT[] = {1, 1, 2};
  int rowadrT[] = {0, 1, 2};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, nullptr, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(1, 0, 2, 0, 4, 0, 2, 0, 13));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse7) {
  //     1 2
  // M = 0 3
  //     4 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 3, 4};
  int colind[] = {0, 1, 1, 0};
  int rownnz[] = {2, 1, 1};
  int rowadr[] = {0, 2, 3};

  mjtNum matT[] = {1, 4, 2, 3};
  int colindT[] = {0, 2, 0, 1};
  int rownnzT[] = {2, 2};
  int rowadrT[] = {0, 2};

  mjtNum diag[] = {2, 3, 4};

  int diagindH[2];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 2, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(66, 4, 4, 35));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse8) {
  // M = 1 0 4
  //     2 3 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 4, 2, 3};
  int colind[] = {0, 2, 0, 1};
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 2};

  mjtNum matT[] = {1, 2, 3, 4};
  int colindT[] = {0, 1, 1, 0};
  int rownnzT[] = {2, 1, 1};
  int rowadrT[] = {0, 2, 3};

  mjtNum diag[] = {2, 3};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 2, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(14, 18, 8, 18, 27, 0, 8, 0, 32));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse9) {
  //     1 2 2
  // M = 1 3 4
  //     4 4 4

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 2, 1, 3, 4, 4, 4, 4};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {1, 1, 4, 2, 3, 4, 2, 4, 4};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};

  mjtNum diag[] = {2, 3, 4};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, nullptr, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(69, 77, 80, 77, 99, 108, 80, 108, 120));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse10) {
  //     1 2 3
  // M = 2 3 2
  //     3 1 1

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 2, 3, 2, 3, 2, 3, 1, 1};
  int colind[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnz[] = {3, 3, 3};
  int rowadr[] = {0, 3, 6};

  mjtNum matT[] = {1, 2, 3, 2, 3, 1, 3, 2, 1};
  int colindT[] = {0, 1, 2, 0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 3};
  int rowadrT[] = {0, 3, 6};
  int rowsuperT[] = {2, 1, 0};

  mjtNum diag[] = {1, 2, 1};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, rowsuperT, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(18, 17, 14, 17, 23, 19, 14, 19, 18));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse11) {
  //     1 1 1
  // M = 0 0 0
  //     0 3 3

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 1, 1, 3, 3};
  int colind[] = {0, 1, 2, 1, 2};
  int rownnz[] = {3, 0, 2};
  int rowadr[] = {0, 3, 3};

  mjtNum matT[] = {1, 1, 3, 1, 3};
  int colindT[] = {0, 0, 2, 0, 2};
  int rownnzT[] = {1, 2, 2};
  int rowadrT[] = {0, 1, 3};
  int rowsuperT[] = {0, 1, 0};

  mjtNum diag[] = {1, 1, 1};

  int diagindH[3];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 3, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, rowsuperT, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(1, 1, 1, 1, 10, 10, 1, 10, 10));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse12) {
  //     1 1 1 1
  // M = 0 0 0 0
  //     0 0 3 3

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 3, 3};
  int colind[] = {0, 1, 2, 3, 2, 3};
  int rownnz[] = {4, 0, 2};
  int rowadr[] = {0, 4, 4};

  mjtNum matT[] = {1, 1, 1, 3, 1, 3};
  int colindT[] = {0, 0, 0, 2, 0, 2};
  int rownnzT[] = {1, 1, 2, 2};
  int rowadrT[] = {0, 1, 2, 4};
  int rowsuperT[] = {1, 0, 1, 0};

  mjtNum diag[] = {1, 1, 1};

  int diagindH[4];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 4, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, rowsuperT, diag, diagindH, data.get());

  EXPECT_THAT(dense,
              ElementsAre(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10, 10, 1, 1, 10, 10));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse13) {
  //     1 1 0 0 0
  // M = 1 1 0 0 0
  //     1 1 0 0 0

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 1, 1};
  int colind[] = {0, 1, 0, 1, 0, 1};
  int rownnz[] = {2, 2, 2};
  int rowadr[] = {0, 2, 4};

  mjtNum matT[] = {1, 1, 1, 1, 1, 1};
  int colindT[] = {0, 1, 2, 0, 1, 2};
  int rownnzT[] = {3, 3, 0, 0, 0};
  int rowadrT[] = {0, 3, 6, 6, 6};
  int rowsuperT[] = {1, 0, 2, 1, 0};

  mjtNum diag[] = {1, 1, 1};

  int diagindH[5];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 3, 5, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, rowsuperT, diag, diagindH, data.get());

  EXPECT_THAT(dense, ElementsAre(3, 3, 0, 0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparse14) {
  // M = 1 1 1 1 2 2 2

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  mjtNum mat[] = {1, 1, 1, 1, 2, 2, 2};
  int colind[] = {0, 1, 2, 3, 4, 5, 6};
  int rownnz[] = {7};
  int rowadr[] = {0};

  mjtNum matT[] = {1, 1, 1, 1, 2, 2, 2};
  int colindT[] = {0, 0, 0, 0, 0, 0, 0};
  int rownnzT[] = {1, 1, 1, 1, 1, 1, 1};
  int rowadrT[] = {0, 1, 2, 3, 4, 5, 6};
  int rowsuperT[] = {3, 2, 1, 0, 2, 1, 0};

  int diagindH[7];
  std::vector<mjtNum> dense;
  SqrMatTDSplitCol(dense, 1, 7, mat, rownnz, rowadr, colind, matT, rownnzT,
                   rowadrT, colindT, rowsuperT, nullptr, diagindH, data.get());

  EXPECT_THAT(dense,
              ElementsAre(1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1,
                          2, 2, 2, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 4, 4, 4, 2,
                          2, 2, 2, 4, 4, 4, 2, 2, 2, 2, 4, 4, 4));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseSymbolic) {
  // Simple dense 2x2 matrix:
  //     1 2
  // M = 3 4
  //
  // M'M (lower triangle) should have 3 elements: (0,0), (1,0), (1,1)

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  // M in CSR: row 0 has cols 0,1; row 1 has cols 0,1
  int colind[] = {0, 1, 0, 1};
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 2};

  // compute transpose using mju_transposeSparse
  mjtNum mat[] = {1, 2, 3, 4};
  mjtNum matT[4];
  int colindT[4];
  int rownnzT[2];
  int rowadrT[2];
  mju_transposeSparse(matT, mat, 2, 2, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind, nullptr);

  // use old function as ground truth
  int rownnzH_expected[] = {0, 0};
  int rowadrH_expected[] = {0, 0};
  int nnz_expected = mju_sqrMatTDSparseCount(
      rownnzH_expected, rowadrH_expected, 2, rownnz, rowadr, colind, rownnzT,
      rowadrT, colindT, nullptr, data.get(), /*flg_upper=*/0);

  // verify: lower triangle should have 3 elements: (0,0), (1,0), (1,1)
  EXPECT_EQ(nnz_expected, 3);
  EXPECT_THAT(rownnzH_expected, ElementsAre(1, 2));
  EXPECT_THAT(rowadrH_expected, ElementsAre(0, 1));

  // test count mode of new function
  int rownnzH[] = {0, 0};
  int rowadrH[] = {0, 0};

  int nnz = mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, nullptr, nullptr, 2, 2,
                                       rownnz, rowadr, colind, nullptr, rownnzT,
                                       rowadrT, colindT, nullptr, data.get());

  EXPECT_EQ(nnz, nnz_expected);
  EXPECT_THAT(rownnzH, ElementsAre(rownnzH_expected[0], rownnzH_expected[1]));
  EXPECT_THAT(rowadrH, ElementsAre(rowadrH_expected[0], rowadrH_expected[1]));

  // test fill mode
  std::vector<int> colindH(nnz, -1);

  mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, colindH.data(), nullptr, 2, 2,
                             rownnz, rowadr, colind, nullptr, rownnzT, rowadrT,
                             colindT, nullptr, data.get());

  // verify: row 0 should have {0}, row 1 should have {0, 1}
  EXPECT_THAT(colindH, ElementsAre(0, 0, 1));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseSymbolicUpper) {
  // Test flg_upper=1: count both lower and upper triangle
  // Same matrix as previous test

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  int colind[] = {0, 1, 0, 1};
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 2};

  mjtNum mat[] = {1, 2, 3, 4};
  mjtNum matT[4];
  int colindT[4];
  int rownnzT[2];
  int rowadrT[2];
  mju_transposeSparse(matT, mat, 2, 2, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind, nullptr);

  // use old function as ground truth with flg_upper=1
  int rownnzH_expected[] = {0, 0};
  int rowadrH_expected[] = {0, 0};
  int nnz_expected = mju_sqrMatTDSparseCount(
      rownnzH_expected, rowadrH_expected, 2, rownnz, rowadr, colind, rownnzT,
      rowadrT, colindT, nullptr, data.get(), /*flg_upper=*/1);

  // test new function with diagind (upper triangle)
  int rownnzH[] = {0, 0};
  int rowadrH[] = {0, 0};
  int diagindH[] = {0, 0};
  int nnz = mju_sqrMatTDSparseSymbolic(
      rownnzH, rowadrH, nullptr, diagindH, 2, 2, rownnz, rowadr, colind,
      nullptr, rownnzT, rowadrT, colindT, nullptr, data.get());

  EXPECT_EQ(nnz, nnz_expected);
  EXPECT_THAT(rownnzH, ElementsAre(rownnzH_expected[0], rownnzH_expected[1]));
  EXPECT_THAT(rowadrH, ElementsAre(rowadrH_expected[0], rowadrH_expected[1]));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseSymbolicSupernode) {
  // Test supernode exploitation with a matrix that has supernodes
  // M has two rows with identical sparsity pattern

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  // 3x2 matrix where rows 1 and 2 have same pattern
  //     1 0
  // M = 2 3
  //     4 5
  int colind[] = {0, 0, 1, 0, 1};
  int rownnz[] = {1, 2, 2};
  int rowadr[] = {0, 1, 3};

  mjtNum mat[] = {1, 2, 3, 4, 5};
  mjtNum matT[5];
  int colindT[5];
  int rownnzT[2];
  int rowadrT[2];
  mju_transposeSparse(matT, mat, 3, 2, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind, nullptr);

  // compute rowsuper and rowsuperT
  int rowsuper[3];
  int rowsuperT[2];
  mju_superSparse(3, rowsuper, rownnz, rowadr, colind);
  mju_superSparse(2, rowsuperT, rownnzT, rowadrT, colindT);

  // use old function as ground truth
  int rownnzH_expected[] = {0, 0};
  int rowadrH_expected[] = {0, 0};
  int nnz_expected = mju_sqrMatTDSparseCount(
      rownnzH_expected, rowadrH_expected, 2, rownnz, rowadr, colind, rownnzT,
      rowadrT, colindT, rowsuperT, data.get(), /*flg_upper=*/0);

  // test new function with supernodes
  int rownnzH[] = {0, 0};
  int rowadrH[] = {0, 0};
  int nnz = mju_sqrMatTDSparseSymbolic(
      rownnzH, rowadrH, nullptr, nullptr, 3, 2, rownnz, rowadr, colind,
      rowsuper, rownnzT, rowadrT, colindT, rowsuperT, data.get());

  EXPECT_EQ(nnz, nnz_expected);
  EXPECT_THAT(rownnzH, ElementsAre(rownnzH_expected[0], rownnzH_expected[1]));
  EXPECT_THAT(rowadrH, ElementsAre(rowadrH_expected[0], rowadrH_expected[1]));

  // test fill mode with supernodes
  std::vector<int> colindH(nnz, -1);
  mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, colindH.data(), nullptr, 3, 2,
                             rownnz, rowadr, colind, rowsuper, rownnzT, rowadrT,
                             colindT, rowsuperT, data.get());

  // verify all filled
  for (int i = 0; i < nnz; i++) {
    EXPECT_GE(colindH[i], 0) << "colindH[" << i << "] not filled";
  }

  // verify numeric phase with supernodes
  std::vector<mjtNum> resH(nnz);
  mjtNum diag[] = {1, 1, 1, 1, 1};  // dummy diagonal
  mju_sqrMatTDSparseNumeric(resH.data(), 2, rownnzH, rowadrH, colindH.data(),
                            nullptr, mat, rownnz, rowadr, colind, matT, rownnzT,
                            rowadrT, colindT, rowsuperT, diag, data.get());

  // ground truth numeric
  std::vector<mjtNum> res_expected(4);
  std::vector<int> colindH_expected(4);
  int rownnzH_exp[] = {0, 0};
  int rowadrH_exp[] = {0, 2};
  mju_sqrMatTDSparse(res_expected.data(), mat, matT, diag, 3, 2, rownnzH_exp,
                     rowadrH_exp, colindH_expected.data(), rownnz, rowadr,
                     colind, nullptr, rownnzT, rowadrT, colindT, rowsuperT,
                     data.get(), nullptr);

  // compare values (sparse result vs sparse ground truth)
  for (int r = 0; r < 2; r++) {
    for (int i = 0; i < rownnzH[r]; i++) {
      // find matching col in ground truth
      int c = colindH[rowadrH[r] + i];
      mjtNum val = resH[rowadrH[r] + i];

      bool found = false;
      for (int j = 0; j < rownnzH_exp[r]; j++) {
        if (colindH_expected[rowadrH_exp[r] + j] == c) {
          EXPECT_NEAR(val, res_expected[rowadrH_exp[r] + j], 1e-14);
          found = true;
          break;
        }
      }
      EXPECT_TRUE(found) << "Column " << c
                         << " not found in ground truth for row " << r;
    }
  }
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseNumeric) {
  // Test numeric phase using symbolic phase + existing function as ground truth
  //     1 2
  // M = 3 4

  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  int colind[] = {0, 1, 0, 1};
  int rownnz[] = {2, 2};
  int rowadr[] = {0, 2};
  mjtNum mat[] = {1, 2, 3, 4};

  // compute transpose
  mjtNum matT[4];
  int colindT[4];
  int rownnzT[2];
  int rowadrT[2];
  mju_transposeSparse(matT, mat, 2, 2, rownnzT, rowadrT, colindT, nullptr,
                      rownnz, rowadr, colind, nullptr);

  // compute supernodes
  int rowsuperT[2];
  mju_superSparse(2, rowsuperT, rownnzT, rowadrT, colindT);

  mjtNum diag[] = {2, 3};  // diagonal weighting matrix

  // test both diagind cases: lower-only (diagind=NULL) and both triangles
  // (diagind!=NULL)
  for (int use_diagind = 0; use_diagind <= 1; use_diagind++) {
    // compute sparsity pattern using symbolic phase
    int rownnzH[] = {0, 0};
    int rowadrH[] = {0, 0};
    int diagindH[] = {0, 0};
    int nnz = mju_sqrMatTDSparseSymbolic(
        rownnzH, rowadrH, nullptr, use_diagind ? diagindH : nullptr, 2, 2,
        rownnz, rowadr, colind, nullptr, rownnzT, rowadrT, colindT, nullptr,
        data.get());

    std::vector<int> colindH(nnz);
    mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, colindH.data(),
                               use_diagind ? diagindH : nullptr, 2, 2, rownnz,
                               rowadr, colind, nullptr, rownnzT, rowadrT,
                               colindT, nullptr, data.get());

    // compute values using numeric phase
    std::vector<mjtNum> resH(nnz);
    mju_sqrMatTDSparseNumeric(resH.data(), 2, rownnzH, rowadrH, colindH.data(),
                              use_diagind ? diagindH : nullptr, mat, rownnz,
                              rowadr, colind, matT, rownnzT, rowadrT, colindT,
                              rowsuperT, diag, data.get());

    // compute ground truth using existing mju_sqrMatTDSparse
    // use uncompressed storage to give the old function enough room
    std::vector<mjtNum> res_expected(4);  // 2x2 uncompressed
    std::vector<int> colindH_expected(4);
    int rownnzH_exp[] = {0, 0};
    int rowadrH_exp[] = {0, 2};
    int diagind_exp[] = {0, 0};
    mju_sqrMatTDSparse(res_expected.data(), mat, matT, diag, 2, 2, rownnzH_exp,
                       rowadrH_exp, colindH_expected.data(), rownnz, rowadr,
                       colind, nullptr, rownnzT, rowadrT, colindT, nullptr,
                       data.get(), use_diagind ? diagind_exp : nullptr);

    // check that rownnz matches (nnz may differ due to compressed vs
    // uncompressed storage)
    EXPECT_EQ(rownnzH[0], rownnzH_exp[0])
        << "rownnz[0] mismatch for use_diagind=" << use_diagind;
    EXPECT_EQ(rownnzH[1], rownnzH_exp[1])
        << "rownnz[1] mismatch for use_diagind=" << use_diagind;

    // compare column indices and values for each row
    for (int r = 0; r < 2; r++) {
      for (int j = 0; j < rownnzH[r]; j++) {
        int idx = rowadrH[r] + j;
        int idx_exp = rowadrH_exp[r] + j;
        EXPECT_EQ(colindH[idx], colindH_expected[idx_exp])
            << "colind mismatch at row " << r << " pos " << j
            << " for use_diagind=" << use_diagind;
        EXPECT_NEAR(resH[idx], res_expected[idx_exp], 1e-10)
            << "value mismatch at row " << r << " pos " << j
            << " for use_diagind=" << use_diagind;
      }
    }
  }

  // Test superset sparsity pattern (as in HessianConeFolded where M is merged
  // into H_colind):
  // - col 0 is unconstrained (rownnzT[0] == 0) but present in H_colind
  // - cols 1, 2 form a JT supernode, with an extra off-diagonal (2, 0) in H
  int J_colind[] = {1, 2};
  int J_rownnz[] = {2};
  int J_rowadr[] = {0};
  mjtNum J[] = {2, 3};
  mjtNum D[] = {5};

  mjtNum JT[2];
  int JT_colind[2], JT_rownnz[3], JT_rowadr[3], JT_rowsuper[3];
  mju_transposeSparse(JT, J, 1, 3, JT_rownnz, JT_rowadr, JT_colind, JT_rowsuper,
                      J_rownnz, J_rowadr, J_colind, nullptr);

  int H_rownnz[] = {1, 1, 3};
  int H_rowadr[] = {0, 1, 2};
  int H_colind[] = {0, 1, 0, 1, 2};
  mjtNum H[] = {-99, -99, -99, -99, -99};

  mju_sqrMatTDSparseNumeric(H, 3, H_rownnz, H_rowadr, H_colind, nullptr, J,
                            J_rownnz, J_rowadr, J_colind, JT, JT_rownnz,
                            JT_rowadr, JT_colind, JT_rowsuper, D, data.get());
  EXPECT_THAT(AsVector(H, 5), ElementsAre(0, 20, 0, 30, 45));
}

TEST_F(EngineUtilSparseTest, MjuMulMatTVec) {
  int nr = 2;
  int nc = 3;
  mjtNum mat[] = {1, 2, 0, 0, 3, 4};

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
  mjtNum A[] = {0, 0, 0, 5, 4, 2, 4, 3, 2};

  mju_addToSymSparse(A, mat, 3, rownnz, rowadr, colind, /*flg_upper=*/1);
  EXPECT_THAT(AsVector(A, 9), ElementsAre(1, 2, 4, 7, 7, 2, 8, 3, 7));

  // same as A
  mjtNum B[] = {0, 0, 0, 5, 4, 2, 4, 3, 2};

  mju_addToSymSparse(B, mat, 3, rownnz, rowadr, colind, /*flg_upper=*/0);
  EXPECT_THAT(AsVector(B, 9), ElementsAre(1, 0, 0, 7, 7, 2, 8, 3, 7));
}

TEST_F(EngineUtilSparseTest, MjuMulSymVecSparse) {
  constexpr int n = 4;
  constexpr int nnz = 9;

  mjtNum mat[n * n] = {1, 0, 0, 0, 0, 2, 0, 0, 3, 0, 4, 0, 5, 6, 7, 8};

  // dense, full matrix
  mjtNum sym[n * n] = {1, 0, 3, 5, 0, 2, 0, 6, 3, 0, 4, 7, 5, 6, 7, 8};

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

  for (int i = 0; i < n; i++) {
    EXPECT_EQ(res[i], res2[i]);
  }
}

TEST_F(EngineUtilSparseTest, MjuDenseToSparse) {
  int nr = 2;
  int nc = 2;
  mjtNum mat[] = {1, 2, 0, 3};

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
  const int* chain2_a = nullptr;
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
  const mjtNum mat[nr * nc] = {1, 2, 0, 0, 0, 0, 0, 3, 4, 0,
                               0, 0, 5, 6, 0, 0, 0, 0, 0, 7};

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
  mju_blockDiag(res, mat, nc, nc, nb, perm_r, perm_c, block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res, ElementsAre(1, 2, 0, 0, 0, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 7,
                               0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, BlockDiagPerm) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {1, 2, 0, 0, 0, 0, 0, 3, 4, 0,
                               0, 0, 5, 6, 0, 0, 0, 0, 0, 7};

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
  mju_blockDiag(res, mat_p, nc, nc, nb, perm_r, perm_c, block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res, ElementsAre(1, 2, 0, 0, 0, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 7,
                               0, 0, 0, 0));
}

TEST_F(EngineUtilSparseTest, BlockDiagLessCols) {
  // 4x5 matrix with 3 blocks
  constexpr int nr = 4;
  constexpr int nc = 5;
  const mjtNum mat[nr * nc] = {1, 2, 0, 0, 0, 0, 0, 3, 4, 0,
                               0, 0, 5, 6, 0, 0, 0, 0, 0, 7};

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
  mju_blockDiag(res2, mat_p, nc, nc_res, nb, perm_r, perm_c, block_nr, block_nc,
                block_r, block_c);
  EXPECT_THAT(res2, ElementsAre(1, 2, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0));
}

TEST_F(EngineUtilSparseTest, PermuteMat) {
  const mjtNum mat[] = {1, 2, 0, 0, 0, 0, 3, 4, 0, 0, 5, 6};
  const int perm_r[] = {2, 0, 1};
  const int perm_c[] = {3, 2, 0, 1};
  mjtNum gather[3 * 4];
  PermuteMat(gather, mat, 3, 4, perm_r, perm_c, false, false);
  EXPECT_THAT(gather, ElementsAre(6, 5, 0, 0, 0, 0, 1, 2, 4, 3, 0, 0));
  mjtNum scatter[3 * 4];
  PermuteMat(scatter, gather, 3, 4, perm_r, perm_c, true, true);
  EXPECT_THAT(scatter, ElementsAre(1, 2, 0, 0, 0, 0, 3, 4, 0, 0, 5, 6));
  mjtNum mixed[3 * 4];
  PermuteMat(mixed, mat, 3, 4, perm_r, perm_c, true, false);
  EXPECT_THAT(mixed, ElementsAre(4, 3, 0, 0, 6, 5, 0, 0, 0, 0, 1, 2));
  mjtNum mixed_back[3 * 4];
  PermuteMat(mixed_back, mixed, 3, 4, perm_r, perm_c, false, true);
  EXPECT_THAT(mixed_back, ElementsAre(1, 2, 0, 0, 0, 0, 3, 4, 0, 0, 5, 6));
}

TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseSymbolicEmptyRowAndGuards) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  // 2x3 matrix M with empty column 1: [[1, 0, 2], [0, 0, 3]]
  int colind[] = {0, 2, 2};
  int rownnz[] = {2, 1};
  int rowadr[] = {0, 2};
  int colindT[] = {0, 0, 1};
  int rownnzT[] = {1, 0, 2};
  int rowadrT[] = {0, 1, 1};

  int rownnzH[] = {0, 0, 0};
  int rowadrH[] = {0, 0, 0};
  int diagindH[] = {0, 0, 0};

  int nnz = mju_sqrMatTDSparseSymbolic(
      rownnzH, rowadrH, nullptr, diagindH, 2, 3, rownnz, rowadr, colind,
      nullptr, rownnzT, rowadrT, colindT, nullptr, data.get());
  EXPECT_EQ(nnz, 4);
  EXPECT_THAT(rownnzH, ElementsAre(2, 0, 2));
  EXPECT_THAT(rowadrH, ElementsAre(0, 2, 2));

  std::vector<int> colindH(nnz, -1);
  mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, colindH.data(), diagindH, 2, 3,
                             rownnz, rowadr, colind, nullptr, rownnzT, rowadrT,
                             colindT, nullptr, data.get());
  EXPECT_EQ(diagindH[0], 0);
  EXPECT_EQ(diagindH[1], -1);
  EXPECT_EQ(diagindH[2], 3);

  EXPECT_EQ(
      mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, nullptr, diagindH, 0, 3,
                                 rownnz, rowadr, colind, nullptr, rownnzT,
                                 rowadrT, colindT, nullptr, data.get()),
      0);
  EXPECT_THAT(rownnzH, ElementsAre(0, 0, 0));
  EXPECT_THAT(rowadrH, ElementsAre(0, 0, 0));
  EXPECT_THAT(diagindH, ElementsAre(-1, -1, -1));

  // Fill mode with nr == 0, nc > 0 must preserve caller-provided res_rowadr
  int shifted_rowadrH[] = {0, 2, 5};
  rownnzH[0] = rownnzH[1] = rownnzH[2] = 7;
  diagindH[0] = diagindH[1] = diagindH[2] = 7;
  EXPECT_EQ(mju_sqrMatTDSparseSymbolic(rownnzH, shifted_rowadrH, colindH.data(),
                                       diagindH, 0, 3, rownnz, rowadr, colind,
                                       nullptr, rownnzT, rowadrT, colindT,
                                       nullptr, data.get()),
            0);
  EXPECT_THAT(rownnzH, ElementsAre(0, 0, 0));
  EXPECT_THAT(shifted_rowadrH, ElementsAre(0, 2, 5));
  EXPECT_THAT(diagindH, ElementsAre(-1, -1, -1));

  EXPECT_EQ(
      mju_sqrMatTDSparseSymbolic(rownnzH, rowadrH, nullptr, diagindH, 2, 0,
                                 rownnz, rowadr, colind, nullptr, rownnzT,
                                 rowadrT, colindT, nullptr, data.get()),
      0);
}

TEST_F(EngineUtilSparseTest, MjuSortSparseRowAndPermuteLowerCSR) {
  int colind[] = {4, 1, 3, 0, 2};
  mjtNum val[] = {40, 10, 30, 0, 20};
  mju_sortSparseRow(5, colind, val);
  EXPECT_THAT(colind, ElementsAre(0, 1, 2, 3, 4));
  EXPECT_THAT(val, ElementsAre(0, 10, 20, 30, 40));

  // symmetric 3x3 matrix in lower-triangular CSR:
  // [10,  2,  3]
  // [ 2, 20,  4]
  // [ 3,  4, 30]
  int src_rownnz[] = {1, 2, 3};
  int src_rowadr[] = {0, 1, 3};
  int src_colind[] = {0, 0, 1, 0, 1, 2};
  mjtNum src_val[] = {10, 2, 20, 3, 4, 30};

  // permute old -> new: 0 -> 2, 1 -> 0, 2 -> 1
  int perm[] = {2, 0, 1};
  int dst_rownnz[3] = {0};
  int dst_rowadr[3] = {0};
  int dst_colind[6] = {0};
  mjtNum dst_val[6] = {0};
  mju_permuteLowerCSR(3, perm, dst_rownnz, dst_rowadr, dst_colind, dst_val,
                      src_rownnz, src_rowadr, src_colind, src_val);

  mjtNum dense_src[9] = {0};
  mjtNum dense_dst[9] = {0};
  mju_sym2dense(dense_src, src_val, 3, src_rownnz, src_rowadr, src_colind);
  mju_sym2dense(dense_dst, dst_val, 3, dst_rownnz, dst_rowadr, dst_colind);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      EXPECT_EQ(dense_dst[perm[i] * 3 + perm[j]], dense_src[i * 3 + j]);
    }
  }
}

TEST_F(EngineUtilSparseTest, MjuMinDegreeGrid) {
  // 8x8 grid graph (nb = 64, words = 1) with uniform block size 3
  constexpr int side = 8;
  constexpr int nb = side * side;
  constexpr int words = (nb + 63) / 64;
  std::vector<uint64_t> adj(nb * words, 0);
  std::vector<int> bsz(nb, 3);
  for (int r = 0; r < side; r++) {
    for (int c = 0; c < side; c++) {
      int u = r * side + c;
      auto connect = [&](int v) {
        adj[u * words + (v >> 6)] |= 1ULL << (v & 63);
        adj[v * words + (u >> 6)] |= 1ULL << (u & 63);
      };
      if (r + 1 < side) connect((r + 1) * side + c);
      if (c + 1 < side) connect(r * side + (c + 1));
    }
  }

  std::vector<int> perm(nb, -1);
  std::vector<int> scratch(5 * nb + words, 0);
  std::vector<uint64_t> nz_bits(words, 0);
  size_t nL_perm = 0;
  mjtSize Lflops_perm = 0;
  int ok = mju_minDegree(perm.data(), adj.data(), nb, bsz.data(), &nL_perm,
                         &Lflops_perm, /*Lflops_max=*/1000000, scratch.data(),
                         nz_bits.data());
  EXPECT_EQ(ok, 1);
  EXPECT_GT(nL_perm, 0u);
  EXPECT_GT(Lflops_perm, 0u);

  std::vector<int> seen(nb, 0);
  for (int i = 0; i < nb; i++) {
    ASSERT_GE(perm[i], 0);
    ASSERT_LT(perm[i], nb);
    seen[perm[i]]++;
  }
  for (int i = 0; i < nb; i++) {
    EXPECT_EQ(seen[i], 1);
  }
}

// shuffled rows longer than 24 entries take the gapped pass of the sort
TEST_F(EngineUtilSparseTest, MjuSortSparseRowLong) {
  constexpr int nnz = 40;
  int colind[nnz];
  mjtNum val[nnz];
  for (int i = 0; i < nnz; i++) {
    colind[i] = (i * 17) % nnz;  // 17 is coprime with 40: a permutation
    val[i] = 10 * colind[i];
  }
  mju_sortSparseRow(nnz, colind, val);
  for (int i = 0; i < nnz; i++) {
    EXPECT_EQ(colind[i], i);
    EXPECT_EQ(val[i], 10 * i);
  }
}

// grid graph over several bitset words (nb = 144, words = 3): a complete
// permutation without a flop cap, and an early abort under a tight one
TEST_F(EngineUtilSparseTest, MjuMinDegreeMultiWordAndAbort) {
  constexpr int side = 12;
  constexpr int nb = side * side;
  constexpr int words = (nb + 63) / 64;
  auto grid = [&]() {
    std::vector<uint64_t> adj(nb * words, 0);
    for (int r = 0; r < side; r++) {
      for (int c = 0; c < side; c++) {
        int u = r * side + c;
        auto connect = [&](int v) {
          adj[u * words + (v >> 6)] |= 1ULL << (v & 63);
          adj[v * words + (u >> 6)] |= 1ULL << (u & 63);
        };
        if (r + 1 < side) connect((r + 1) * side + c);
        if (c + 1 < side) connect(r * side + (c + 1));
      }
    }
    return adj;
  };
  std::vector<int> bsz(nb, 3);
  std::vector<int> perm(nb, -1);
  std::vector<int> scratch(5 * nb + words, 0);
  std::vector<uint64_t> nz_bits(words, 0);

  // no cap: a permutation of the blocks
  std::vector<uint64_t> adj = grid();
  size_t nL = 0;
  mjtSize flops = 0;
  ASSERT_EQ(mju_minDegree(perm.data(), adj.data(), nb, bsz.data(), &nL, &flops,
                          /*max_flops=*/0, scratch.data(), nz_bits.data()),
            1);
  EXPECT_GT(nL, 0u);
  EXPECT_GT(flops, 0u);
  std::vector<int> seen(nb, 0);
  for (int i = 0; i < nb; i++) {
    ASSERT_GE(perm[i], 0);
    ASSERT_LT(perm[i], nb);
    seen[perm[i]]++;
  }
  for (int i = 0; i < nb; i++) {
    EXPECT_EQ(seen[i], 1);
  }

  // cap at half the flops: aborts once the count reaches 4/5 of the cap
  adj = grid();
  size_t nL_cap = 0;
  mjtSize flops_cap = 0;
  EXPECT_EQ(mju_minDegree(perm.data(), adj.data(), nb, bsz.data(), &nL_cap,
                          &flops_cap, /*max_flops=*/flops / 2, scratch.data(),
                          nz_bits.data()),
            0);
  EXPECT_GE(5 * flops_cap, 4 * (flops / 2));
  EXPECT_LT(flops_cap, flops);
  EXPECT_LT(nL_cap, nL);
}

// Verify mju_sqrMatTDSparseSymbolic with supernodes larger than mjMAXSUPER (8),
// both non-zero and all-zero colsuper, and both lower-only and full (diagind).
TEST_F(EngineUtilSparseTest, MjuSqrMatTDSparseSymbolicLargeSupernodes) {
  MjModelPtr model = LoadModelFromString("<mujoco/>");
  MjDataPtr data = MakeData(model);

  // build M (nr = 25, nc = 30) with column supernodes of sizes {12, 1, 10, 7}
  // and row supernodes of sizes {10, 6, 9}, plus an empty column supernode
  const int nr = 25, nc = 30;
  std::vector<int> row_blocks = {10, 6, 9};
  std::vector<int> col_blocks = {12, 1, 10, 7};
  // block pattern (3 x 4): col block 1 is empty
  const int pat[3][4] = {{1, 0, 1, 0}, {1, 0, 0, 1}, {0, 0, 1, 1}};

  std::vector<int> rownnz(nr, 0), rowadr(nr, 0), colind;
  int r0 = 0;
  for (int rb = 0; rb < 3; rb++) {
    for (int ra = 0; ra < row_blocks[rb]; ra++) {
      int r = r0 + ra;
      rowadr[r] = colind.size();
      int c0 = 0;
      for (int cb = 0; cb < 4; cb++) {
        if (pat[rb][cb]) {
          for (int ca = 0; ca < col_blocks[cb]; ca++) {
            colind.push_back(c0 + ca);
          }
        }
        c0 += col_blocks[cb];
      }
      rownnz[r] = static_cast<int>(colind.size()) - rowadr[r];
    }
    r0 += row_blocks[rb];
  }

  std::vector<int> rownnzT(nc), rowadrT(nc), colindT(colind.size());
  std::vector<int> rowsuper(nr), rowsuperT(nc), zero_super(nc, 0);
  mju_superSparse(nr, rowsuper.data(), rownnz.data(), rowadr.data(),
                  colind.data());
  mju_transposeSparse(nullptr, nullptr, nr, nc, rownnzT.data(), rowadrT.data(),
                      colindT.data(), rowsuperT.data(), rownnz.data(),
                      rowadr.data(), colind.data(), rowsuper.data());
  EXPECT_EQ(rowsuperT[0], 11);  // 12-wide supernode (> mjMAXSUPER)
  EXPECT_EQ(rowsuper[0], 9);    // 10-wide row supernode

  for (int use_upper = 0; use_upper <= 1; use_upper++) {
    // reference: no supernodes
    std::vector<int> ref_nnz(nc), ref_adr(nc), ref_diag(nc);
    int nnz_ref = mju_sqrMatTDSparseSymbolic(
        ref_nnz.data(), ref_adr.data(), nullptr,
        use_upper ? ref_diag.data() : nullptr, nr, nc, rownnz.data(),
        rowadr.data(), colind.data(), nullptr, rownnzT.data(), rowadrT.data(),
        colindT.data(), nullptr, data.get());
    std::vector<int> ref_col(nnz_ref, -1);
    mju_sqrMatTDSparseSymbolic(ref_nnz.data(), ref_adr.data(), ref_col.data(),
                               use_upper ? ref_diag.data() : nullptr, nr, nc,
                               rownnz.data(), rowadr.data(), colind.data(),
                               nullptr, rownnzT.data(), rowadrT.data(),
                               colindT.data(), nullptr, data.get());

    // test both full supernodes and all-zero colsuper (fast-path drop)
    for (const int* cs : {rowsuperT.data(), zero_super.data()}) {
      std::vector<int> got_nnz(nc), got_adr(nc), got_diag(nc);
      int nnz_got = mju_sqrMatTDSparseSymbolic(
          got_nnz.data(), got_adr.data(), nullptr,
          use_upper ? got_diag.data() : nullptr, nr, nc, rownnz.data(),
          rowadr.data(), colind.data(), rowsuper.data(), rownnzT.data(),
          rowadrT.data(), colindT.data(), cs, data.get());
      ASSERT_EQ(nnz_got, nnz_ref);
      EXPECT_EQ(got_nnz, ref_nnz);
      EXPECT_EQ(got_adr, ref_adr);

      std::vector<int> got_col(nnz_got, -1);
      mju_sqrMatTDSparseSymbolic(
          got_nnz.data(), got_adr.data(), got_col.data(),
          use_upper ? got_diag.data() : nullptr, nr, nc, rownnz.data(),
          rowadr.data(), colind.data(), rowsuper.data(), rownnzT.data(),
          rowadrT.data(), colindT.data(), cs, data.get());
      EXPECT_EQ(got_col, ref_col);
      if (use_upper) {
        EXPECT_EQ(got_diag, ref_diag);
      }
    }
  }
}

TEST_F(EngineUtilSparseTest, MjuAddToSclSparseIncParity) {
  auto ref_add_inc = [](mjtNum* dst, const mjtNum* src, int nnzdst,
                        const int* inddst, int nnzsrc, const int* indsrc,
                        mjtNum scl) {
    if (nnzdst <= 0 || nnzsrc <= 0) return;
    int adrs = 0, adrd = 0;
    while (adrs < nnzsrc && adrd < nnzdst) {
      if (indsrc[adrs] == inddst[adrd]) {
        dst[adrd] += scl * src[adrs];
        adrs++;
        adrd++;
      } else if (indsrc[adrs] < inddst[adrd]) {
        adrs++;
      } else {
        adrd++;
      }
    }
  };

  auto check_case = [&](const std::vector<int>& inddst,
                        const std::vector<int>& indsrc, mjtNum scl) {
    const int nd = static_cast<int>(inddst.size());
    const int ns = static_cast<int>(indsrc.size());
    std::vector<mjtNum> dst_ref(nd), src(ns);
    for (int i = 0; i < nd; ++i) {
      dst_ref[i] = mjtNum(1.5) / (i + 2) - mjtNum(0.3) * (i % 3);
    }
    for (int i = 0; i < ns; ++i) {
      src[i] = mjtNum(2.25) / (i + 3) + mjtNum(-0.7) * (i & 1);
    }
    std::vector<mjtNum> dst_got = dst_ref;

    ref_add_inc(dst_ref.data(), src.data(), nd, inddst.data(), ns,
                indsrc.data(), scl);
    mju_addToSclSparseInc(dst_got.data(), src.data(), nd, inddst.data(), ns,
                          indsrc.data(), scl);
    if (nd > 0) {
      EXPECT_EQ(
          std::memcmp(dst_got.data(), dst_ref.data(), nd * sizeof(mjtNum)), 0)
          << "nd=" << nd << " ns=" << ns << " scl=" << scl;
    }
  };

  // empty and zero-scale cases (including -0.0)
  check_case({}, {0, 1, 2}, mjtNum(2.5));
  check_case({0, 1, 2}, {}, mjtNum(2.5));
  check_case({0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}, mjtNum(0));
  check_case({0, 1, 2, 3, 4, 5}, {0, 1, 2, 3, 4, 5}, mjtNum(-0.0));

  // negative nnz guard
  mju_addToSclSparseInc(nullptr, nullptr, -1, nullptr, 3, nullptr, mjtNum(1.0));
  mju_addToSclSparseInc(nullptr, nullptr, 3, nullptr, -1, nullptr, mjtNum(1.0));

  // disjoint, identical, subset, and partial overlap
  check_case({10, 11, 12, 13}, {0, 1, 2, 3}, mjtNum(-1.5));
  check_case({5, 6, 7, 8, 9, 10}, {5, 6, 7, 8, 9, 10}, mjtNum(-0.75));
  check_case({4, 5, 6, 7, 8, 9, 10, 11}, {4, 6, 7, 10, 11}, mjtNum(1.75));
  check_case({0, 2, 5, 7, 10}, {0, 3, 5, 8, 10}, mjtNum(-0.625));
  check_case({1, 4, 6, 9, 12, 15}, {0, 4, 5, 9, 11, 15, 20}, mjtNum(3.125));
}

}  // namespace
}  // namespace mujoco
