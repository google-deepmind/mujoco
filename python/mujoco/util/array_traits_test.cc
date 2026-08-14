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

#include "array_traits.h"

#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace mujoco::util {
namespace {

using ::testing::ElementsAre;

TEST(ArrayTraitsTest, CArrayType) {
  struct Foo {};
  static_assert(std::is_same_v<c_array_t<int, 3, 4, 5>,
                               int[3][4][5]>);
  static_assert(std::is_same_v<c_array_t<double>,
                               double>);
  static_assert(std::is_same_v<c_array_t<Foo, 7>,
                               Foo[7]>);
  static_assert(std::is_same_v<c_array_t<Foo*, 3, 4, 5, 6>,
                               Foo*[3][4][5][6]>);
}

TEST(ArrayTraitsTest, ArrayNdim) {
  struct Foo {};
  EXPECT_EQ(array_ndim_v<int[3][4][5]>, 3);
  EXPECT_EQ(array_ndim_v<double(*)[3][4]>, 2);
  EXPECT_EQ(array_ndim_v<Foo*[3][4]>, 2);
  EXPECT_EQ(array_ndim_v<Foo(&)[3][4][5][6]>, 4);
}

TEST(ArrayTraitsTest, ArrayScalarType) {
  struct Foo {};
  static_assert(std::is_same_v<
      array_scalar_t<int[3][4][5]>,
      int
  >);
  static_assert(std::is_same_v<
      array_scalar_t<double(*)[3][4]>,
      double
  >);
  static_assert(std::is_same_v<
      array_scalar_t<Foo*[3][4]>,
      Foo*
  >);
  static_assert(std::is_same_v<
      array_scalar_t<Foo(&)[3][4][5][6]>,
      Foo
  >);
}

TEST(ArrayTraitsTest, MakeEigen) {
  {
    auto eigen = MakeEigen<float[3]>();
    static_assert(std::is_same_v<
        decltype(eigen),
        Eigen::Vector3f
    >);
  }
  {
    auto eigen = MakeEigen<int[2][3]>();
    static_assert(std::is_same_v<
        decltype(eigen),
        Eigen::Matrix<int, 2, 3, Eigen::RowMajor>
    >);
  }
  {
    auto eigen = MakeEigen<double[2][3][4]>();
    static_assert(std::is_same_v<
        decltype(eigen),
        Eigen::Tensor<double, 3, Eigen::RowMajor, Eigen::DenseIndex>
    >);
    EXPECT_THAT(eigen.dimensions(), ElementsAre(2, 3, 4));
  }
}



}  // namespace
}  // namespace mujoco::util
