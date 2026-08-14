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

#include "tuple_tools.h"

#include <tuple>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace mujoco::util {
namespace {

TEST(TupleToolsTest, Slice) {
  auto tuple = std::make_tuple(2, 3, 5, 7, 11, 13);
  EXPECT_EQ((tuple_slice<1, 5>(tuple)), std::make_tuple(3, 5, 7, 11));
  EXPECT_EQ((tuple_slice<2, 3>(tuple)), std::make_tuple(5));

  // negative indices
  EXPECT_EQ((tuple_slice<1, -2>(tuple)), std::make_tuple(3, 5, 7));
  EXPECT_EQ((tuple_slice<-4, 6>(tuple)), std::make_tuple(5, 7, 11, 13));
  EXPECT_EQ((tuple_slice<-3, -1>(tuple)), std::make_tuple(7, 11));

  // empty slices
  EXPECT_EQ((tuple_slice<0, 0>(tuple)), std::make_tuple());
  EXPECT_EQ((tuple_slice<3, 3>(tuple)), std::make_tuple());
  EXPECT_EQ((tuple_slice<-2, -2>(tuple)), std::make_tuple());

  // specify void as the End argument
  EXPECT_EQ((tuple_slice<2, void>(tuple)), std::make_tuple(5, 7, 11, 13));
  EXPECT_EQ((tuple_slice<-2, void>(tuple)), std::make_tuple(11, 13));

  // omit the End argument
  EXPECT_EQ((tuple_slice<2>(tuple)), std::make_tuple(5, 7, 11, 13));
  EXPECT_EQ((tuple_slice<-2>(tuple)), std::make_tuple(11, 13));

  // empty input tuples
  EXPECT_EQ((tuple_slice<0>(std::make_tuple())), std::make_tuple());
  EXPECT_EQ((tuple_slice<0, 0>(std::make_tuple())), std::make_tuple());
}

}  // namespace
}  // namespace mujoco::util
