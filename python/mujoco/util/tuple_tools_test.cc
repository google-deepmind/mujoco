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

#include <gtest/gtest.h>

namespace mujoco::util {
namespace {

TEST(TupleToolsTest, StringIsInTuple) {
  constexpr auto names = std::make_tuple("res", "n", "vec");
  static_assert(string_is_in_tuple("n", names));
  static_assert(string_is_in_tuple("vec", names));
  static_assert(!string_is_in_tuple("ve", names));
  static_assert(!string_is_in_tuple("n", std::make_tuple()));
  EXPECT_TRUE(string_is_in_tuple("res", names));
  EXPECT_FALSE(string_is_in_tuple("mat", names));
}

TEST(TupleToolsTest, IsSubsetStrings) {
  constexpr auto names = std::make_tuple("res", "n", "vec");
  static_assert(is_subset_strings(std::make_tuple(), names));
  static_assert(is_subset_strings(std::make_tuple("n"), names));
  static_assert(is_subset_strings(std::make_tuple("vec", "res"), names));
  static_assert(!is_subset_strings(std::make_tuple("n", "mat"), names));
  static_assert(!is_subset_strings(std::make_tuple("n"), std::make_tuple()));
  EXPECT_TRUE(is_subset_strings(names, names));
  EXPECT_FALSE(is_subset_strings(std::make_tuple("mat"), names));
}

}  // namespace
}  // namespace mujoco::util
