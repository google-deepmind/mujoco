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

#include "func_wrap.h"

#include <string>
#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <Eigen/Eigen>
#include "func_traits.h"

namespace {
struct BoxedDouble {
  double value;
};
struct BoxedInt {
  int value;
};
}  // namespace

namespace mujoco::util {
template <> struct wrapped<BoxedDouble> {
  static BoxedDouble unwrap(const std::string& wrapped) {
    return BoxedDouble{std::stod(wrapped)};
  }
};
template <> struct wrapped<BoxedInt> {
  static BoxedInt unwrap(int wrapped) {
    return BoxedInt{wrapped};
  }
};
template <typename T, int N> struct wrapped<T(*)[N]> {
  using Array = T[N];
  static Array* unwrap(Eigen::Ref<Eigen::Vector<T, N>> wrapped) {
    return reinterpret_cast<Array*>(wrapped.data());
  }
};
template <typename T, int N> struct wrapped<const T(*)[N]> {
  using Array = const T[N];
  static Array* unwrap(const Eigen::Vector<T, N>& wrapped) {
    return reinterpret_cast<Array*>(wrapped.data());
  }
};
}  // namespace mujoco::util

namespace {
using ::mujoco::util::func_arg_t;
using ::mujoco::util::UnwrapArgs;
using ::mujoco::util::ReturnArrayArg0;

double add(BoxedDouble x, float y, BoxedInt z) {
  return x.value + y + z.value;
}

void add_array4(double (*out)[4], const double (*x)[4], const double (*y)[4]) {
  for (int i = 0; i < 4; ++i) {
    (*out)[i] = (*x)[i] + (*y)[i];
  }
}

TEST(FuncWrapTest, UnwrapArgs) {
  {
    auto wrapped_add = UnwrapArgs(add);
    static_assert(std::is_same_v<
        func_arg_t<decltype(wrapped_add), 0>, const std::string&
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(wrapped_add), 1>, float
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(wrapped_add), 2>, int
    >);
    // Use binary powers so that we can do exact floating point comparison.
    EXPECT_EQ(wrapped_add("1.6e+1", 5e-1, 2), 18.5);
  }
  {
    Eigen::Vector4d out;
    UnwrapArgs(add_array4)(out, {1, 3, 5, 7}, {2, 6, 9, 11});
    EXPECT_THAT(out, ::testing::ElementsAre(3, 9, 14, 18));
  }
}

TEST(FuncWrapTest, ReturnArrayArg0) {
  auto out = UnwrapArgs(ReturnArrayArg0(add_array4))({1, 3, 5, 7},
                                                     {2, 6, 9, 11});
  static_assert(std::is_same_v<decltype(out), Eigen::Vector4d>);
  EXPECT_THAT(out, ::testing::ElementsAre(3, 9, 14, 18));
}
}  // namespace
