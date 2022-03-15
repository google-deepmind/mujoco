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

#include "func_traits.h"

#include <functional>
#include <type_traits>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace mujoco::util {
namespace {

TEST(FuncTraitsTest, IsCallable) {
  EXPECT_FALSE(is_callable_v<int>);
  EXPECT_TRUE(is_callable_v<double(double)>);
  EXPECT_TRUE(is_callable_v<void(*)(void)>);
  EXPECT_TRUE((is_callable_v<int(&)(int, float)>));
  EXPECT_TRUE(is_callable_v<std::function<void(void)>>);
  {
    auto lambda = [](){};
    EXPECT_TRUE(is_callable_v<decltype(lambda)>);
  }
  {
    auto mutable_lambda = []() mutable {};
    EXPECT_TRUE(is_callable_v<decltype(mutable_lambda)>);
  }
  {
    struct Functor { void operator()() {} };
    EXPECT_TRUE(is_callable_v<Functor>);
  }
  {
    struct ConstFunctor { void operator()() const {} };
    EXPECT_TRUE(is_callable_v<ConstFunctor>);
  }
  {
    struct NotCallable { void Foo() {} };
    EXPECT_FALSE(is_callable_v<NotCallable>);
    EXPECT_FALSE(is_callable_v<decltype(&NotCallable::Foo)>);
  }
}

TEST(FuncTraitsTest, FuncArgType) {
  static_assert(std::is_same_v<
      func_arg_t<void()>,
      void
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int)>,
      int
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&)>,
      int&&
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&), 0>,
      int&&
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&), 1>,
      char&
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&), 2>,
      const float&
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&), 3>,
      void
  >);
  static_assert(std::is_same_v<
      func_arg_t<bool(int&&, char&, const float&), 7>,
      void
  >);

  {
    auto lambda = [](bool, double&, float&&){};
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda)>,
        bool
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda), 0>,
        bool
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda), 1>,
        double&
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda), 2>,
        float&&
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda), 3>,
        void
    >);
    static_assert(std::is_same_v<
        func_arg_t<decltype(lambda), 10>,
        void
    >);
  }
  {
    struct Functor { void operator()(char, void*) {} };
    static_assert(std::is_same_v<
        func_arg_t<Functor>,
        char
    >);
    static_assert(std::is_same_v<
        func_arg_t<Functor, 0>,
        char
    >);
    static_assert(std::is_same_v<
        func_arg_t<Functor, 1>,
        void*
    >);
    static_assert(std::is_same_v<
        func_arg_t<Functor, 2>,
        void
    >);
    static_assert(std::is_same_v<
        func_arg_t<Functor, 5>,
        void
    >);
  }
}

TEST(FuncTraitsTest, FuncArgCount) {
  EXPECT_EQ(func_arg_count_v<void()>, 0);
  EXPECT_EQ(func_arg_count_v<bool(int)>, 1);
  EXPECT_EQ(func_arg_count_v<bool(int&&, char&, const float&)>, 3);
  {
    auto lambda = [](bool, double&, float&&){};
    EXPECT_EQ(func_arg_count_v<decltype(lambda)>, 3);
  }
  {
    struct Functor { void operator()(char, void*) {} };
    EXPECT_EQ(func_arg_count_v<Functor>, 2);
  }
}

}  // namespace
}  // namespace mujoco::util
