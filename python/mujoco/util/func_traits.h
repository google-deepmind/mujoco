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

#ifndef MUJOCO_PYTHON_FUNC_TRAITS_H_
#define MUJOCO_PYTHON_FUNC_TRAITS_H_

#include <tuple>
#include <type_traits>

namespace mujoco::util {

// Forward declaration so that the public interface appears at the top of file.
namespace _impl {
template <typename, typename = void> struct is_callable;
template <typename, int, bool = false> struct func_arg;
}  // namespace _impl

// True if T is callable, i.e. if T is either a function pointer/reference or
// is an instance of a type with operator().
template <typename T>
static constexpr bool is_callable_v =
    _impl::is_callable<std::remove_reference_t<T>>::value;

// Type of the Nth argument of a function or functor, where N=0 refers to the
// first argument. If N exceeds the number arguments for F then
// func_arg_t<F, N> is void.
template <typename F, int N = 0>
using func_arg_t = typename _impl::func_arg<F, N, (N > 0)>::type;

template <typename F>
static constexpr int func_arg_count_v = _impl::func_arg<F, 0>::count;

// =====================================================================
// IMPLEMENTATION DETAIL. FOR INTERNAL USE WITHIN THIS HEADER FILE ONLY.
// =====================================================================
namespace _impl {
template <typename T, typename>
struct is_callable {
  static constexpr bool value = false;
};

template <typename T>
struct is_callable<T, std::void_t<decltype(&T::operator())>> {
  static constexpr bool value = true;
};

template <typename Return, typename... Args>
struct is_callable<Return(Args...)> {
  static constexpr bool value = true;
};

template <typename Return, typename... Args>
struct is_callable<Return (*)(Args...)> {
  static constexpr bool value = true;
};

// Support functors by looking at its member function Func::operator().
template <typename Func, int N, bool Recursing>
struct func_arg {
  using call = decltype(
      &std::remove_const_t<std::remove_reference_t<Func>>::operator());
  using type = typename func_arg<call, N>::type;
  static constexpr int count = func_arg<call, N>::count;
};

// Base case (N == 0) for function: resolve to Arg0.
template <typename Ret, typename Arg0, typename... Args>
struct func_arg<Ret(Arg0, Args...), 0> {
  using type = Arg0;
  static constexpr int count = 1 + std::tuple_size_v<std::tuple<Args...>>;
};

// Recursive case (N > 0) for function: discard Arg0 it and resolve to N-1.
template <typename Ret, int N, typename Arg0, typename... Args>
struct func_arg<Ret(Arg0, Args...), N, true> {
  using type = typename func_arg<Ret(Args...), N - 1, (N > 1)>::type;
  static constexpr int count = 1 + std::tuple_size_v<std::tuple<Args...>>;
};

// Specialization for non-const member functions.
template <typename C, typename Ret, int N, typename... Args>
struct func_arg<Ret (C::*)(Args...), N> {
  using type = typename func_arg<Ret(Args...), N, (N > 0)>::type;
  static constexpr int count = std::tuple_size_v<std::tuple<Args...>>;
};

// Specialization for const member functions (matches lambda::operator()).
template <typename C, typename Ret, int N, typename... Args>
struct func_arg<Ret (C::*)(Args...) const, N> {
  using type = typename func_arg<Ret(Args...), N, (N > 0)>::type;
  static constexpr int count = std::tuple_size_v<std::tuple<Args...>>;
};

// Functions with no argument: always resolve to void.
template <typename Ret, int N, bool Recursing>
struct func_arg<Ret(), N, Recursing> {
  using type = void;
  static constexpr int count = 0;
};
}  // namespace _impl
}  // namespace mujoco::util

#endif  // MUJOCO_PYTHON_FUNC_TRAITS_H_
