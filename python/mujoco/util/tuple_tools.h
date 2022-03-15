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

#ifndef MUJOCO_PYTHON_UTIL_TUPLE_TOOLS_H_
#define MUJOCO_PYTHON_UTIL_TUPLE_TOOLS_H_

#include <optional>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>

#include "crossplatform.h"

namespace mujoco::util {

// Forward declaration so that the public interface appears at the top of file.
namespace _impl {
template <int N> struct head_slicer;
template <int N> struct tail_slicer;
}  // namespace _impl

// Removes the first `Begin` elements from the from a tuple.
// If `Begin` is negative, the resulting tuple is obtained by removing the first
// `N - Begin` elements, where N is the size of the input tuple.
//
// For consistency with the <Begin, End> form (see below), the second optional
// template argument can be explicitly spelled out as void,
// e.g. tuple_slice<2, void>(tuple), to indicate that the slice takes every
// element up to the end of the tuple.
//
// This function has the same semantic as Python's list slicing x[Begin:].
template <int Begin, typename End = void, typename Tuple>
MUJOCO_ALWAYS_INLINE
constexpr auto tuple_slice(Tuple&& tuple) {
  static_assert(
      std::is_void_v<End>,
      "argument End must be either omitted or explicitly specified as void");
  constexpr int Size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
  static_assert(Begin >= -Size && Begin <= Size);
  constexpr int NHead = (Begin >= 0) ? Begin : (Size + Begin);
  return std::apply(_impl::head_slicer<NHead>(), std::forward<Tuple>(tuple));
}

// Extracts a contiguous slice of elements from a tuple so that the resulting
// tuple begins with the element at index `Begin` and ends with the element
// immediately before the one at index `End`. A negative value of `Begin` or
// `End` is interpreted as indexing an element from the end of the input tuple,
// where -1 refers to the last element.
//
// This function has the same semantic as Python's list slicing x[Begin:End].
template <int Begin, int End, typename Tuple>
MUJOCO_ALWAYS_INLINE
constexpr auto tuple_slice(Tuple&& tuple) {
  constexpr int Size = std::tuple_size_v<std::remove_reference_t<Tuple>>;
  static_assert(End >= -Size && End <= Size);
  constexpr int NTail = (End >= 0) ? (Size - End) : (-End);
  static_assert(
      (Begin >= -Size && Begin <= -NTail) ||
      (Begin >= 0 && Begin <= Size - NTail),
      "Begin should refer to an element that comes before End");
  constexpr int NHead = (Begin >= 0) ? Begin : (Size + Begin);
  return tuple_slice<NHead>(
      std::apply(_impl::tail_slicer<NTail>(), std::forward<Tuple>(tuple)));
}

// Compile-time function to check whether a string occurs in a tuple.
// Should ideally be declared consteval if we switch to C++20.
template <typename Str, typename Tuple>
static constexpr bool string_is_in_tuple(Str str, Tuple&& tuple) {
  if constexpr (std::tuple_size_v<std::remove_reference_t<Tuple>> == 0) {
    return false;
  } else if (std::string_view(str) == std::string_view(std::get<0>(tuple))) {
    return true;
  } else {
    return string_is_in_tuple(str, util::tuple_slice<1, void>(tuple));
  }
}

// Compile-time function to check whether the elements of one tuple is a subset
// another. Should ideally be declared consteval if we switch to C++20.
template <typename Tuple1, typename Tuple2>
constexpr bool is_subset_strings(Tuple1 tuple1, Tuple2 tuple2) {
  if constexpr (std::tuple_size_v<Tuple1> == 0) {
    return true;
  } else if (string_is_in_tuple(std::get<0>(tuple1), tuple2)) {
    return is_subset_strings(util::tuple_slice<1, void>(tuple1), tuple2);
  } else {
    return false;
  }
}

// =====================================================================
// IMPLEMENTATION DETAIL. FOR INTERNAL USE WITHIN THIS HEADER FILE ONLY.
// =====================================================================
namespace _impl{
template <int N>
struct head_slicer {
  template <typename T, typename... U>
  MUJOCO_ALWAYS_INLINE
  constexpr auto operator()(T&& t, U&&... u) const {
    return head_slicer<N-1>()(std::forward<U>(u)...);
  }
};

template <>
struct head_slicer<0> {
  template <typename... T>
  MUJOCO_ALWAYS_INLINE
  constexpr auto operator()(T&&... t) const {
    return std::forward_as_tuple(t...);
  }
};

template <int N>
struct move_head_to_tail {
  template <typename T, typename... U>
  MUJOCO_ALWAYS_INLINE
  static constexpr auto move(T&& t, U&&... u) {
    return move_head_to_tail<N-1>::move(
        std::forward<U>(u)..., std::forward<T>(t));
  }
};

template <>
struct move_head_to_tail<0> {
  template <typename... T>
  MUJOCO_ALWAYS_INLINE
  static constexpr auto move(T&&... t) {
    return std::forward_as_tuple(t...);
  }
};

template <int N>
struct tail_slicer {
  template <typename... T>
  MUJOCO_ALWAYS_INLINE
  constexpr auto operator()(T&&... t) const {
    constexpr int Size = std::tuple_size_v<std::tuple<T...>>;
    constexpr int NHead = Size - N;
    return std::apply(head_slicer<N>(),
        move_head_to_tail<NHead>::move(std::forward<T>(t)...));
  }
};
}  // namespace _impl
}  // namespace mujoco::util

#endif  // MUJOCO_PYTHON_UTIL_TUPLE_TOOLS_H_
