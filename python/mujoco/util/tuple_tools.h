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

#include <string_view>
#include <tuple>

namespace mujoco::util {

// Compile-time function to check whether a string occurs in a tuple.
// Should ideally be declared consteval if we switch to C++20.
template <typename Str, typename Tuple>
constexpr bool string_is_in_tuple(Str str, const Tuple& tuple) {
  return std::apply(
      [&](const auto&... elem) {
        return ((std::string_view(str) == std::string_view(elem)) || ...);
      },
      tuple);
}

// Compile-time function to check whether the elements of one tuple is a subset
// another. Should ideally be declared consteval if we switch to C++20.
template <typename Tuple1, typename Tuple2>
constexpr bool is_subset_strings(const Tuple1& tuple1, const Tuple2& tuple2) {
  return std::apply(
      [&](const auto&... elem) {
        return (string_is_in_tuple(elem, tuple2) && ...);
      },
      tuple1);
}

}  // namespace mujoco::util

#endif  // MUJOCO_PYTHON_UTIL_TUPLE_TOOLS_H_
