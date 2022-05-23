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

#ifndef MUJOCO_PYTHON_FUNCTIONS_H_
#define MUJOCO_PYTHON_FUNCTIONS_H_

#include <array>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <Eigen/Core>
#include <mujoco/mujoco.h>
#include "errors.h"
#include "structs.h"
#include "util/array_traits.h"
#include "util/crossplatform.h"
#include "util/func_wrap.h"
#include "util/tuple_tools.h"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

// Performs a compile-time check that the omitted argument name list is a
// subset of the underlying function parameter names, and returns a helper that
// defines a pybind11 function whose argument names obtained by removing
// __VA_ARGS__ from the param_names tuple in the given MjTraits.
// (This has to be implemented as a a macro because we cannot perform a
// constexpr comparison of a tuple that is passed as a function argument.)
#define DEF_WITH_OMITTED_PY_ARGS(MJTRAITS, ...)                                \
  static_assert(                                                               \
      util::is_subset_strings(std::make_tuple(__VA_ARGS__),                    \
                              MJTRAITS::param_names),                          \
      "omitted argument names is not a subset of function parameter names");   \
  DefWithOmittedPyArgsImpl<MJTRAITS, decltype(std::make_tuple(__VA_ARGS__))> { \
    std::make_tuple(__VA_ARGS__)                                               \
  }

namespace mujoco::util {
template <typename RawMj>
struct wrapped<RawMj*, python::enable_if_mj_struct_t<RawMj>> {
  MUJOCO_ALWAYS_INLINE
  static constexpr RawMj* unwrap(python::MjWrapper<RawMj>& wrapper) {
    return wrapper.get();
  }
};

// We use std::optional on pointer arguments to indicate that Python callers
// can pass None.
template <typename RawMj>
struct wrapped<std::optional<RawMj*>, python::enable_if_mj_struct_t<RawMj>> {
  MUJOCO_ALWAYS_INLINE
  static constexpr std::optional<RawMj*> unwrap(
      std::optional<python::MjWrapper<RawMj>*> wrapper) {
    if (wrapper.has_value()) {
      return (*wrapper)->get();
    }
    return std::nullopt;
  }
};

template <typename Arr>
using enable_if_arithmetic_array_t = std::enable_if_t<
    std::is_array_v<Arr> &&
    std::is_arithmetic_v<std::remove_all_extents_t<Arr>>>;

template <typename Arr>
struct wrapped<Arr*, enable_if_arithmetic_array_t<Arr>> {
  MUJOCO_ALWAYS_INLINE
  static constexpr Arr* unwrap(Eigen::Ref<array_eigen_t<Arr>> wrapper) {
    return reinterpret_cast<Arr*>(wrapper.data());
  }
};
}  // namespace mujoco::util

namespace mujoco::python {
namespace _impl {
template <typename, typename>
struct py_arg_helper {};

template <typename... PyArg, typename... OmittedArg>
struct py_arg_helper<std::tuple<PyArg...>, std::tuple<OmittedArg...>> {
  std::tuple<PyArg...> py_args;
  std::tuple<OmittedArg...> omitted_args;

  static constexpr int n_py_args = std::tuple_size_v<decltype(py_args)>;
  static constexpr int n_omitted_args =
      std::tuple_size_v<decltype(omitted_args)>;

  template <typename... T>
  MUJOCO_ALWAYS_INLINE
  constexpr void def(::pybind11::module_& m, T&&... t) {
    constexpr int NExtras = std::tuple_size_v<std::tuple<T...>>;
    unpack_tuple_as_py_args<0, NExtras>(m, std::forward<T>(t)...);
  }

  template <int ArgIdx, int NExtras, typename... T>
  MUJOCO_ALWAYS_INLINE
  constexpr void unpack_tuple_as_py_args(::pybind11::module_& m, T&&... t) {
    if constexpr (ArgIdx == n_py_args) {
      if constexpr (std::tuple_size_v<std::tuple<T...>> ==
                    NExtras + n_py_args - n_omitted_args) {
        m.def(std::forward<T>(t)...);
      } else {
        // This should ideally be a static_assert, but we need C++20 consteval
        // to do that. When using the DEF_WITH_OMITTED_PY_ARGS macro, the
        // static_assert in that macro would trigger first, rendering this
        // branch unreachable.
        throw UnexpectedError(
            "omitted argument names do not match the underlying function "
            "parameter names");
      }
    } else if (is_omitted<ArgIdx>()) {
      unpack_tuple_as_py_args<ArgIdx+1, NExtras>(m, std::forward<T>(t)...);
    } else {
      unpack_tuple_as_py_args<ArgIdx+1, NExtras>(
          m, std::forward<T>(t)..., ::pybind11::arg(std::get<ArgIdx>(py_args)));
    }
  }

  template <int ArgIdx, int OmittedIdx = 0>
  MUJOCO_ALWAYS_INLINE
  constexpr bool is_omitted() {
    if constexpr (OmittedIdx == n_omitted_args) {
      return false;
    // string_view comparison can be constexpr
    } else if (std::string_view(std::get<ArgIdx>(py_args)) ==
               std::string_view(std::get<OmittedIdx>(omitted_args))) {
      return true;
    } else {
      return is_omitted<ArgIdx, OmittedIdx + 1>();
    }
  }
};
}  // namespace _impl

template <typename Tuple>
MUJOCO_ALWAYS_INLINE
static constexpr auto WithNamedArgs(Tuple&& py_args) {
  using ArgTuple = std::remove_cv_t<std::remove_reference_t<Tuple>>;
  return _impl::py_arg_helper<ArgTuple, std::tuple<>>{
    std::forward<Tuple>(py_args), std::tuple<>()};
}

template <typename Tuple1, typename Tuple2>
MUJOCO_ALWAYS_INLINE
static constexpr auto WithNamedArgs(Tuple1&& py_args, Tuple2&& omitted_args) {
  using ArgTuple = std::remove_cv_t<std::remove_reference_t<Tuple1>>;
  using OmittedTuple = std::remove_cv_t<std::remove_reference_t<Tuple2>>;
  return _impl::py_arg_helper<ArgTuple, OmittedTuple>{
    std::forward<Tuple1>(py_args), std::forward<Tuple2>(omitted_args)};
}

template <typename MjTraits>
MUJOCO_ALWAYS_INLINE
static constexpr void Def(::pybind11::module_& m) {
  WithNamedArgs(MjTraits::param_names).def(
      m, MjTraits::name,
      util::UnwrapArgs(InterceptMjErrors(MjTraits::GetFunc())),
      ::pybind11::doc(MjTraits::doc),
      ::pybind11::call_guard<::pybind11::gil_scoped_release>());
}

template <typename MjTraits, typename Func>
MUJOCO_ALWAYS_INLINE
static constexpr void Def(::pybind11::module_& m, Func&& func) {
  WithNamedArgs(MjTraits::param_names).def(
      m, MjTraits::name, util::UnwrapArgs(std::forward<Func>(func)),
      ::pybind11::doc(MjTraits::doc),
      ::pybind11::call_guard<::pybind11::gil_scoped_release>());
}

template <typename MjTraits, typename OmittedArgs, typename Func>
MUJOCO_ALWAYS_INLINE
static constexpr void Def(
    ::pybind11::module_& m, OmittedArgs&& omitted_args, Func&& func) {
  WithNamedArgs(MjTraits::param_names, std::forward<OmittedArgs>(omitted_args))
      .def(m, MjTraits::name, util::UnwrapArgs(std::forward<Func>(func)),
           ::pybind11::doc(MjTraits::doc),
           ::pybind11::call_guard<::pybind11::gil_scoped_release>());
}

template <typename MjTraits, typename Func>
MUJOCO_ALWAYS_INLINE
static constexpr void DefWithGil(::pybind11::module_& m, Func&& func) {
  WithNamedArgs(MjTraits::param_names).def(
      m, MjTraits::name, util::UnwrapArgs(std::forward<Func>(func)),
      ::pybind11::doc(MjTraits::doc));
}

// Should only be invoked via the DEF_WITH_OMITTED_PY_ARGS macro.
template <typename MjTraits, typename OmittedArgsTuple>
struct DefWithOmittedPyArgsImpl {
  OmittedArgsTuple omitted_args;
  template <typename Func>
  constexpr auto operator()(::pybind11::module_& m, Func&& func) {
    return Def<MjTraits>(m, omitted_args, std::forward<Func>(func));
  }
};
}  // namespace mujoco::python

#endif  // MUJOCO_PYTHON_FUNCTIONS_H_
