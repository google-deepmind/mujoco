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

#ifndef MUJOCO_PYTHON_UTIL_ARRAY_TRAITS_H_
#define MUJOCO_PYTHON_UTIL_ARRAY_TRAITS_H_

#include <type_traits>

#include "crossplatform.h"
#include <Eigen/Eigen>
#include <unsupported/Eigen/CXX11/Tensor>

namespace mujoco::util {

// Forward declaration so that the public interface appears at the top of file.
namespace _impl {
template <typename T, int... N> struct c_array;
template <typename T> struct c_array_traits;
}  // namespace _impl

// Array type from scalar type and extents. This is intended to be used to
// deduce array extents as template integer parameters.
// For example c_array_t<double, 9, 4, 7> is the same as double[9][4][7].
template <typename T, int... N>
using c_array_t = typename _impl::c_array<T, N...>::type;

// Scalar type from an array, reference-to-array, or pointer-to-array type.
template <typename T>
using array_scalar_t = typename _impl::c_array_traits<T>::scalar_type;

// The number of dimensions of an array type. If the array type is regarded as
// a tensor then this corresponds to the tensor rank.
template <typename T>
static constexpr int array_ndim_v = _impl::c_array_traits<T>::ndim;

// Makes an Eigen::Matrix or Eigen::Tensor object with the same data type and
// shape as the given array type. The Eigen object returned is always row-major
// (i.e. C ordering) and dense. If the array type is one- or two-dimensional
// then an Eigen::Matrix with compile-time constant shape is returned.
// Otherwise, an Eigen::Tensor is returned.
template <typename ArrType>
constexpr auto MUJOCO_ALWAYS_INLINE MakeEigen() {
  return _impl::c_array_traits<ArrType>::template MakeEigen<>();
}

template <typename ArrType>
using array_eigen_t = std::conditional_t<
    std::is_const_v<ArrType>,
    const decltype(MakeEigen<std::remove_const_t<ArrType>>()),
    // Still need remove_const here since the type substitution in the false
    // branch always occurs regardless of the condition, and
    // MakeEigen<const T>() is invalid.
    decltype(MakeEigen<std::remove_const_t<ArrType>>())>;

// =====================================================================
// IMPLEMENTATION DETAIL. FOR INTERNAL USE WITHIN THIS HEADER FILE ONLY.
// =====================================================================
namespace _impl {
template <typename T, int... N>
struct c_array {};

template <typename T>
struct c_array<T> {
  using type = T;
  static constexpr int ndim = 0;
};

template <typename T, int M, int... N>
struct c_array<T, M, N...> {
  using type = typename c_array<T, N...>::type[M];
  static constexpr int ndim = c_array<T, N...>::ndim + 1;
};

template <typename T>
struct c_array_traits {
  static constexpr int ndim = 0;
  using scalar_type = std::remove_reference_t<T>;

  template <int... N>
  static constexpr auto MUJOCO_ALWAYS_INLINE MakeEigen() {
    if constexpr (c_array<T, N...>::ndim <= 2) {
      return Eigen::Matrix<T, N..., Eigen::RowMajor>();
    } else {
      return Eigen::Tensor<
          T, c_array<T, N...>::ndim, Eigen::RowMajor, Eigen::DenseIndex>(N...);
    }
  }
};

template <typename T, int N>
struct c_array_traits<T[N]> {
  // Recursively peel off the innermost extent.
  static constexpr int ndim = c_array_traits<T>::ndim + 1;
  using scalar_type = typename c_array_traits<T>::scalar_type;

  template <int... M>
  static constexpr auto MUJOCO_ALWAYS_INLINE MakeEigen() {
    return c_array_traits<T>::template MakeEigen<M..., N>();
  }
};

template <typename T, int N>
struct c_array_traits<T(&)[N]> {
  // Delegate everything to the T[N] case.
  static constexpr int ndim = c_array_traits<T[N]>::ndim;
  using scalar_type = typename c_array_traits<T[N]>::scalar_type;

  template <int... M>
  static constexpr auto MUJOCO_ALWAYS_INLINE MakeEigen() {
    return c_array_traits<T[N]>::template MakeEigen<M...>();
  }
};

template <typename T, int N>
struct c_array_traits<T(*)[N]> {
  // Delegate everything to the T[N] case.
  static constexpr int ndim = c_array_traits<T[N]>::ndim;
  using scalar_type = typename c_array_traits<T[N]>::scalar_type;

  template <int... M>
  static constexpr auto MUJOCO_ALWAYS_INLINE MakeEigen() {
    return c_array_traits<T[N]>::template MakeEigen<M...>();
  }
};

}  // namespace _impl
}  // namespace mujoco::util

#endif  // MUJOCO_PYTHON_UTIL_ARRAY_TRAITS_H_
