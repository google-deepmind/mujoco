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

#ifndef MUJOCO_PYTHON_UTIL_FUNC_WRAP_H_
#define MUJOCO_PYTHON_UTIL_FUNC_WRAP_H_

#include <type_traits>
#include <utility>

#include <Eigen/Eigen>
#include "array_traits.h"
#include "crossplatform.h"
#include "func_traits.h"

namespace mujoco::util {

// Represents an argument type T of a C++ function that is callable from Python
// via pybind11. Template specializations of this struct defines how to unwrap
// arguments from pybind11 before passing them the underlying C++ function.
//
// This is used to help bind functions whose argument types are not related
// to the types that are registered with pybind11, and where it is not
// desirable/appropriate for the function's argument types to be registered.
//
// Usage:
//   In the compilation unit that binds a function, specialize the template for
//   each argument type that needs to be unwrapped, e.g. if a function expects
//   an argument of type SomeArgType, but the type that is known to pybind11
//   is SomeWrapperType, then the specialization looks like:
//
//      template <> wrapped py_arg<SomeArgType*> {
//        static constexpr SomeArgType* unwrap(SomeWrapperType* wrapped_arg) {
//          return wrapped_arg->get_the_underlying_thing();
//        }
//      };
template <typename T, typename = void>
struct wrapped {
  MUJOCO_ALWAYS_INLINE
  static constexpr T unwrap(T arg)  {
    return arg;
  }
};

// The wrapper type for T that can be unwrapped via wrapped<T>::unwrap.
template <typename T> using wrapper_t =
    typename util::func_arg_t<decltype(wrapped<T>::unwrap)>;

namespace _impl {

template <typename T, typename = void>
struct arg_type_deducer {
  static_assert(util::is_callable_v<T>, "not a Callable type");
  template <typename WrapOp>
  static constexpr auto WrapFunc(T&& callable) {
    using Call = decltype(&std::remove_reference_t<T>::operator());
    return arg_type_deducer<T, Call>::template WrapFunc<WrapOp>(
        std::forward<T>(callable));
  }
};

template <typename Return, typename... Args>
using func_t = Return(Args...);

// Specializations to deduce argument types for vanilla function references.
template <typename Return, typename... Args>
struct arg_type_deducer<func_t<Return, Args...>&> {
  template <typename WrapOp>
  static constexpr auto WrapFunc(Return (&func)(Args...)) {
    return WrapOp::template WrapFunc<Return, Args...>(func);
  }
};

// Specializations to deduce argument types for vanilla function pointers.
template <typename Return, typename... Args>
struct arg_type_deducer<Return (*)(Args...)> {
  template <typename WrapOp>
  static constexpr auto WrapFunc(Return (*func)(Args...)) {
    return WrapOp::template WrapFunc<Return, Args...>(*func);
  }
};

// Specialization to deduce argument types for non-const operator().
template <typename Callable, typename Return, typename... Args>
struct arg_type_deducer<
    Callable, Return (std::remove_reference_t<Callable>::*)(Args...)> {
  template <typename WrapOp>
  static constexpr auto WrapFunc(Callable&& callable) {
    return WrapOp::template WrapFunc<Return, Args...>(
        std::forward<Callable>(callable));
  }
};

// Specialization to deduce argument types for const operator().
template <typename Callable, typename Return, typename... Args>
struct arg_type_deducer<
    Callable, Return (std::remove_reference_t<Callable>::*)(Args...) const> {
  template <typename WrapOp>
  static constexpr auto WrapFunc(Callable&& callable) {
    return WrapOp::template WrapFunc<Return, Args...>(
        std::forward<Callable>(callable));
  }
};

template <typename WrapOp, typename Callable>
constexpr auto WrapFunc(Callable&& callable) {
  return arg_type_deducer<Callable>::template WrapFunc<WrapOp>(
      std::forward<Callable>(callable));
}

struct UnwrapArgs {
  template <typename Return, typename... Args, typename Callable>
  static constexpr auto WrapFunc(Callable&& callable) {
    return [callable](wrapper_t<Args>... wrapped_args)
              MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE {
      return callable(wrapped<Args>::unwrap(wrapped_args)...);
    };
  }
};

template <bool OutArgProvided>
struct ReturnArrayArg0 {
  template <typename Return, typename OutArg, typename... InArgs,
            typename Callable>
  static constexpr auto WrapFunc(Callable&& callable) {
    using OutArray = std::remove_reference_t<std::remove_pointer_t<OutArg>>;
    using OutScalar = util::array_scalar_t<OutArray>;
    static_assert(
        std::is_array_v<OutArray> && std::is_arithmetic_v<OutScalar>,
        "output is not an array of arithmetic type");
    static_assert(
        std::is_void_v<Return>,
        "callable under ReturnArrayArg0 cannot return a value");

    // MSVC has a bug with `if constexpr`, as a workaround we precompute the
    // condition into a constexpr variable first.
    // https://developercommunity.visualstudio.com/t/1509806
    constexpr bool OutArgIsRef = std::is_same_v<OutArg, OutArray&>;

    if constexpr (OutArgProvided) {
      using EigenOutType = Eigen::Ref<decltype(util::MakeEigen<OutArray>())>;
      return [callable](InArgs... args, EigenOutType eigen_out)
          MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE {
        if constexpr (OutArgIsRef) {
          callable(*reinterpret_cast<OutArray*>(eigen_out.data()), args...);
        } else {
          callable(reinterpret_cast<OutArray*>(eigen_out.data()), args...);
        }
      };
    } else {
      return [callable](InArgs... args) MUJOCO_ALWAYS_INLINE_LAMBDA_MUTABLE {
        auto eigen_out = util::MakeEigen<OutArray>();
        if constexpr (OutArgIsRef) {
          callable(*reinterpret_cast<OutArray*>(eigen_out.data()), args...);
        } else {
          callable(reinterpret_cast<OutArray*>(eigen_out.data()), args...);
        }
        return eigen_out;
      };
    }
  }
};

}  // namespace _impl

// Makes a callable that unwraps each argument before passing it to the
// given callable. Specifically, given f(T1 x1, T2 x2, ...) this function
// returns a callable
// g(wrapper_t<T1> w1, wrapper_t<T2> w2, ...) = f(unwrap(w1), unwrap(w2), ...).
template <typename Callable>
constexpr auto UnwrapArgs(Callable&& callable) {
  return _impl::WrapFunc<_impl::UnwrapArgs>(std::forward<Callable>(callable));
}

template <bool OutArgProvided = false, typename Callable>
constexpr auto ReturnArrayArg0(Callable&& callable) {
  return _impl::WrapFunc<_impl::ReturnArrayArg0<OutArgProvided>>(
      std::forward<Callable>(callable));
}

}  // namespace mujoco::util

#endif  // MUJOCO_PYTHON_UTIL_FUNC_WRAP_H_
