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

#include <cmath>
#include <cstdint>
#include <sstream>
#include <type_traits>

#include <Python.h>

#include "util/crossplatform.h"
#include "enum_traits.h"
#include "util/tuple_tools.h"
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

namespace mujoco::python {
namespace {
namespace py = ::pybind11;

inline void ZeroDenominatorCheck(double b) {
  if (b == 0) {
    PyErr_SetString(PyExc_ZeroDivisionError, "division by zero");
    throw py::error_already_set();
  }
}

template <typename T>
inline T FloorDiv(T a, T b) {
  ZeroDenominatorCheck(b);
  return std::floor(static_cast<double>(a) / static_cast<double>(b));
}

template <typename Trait>
void DefEnum(py::module_& m) {
  py::enum_<typename Trait::type> e(m, Trait::name);
  for (const auto& [name, enumerator] : Trait::values) {
    e.value(name, enumerator);
  }

  e.def(py::init([](int value) {
          for (const auto& [name, enumerator] : Trait::values) {
            if (value == enumerator) {
              return static_cast<typename Trait::type>(value);
            }
          }
          std::ostringstream err;
          err << "Invalid int value for " << Trait::name << ": " << value;
          throw py::value_error(err.str());
        }),
        py::arg("value"), py::prepend());

  // Arithmetic operators
  e.def(-py::self);
  e.def(py::self + std::int64_t());
  e.def(py::self + double());
  e.def(std::int64_t() + py::self);
  e.def(double() + py::self);

  e.def(py::self - std::int64_t());
  e.def(py::self - double());
  e.def(std::int64_t() - py::self);
  e.def(double() - py::self);

  e.def(py::self * std::int64_t());
  e.def(py::self * double());
  e.def(std::int64_t() * py::self);
  e.def(double() * py::self);

  e.def("__truediv__",
        [](const typename Trait::type& a, double b) -> double {
          ZeroDenominatorCheck(b);
          return static_cast<double>(a) / b;
        });
  e.def("__rtruediv__",
        [](const typename Trait::type& b, double a) -> double {
          ZeroDenominatorCheck(b);
          return a / static_cast<double>(b);
        });
  e.def("__floordiv__",
        [](const typename Trait::type& a, std::int64_t b) -> std::int64_t {
          return FloorDiv<std::int64_t>(a, b);
        });
  e.def("__floordiv__",
        [](const typename Trait::type& a, double b) -> double {
          return FloorDiv<double>(a, b);
        });
  e.def("__rfloordiv__",
        [](const typename Trait::type& b, std::int64_t a) -> std::int64_t {
          return FloorDiv<std::int64_t>(a, b);
        });
  e.def("__rfloordiv__",
        [](const typename Trait::type& b, double a) -> double {
          return FloorDiv<double>(a, b);
        });
  e.def("__mod__",
        [](const typename Trait::type& a, std::int64_t b) -> std::int64_t {
          return a - FloorDiv<std::int64_t>(a, b) * b;
        });
  e.def("__mod__",
        [](const typename Trait::type& a, double b) -> double {
          return a - FloorDiv<double>(a, b) * b;
        });
  e.def("__rmod__",
        [](const typename Trait::type& b, std::int64_t a) -> std::int64_t {
          return a - FloorDiv<std::int64_t>(a, b) * b;
        });
  e.def("__rmod__",
        [](const typename Trait::type& b, double a) -> double {
          return a - FloorDiv<double>(a, b) * b;
        });

  // Bitwise operators
  e.def(py::self & std::int64_t());
  e.def(std::int64_t() & py::self);
  e.def(py::self | std::int64_t());
  e.def(std::int64_t() | py::self);
  e.def(py::self ^ std::int64_t());
  e.def(std::int64_t() ^ py::self);

  // Bit shifts
  e.def(py::self << std::int64_t());
  e.def(py::self >> std::int64_t());
}

template <typename Tuple>
void DefAllEnums(py::module_& m, Tuple&& tuple) {
  using TupleNoRef = std::remove_reference_t<Tuple>;
  if constexpr (std::tuple_size_v<TupleNoRef> != 0) {
    using Trait = std::remove_reference_t<std::tuple_element_t<0, TupleNoRef>>;
    DefEnum<Trait>(m);
    DefAllEnums(m, util::tuple_slice<1, void>(tuple));
  }
}

PYBIND11_MODULE(_enums, pymodule) {
  DefAllEnums(pymodule, python_traits::kAllEnums);
}
}  // namespace
}  // namespace mujoco::python
