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

#include <sstream>
#include <type_traits>

#include "util/crossplatform.h"
#include "enum_traits.h"
#include "util/tuple_tools.h"
#include <pybind11/pybind11.h>

namespace mujoco::python {
namespace {
namespace py = ::pybind11;
template <typename Trait>
MUJOCO_ALWAYS_INLINE
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
}

template <typename Tuple>
MUJOCO_ALWAYS_INLINE
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
