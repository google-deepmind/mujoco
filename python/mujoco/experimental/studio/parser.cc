// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string_view>

#include <mujoco/mujoco.h>
#include "third_party/mujoco/src/experimental/platform/sim/model_holder.h"
#include "structs.h"
#include <pybind11/pybind11.h>

namespace mujoco::python {

// Loads, parses, and compiles a MuJoCo model from the given file. Returns the
// python mjData object (which also contains the compiled mjModel).
py::object Parse(std::string_view filepath) {
  auto holder = platform::ModelHolder::FromFile(filepath);
  if (!holder->ok()) {
    throw py::value_error(
        std::string("Failed to load model from '") +
        std::string(filepath) + "': " + std::string(holder->error()));
  }
  mjModel* model = holder->ReleaseModel();
  mjData* data = holder->ReleaseData();
  py::object py_model = py::cast(MjModelWrapper(model));
  py::object py_data =
      py::cast(MjDataWrapper(py::cast<MjModelWrapper*>(py_model), data));
  return py_data;
}

}  // namespace mujoco::python

PYBIND11_MODULE(parser, m) {
  m.def("parse", &mujoco::python::Parse,
        pybind11::return_value_policy::take_ownership);
}
