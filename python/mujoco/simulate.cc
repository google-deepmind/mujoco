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

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include <glfw_adapter.h>
#include <glfw_dispatch.h>
#include <simulate.h>
#include "structs.h"
#include <pybind11/pybind11.h>

namespace mujoco::python {
namespace {
template <typename T, int N>
constexpr inline std::size_t sizeof_arr(const T(&arr)[N]) {
  return sizeof(arr);
}

PYBIND11_MODULE(_simulate, pymodule) {
  namespace py = ::pybind11;
  using SimulateMutex = decltype(mujoco::Simulate::mtx);

  py::class_<SimulateMutex>(pymodule, "SimulateMutex")
      .def(
          "__enter__", [](SimulateMutex& mtx) { mtx.lock(); },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "__exit__",
          [](SimulateMutex& mtx, py::handle, py::handle, py::handle) {
            mtx.unlock();
          },
          py::call_guard<py::gil_scoped_release>());

  py::class_<mujoco::Simulate>(pymodule, "Simulate")
      .def(py::init([]() {
        return std::make_unique<mujoco::Simulate>(
            std::make_unique<mujoco::GlfwAdapter>());
      }))
      .def(
          "render_loop",
          [](mujoco::Simulate& simulate) { simulate.RenderLoop(); },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "load",
          [](mujoco::Simulate& simulate, MjModelWrapper& m, MjDataWrapper& d,
             const std::string& path) {
            simulate.Load(m.get(), d.get(), path.c_str());
          },
          py::call_guard<py::gil_scoped_release>())
      .def("apply_pose_perturbations",
           &mujoco::Simulate::ApplyPosePerturbations,
           py::call_guard<py::gil_scoped_release>())
      .def("apply_force_perturbations",
           &mujoco::Simulate::ApplyForcePerturbations,
           py::call_guard<py::gil_scoped_release>())

      .def(
          "lock",
          [](mujoco::Simulate& simulate) -> SimulateMutex& {
            return simulate.mtx;
          },
          py::call_guard<py::gil_scoped_release>(),
          py::return_value_policy::reference)
      .def_readonly("ctrl_noise_std", &mujoco::Simulate::ctrl_noise_std,
                    py::call_guard<py::gil_scoped_release>())
      .def_readonly("ctrl_noise_rate", &mujoco::Simulate::ctrl_noise_rate,
                    py::call_guard<py::gil_scoped_release>())

      .def_readonly("real_time_index", &mujoco::Simulate::real_time_index,
                    py::call_guard<py::gil_scoped_release>())
      .def_readwrite("speed_changed", &mujoco::Simulate::speed_changed,
                     py::call_guard<py::gil_scoped_release>())
      .def_readwrite("measured_slowdown", &mujoco::Simulate::measured_slowdown,
                     py::call_guard<py::gil_scoped_release>())
      .def_readonly("refresh_rate", &mujoco::Simulate::refresh_rate,
                    py::call_guard<py::gil_scoped_release>())

      .def_readonly("busywait", &mujoco::Simulate::busywait,
                    py::call_guard<py::gil_scoped_release>())
      .def_readonly("run", &mujoco::Simulate::run,
                    py::call_guard<py::gil_scoped_release>())

      .def_property(
          "exitrequest",
          [](mujoco::Simulate& simulate) {
            return simulate.exitrequest.load();
          },
          [](mujoco::Simulate& simulate, bool exitrequest) {
            simulate.exitrequest.store(exitrequest);
          },
          py::call_guard<py::gil_scoped_release>())

      .def_property_readonly(
          "uiloadrequest",
          [](mujoco::Simulate& simulate) {
            return simulate.uiloadrequest.load();
          },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "uiloadrequest_decrement",
          [](mujoco::Simulate& simulate) {
            simulate.uiloadrequest.fetch_sub(1);
          },
          py::call_guard<py::gil_scoped_release>())

      .def_property(
          "droploadrequest",
          [](mujoco::Simulate& simulate) {
            return simulate.droploadrequest.load();
          },
          [](mujoco::Simulate& simulate, bool droploadrequest) {
            simulate.droploadrequest.store(droploadrequest);
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property_readonly(
          "dropfilename",
          [](mujoco::Simulate& simulate) -> std::string {
            return simulate.dropfilename;
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property_readonly(
          "filename",
          [](mujoco::Simulate& simulate) -> std::string {
            return simulate.filename;
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property(
          "load_error",
          [](mujoco::Simulate& simulate) -> std::string {
            return simulate.load_error;
          },
          [](mujoco::Simulate& simulate, const std::string& error) {
            const auto max_length = sizeof_arr(simulate.load_error);
            std::strncpy(simulate.load_error, error.c_str(), max_length - 1);
            simulate.load_error[max_length - 1] = '\0';
          });

  pymodule.def("set_glfw_dlhandle", [](std::uintptr_t dlhandle) {
    mujoco::Glfw(reinterpret_cast<void*>(dlhandle));
  });
}

}  // namespace
}  // namespace mujoco::python
