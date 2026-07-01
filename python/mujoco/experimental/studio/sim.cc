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

// Python bindings for MuJoCo platform simulation components.

#include <tuple>

#include <mujoco/mujoco.h>
#include <mujoco/experimental/platform/sim/step_control.h>
#include "structs.h"
#include <pybind11/pybind11.h>

namespace py = pybind11;

using StepControl = mujoco::platform::StepControl;

PYBIND11_MODULE(sim, m, pybind11::mod_gil_not_used()) {
  py::module_::import("mujoco._structs");
  m.doc() = "MuJoCo platform simulation bindings for Link.";

  py::enum_<StepControl::Status>(m, "StepStatus")
      .value("OK", StepControl::Status::kOk)
      .value("PAUSED", StepControl::Status::kPaused)
      .value("VISCOUS_PAUSED", StepControl::Status::kViscousPaused)
      .value("AUTO_RESET", StepControl::Status::kAutoReset)
      .value("DIVERGED", StepControl::Status::kDiverged);

  py::enum_<StepControl::PauseState>(m, "PauseState")
      .value("UNPAUSED", StepControl::PauseState::kUnpaused)
      .value("NORMAL_PAUSED", StepControl::PauseState::kNormalPaused)
      .value("VISCOUS_PAUSED", StepControl::PauseState::kViscousPaused);

  py::class_<StepControl>(m, "StepControl")
      .def(py::init<>())
      .def(
          "advance",
          [](StepControl& self, py::object model_obj, py::object data_obj) {
            mjModel* m = nullptr;
            mjData* d = nullptr;
            if (!model_obj.is_none()) {
              m = py::cast<mujoco::python::MjModelWrapper&>(model_obj).get();
            }
            if (!data_obj.is_none()) {
              d = py::cast<mujoco::python::MjDataWrapper&>(data_obj).get();
            }
            py::gil_scoped_release no_gil;
            return self.Advance(m, d);
          },
          py::arg("model"), py::arg("data"),
          "Step physics forward, respecting speed settings and refresh budget.")
      .def("force_sync", &StepControl::ForceSync,
           "Ensures the next Advance() will synchronize time and step once.")
      .def("get_speed", &StepControl::GetSpeed,
           "Returns the desired simulation speed as a percentage of real time.")
      .def("get_speed_measured", &StepControl::GetSpeedMeasured,
           "Returns the measured simulation speed.")
      .def("set_speed", &StepControl::SetSpeed, py::arg("speed"),
           "Sets the desired speed (clamped to [0.1%, 100%]).")
      .def("set_pause_state", &StepControl::SetPauseState, py::arg("state"),
           "Sets the pause state of the simulation.")
      .def("get_pause_state", &StepControl::GetPauseState,
           "Returns the current pause state.")
      .def("request_single_step", &StepControl::RequestSingleStep,
           "Request a single step if paused.")
      .def(
          "get_noise_parameters",
          [](const StepControl& self) {
            float noise_scale, noise_rate;
            self.GetNoiseParameters(noise_scale, noise_rate);
            return std::make_tuple(noise_scale, noise_rate);
          },
          "Returns the noise parameters.")
      .def("set_noise_parameters", &StepControl::SetNoiseParameters,
           py::arg("noise_scale"), py::arg("noise_rate"),
           "Sets the noise parameters.");
}
