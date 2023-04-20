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

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include <glfw_adapter.h>
#include <glfw_dispatch.h>
#include <simulate.h>
#include "structs.h"
#include <pybind11/gil.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

namespace mujoco::python {
namespace {
namespace py = ::pybind11;

template <typename T, int N>
constexpr inline std::size_t sizeof_arr(const T(&arr)[N]) {
  return sizeof(arr);
}

class SimulateWrapper : public mujoco::Simulate {
 public:
  SimulateWrapper(std::unique_ptr<PlatformUIAdapter> platform_ui_adapter,
                  py::object scn, py::object cam,
                  py::object opt, py::object pert,  bool fully_managed)
      : Simulate(std::move(platform_ui_adapter),
                 scn.cast<MjvSceneWrapper&>().get(),
                 cam.cast<MjvCameraWrapper&>().get(),
                 opt.cast<MjvOptionWrapper&>().get(),
                 pert.cast<MjvPerturbWrapper&>().get(),
                 fully_managed),
        m_(py::none()),
        d_(py::none()),
        scn_(scn),
        cam_(cam),
        opt_(opt),
        pert_(pert) {}

  void Load(py::object m, py::object d, const std::string& path) {
    mjModel* m_raw = m.cast<MjModelWrapper&>().get();
    mjData* d_raw = d.cast<MjDataWrapper&>().get();
    {
      py::gil_scoped_release no_gil;
      Simulate::Load(m_raw, d_raw, path.c_str());
    }
    m_ = m;
    d_ = d;
    m_raw_ = m_raw;
    d_raw_ = d_raw;
  }

 private:
  // Hold references to keep these Python objects alive for as long as the
  // simulate object.
  py::object m_;
  py::object d_;
  py::object scn_;
  py::object cam_;
  py::object opt_;
  py::object pert_;

  mjModel* m_raw_ = nullptr;
  mjData* d_raw_ = nullptr;
};

PYBIND11_MODULE(_simulate, pymodule) {
  py::class_<SimulateMutex>(pymodule, "Mutex")
      .def(
          "__enter__", [](SimulateMutex& mtx) { mtx.lock(); },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "__exit__",
          [](SimulateMutex& mtx, py::handle, py::handle, py::handle) {
            mtx.unlock();
          },
          py::call_guard<py::gil_scoped_release>());

  py::class_<SimulateWrapper>(pymodule, "Simulate")
      .def_readonly_static("MAX_GEOM", &mujoco::Simulate::kMaxGeom)
      .def(py::init([](py::object scn, py::object cam, py::object opt,
                       py::object pert, bool fully_managed) {
        return std::make_unique<SimulateWrapper>(
            std::make_unique<mujoco::GlfwAdapter>(), scn, cam, opt, pert,
            fully_managed);
      }))
      .def("load", &SimulateWrapper::Load)
      .def("sync", &mujoco::Simulate::Sync,
           py::call_guard<py::gil_scoped_release>())

      .def(
          "render_loop",
          [](SimulateWrapper& simulate) { simulate.RenderLoop(); },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "lock",
          [](SimulateWrapper& simulate) -> SimulateMutex& {
            return simulate.mtx;
          },
          py::call_guard<py::gil_scoped_release>(),
          py::return_value_policy::reference_internal)
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
          [](SimulateWrapper& simulate) { return simulate.exitrequest.load(); },
          [](SimulateWrapper& simulate, int exitrequest) {
            simulate.exitrequest.store(exitrequest);
          },
          py::call_guard<py::gil_scoped_release>())

      .def_property_readonly(
          "uiloadrequest",
          [](SimulateWrapper& simulate) {
            return simulate.uiloadrequest.load();
          },
          py::call_guard<py::gil_scoped_release>())
      .def(
          "uiloadrequest_decrement",
          [](SimulateWrapper& simulate) {
            simulate.uiloadrequest.fetch_sub(1);
          },
          py::call_guard<py::gil_scoped_release>())

      .def_property(
          "droploadrequest",
          [](SimulateWrapper& simulate) {
            return simulate.droploadrequest.load();
          },
          [](SimulateWrapper& simulate, bool droploadrequest) {
            simulate.droploadrequest.store(droploadrequest);
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property_readonly(
          "dropfilename",
          [](SimulateWrapper& simulate) -> std::string {
            return simulate.dropfilename;
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property_readonly(
          "filename",
          [](SimulateWrapper& simulate) -> std::string {
            return simulate.filename;
          },
          py::call_guard<py::gil_scoped_release>())
      .def_property(
          "load_error",
          [](SimulateWrapper& simulate) -> std::string {
            return simulate.load_error;
          },
          [](SimulateWrapper& simulate, const std::string& error) {
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
