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

#include <array>
#include <cstdio>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include <mujoco/mujoco.h>
#include <mujoco/simulate.h>
#include "raw.h"
#include "structs.h"
#include <pybind11/buffer_info.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace mujoco::python {

namespace {

namespace py = ::pybind11;

const auto simulate_doc = R"(
Python wrapper for the Simulate class
)";

// We define SimulateWrapper here instead of in structs because
// we do not want to make _structs dependent on gflw

PYBIND11_MODULE(_simulate, pymodule) {
  namespace py = ::pybind11;

  py::class_<mujoco::raw::Simulate>(pymodule, "Simulate")
    .def(py::init<>())
    .def("renderloop",
          [](mujoco::raw::Simulate& simulate) {
            simulate.renderloop();
          },
      py::call_guard<py::gil_scoped_release>())
    .def("load",
          [](mujoco::raw::Simulate& simulate, std::string filename, const MjModelWrapper& m, MjDataWrapper& d, bool delete_old_m_d) {
            const raw::MjModel* m_ptr = m.get();
            raw::MjData* d_ptr = d.get();
            simulate.load(filename.c_str(), (mjModel*)m_ptr, d_ptr, delete_old_m_d);
          },
      py::call_guard<py::gil_scoped_release>())
    .def("applyposepertubations", &mujoco::raw::Simulate::applyposepertubations)
    .def("applyforceperturbations", &mujoco::raw::Simulate::applyforceperturbations)

    .def("lock", // TODO wrap mutex properly as as seperate pybind11 object?
      [](mujoco::raw::Simulate& simulate) {
            simulate.mtx.lock();
          },
      py::call_guard<py::gil_scoped_release>())
    .def("unlock",
      [](mujoco::raw::Simulate& simulate) {
            simulate.mtx.unlock();
          },
      py::call_guard<py::gil_scoped_release>())
    .def_readwrite("ctrlnoisestd", &mujoco::raw::Simulate::ctrlnoisestd)
    .def_readwrite("ctrlnoiserate", &mujoco::raw::Simulate::ctrlnoiserate)
    .def_readwrite("slow_down", &mujoco::raw::Simulate::slow_down)
    .def_readwrite("speed_changed", &mujoco::raw::Simulate::speed_changed)
    .def("getrefreshRate",
      [](mujoco::raw::Simulate& simulate) {
            return simulate.vmode.refreshRate;
          })

    .def_readwrite("busywait", &mujoco::raw::Simulate::busywait)
    .def_readwrite("run", &mujoco::raw::Simulate::run)
    //.def_readwrite("exitrequest", &mujoco::raw::Simulate::exitrequest)
    .def("getexitrequest",
      [](mujoco::raw::Simulate& simulate) {
        return simulate.exitrequest.load();
      }
    )
    .def("setexitrequest",
      [](mujoco::raw::Simulate& simulate, bool exitrequest) {
        simulate.exitrequest.store(exitrequest);
      }
    )
    // .def_readwrite("uiloadrequest", &mujoco::raw::Simulate::uiloadrequest)
    .def("getuiloadrequest",
      [](mujoco::raw::Simulate& simulate) {
        return simulate.uiloadrequest.load();
      }
    )
    .def("setuiloadrequest",
      [](mujoco::raw::Simulate& simulate, int uiloadrequest) {
        simulate.uiloadrequest.store(uiloadrequest);
      }
    )
    .def("uiloadrequest_fetch_sub",
      [](mujoco::raw::Simulate& simulate, int arg) {
        simulate.uiloadrequest.fetch_sub(arg);
      }
    )
    // .def_readwrite("droploadrequest", &mujoco::raw::Simulate::droploadrequest)
    .def("getdroploadrequest",
      [](mujoco::raw::Simulate& simulate) {
        return simulate.droploadrequest.load();
      }
    )
    .def("setdroploadrequest",
      [](mujoco::raw::Simulate& simulate, bool droploadrequest) {
        simulate.droploadrequest.store(droploadrequest);
      }
    )
    .def("getdropfilename",
      [](mujoco::raw::Simulate& simulate) {
        return (char*)simulate.dropfilename;
      }
    )
    .def("getfilename",
      [](mujoco::raw::Simulate& simulate) {
        return (char*)simulate.filename;
      }
    )
    .def("setloadError",
      [](mujoco::raw::Simulate& simulate, std::string& loadError) {
        strncpy(simulate.loadError, loadError.c_str(), simulate.kMaxFilenameLength);
      }
    );
}

}  // namespace

}
