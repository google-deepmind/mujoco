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

PYBIND11_MODULE(_simulate, pymodule) {
  namespace py = ::pybind11;

  py::class_<mujoco::Simulate>(pymodule, "Simulate")
    .def(py::init<>())
    .def("renderloop",
          [](mujoco::Simulate& simulate) {
            simulate.renderloop();
          },
      py::call_guard<py::gil_scoped_release>())
    .def("load",
          [](mujoco::Simulate& simulate, std::string filename, const MjModelWrapper& m, MjDataWrapper& d) {
            const raw::MjModel* m_ptr = m.get();
            raw::MjData* d_ptr = d.get();
            simulate.load(filename.c_str(), (mjModel*)m_ptr, d_ptr);
          },
      py::call_guard<py::gil_scoped_release>())
    .def("applyposepertubations", &mujoco::Simulate::applyposepertubations)
    .def("applyforceperturbations", &mujoco::Simulate::applyforceperturbations)

    .def("lock", // TODO wrap mutex properly as as seperate pybind11 object?
      [](mujoco::Simulate& simulate) {
            simulate.mtx.lock();
          },
      py::call_guard<py::gil_scoped_release>())
    .def("unlock",
      [](mujoco::Simulate& simulate) {
            simulate.mtx.unlock();
          },
      py::call_guard<py::gil_scoped_release>())
    .def_readwrite("ctrlnoisestd", &mujoco::Simulate::ctrlnoisestd)
    .def_readwrite("ctrlnoiserate", &mujoco::Simulate::ctrlnoiserate)

    .def_readwrite("realtimeindex", &mujoco::Simulate::realTimeIndex)
    .def_readwrite("speedchanged", &mujoco::Simulate::speedChanged)
    .def_readwrite("measuredslowdown", &mujoco::Simulate::measuredSlowdown)
    .def_readwrite("refreshrate", &mujoco::Simulate::refreshRate)

    .def_readwrite("busywait", &mujoco::Simulate::busywait)
    .def_readwrite("run", &mujoco::Simulate::run)

    .def("getexitrequest",
      [](mujoco::Simulate& simulate) {
        return simulate.exitrequest.load();
      }
    )
    .def("setexitrequest",
      [](mujoco::Simulate& simulate, bool exitrequest) {
        simulate.exitrequest.store(exitrequest);
      }
    )

    .def("getuiloadrequest",
      [](mujoco::Simulate& simulate) {
        return simulate.uiloadrequest.load();
      }
    )
    .def("setuiloadrequest",
      [](mujoco::Simulate& simulate, int uiloadrequest) {
        simulate.uiloadrequest.store(uiloadrequest);
      }
    )
    .def("uiloadrequest_fetch_sub",
      [](mujoco::Simulate& simulate, int arg) {
        simulate.uiloadrequest.fetch_sub(arg);
      }
    )

    .def("getdroploadrequest",
      [](mujoco::Simulate& simulate) {
        return simulate.droploadrequest.load();
      }
    )
    .def("setdroploadrequest",
      [](mujoco::Simulate& simulate, bool droploadrequest) {
        simulate.droploadrequest.store(droploadrequest);
      }
    )
    .def("getdropfilename",
      [](mujoco::Simulate& simulate) {
        return (char*)simulate.dropfilename;
      }
    )
    .def("getfilename",
      [](mujoco::Simulate& simulate) {
        return (char*)simulate.filename;
      }
    )
    .def("setloadError",
      [](mujoco::Simulate& simulate, std::string& loadError) {
        strncpy(simulate.loadError, loadError.c_str(), simulate.kMaxFilenameLength);
      }
    );

  pymodule.def("setglfwdlhandle", [](std::uintptr_t dlhandle) { mujoco::setglfwdlhandle(reinterpret_cast<void*>(dlhandle)); });
}

}  // namespace

}
