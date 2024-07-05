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

#include <utility>
#include <vector>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>

namespace mujoco::python {
namespace {

namespace py = ::pybind11;

template <auto N>
py::tuple MakeTuple(
    const char* (&strings)[N]) {
  py::list result;
  for (int i = 0; i < N; i++) {
    result.append(py::str(strings[i]));
  }
  return std::move(result);
}

template <auto N>
py::tuple MakeTuple(const char* (&strings)[N][3]) {
  py::list result;
  for (int i = 0; i < N; i++) {
    result.append(py::make_tuple(
        py::str(strings[i][0]),
        py::str(strings[i][1]),
        py::str(strings[i][2])));
  }
  return std::move(result);
}

PYBIND11_MODULE(_constants, pymodule) {
  #define X(var) pymodule.attr(#var) = var

  // from mjmodel.h
  X(mjPI);
  X(mjMAXVAL);
  X(mjMINMU);
  X(mjMINIMP);
  X(mjMAXIMP);
  X(mjMAXCONPAIR);
  X(mjNEQDATA);
  X(mjNDYN);
  X(mjNGAIN);
  X(mjNBIAS);
  X(mjNREF);
  X(mjNIMP);
  X(mjNSOLVER);

  // from mjvisualize.h
  X(mjNGROUP);
  X(mjMAXLIGHT);
  X(mjMAXOVERLAY);
  X(mjMAXLINE);
  X(mjMAXLINEPNT);
  X(mjMAXPLANEGRID);

  // from mujoco.h
  X(mjVERSION_HEADER);

  // from mjtnum.h
  X(mjMINVAL);

  #undef X
  pymodule.attr("mjDISABLESTRING") = MakeTuple(mjDISABLESTRING);
  pymodule.attr("mjENABLESTRING") = MakeTuple(mjENABLESTRING);
  pymodule.attr("mjTIMERSTRING") = MakeTuple(mjTIMERSTRING);
  pymodule.attr("mjLABELSTRING") = MakeTuple(mjLABELSTRING);
  pymodule.attr("mjFRAMESTRING") = MakeTuple(mjFRAMESTRING);
  pymodule.attr("mjVISSTRING") = MakeTuple(mjVISSTRING);
  pymodule.attr("mjRNDSTRING") = MakeTuple(mjRNDSTRING);
}
}  // namespace
}  // namespace mujoco::python
