// Copyright 2024 DeepMind Technologies Limited
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

#include "raw.h"
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace py = ::pybind11;

namespace mujoco::python {

struct MjSpec {
  MjSpec();
  MjSpec(raw::MjSpec* ptr, const py::dict& assets_ = {});

  // copy constructor and assignment
  MjSpec(const MjSpec& other);
  MjSpec& operator=(const MjSpec& other);

  // move constructor and move assignment
  MjSpec(MjSpec&& other);
  MjSpec& operator=(MjSpec&& other);
  ~MjSpec();

  raw::MjModel* Compile();

  raw::MjSpec* ptr;
  py::dict assets;
  bool override_assets = true;
  MjSpec* parent = nullptr;
};
}  // namespace mujoco::python
