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

#include "specs_wrapper.h"

#include <cstddef>  // IWYU pragma: keep
#include <string>
#include <string_view>  // IWYU pragma: keep
#include <vector>       // IWYU pragma: keep

#include <mujoco/mjspec.h>  // IWYU pragma: keep
#include <mujoco/mujoco.h>
#include "errors.h"
#include "indexers.h"  // IWYU pragma: keep
#include "raw.h"
#include "structs.h"  // IWYU pragma: keep
#include <pybind11/cast.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace py = ::pybind11;

namespace mujoco::python {

MjSpec::MjSpec() : ptr(mj_makeSpec()) {}
MjSpec::MjSpec(raw::MjSpec* ptr, const py::dict& assets_) : ptr(ptr) {
  for (const auto [key, value] : assets_) {
    assets[key] = value;
  }
}

// copy constructor and assignment
MjSpec::MjSpec(const MjSpec& other) : ptr(mj_copySpec(other.ptr)) {
  override_assets = other.override_assets;
  for (const auto [key, value] : other.assets) {
    assets[key] = value;
  }
  parent = other.parent;
}

MjSpec& MjSpec::operator=(const MjSpec& other) {
  override_assets = other.override_assets;
  ptr = mj_copySpec(other.ptr);
  for (const auto [key, value] : other.assets) {
    assets[key] = value;
  }
  parent = other.parent;
  return *this;
}

// move constructor and move assignment
MjSpec::MjSpec(MjSpec&& other) : ptr(other.ptr) {
  override_assets = other.override_assets;
  other.ptr = nullptr;
  for (const auto [key, value] : other.assets) {
    assets[key] = value;
  }
  other.assets.clear();
  parent = other.parent;
  other.parent = nullptr;
}

MjSpec& MjSpec::operator=(MjSpec&& other) {
  override_assets = other.override_assets;
  ptr = other.ptr;
  other.ptr = nullptr;
  for (const auto [key, value] : other.assets) {
    assets[key] = value;
  }
  other.assets.clear();
  parent = other.parent;
  other.parent = nullptr;
  return *this;
}

MjSpec::~MjSpec() { mj_deleteSpec(ptr); }

raw::MjModel* MjSpec::Compile() {
  if (assets.empty()) {
    auto m = mj_compile(ptr, 0);
    if (!m || mjs_isWarning(ptr)) {
      throw py::value_error(mjs_getError(ptr));
    }
    return m;
  }
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  for (const auto& asset : assets) {
    std::string buffer_name =
        _impl::StripPath(py::cast<std::string>(asset.first).c_str());
    std::string buffer = py::cast<std::string>(asset.second);
    const int vfs_error = InterceptMjErrors(mj_addBufferVFS)(
        &vfs, buffer_name.c_str(), buffer.c_str(), buffer.size());
    if (vfs_error) {
      mj_deleteVFS(&vfs);
      if (vfs_error == 2) {
        throw py::value_error("Repeated file name in assets dict: " +
                              buffer_name);
      } else {
        throw py::value_error("Asset failed to load: " + buffer_name);
      }
    }
  }
  auto m = mj_compile(ptr, &vfs);
  if (!m || mjs_isWarning(ptr)) {
    throw py::value_error(mjs_getError(ptr));
  }
  mj_deleteVFS(&vfs);
  return m;
}

}  // namespace mujoco::python
