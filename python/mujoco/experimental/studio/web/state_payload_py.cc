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

// Python bindings for the state payload wire format (state_payload.h).
//
// WebViewer serializes the /state WebSocket payload with this module each
// frame; the browser parses it with the same header (web_client_session).

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "state_payload.h"
#include "structs.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Serialize the complete state WebSocket payload (see state_payload.h):
// physics state, render state and extra geoms as tagged blocks.
static py::bytes SerializeStatePayload(
    uint32_t model_crc32, int physics_spec, const py::bytes& physics_state,
    const mujoco::python::MjvCameraWrapper& camera,
    const mujoco::python::MjvPerturbWrapper& perturb,
    const mujoco::python::MjvOptionWrapper& vis_options,
    const mujoco::python::MjModelWrapper& model,
    const std::vector<uint8_t>& render_flags,
    const std::vector<mujoco::python::MjvGeomWrapper>& extra_geoms) {
  std::vector<mjvGeom> geoms;
  geoms.reserve(extra_geoms.size());
  for (const mujoco::python::MjvGeomWrapper& geom_wrapper : extra_geoms) {
    if (geom_wrapper.get()) {
      geoms.push_back(*geom_wrapper.get());
    }
  }

  std::string physics = physics_state;
  const std::vector<std::byte> buffer = mujoco::studio::SerializeStatePayload(
      model_crc32, physics_spec, physics.data(), physics.size(), *camera.get(),
      *perturb.get(), *vis_options.get(), model.get()->opt, model.get()->vis,
      model.get()->stat, render_flags, geoms.data(), geoms.size());
  return py::bytes(reinterpret_cast<const char*>(buffer.data()), buffer.size());
}

// Upper bound of a serialized payload for a model whose physics state is
// `physics_bytes` long. Used to size the StateServer's shared memory.
static size_t MaxStatePayloadSize(size_t physics_bytes) {
  return mujoco::studio::MaxStatePayloadSize(physics_bytes);
}

PYBIND11_MODULE(state_payload, m, pybind11::mod_gil_not_used()) {
  py::module_::import("mujoco._structs");
  m.doc() = "MuJoCo web viewer state payload serialization";

  m.def("serialize_state_payload", &SerializeStatePayload);
  m.def("max_state_payload_size", &MaxStatePayloadSize);
  m.attr("MAX_EXTRA_GEOMS") = mujoco::studio::kMaxExtraGeoms;
}
