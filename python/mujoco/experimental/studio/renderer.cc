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

#include <mujoco/experimental/platform/hal/renderer.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <mujoco/experimental/platform/hal/graphics_mode.h>
#include "structs.h"
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace mujoco::python {

class Renderer {
 public:
  using RendererImpl = mujoco::platform::Renderer;
  using GraphicsMode = mujoco::platform::GraphicsMode;

  Renderer(const std::string& graphics_mode_str) {
    py::gil_scoped_release no_gil;
    const GraphicsMode mode = mujoco::platform::GraphicsModeFromString(
        graphics_mode_str, GraphicsMode::FilamentOpenGl);
    impl_ = std::make_unique<RendererImpl>(nullptr, mode);
  }

  void Init(const MjModelWrapper& model) {
    py::gil_scoped_release no_gil;
    impl_->Init(model.get());
  }

  pybind11::bytes Render(const MjModelWrapper& model, MjDataWrapper& data,
                         std::optional<MjvPerturbWrapper>& perturb,
                         std::optional<MjvCameraWrapper>& camera,
                         std::optional<MjvOptionWrapper>& vis_option, int width,
                         int height) {
    std::vector<std::byte> pixels(width * height * 3);
    {
      py::gil_scoped_release no_gil;
      impl_->Render(
          model.get(), data.get(), perturb ? perturb.value().get() : nullptr,
          camera ? camera.value().get() : nullptr,
          vis_option ? vis_option.value().get() : nullptr, width, height, pixels);
    }
    return pybind11::bytes((const char*)pixels.data(), pixels.size());
  }

  pybind11::memoryview GetRenderFlags() {
    return pybind11::memoryview::from_buffer(
        impl_->GetRenderFlags(), {static_cast<pybind11::ssize_t>(mjNRNDFLAG)},
        {sizeof(mjtByte)});
  }

 private:
  std::unique_ptr<RendererImpl> impl_;
};

}  // namespace mujoco::python

PYBIND11_MODULE(renderer, m, pybind11::mod_gil_not_used()) {
  pybind11::module_::import("mujoco._structs");
  pybind11::class_<mujoco::python::Renderer>(m, "Renderer")
      .def(pybind11::init<const std::string&>())
      .def("Init", &mujoco::python::Renderer::Init)
      .def("Render", &mujoco::python::Renderer::Render)
      .def("get_render_flags", &mujoco::python::Renderer::GetRenderFlags,
           pybind11::keep_alive<0, 1>());
}
