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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>
#include <mujoco/experimental/studio/hal/filament_renderer.h>
#include <mujoco/experimental/studio/hal/graphics_mode.h>
#include "structs.h"
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

class Viewer {
 public:
  Viewer(uint64_t native_window, int gfx_mode) {
    py::gil_scoped_release no_gil;
    renderer_ = std::make_unique<mujoco::studio::FilamentRenderer>(
        reinterpret_cast<void*>(native_window),
        static_cast<mujoco::studio::GraphicsMode>(gfx_mode));
  }

  void InitRenderer(const mujoco::python::MjModelWrapper& model) {
    py::gil_scoped_release no_gil;
    renderer_->Init(model.get());
  }

  intptr_t UploadImage(intptr_t tex_id, const std::string img, int width,
                       int height, int bpp) {
    py::gil_scoped_release no_gil;
    auto bytes = reinterpret_cast<const std::byte*>(img.data());
    const size_t num_bytes = img.size();
    return renderer_->UploadImage(tex_id, {bytes, num_bytes}, width, height);
  }

  bool RenderToPixels(int gfx_mode) {
    return IsHeadless(
        static_cast<mujoco::studio::GraphicsMode>(gfx_mode));
  }

  void Render(
      int width, int height,
      const mujoco::python::MjModelWrapper& model,
      mujoco::python::MjDataWrapper& data,
      mujoco::python::MjvPerturbWrapper& perturb,
      mujoco::python::MjvCameraWrapper& camera,
      mujoco::python::MjvOptionWrapper& vis_options,
      const std::vector<uint8_t>& render_flags,
      const std::vector<mujoco::python::MjvGeomWrapper>& extra_geoms = {},
      std::optional<pybind11::buffer> pixels = std::nullopt) {
    std::byte* pixel_data = nullptr;
    size_t pixel_size = 0;
    pybind11::buffer_info info;
    if (pixels.has_value()) {
      info = pixels->request();
      if (!PyBuffer_IsContiguous(info.view(), 'C')) {
        throw pybind11::value_error("Buffer must be C-contiguous.");
      }
      pixel_data = static_cast<std::byte*>(info.ptr);
      pixel_size = static_cast<size_t>(info.size * info.itemsize);
    }

    std::vector<mjvGeom> geoms;
    geoms.reserve(extra_geoms.size());
    for (const auto& geom_wrapper : extra_geoms) {
      if (geom_wrapper.get()) {
        geoms.push_back(*geom_wrapper.get());
      }
    }

    py::gil_scoped_release no_gil;
    // Update render flags before rendering.
    for (size_t i = 0; i < mjNRNDFLAG && i < render_flags.size(); ++i) {
      renderer_->GetRenderFlags()[i] = render_flags[i];
    }
    renderer_->Sync(model.get(), data.get(), perturb.get(), camera.get(),
                    vis_options.get(), width, height, geoms);
    renderer_->Submit(width, height, {pixel_data, pixel_size});
  }

 private:
  std::unique_ptr<mujoco::studio::FilamentRenderer> renderer_;
};

PYBIND11_MODULE(native_viewer_cc, m, pybind11::mod_gil_not_used()) {
  pybind11::module_::import("mujoco._structs");
  m.def(
      "set_imgui_context",
      [](intptr_t ptr) {
        ImGui::SetCurrentContext(reinterpret_cast<ImGuiContext*>(ptr));
      },
      pybind11::arg("ptr"), "Set ImGui context pointer.");
  pybind11::class_<Viewer>(m, "Viewer")
      .def(pybind11::init<uintptr_t, int>())
      .def("InitRenderer", &Viewer::InitRenderer)
      .def("RenderToPixels", &Viewer::RenderToPixels)
      .def("Render", &Viewer::Render, pybind11::arg("width"),
           pybind11::arg("height"), pybind11::arg("model"),
           pybind11::arg("data"), pybind11::arg("perturb"),
           pybind11::arg("camera"), pybind11::arg("vis_options"),
           pybind11::arg("render_flags"),
           pybind11::arg("extra_geoms") =
               std::vector<mujoco::python::MjvGeomWrapper>(),
           pybind11::arg("pixels") = pybind11::none())
      .def("UploadImage", &Viewer::UploadImage);
}
