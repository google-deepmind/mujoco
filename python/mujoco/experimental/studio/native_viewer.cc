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
#include <string>
#include <string_view>
#include <vector>

#include <fstream>
#include <implot.h>
#include <mujoco/mujoco.h>
#include "third_party/mujoco/src/experimental/platform/hal/graphics_mode.h"
#include "third_party/mujoco/src/experimental/platform/hal/renderer.h"
#include "third_party/mujoco/src/experimental/platform/hal/window.h"
#include "structs.h"
#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

static bool IsCuda() {
#ifdef CUDA
  return true;
#else
  return false;
#endif
}

static bool IsCrd() {
  const char* display = getenv("DISPLAY");
  return display ? strcmp(display, ":20") == 0 : false;
}

static std::vector<std::byte> LoadAsset(std::string_view path) {
  std::string file_path = "assets/" +
      std::string(path.substr(path.find(':') + 1));

  std::ifstream file(file_path, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return {};
  }
  auto file_size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<std::byte> buffer(file_size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), file_size)) {
    return {};
  }
  return buffer;
}

// Holds loaded resource data for the MuJoCo resource provider.
struct ResourceData {
  std::vector<std::byte> bytes;
};

class Viewer {
 public:
  Viewer(const std::string& title, int width, int height,
         std::string graphics_mode_str) {
    // Register resource providers for font and filament assets.
    mjpResourceProvider resource_provider;
    mjp_defaultResourceProvider(&resource_provider);

    resource_provider.open = [](mjResource* resource) {
      auto* data = new ResourceData();
      data->bytes = LoadAsset(resource->name);
      resource->data = data;
      return static_cast<int>(data->bytes.size());
    };
    resource_provider.read = [](mjResource* resource, const void** buffer) {
      auto* data = static_cast<ResourceData*>(resource->data);
      *buffer = data->bytes.data();
      return static_cast<int>(data->bytes.size());
    };
    resource_provider.close = [](mjResource* resource) {
      delete static_cast<ResourceData*>(resource->data);
      resource->data = nullptr;
    };
    resource_provider.prefix = "font";
    mjp_registerResourceProvider(&resource_provider);
    resource_provider.prefix = "filament";
    mjp_registerResourceProvider(&resource_provider);

    mujoco::platform::Window::Config config;
    using GraphicsMode = mujoco::platform::GraphicsMode;
    config.gfx_mode = mujoco::platform::GraphicsModeFromString(
        graphics_mode_str, GraphicsMode::FilamentOpenGl);
    window_ = std::make_unique<mujoco::platform::Window>("PyStudio " + title,
                                                         width, height, config);
    ImPlot::CreateContext();

    renderer_ = std::make_unique<mujoco::platform::Renderer>(
        window_->GetNativeWindowHandle(), config.gfx_mode);
  }

  void InitRenderer(const mujoco::python::MjModelWrapper& model) {
    renderer_->Init(model.get());
  }

  bool NewFrame() {
    const mujoco::platform::Window::Status status = window_->NewFrame();
    return status == mujoco::platform::Window::Status::kRunning;
  }

  intptr_t UploadImage(intptr_t tex_id, const std::string img, int width,
                       int height, int bpp) {
    return renderer_->UploadImage(tex_id, (const std::byte*)img.data(), width,
                                  height, bpp);
  }

  int RenderToTexture(const mujoco::python::MjModelWrapper& model,
                      mujoco::python::MjDataWrapper& data,
                      mujoco::python::MjvCameraWrapper& cam, int width,
                      int height, int tex_id) {
    const int bytes_per_pixel = 3;
    std::vector<std::byte> bytes(width * height * bytes_per_pixel);
    renderer_->RenderToTexture(model.get(), data.get(), cam.get(), width,
                               height, bytes.data());
    return renderer_->UploadImage(tex_id, bytes.data(), width, height,
                                  bytes_per_pixel);
  }

  std::string GetDropFile() {
    return window_->GetDropFile();
  }

  void Present(const mujoco::python::MjModelWrapper& model,
               mujoco::python::MjDataWrapper& data,
               mujoco::python::MjvPerturbWrapper& perturb,
               mujoco::python::MjvCameraWrapper& camera,
               mujoco::python::MjvOptionWrapper& vis_options,
               const std::vector<uint8_t>& render_flags) {
    const float width = window_->GetWidth();
    const float height = window_->GetHeight();
    const float scale = window_->GetScale();

    if (mujoco::platform::IsHeadless(window_->GetGraphicsMode())) {
      pixels_.resize(width * height * 3);
    } else {
      pixels_.clear();
    }

    // Update render flags before rendering.
    mjtByte* flags = renderer_->GetRenderFlags();
    for (size_t i = 0; i < mjNRNDFLAG && i < render_flags.size(); ++i) {
      flags[i] = render_flags[i];
    }

    renderer_->Render(model.get(), data.get(), perturb.get(), camera.get(),
                      vis_options.get(), width * scale, height * scale,
                      pixels_);

    window_->EndFrame();
    window_->Present(pixels_);
  }

 private:
  std::unique_ptr<mujoco::platform::Window> window_;
  std::unique_ptr<mujoco::platform::Renderer> renderer_;
  std::vector<std::byte> pixels_;
};

PYBIND11_MODULE(native_viewer_cc, m) {
  pybind11::class_<Viewer>(m, "Viewer")
      .def(pybind11::init<const std::string&, int, int, const std::string&>())
      .def("InitRenderer", &Viewer::InitRenderer)
      .def("NewFrame", &Viewer::NewFrame)
      .def("Present", &Viewer::Present)
      .def("UploadImage", &Viewer::UploadImage)
      .def("RenderToTexture", &Viewer::RenderToTexture)
      .def("GetDropFile", &Viewer::GetDropFile);
  m.def("IsCrd", &IsCrd);
  m.def("IsCuda", &IsCuda);
}
