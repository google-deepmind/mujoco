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

#include <imgui.h>
#include <implot.h>
#include <mujoco/experimental/platform/hal/graphics_mode.h>
#include <mujoco/experimental/platform/hal/window.h>
#include <mujoco/experimental/platform/resources.h>
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

static intptr_t GetImGuiContext() {
  return reinterpret_cast<intptr_t>(ImGui::GetCurrentContext());
}

static intptr_t GetImPlotContext() {
  return reinterpret_cast<intptr_t>(ImPlot::GetCurrentContext());
}

class Window {
 public:
  Window(const std::string& title, int width, int height,
         std::string graphics_mode_str) {
    pybind11::gil_scoped_release no_gil;

    mujoco::platform::RegisterResourceProviders();

    mujoco::platform::Window::Config config;
    config.gfx_mode = mujoco::platform::GraphicsModeFromString(
        graphics_mode_str, mujoco::platform::GraphicsMode::FilamentOpenGl);
    window_ = std::make_unique<mujoco::platform::Window>(
        title, width, height, config);
    ImPlot::CreateContext();
  }

  bool NewFrame() {
    pybind11::gil_scoped_release no_gil;
    const mujoco::platform::Window::Status window_status = window_->NewFrame();
    return window_status == mujoco::platform::Window::Status::kRunning;
  }

  void Present(pybind11::bytes pixels) {
    pybind11::gil_scoped_release no_gil;
    std::string_view sv(pixels);
    window_->EndFrame();
    window_->Present({(std::byte*)sv.data(), sv.size()});
  }

  int GetWidth() { return window_->GetWidth(); }
  int GetHeight() { return window_->GetHeight(); }

  uint64_t GetNativeWindowHandle() {
    return reinterpret_cast<uint64_t>(window_->GetNativeWindowHandle());
  }

  std::string GetDropFile() { return window_->GetDropFile(); }

 private:
  std::unique_ptr<mujoco::platform::Window> window_;
};

PYBIND11_MODULE(window, m, pybind11::mod_gil_not_used()) {
  m.def("IsCrd", &IsCrd);
  m.def("IsCuda", &IsCuda);
  m.def("GetImGuiContext", &GetImGuiContext);
  m.def("GetImPlotContext", &GetImPlotContext);

  pybind11::class_<Window> cls(m, "Window");
  cls.def(pybind11::init<const std::string&, int, int, const std::string&>());
  cls.def("NewFrame", &Window::NewFrame);
  cls.def("Present", &Window::Present);
  cls.def("GetWidth", &Window::GetWidth);
  cls.def("GetHeight", &Window::GetHeight);
  cls.def("GetDropFile", &Window::GetDropFile);
  cls.def("GetNativeWindowHandle", &Window::GetNativeWindowHandle);
}
