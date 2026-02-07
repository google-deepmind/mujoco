// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_WINDOW_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_WINDOW_H_

#include <string>
#include <string_view>

#include "experimental/platform/helpers.h"
#include <SDL_video.h>

namespace mujoco::platform {

// Platform-independent window abstraction using SDL2.
//
// Initializes the SDL2 and ImGui libraries, creates/owns the native window, and
// handles events from the window.
class Window {
 public:
  // Configures the window for the specified rendering backend.
  enum RenderConfig {
    kClassicOpenGL,
    kFilamentVulkan,
    kFilamentOpenGL,
    kFilamentWebGL,
  };

  struct Config {
    RenderConfig render_config = kClassicOpenGL;
    bool enable_keyboard = true;
    bool load_fonts = true;
  };

  Window(std::string_view title, int width, int height, Config config,
         const LoadAssetFn& load_asset_fn);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;

  // The status of the window.
  enum Status {
    kRunning,
    kQuitting,
  };

  // Processes all pendings window events and prepares ImGui for input handling
  // and GUI rendering. Returns the status of the window.
  Status NewFrame();

  // Finalizes ImGui input handling. Must call NewFrame first.
  void EndFrame();

  // Swaps and presents the window buffer.
  void Present();

  // Sets the title of the window.
  void SetTitle(std::string_view title);

  // Returns information related to the current size of the window.
  int GetWidth() const { return width_; }
  int GetHeight() const { return height_; }
  float GetScale() const { return scale_; }
  float GetAspectRatio() const {
    return height_ > 0
               ? static_cast<float>(width_) / static_cast<float>(height_)
               : 1.0f;
  }

  // Returns the path to a file that was dropped on the window. Once called,
  // the value will be cleared until the next time a file is dropped.
  std::string GetDropFile();

  // Returns the handle to the underlying native window.
  void* GetNativeWindowHandle() { return native_window_; }

 private:
  int width_ = 0;
  int height_ = 0;
  float scale_ = 1.0f;
  Config config_;
  void* native_window_ = nullptr;
  SDL_Window* sdl_window_ = nullptr;
  bool should_exit_ = false;
  std::string drop_file_;
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_WINDOW_H_
