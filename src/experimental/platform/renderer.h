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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_

#include <chrono>
#include <cstddef>
#include <memory>
#include <ratio>
#include <span>

#include <mujoco/mujoco.h>
#include "experimental/platform/renderer_backend.h"


namespace mujoco::platform {

// Renders the mujoco simulation and the imgui state.
//
// The Renderer is built around a specific backend configuration
// (see renderer_backend.h).
//
// Currently we support two rendering engines: the Classic MuJoCo OpenGL
// renderer and a Filament-based renderer.
//
// The Classic renderer is implemented using the OpenGL 2.0 fixed function
// pipeline. It assumes that the caller has correctly initialized the OpenGL
// context (e.g. using EGL).
//
// The Filament renderer is a modern physically-based renderer that supports
// OpenGL 3.0, Vulkan, and WebGL (as well as other graphics libraries). Unlike
// the Classic renderer, the Filament engine itself will manage its graphics
// context. If rendering to a window surface (e.g. x11), it requires a pointer
// to the native window to do so.
//
// For OpenGL, two different Filament configurations are available: normal and
// headless. Normal rendering assumes that we will be rendering to an x11
// window and therefore will use a x11-based context. Headless assumes that we
// will be rendering to a texture and will use an EGL context. Vulkan and WebGL
// have no need for such a distinction.
class Renderer {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;
  using Seconds = std::chrono::duration<double>;
  using Milliseconds = std::chrono::duration<double, std::milli>;

  explicit Renderer(void* native_window = nullptr);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  // Initializes the renderer with the given mjModel.
  void Init(const mjModel* model);

  // Renders the simulation and ux state. Renders into `pixels` if provided,
  // otherwise renders to the `native_window` provided at construction.
  void Render(const mjModel* model, mjData* data, const mjvPerturb* perturb,
              mjvCamera* camera, const mjvOption* vis_option, int width,
              int height, std::span<std::byte> pixels = {});

  // Populates the given output buffer with RGB888 pixel data. The size of the
  // output buffer must be at least width * height * 3.
  void RenderToTexture(const mjModel* model, mjData* data, mjvCamera* camera,
                       int width, int height, std::byte* output);

  // Uploads an image to the backend for GUI rendering, returning the texture
  // ID for the texture. The ID can be used in subsequent calls to update the
  // texture data. A nullptr pixels argument will free the texture if it exists.
  // A texture ID of 0 will create a new texture.
  int UploadImage(int texture_id, const std::byte* pixels, int width,
                  int height, int bpp);

  // Rendering flags.
  mjtByte* GetRenderFlags() { return scene_.flags; }

  // Returns the current frame rate.
  double GetFps();

  // Returns the statically-defined backend for which this renderer is
  // configured.
  static RendererBackend GetBackend();

 private:
  // Resets the renderer; no rendering will occur until Init() is called again.
  void Deinit();

  void UpdateFps();

  void* native_window_ = nullptr;
  std::shared_ptr<void> graphics_api_context_ = nullptr;
  mjrContext render_context_;
  mjvScene scene_;
  bool initialized_ = false;
  mjtNum last_update_time_ = -1;
  int frames_ = 0;
  TimePoint last_fps_update_;
  double fps_ = 0;
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_
