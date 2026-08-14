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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_CLASSIC_RENDERER_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_CLASSIC_RENDERER_H_

#include <chrono>
#include <cstddef>
#include <memory>
#include <span>

#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/hal/renderer.h"

namespace mujoco::platform {

// Renders the mujoco simulation and the imgui state using the Classic MuJoCo
// OpenGL renderer.
class ClassicRenderer : public Renderer {
 public:
  ClassicRenderer(void* native_window, GraphicsMode gfx_mode);
  ~ClassicRenderer();

  ClassicRenderer(const ClassicRenderer&) = delete;
  ClassicRenderer& operator=(const ClassicRenderer&) = delete;

  // Initializes the renderer with the given mjModel.
  void Init(const mjModel* model) override;

  // Renders the simulation and ux state. Renders into `pixels` if provided,
  // otherwise renders to the `native_window` provided at construction.
  void Render(const mjModel* model, mjData* data, const mjvPerturb* perturb,
              mjvCamera* camera, const mjvOption* vis_option, int width,
              int height, std::span<std::byte> pixels = {},
              std::span<mjvGeom> extra_geoms = {}) override;

  // Populates the given output buffer with RGB888 pixel data. The size of the
  // output buffer must be at least width * height * 3.
  void RenderToTexture(const mjModel* model, mjData* data, mjvCamera* camera,
                       int width, int height, std::byte* output) override;

  // Uploads an image to the backend for GUI rendering, returning the texture
  // ID for the texture. The ID can be used in subsequent calls to update the
  // texture data. A nullptr pixels argument will free the texture if it exists.
  // A texture ID of 0 will create a new texture.
  int UploadImage(int texture_id, const std::byte* pixels, int width,
                  int height, int bpp) override;

  // Rendering flags.
  mjtByte* GetRenderFlags() override { return scene_.flags; }

  // Returns the current frame rate.
  double GetFps() override;

 private:
  using Clock = std::chrono::steady_clock;
  using TimePoint = std::chrono::time_point<Clock>;

  // Resets the renderer; no rendering will occur until Init() is called again.
  void Deinit();

  void UpdateFps();

  void DoReadPixels(int width, int height, unsigned char* rgb);

  GraphicsMode gfx_ = GraphicsMode::ClassicOpenGl;
  std::shared_ptr<void> graphics_api_context_ = nullptr;
  mjrContext render_context_;
  mjvScene scene_;
  TimePoint last_fps_update_;
  int framebuffer_mode_ = 0;
  int frames_ = 0;
  double fps_ = 0;
  bool initialized_ = false;
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_CLASSIC_RENDERER_H_
