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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_RENDERER_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_RENDERER_H_

#include <cstddef>
#include <span>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

// Renders the mujoco simulation and the imgui state.
class Renderer {
 public:
  Renderer() = default;
  virtual ~Renderer() = default;

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  // Initializes the renderer with the given mjModel.
  virtual void Init(const mjModel* model) = 0;

  // Renders the simulation and ux state. Renders into `pixels` if provided,
  // otherwise renders to the `native_window` provided at construction.
  virtual void Render(const mjModel* model, mjData* data,
                      const mjvPerturb* perturb, mjvCamera* camera,
                      const mjvOption* vis_option, int width, int height,
                      std::span<std::byte> pixels = {},
                      std::span<mjvGeom> extra_geoms = {}) = 0;

  // Populates the given output buffer with RGB888 pixel data. The size of the
  // output buffer must be at least width * height * 3.
  virtual void RenderToTexture(const mjModel* model, mjData* data,
                               mjvCamera* camera, int width, int height,
                               std::byte* output) = 0;

  // Uploads an image to the backend for GUI rendering, returning the texture
  // ID for the texture. The ID can be used in subsequent calls to update the
  // texture data. A nullptr pixels argument will free the texture if it exists.
  // A texture ID of 0 will create a new texture.
  virtual int UploadImage(int texture_id, const std::byte* pixels, int width,
                          int height, int bpp) = 0;

  // Rendering flags.
  virtual mjtByte* GetRenderFlags() = 0;

  // Returns the current frame rate.
  virtual double GetFps() = 0;
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_RENDERER_H_
