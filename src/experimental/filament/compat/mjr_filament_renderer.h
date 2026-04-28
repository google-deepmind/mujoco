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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MJR_FILAMENT_RENDERER_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MJR_FILAMENT_RENDERER_H_

#include <cstdint>
#include <memory>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/compat/imgui_bridge.h"
#include "experimental/filament/compat/scene_bridge.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Subclass of the FilamentContext that implements the legacy mjr API.
class MjrFilamentRenderer : public FilamentContext {
 public:
  explicit MjrFilamentRenderer(const mjrFilamentConfig* config);
  ~MjrFilamentRenderer() = default;

  // Initializes the renderer with the given model.
  void Init(const mjModel* model);

  // Renders the given mjvScene to the viewport.
  void Render(const mjrRect& viewport, const mjvScene* scene);

  // Configures the renderer to render to the window (0) or an offscreen
  // texture (1 or 2). Rendering to the window always includes UX data from
  // ImGui. A value of 1 indicates the UX should not be included in the
  // offscreen render, whereas 2 indicates that it should.
  void SetFrameBuffer(int framebuffer);

  // Renders the scene to a texture if the framebuffer is not 0.
  void ReadPixels(mjrRect viewport, unsigned char* rgb, float* depth);

  // Uploads the mesh data from the model to the GPU.
  void UploadMesh(const mjModel* model, int id);

  // Uploads the texture data from the model to the GPU.
  void UploadTexture(const mjModel* model, int id);

  // Uploads the height field data from the model to the GPU.
  void UploadHeightField(const mjModel* model, int id);

  // Uploads a texture that can be used with ImGui to the GPU.
  uintptr_t UploadGuiImage(uintptr_t tex_id, const uint8_t* pixels, int width,
                           int height, int bpp);

  // Renders an ImGui window containing Filament-specific editor UI.
  void UpdateGui();

  MjrFilamentRenderer(const MjrFilamentRenderer&) = delete;
  MjrFilamentRenderer& operator=(const MjrFilamentRenderer&) = delete;

 private:
  enum class FrameBufferMode {
    Window,
    OffScreen,
    OffScreenWithGui,
  };

  FrameBufferMode mode_ = FrameBufferMode::Window;
  mjrRenderRequest render_requests_[2];
  std::unique_ptr<SceneBridge> scene_bridge_;
  std::unique_ptr<ImguiBridge> imgui_bridge_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MJR_FILAMENT_RENDERER_H_
