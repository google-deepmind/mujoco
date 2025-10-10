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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_

#include <cstdint>
#include <memory>

#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture_util.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Manages the filament renderer that is exposed via the mjr functions.
class FilamentContext {
 public:
  FilamentContext(const mjrFilamentConfig* config, const mjModel* model,
                  mjrContext* con);
  ~FilamentContext();

  void Render(const mjrRect& viewport, const mjvScene* scene,
              const mjrContext* con);

  void SetFrameBuffer(int framebuffer);

  void ReadPixels(mjrRect viewport, unsigned char* rgb, float* depth);

  void UploadMesh(const mjModel* model, int id);

  void UploadTexture(const mjModel* model, int id);

  void UploadHeightField(const mjModel* model, int id);

  void UploadFont(const uint8_t* pixels, int width, int height, int id);

  FilamentContext(const FilamentContext&) = delete;
  FilamentContext& operator=(const FilamentContext&) = delete;

 private:
  void PrepareRenderTargets(int width, int height);
  void DestroyRenderTargets();

  SceneView* GetSceneView(const mjvScene* scene);

  mjrFilamentConfig config_;
  mjrContext* context_ = nullptr;
  const mjModel* model_ = nullptr;
  filament::Engine* engine_ = nullptr;
  filament::SwapChain* swap_chain_ = nullptr;
  filament::Renderer* renderer_ = nullptr;
  filament::RenderTarget* color_target_ = nullptr;
  filament::RenderTarget* depth_target_ = nullptr;
  filament::Texture* target_textures_[kNumRenderTargetTextureTypes] = {
      nullptr, nullptr, nullptr};

  bool render_to_texture_ = false;
  bool render_gui_ = false;

  std::unique_ptr<ObjectManager> object_manager_;
  std::unique_ptr<SceneView> scene_view_;
  std::unique_ptr<GuiView> gui_view_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_
