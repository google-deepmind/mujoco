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

#include <backend/Platform.h>
#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/render_target_util.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Manages the filament renderer that is exposed via the mjr functions.
class FilamentContext {
 public:
  explicit FilamentContext(const mjrFilamentConfig* config);
  ~FilamentContext();

  void Init(const mjModel* model);

  void Render(const mjrRect& viewport, const mjvScene* scene);

  void SetFrameBuffer(int framebuffer);

  void ReadPixels(mjrRect viewport, unsigned char* rgb, float* depth);

  void UploadMesh(const mjModel* model, int id);

  void UploadTexture(const mjModel* model, int id);

  void UploadHeightField(const mjModel* model, int id);

  uintptr_t UploadGuiImage(uintptr_t tex_id, const uint8_t* pixels, int width,
                           int height, int bpp);

  double GetFrameRate() const;

  void UpdateGui();

  FilamentContext(const FilamentContext&) = delete;
  FilamentContext& operator=(const FilamentContext&) = delete;

 private:
  enum SwapChainType {
    kWindowSwapChain,
    kOffscreenSwapChain,
  };

  void PrepareRenderTargets(int width, int height);
  void DestroyRenderTargets();

  mjrFilamentConfig config_;

  filament::Engine* engine_ = nullptr;
  filament::Renderer* renderer_ = nullptr;
  filament::SwapChain* window_swap_chain_ = nullptr;
  filament::SwapChain* offscreen_swap_chain_ = nullptr;
  std::unique_ptr<filament::backend::Platform> platform_;

  SceneView::DrawMode last_render_mode_ = SceneView::DrawMode::kNormal;
  SwapChainType scene_swap_chain_target_ = kWindowSwapChain;
  SwapChainType gui_swap_chain_target_ = kWindowSwapChain;
  std::unique_ptr<RenderTargetAndTextures> color_target_;
  std::unique_ptr<RenderTargetAndTextures> depth_target_;
  std::unique_ptr<ObjectManager> object_manager_;
  std::unique_ptr<SceneView> scene_view_;
  std::unique_ptr<GuiView> gui_view_;
  int window_width_ = 0;
  int window_height_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_
