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

#include "experimental/filament/filament/mjr_filament_renderer.h"

#include <cstdint>
#include <memory>

#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/imgui_bridge.h"
#include "experimental/filament/filament/imgui_editor.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/filament/scene_bridge.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

MjrFilamentRenderer::MjrFilamentRenderer(const mjrFilamentConfig* config)
    : FilamentContext(config) {
}

void MjrFilamentRenderer::Init(const mjModel* model) {
  scene_view_ = std::make_unique<SceneView>(GetEngine());
  scene_bridge_ = std::make_unique<SceneBridge>(GetObjectManager(),
                                                scene_view_.get(), model);
  imgui_bridge_ =
      std::make_unique<ImguiBridge>(GetObjectManager(), scene_view_.get());

  SetClearColor(ReadElement(model, "filament.clearColor",
                            filament::math::float4(0, 0, 0, 1)));
}

void MjrFilamentRenderer::Render(const mjrRect& viewport, const mjvScene* scene) {
  scene_bridge_->Update(viewport, scene);
  // Update the UX renderable entity after processing the scene in case there
  // are any elements in the scene which generate UX draw calls (e.g. labels).
  if (imgui_bridge_ && gui_swap_chain_target_ == scene_swap_chain_target_) {
    // Prepare the filament Renderable that contains the GUI draw commands. We
    // must call this function even if we do not plan on rendering the GUI to
    // ensure the ImGui state is updated.
    imgui_bridge_->Update();
  }

  last_render_mode_ = DrawMode::Color;
  if (scene->flags[mjRND_SEGMENT]) {
    last_render_mode_ = DrawMode::Segmentation;
  } else if (scene->flags[mjRND_DEPTH]) {
    last_render_mode_ = DrawMode::Depth;
  }
  last_camera_ = mjv_averageCamera(scene->camera, scene->camera + 1);

  if (scene_swap_chain_target_ == kWindowSwapChain) {
    RenderRequest request;
    request.scene = scene_view_.get();
    request.draw_mode = last_render_mode_;
    request.camera = last_camera_;
    request.draw_ux = (gui_swap_chain_target_ == kWindowSwapChain);
    request.width = viewport.width;
    request.height = viewport.height;
    FilamentContext::Render({&request, 1});
  }
}

void MjrFilamentRenderer::SetFrameBuffer(int framebuffer) {
  switch (framebuffer) {
    case mjFB_WINDOW:
      scene_swap_chain_target_ = kWindowSwapChain;
      gui_swap_chain_target_ = kWindowSwapChain;
      break;
    case mjFB_OFFSCREEN:
      scene_swap_chain_target_ = kOffscreenSwapChain;
      gui_swap_chain_target_ = kWindowSwapChain;
      break;
    case 2:  // No official constant fo this.
      scene_swap_chain_target_ = kOffscreenSwapChain;
      gui_swap_chain_target_ = kOffscreenSwapChain;
      break;
    default:
      mju_error("Invalid framebuffer mode: %d", framebuffer);
  }
}

void MjrFilamentRenderer::ReadPixels(mjrRect viewport, unsigned char* rgb,
                                     float* depth) {
  if (scene_swap_chain_target_ != kOffscreenSwapChain) {
    mju_error("ReadPixels is only supported for offscreen rendering.");
  }

  RenderRequest request;
  request.scene = scene_view_.get();
  request.camera = last_camera_;
  request.draw_ux = (gui_swap_chain_target_ == kOffscreenSwapChain);
  request.width = viewport.width;
  request.height = viewport.height;

  if (rgb) {
    request.draw_mode = last_render_mode_;

    RenderTargetConfig config;
    DefaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target = std::make_unique<RenderTarget>(GetEngine(), config);
    target->Prepare(request.width, request.height);
    request.target = target.get();

    ReadPixelsRequest read_request;
    read_request.output = rgb;
    read_request.num_bytes = viewport.width * viewport.height * 3;
    const FrameHandle frame =
        FilamentContext::Render({&request, 1}, {&read_request, 1});
    FilamentContext::WaitForFrame(frame);
  }

  if (depth) {
    request.draw_mode = DrawMode::Depth;

    RenderTargetConfig config;
    DefaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_R32F;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target = std::make_unique<RenderTarget>(GetEngine(), config);
    target->Prepare(request.width, request.height);
    request.target = target.get();

    ReadPixelsRequest read_request;
    read_request.output = reinterpret_cast<uint8_t*>(depth);
    read_request.num_bytes = viewport.width * viewport.height * sizeof(float);
    const FrameHandle frame =
        FilamentContext::Render({&request, 1}, {&read_request, 1});
    FilamentContext::WaitForFrame(frame);
  }
}

void MjrFilamentRenderer::UploadMesh(const mjModel* model, int id) {
  if (!scene_bridge_) {
    mju_error("SceneBridge is not initialized.");
  }
  scene_bridge_->UploadMesh(model, id);
}

void MjrFilamentRenderer::UploadTexture(const mjModel* model, int id) {
  if (!scene_bridge_) {
    mju_error("SceneBridge is not initialized.");
  }
  scene_bridge_->UploadTexture(model, id);
}

void MjrFilamentRenderer::UploadHeightField(const mjModel* model, int id) {
  if (!scene_bridge_) {
    mju_error("SceneBridge is not initialized.");
  }
  scene_bridge_->UploadHeightField(model, id);
}

uintptr_t MjrFilamentRenderer::UploadGuiImage(uintptr_t tex_id,
                                          const uint8_t* pixels, int width,
                                          int height, int bpp) {
  if (imgui_bridge_) {
    return imgui_bridge_->UploadImage(tex_id, pixels, width, height, bpp);
  }
  return 0;
}

void MjrFilamentRenderer::UpdateGui() { DrawGui(scene_bridge_.get()); }

}  // namespace mujoco
