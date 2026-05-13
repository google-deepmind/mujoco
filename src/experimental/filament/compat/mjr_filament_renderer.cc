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

#include "experimental/filament/compat/mjr_filament_renderer.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/imgui_bridge.h"
#include "experimental/filament/compat/scene_bridge.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/render_context_filament_cpp.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

MjrFilamentRenderer::MjrFilamentRenderer(const mjrFilamentConfig* config) {
  filament_context_ = std::make_unique<FilamentContext>(config);
}

void MjrFilamentRenderer::Init(const mjModel* model) {
  scene_bridge_ = std::make_unique<SceneBridge>(filament_context_.get(), model);
  imgui_bridge_ = std::make_unique<ImguiBridge>(filament_context_.get());
  scene_bridge_->SetDrawTextFunction(DrawTextAt);
}

void MjrFilamentRenderer::Render(const mjrRect& viewport,
                                 const mjvScene* scene) {
  scene_bridge_->Update(viewport, scene);
  // Update the UX renderable entity after processing the scene in case there
  // are any elements in the scene which generate UX draw calls (e.g. labels).
  if (mode_ != FrameBufferMode::OffScreen) {
    imgui_bridge_->Update();
  }

  if (mode_ == FrameBufferMode::Window) {
    mjrRenderRequest reqs[2];
    mjr_defaultRenderRequest(&reqs[0]);
    reqs[0].scene = scene_bridge_->GetScene();
    reqs[0].draw_mode = scene_bridge_->GetDrawMode();
    reqs[0].camera = scene_bridge_->GetCamera();
    reqs[0].viewport = viewport;

    mjr_defaultRenderRequest(&reqs[1]);
    reqs[1].scene = imgui_bridge_->GetScene();
    reqs[1].draw_mode = mjDRAW_MODE_COLOR;
    reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
    reqs[1].viewport = viewport;
    filament_context_->Render(reqs);
  }
}

void MjrFilamentRenderer::SetFrameBuffer(int framebuffer) {
  switch (framebuffer) {
    case mjFB_WINDOW:
      mode_ = FrameBufferMode::Window;
      break;
    case mjFB_OFFSCREEN:
      mode_ = FrameBufferMode::OffScreen;
      break;
    case 2:  // No official constant fo this.
      mode_ = FrameBufferMode::OffScreenWithGui;
      break;
    default:
      mju_error("Invalid framebuffer mode: %d", framebuffer);
  }
}

void MjrFilamentRenderer::ReadPixels(mjrRect viewport, unsigned char* rgb,
                                     float* depth) {
  if (mode_ == FrameBufferMode::Window) {
    mju_error("ReadPixels is only supported for offscreen rendering.");
  }

  mjrRenderRequest reqs[2];
  mjr_defaultRenderRequest(&reqs[0]);

  reqs[0].scene = scene_bridge_->GetScene();
  reqs[0].draw_mode = scene_bridge_->GetDrawMode();
  reqs[0].camera = scene_bridge_->GetCamera();
  reqs[0].viewport = viewport;

  mjr_defaultRenderRequest(&reqs[1]);
  reqs[1].scene = imgui_bridge_->GetScene();
  reqs[1].draw_mode = mjDRAW_MODE_COLOR;
  reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
  reqs[1].viewport = viewport;

  if (rgb) {
    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);
    config.width = viewport.width;
    config.height = viewport.height;
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target = CreateRenderTarget(filament_context_.get(), config);

    reqs[0].target = target.get();
    reqs[1].target = target.get();

    mjrReadPixelsRequest read_request;
    mjr_defaultReadPixelsRequest(&read_request);
    read_request.target = target.get();
    read_request.output = rgb;
    read_request.num_bytes = viewport.width * viewport.height * 3;

    const size_t num_requests =
        (mode_ == FrameBufferMode::OffScreenWithGui) ? 2 : 1;
    const mjrFrameHandle frame = filament_context_->Render(
        {&reqs[0], num_requests}, {&read_request, 1});
    filament_context_->WaitForFrame(frame);
  }

  if (depth) {
    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);
    config.width = viewport.width;
    config.height = viewport.height;
    config.color_format = mjPIXEL_FORMAT_R32F;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target = CreateRenderTarget(filament_context_.get(), config);

    reqs[0].draw_mode = mjDRAW_MODE_DEPTH;
    reqs[0].target = target.get();

    mjrReadPixelsRequest read_request;
    mjr_defaultReadPixelsRequest(&read_request);
    read_request.target = target.get();
    read_request.output = reinterpret_cast<uint8_t*>(depth);
    read_request.num_bytes = viewport.width * viewport.height * sizeof(float);

    const mjrFrameHandle frame = filament_context_->Render(
        {&reqs[0], 1}, {&read_request, 1});
    filament_context_->WaitForFrame(frame);
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
  return imgui_bridge_->UploadImage(tex_id, pixels, width, height, bpp);
}

double MjrFilamentRenderer::GetFrameRate() const {
  mjrFrameStats stats;
  mjr_defaultFrameStats(&stats);
  filament_context_->GetFrameStats(0, &stats);
  return stats.frame_rate;
}

void MjrFilamentRenderer::UpdateGui() {
  mjrf_DEBUG_drawImguiEditor(scene_bridge_->GetScene());
}

}  // namespace mujoco
