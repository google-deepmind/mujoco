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

#include <math/vec4.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/imgui_bridge.h"
#include "experimental/filament/compat/imgui_editor.h"
#include "experimental/filament/compat/scene_bridge.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

MjrFilamentRenderer::MjrFilamentRenderer(const mjrFilamentConfig* config) {
  filament_context_ = std::make_unique<FilamentContext>(config);
}

void MjrFilamentRenderer::Init(const mjModel* model) {
  scene_bridge_ = std::make_unique<SceneBridge>(filament_context_.get(), model);
  imgui_bridge_ = std::make_unique<ImguiBridge>(filament_context_.get());

  mjr_defaultRenderRequest(&render_requests_[0]);
  mjr_defaultRenderRequest(&render_requests_[1]);

  render_requests_[0].scene = scene_bridge_->GetSceneView();
  render_requests_[0].draw_mode = mjDRAW_MODE_COLOR;

  render_requests_[1].scene = imgui_bridge_->GetSceneView();
  render_requests_[1].draw_mode = mjDRAW_MODE_COLOR;

  // The UX camera is a fixed orthographic camera. We only need to change the
  // width/height based on the viewport per frame.
  render_requests_[1].camera.orthographic = true;
  render_requests_[1].camera.pos[0] = 0.0f;
  render_requests_[1].camera.pos[1] = 0.0f;
  render_requests_[1].camera.pos[2] = 1.0f;
  render_requests_[1].camera.forward[0] = 0.0f;
  render_requests_[1].camera.forward[1] = 0.0f;
  render_requests_[1].camera.forward[2] = -1.0f;
  render_requests_[1].camera.up[0] = 0.0f;
  render_requests_[1].camera.up[1] = 1.0f;
  render_requests_[1].camera.up[2] = 0.0f;
  render_requests_[1].camera.frustum_top = 0.0f;
  render_requests_[1].camera.frustum_near = 0.0f;
  render_requests_[1].camera.frustum_far = 1.0f;
}

void MjrFilamentRenderer::Render(const mjrRect& viewport,
                                 const mjvScene* scene) {
  scene_bridge_->Update(viewport, scene);
  // Update the UX renderable entity after processing the scene in case there
  // are any elements in the scene which generate UX draw calls (e.g. labels).
  if (mode_ != FrameBufferMode::OffScreen) {
    imgui_bridge_->Update();
  }

  if (scene->flags[mjRND_SEGMENT]) {
    render_requests_[0].draw_mode = mjDRAW_MODE_SEGMENTATION;
  } else if (scene->flags[mjRND_DEPTH]) {
    render_requests_[0].draw_mode = mjDRAW_MODE_DEPTH;
  } else {
    render_requests_[0].draw_mode = mjDRAW_MODE_COLOR;
  }

  render_requests_[0].width = viewport.width;
  render_requests_[0].height = viewport.height;
  render_requests_[1].width = viewport.width;
  render_requests_[1].height = viewport.height;

  render_requests_[0].camera =
      mjv_averageCamera(scene->camera, scene->camera + 1);
  render_requests_[1].camera.frustum_center = viewport.width / 2.0f;
  render_requests_[1].camera.frustum_width = viewport.width / 2.0f;
  render_requests_[1].camera.frustum_bottom = viewport.height;

  if (mode_ == FrameBufferMode::Window) {
    render_requests_[0].target = nullptr;
    render_requests_[1].target = nullptr;
    filament_context_->Render(render_requests_);
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

  render_requests_[0].width = viewport.width;
  render_requests_[0].height = viewport.height;
  render_requests_[1].width = viewport.width;
  render_requests_[1].height = viewport.height;

  if (rgb) {
    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);
    config.width = viewport.width;
    config.height = viewport.height;
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target =
        std::make_unique<RenderTarget>(filament_context_.get(), config);
    render_requests_[0].target = target.get();
    render_requests_[1].target = target.get();

    const size_t num_requests =
        (mode_ == FrameBufferMode::OffScreenWithGui) ? 2 : 1;

    mjrReadPixelsRequest read_request;
    mjr_defaultReadPixelsRequest(&read_request);
    read_request.output = rgb;
    read_request.num_bytes = viewport.width * viewport.height * 3;
    const mjrFrameHandle frame = filament_context_->Render(
        {&render_requests_[0], num_requests}, {&read_request, 1});
    filament_context_->WaitForFrame(frame);

    render_requests_[0].target = nullptr;
    render_requests_[1].target = nullptr;
  }

  if (depth) {
    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_R32F;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target =
        std::make_unique<RenderTarget>(filament_context_.get(), config);
    target->Prepare(viewport.width, viewport.height);
    render_requests_[0].target = target.get();
    render_requests_[1].target = target.get();

    mjrDrawMode last_draw_mode = render_requests_[0].draw_mode;
    render_requests_[0].draw_mode = mjDRAW_MODE_DEPTH;

    mjrReadPixelsRequest read_request;
    mjr_defaultReadPixelsRequest(&read_request);
    read_request.output = reinterpret_cast<uint8_t*>(depth);
    read_request.num_bytes = viewport.width * viewport.height * sizeof(float);
    const mjrFrameHandle frame = filament_context_->Render(
        {&render_requests_[0], 1}, {&read_request, 1});
    filament_context_->WaitForFrame(frame);

    render_requests_[0].target = nullptr;
    render_requests_[1].target = nullptr;
    render_requests_[0].draw_mode = last_draw_mode;
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

void MjrFilamentRenderer::UpdateGui() { DrawGui(scene_bridge_.get()); }

}  // namespace mujoco
