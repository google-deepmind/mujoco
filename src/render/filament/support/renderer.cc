// Copyright 2026 DeepMind Technologies Limited
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

#include "render/filament/support/renderer.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/support/mjrf_ptr.h"

namespace mujoco {

Renderer::Renderer(const mjrfContextConfig& config)
    : ctx_(CreateContext(config)) {}

Renderer::~Renderer() {
  views_.clear();
  targets_.clear();
  scenes_.clear();
  ctx_.reset();
}

mjrfScene* Renderer::GetScene(std::string_view name) {
  SceneHandle handle = GetSceneHandle(name);
  if (handle != kInvalidScene) {
    return GetScene(handle);
  }

  const SceneHandle hnd{static_cast<int>(scenes_.size())};
  mjrfSceneParams params;
  mjrf_defaultSceneParams(&params);
  scenes_.push_back(CreateScene(ctx_.get(), params));
  scene_lookup_.emplace(name, hnd);
  return scenes_.back().get();
}

mjrfScene* Renderer::GetScene(SceneHandle handle) {
  if (handle.id < 0 || handle.id >= static_cast<int>(scenes_.size())) {
    return nullptr;
  }
  return scenes_[handle.id].get();
}

Renderer::SceneHandle Renderer::GetSceneHandle(std::string_view name) const {
  auto it = scene_lookup_.find(std::string(name));
  if (it != scene_lookup_.end()) {
    return it->second;
  }
  return kInvalidScene;
}

Renderer::RenderTargetHandle Renderer::CreateRenderTarget(
    std::string_view name, const mjrfRenderTargetConfig& config) {
  auto it = render_target_lookup_.find(std::string(name));
  if (it != render_target_lookup_.end()) {
    mju_error("Render target '%.*s' already exists.",
              static_cast<int>(name.size()), name.data());
    return kInvalidRenderTarget;
  }

  const RenderTargetHandle hnd({static_cast<int>(targets_.size())});
  targets_.push_back(mujoco::CreateRenderTarget(ctx_.get(), config));
  target_configs_.push_back(config);
  target_pixels_.emplace_back();
  render_target_lookup_.emplace(name, hnd);
  if (config.width > 0 && config.height > 0) {
    ResizeRenderTarget(hnd, config.width, config.height);
  }
  return hnd;
}

void Renderer::ResizeRenderTarget(RenderTargetHandle handle, int width,
                                  int height) {
  if (handle == kWindowRenderTarget) {
    window_width_ = width;
    window_height_ = height;
    return;
  }

  if (handle.id < 0 || handle.id >= static_cast<int>(targets_.size())) {
    return;
  }

  mjrf_resizeRenderTarget(targets_[handle.id].get(), width, height);

  mjrfRenderTargetConfig& config = target_configs_[handle.id];
  config.width = width;
  config.height = height;
  int bpp = -1;
  switch (config.color_format) {
    case mjPIXEL_FORMAT_R8:
        bpp = 1;
        break;
    case mjPIXEL_FORMAT_RGB8:
        bpp = 3;
        break;
    case mjPIXEL_FORMAT_RGBA8:
        bpp = 4;
        break;
    case mjPIXEL_FORMAT_R32F:
        bpp = 4;
        break;
    case mjPIXEL_FORMAT_DEPTH32F:
        bpp = 4;
        break;
    default:
      mju_error("Unsupported render target color format: %d",
                config.color_format);
  }
  target_pixels_[handle.id].resize(std::max(0, width * height * bpp));
}

std::span<const std::byte> Renderer::GetPixels(RenderTargetHandle handle) {
  if (handle.id < 0 || handle.id >= static_cast<int>(target_pixels_.size())) {
    return {};
  }
  return target_pixels_[handle.id];
}

Renderer::RenderTargetHandle Renderer::GetRenderTargetHandle(
    std::string_view name) const {
  auto it = render_target_lookup_.find(std::string(name));
  if (it == render_target_lookup_.end()) {
    return kInvalidRenderTarget;
  }
  return it->second;
}

Renderer::View* Renderer::GetView(ViewHandle handle) {
  if (handle.id < 0 || handle.id >= static_cast<int>(views_.size())) {
    return nullptr;
  }
  return views_[handle.id].get();
}

Renderer::View* Renderer::GetView(std::string_view name) {
  ViewHandle handle = GetViewHandle(name);
  if (handle != kInvalidView) {
    return GetView(handle);
  }

  const ViewHandle hnd{static_cast<int>(views_.size())};
  views_.emplace_back(std::make_unique<View>());
  view_lookup_.emplace(name, hnd);
  return views_.back().get();
}

Renderer::ViewHandle Renderer::GetViewHandle(std::string_view name) const {
  auto it = view_lookup_.find(std::string(name));
  if (it == view_lookup_.end()) {
    return kInvalidView;
  }
  return it->second;
}

void Renderer::Render(std::span<const ViewHandle> views) {
  std::unordered_set<int> targets;
  std::vector<mjrfReadRequest> read_requests;
  std::vector<mjrfRenderRequest> render_requests;

  render_requests.reserve(views.size());
  for (const ViewHandle& view : views) {
    mjrfRenderRequest request;
    RenderTargetHandle target = UpdateRenderRequest(request, view);
    if (target != kInvalidRenderTarget) {
      render_requests.push_back(request);
      targets.insert(target.id);
    }
  }

  for (int target_id : targets) {
    if (target_id >= 0 && target_id < static_cast<int>(target_pixels_.size())) {
      read_requests.emplace_back();
      UpdateReadRequest(read_requests.back(), RenderTargetHandle{target_id});
    }
  }

  mjrfFrameHandle frame = mjrf_render(
      ctx_.get(), render_requests.data(), render_requests.size(),
      read_requests.data(), read_requests.size());
  mjrf_waitForFrame(ctx_.get(), frame);

  mjrfFrameStats stats;
  mjrf_defaultFrameStats(&stats);
  mjrf_getFrameStats(ctx_.get(), frame, &stats);
  gpu_frame_ms_ = stats.frame_rate > 0 ? 1000.0 / stats.frame_rate : 0;
}

Renderer::RenderTargetHandle Renderer::UpdateRenderRequest(
    mjrfRenderRequest& request, ViewHandle view) {
  mjrf_defaultRenderRequest(&request);
  const View* v = GetView(view);
  if (!v) {
    return kInvalidRenderTarget;
  }

  mjrRect target_viewport = {0, 0, window_width_, window_height_};
  if (v->render_target.id >= 0 &&
             v->render_target.id < static_cast<int>(targets_.size())) {
    request.target = targets_[v->render_target.id].get();
    target_viewport.width = target_configs_[v->render_target.id].width;
    target_viewport.height = target_configs_[v->render_target.id].height;
  } else if (v->render_target == kWindowRenderTarget) {
    request.target = nullptr;
  } else {
    return kInvalidRenderTarget;
  }

  request.scene = GetScene(v->scene);
  if (request.scene == nullptr) {
    return kInvalidRenderTarget;
  }

  request.camera = v->camera;
  if (v->camera.orthographic) {
    request.camera.orthographic = true;
    request.camera.pos[0] = 0.0f;
    request.camera.pos[1] = 0.0f;
    request.camera.pos[2] = 1.0f;
    request.camera.forward[0] = 0.0f;
    request.camera.forward[1] = 0.0f;
    request.camera.forward[2] = -1.0f;
    request.camera.up[0] = 0.0f;
    request.camera.up[1] = 1.0f;
    request.camera.up[2] = 0.0f;
    request.camera.frustum_top = 0.0f;
    request.camera.frustum_near = 0.0f;
    request.camera.frustum_far = 1.0f;
    request.camera.frustum_center = target_viewport.width / 2.0f;
    request.camera.frustum_width = target_viewport.width / 2.0f;
    request.camera.frustum_bottom = target_viewport.height;
  }

  request.viewport.left = static_cast<int>(v->rel_viewport[0] * target_viewport.width);
  request.viewport.bottom = static_cast<int>(v->rel_viewport[1] * target_viewport.height);
  request.viewport.width = static_cast<int>(v->rel_viewport[2] * target_viewport.width);
  request.viewport.height = static_cast<int>(v->rel_viewport[3] * target_viewport.height);
  request.draw_mode = v->draw_mode;
  request.post_processing = v->post_processing;
  request.reflections = v->reflections;
  request.shadows = v->shadows;
  if (v->headlight.has_value()) {
    request.headlight = true;
    request.headlight_color[0] = v->headlight->color[0];
    request.headlight_color[1] = v->headlight->color[1];
    request.headlight_color[2] = v->headlight->color[2];
    request.headlight_intensity = v->headlight->intensity;
  } else {
    request.headlight = false;
  }

  return v->render_target;
}

void Renderer::UpdateReadRequest(mjrfReadRequest& request,
                                 RenderTargetHandle target) {
  mjrf_defaultReadRequest(&request);
  if (target.id < 0 || target.id >= static_cast<int>(targets_.size())) {
    return;
  }

  request.target = targets_[target.id].get();
  request.output = target_pixels_[target.id].data();
  request.num_bytes = target_pixels_[target.id].size();
}

}  // namespace mujoco
