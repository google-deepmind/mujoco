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

#include "experimental/platform/renderer.h"

#include <cstddef>

#include <mujoco/mujoco.h>

#if defined(USE_FILAMENT_OPENGL) || defined(USE_FILAMENT_VULKAN)
#include "experimental/filament/render_context_filament.h"
#elif defined(USE_CLASSIC_OPENGL)
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#else
#error No rendering mode defined.
#endif

namespace mujoco::platform {

Renderer::Renderer(void* native_window) : native_window_(native_window) {
#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_Init();
#endif
}

Renderer::~Renderer() { Deinit(); }

void Renderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    mjr_defaultContext(&render_context_);

#if defined(USE_CLASSIC_OPENGL)
    mjr_makeContext(model, &render_context_, mjFONTSCALE_150);
#else
    mjrFilamentConfig render_config;
    mjr_defaultFilamentConfig(&render_config);
    render_config.native_window = native_window_;
    render_config.enable_gui = true;
#if defined(USE_FILAMENT_OPENGL)
    render_config.graphics_api = mjGFX_OPENGL;
#elif defined(USE_FILAMENT_VULKAN)
    render_config.graphics_api = mjGFX_VULKAN;
#endif
    mjr_makeFilamentContext(model, &render_context_, &render_config);
#endif

    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 2000);
    initialized_ = true;
  }
}

void Renderer::Deinit() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    initialized_ = false;
  }
}

void Renderer::Render(const mjModel* model, mjData* data,
                      const mjvPerturb* perturb, mjvCamera* camera,
                      const mjvOption* vis_option, int width, int height) {
  if (!initialized_) {
    return;
  }

  if (last_update_time_ == data->time) {
    mjv_updateCamera(model, data, camera, &scene_);
  } else {
    mjv_updateScene(model, data, vis_option, perturb, camera, mjCAT_ALL,
                    &scene_);
    last_update_time_ = data->time;
  }

  const mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scene_, &render_context_);

#ifdef USE_CLASSIC_OPENGL
  ImGui_ImplOpenGL3_NewFrame();
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
#endif

#ifdef USE_CLASSIC_OPENGL
  TimePoint now = std::chrono::steady_clock::now();
  TimePoint::duration delta_time = now - last_fps_update_;
  const double interval = std::chrono::duration<double>(delta_time).count();

  ++frames_;
  if (interval > 0.2) {  // only update FPS stat at most 5 times per second
    last_fps_update_ = now;
    fps_ = frames_ / interval;
    frames_ = 0;
  }
#else
  fps_ = mjr_getFrameRate(&render_context_);
#endif
}

void Renderer::RenderToTexture(const mjModel* model, mjData* data,
                               mjvCamera* camera, int width, int height,
                               std::byte* output) {
  if (!initialized_ || last_update_time_ == -1) {
    return;
  }

  const mjrRect viewport = {0, 0, width, height};

  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  mjv_updateCamera(model, data, camera, &scene_);
  mjr_render(viewport, &scene_, &render_context_);
  mjr_readPixels((unsigned char*)output, nullptr, viewport, &render_context_);
  mjr_setBuffer(mjFB_WINDOW, &render_context_);
}

double Renderer::GetFps() { return fps_; }

}  // namespace mujoco::platform
