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
#include <span>

#include <mujoco/mujoco.h>
#include "experimental/platform/renderer_backend.h"

#if defined(MUJOCO_RENDERER_CLASSIC_OPENGL)
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include "experimental/platform/egl_utils.h"
#else
#include "experimental/filament/render_context_filament.h"
#include "experimental/platform/plugin.h"
#endif

namespace mujoco::platform {

Renderer::Renderer(void* native_window) : native_window_(native_window) {
#ifdef MUJOCO_RENDERER_CLASSIC_OPENGL
  if (native_window == nullptr) {
    graphics_api_context_ = CreateEglContext();
  }
  if (ImGui::GetCurrentContext()) {
    ImGui_ImplOpenGL3_Init();
  }
#endif
}

Renderer::~Renderer() {
  Deinit();
  graphics_api_context_.reset();
}

void Renderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    mjr_defaultContext(&render_context_);

#if defined(MUJOCO_RENDERER_CLASSIC_OPENGL)
    mjr_makeContext(model, &render_context_, mjFONTSCALE_150);
#else
    mjrFilamentConfig render_config;
    mjr_defaultFilamentConfig(&render_config);
    render_config.native_window = native_window_;
    render_config.enable_gui = true;
#if defined(MUJOCO_RENDERER_FILAMENT_OPENGL)
    render_config.graphics_api = mjGFX_OPENGL;
#elif defined(MUJOCO_RENDERER_FILAMENT_OPENGL_HEADLESS)
    render_config.graphics_api = mjGFX_OPENGL;
#elif defined(MUJOCO_RENDERER_FILAMENT_VULKAN)
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
                      const mjvOption* vis_option, int width, int height,
                      std::span<std::byte> pixels) {
  if (!initialized_) {
    return;
  }

  mjvCamera default_cam;
  if (camera == nullptr) {
    mjv_defaultCamera(&default_cam);
    camera = &default_cam;
  }
  mjvOption default_opt;
  if (vis_option == nullptr) {
    mjv_defaultOption(&default_opt);
    vis_option = &default_opt;
  }

  mjv_updateScene(model, data, vis_option, perturb, camera, mjCAT_ALL,
                  &scene_);

  const bool render_to_texture = !pixels.empty();
  if (render_to_texture) {
    // mjr_readPixels reads to a RGB buffer (i.e. 3 bytes per pixel).
    if (pixels.size() != width * height * 3) {
      mju_error("Offscreen mode requires a pixel buffer of size %d.",
                width * height * 3);
    }

    #ifdef MUJOCO_RENDERER_CLASSIC_OPENGL
      mjr_resizeOffscreen(width, height, &render_context_);
      mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
    #else
      // The filament backend supports two offscreen framebuffers.
      // mjFB_OFFSCREEN renders just the mjvScene data. +1 also includes the
      // ImGui draw data.
      mjr_setBuffer(mjFB_OFFSCREEN + 1, &render_context_);
    #endif
  }

  const mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scene_, &render_context_);

  // The filament backend knows how to renders the ImGui draw data. For the
  // classic backend, we need to render the ImGui draw data ourselves.
  #ifdef MUJOCO_RENDERER_CLASSIC_OPENGL
    if (ImGui::GetCurrentContext()) {
      ImGui_ImplOpenGL3_NewFrame();
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }
  #endif

  if (render_to_texture) {
    unsigned char* ptr = reinterpret_cast<unsigned char*>(pixels.data());
    mjr_readPixels(ptr, nullptr, viewport, &render_context_);
    mjr_setBuffer(mjFB_WINDOW, &render_context_);
  }

  UpdateFps();
}

void Renderer::RenderToTexture(const mjModel* model, mjData* data,
                               mjvCamera* camera, int width, int height,
                               std::byte* output) {
  if (!initialized_) {
    return;
  }

  const mjrRect viewport = {0, 0, width, height};

  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  mjv_updateCamera(model, data, camera, &scene_);
  mjr_render(viewport, &scene_, &render_context_);
  mjr_readPixels((unsigned char*)output, nullptr, viewport, &render_context_);
  mjr_setBuffer(mjFB_WINDOW, &render_context_);
}

int Renderer::UploadImage(int texture_id, const std::byte* pixels, int width,
                          int height, int bpp) {
#if defined(MUJOCO_RENDERER_CLASSIC_OPENGL)
  return 0;
#else
  return mjr_uploadGuiImage(texture_id,
                            reinterpret_cast<const unsigned char*>(pixels),
                            width, height, bpp, &render_context_);
#endif
}

double Renderer::GetFps() { return fps_; }

void Renderer::UpdateFps() {
#ifdef MUJOCO_RENDERER_CLASSIC_OPENGL
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

RendererBackend Renderer::GetBackend() {
#if defined(MUJOCO_RENDERER_FILAMENT_OPENGL_HEADLESS)
  return RendererBackend::FilamentOpenGlHeadless;
#elif defined(MUJOCO_RENDERER_FILAMENT_OPENGL)
  return RendererBackend::FilamentOpenGl;
#elif defined(MUJOCO_RENDERER_FILAMENT_VULKAN)
  return RendererBackend::FilamentVulkan;
#elif defined(MUJOCO_RENDERER_FILAMENT_WEBGL)
  return RendererBackend::FilamentWebGl;
#elif defined(MUJOCO_RENDERER_CLASSIC_OPENGL)
  return RendererBackend::ClassicOpenGl;
#else
  #error "Unsupported renderer backend."
#endif
}
}  // namespace mujoco::platform

#if !defined(MUJOCO_RENDERER_CLASSIC_OPENGL)
mjPLUGIN_LIB_INIT {
  mujoco::platform::GuiPlugin plugin;
  plugin.name = "Filament";
  plugin.update = [](mujoco::platform::GuiPlugin* self) {
    mjr_updateGui(nullptr);
  };
  mujoco::platform::RegisterGuiPlugin(&plugin);
}
#endif
