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

#include "experimental/platform/hal/renderer.h"

#include <chrono>
#include <cstddef>
#include <span>
#include <utility>

#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <mujoco/mujoco.h>
#if !defined(__EMSCRIPTEN__) && !defined(__APPLE__)
#include "experimental/platform/hal/egl_utils.h"
#endif
#include "experimental/filament/render_context_filament.h"
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/ux/plugin.h"

namespace mujoco::platform {

static void FlipImage(unsigned char* pixels, int width, int height, int bpp) {
  const int row_size = width * bpp;
  for (int i = 0; i < height / 2; ++i) {
    unsigned char* top_row = pixels + i * row_size;
    unsigned char* bottom_row = pixels + (height - 1 - i) * row_size;
    for (int j = 0; j < row_size; ++j) {
      std::swap(top_row[j], bottom_row[j]);
    }
  }
}

Renderer::Renderer(void* native_window, GraphicsMode gfx)
    : native_window_(native_window), gfx_(gfx) {
  if (IsClassic(gfx_)) {
    if (native_window == nullptr) {
#if !defined(__EMSCRIPTEN__) && !defined(__APPLE__)
      graphics_api_context_ = CreateEglContext();
#endif
    }
    if (ImGui::GetCurrentContext()) {
      ImGui_ImplOpenGL3_Init();
    }
  }
}

Renderer::~Renderer() {
  if (IsClassic(gfx_)) {
    if (ImGui::GetCurrentContext()) {
      ImGui_ImplOpenGL3_Shutdown();
    }
  }
  Deinit();
  graphics_api_context_.reset();
}

void Renderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    if (IsClassic(gfx_)) {
      mjr_defaultContext(&render_context_);
      mjr_makeContext(model, &render_context_, mjFONTSCALE_150);
      render_ = [&](mjrRect rect, mjvScene* scene) {
        mjr_render(rect, scene, &render_context_);
      };
      set_buffer_ = [&](int framebuffer) {
        mjr_setBuffer(framebuffer, &render_context_);
      };
      read_pixels_ = [&](unsigned char* pixels, mjrRect rect) {
        mjr_readPixels(pixels, nullptr, rect, &render_context_);
        mjr_setBuffer(mjFB_WINDOW, &render_context_);
      };
    } else {
      mjrf_defaultContext(&render_context_);
      mjrFilamentConfig render_config;
      mjrf_defaultFilamentConfig(&render_config);
      render_config.native_window = native_window_;
      render_config.width = model->vis.global.offwidth;
      render_config.height = model->vis.global.offheight;
      render_config.force_software_rendering = IsSoftware(gfx_);
      render_config.graphics_api = IsOpenGl(gfx_) || IsWebGl(gfx_)
                                       ? mjGRAPHICS_API_OPENGL
                                       : mjGRAPHICS_API_VULKAN;
      mjrf_makeFilamentContext(model, &render_context_, &render_config);
      render_ = [&](mjrRect rect, mjvScene* scene) {
        mjrf_render(rect, scene, &render_context_);
      };
      set_buffer_ = [&](int framebuffer) {
        mjrf_setBuffer(framebuffer, &render_context_);
      };
      read_pixels_ = [&](unsigned char* pixels, mjrRect rect) {
        mjrf_readPixels(pixels, nullptr, rect, &render_context_);
        mjrf_setBuffer(mjFB_WINDOW, &render_context_);
      };
    }

    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 2000);
    initialized_ = true;
  }
}

void Renderer::Deinit() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    if (IsClassic(gfx_)) {
      mjr_freeContext(&render_context_);
    } else {
      mjrf_freeContext(&render_context_);
    }
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
    if (model) {
      mjv_defaultFreeCamera(model, &default_cam);
    } else {
      mjv_defaultCamera(&default_cam);
    }
    camera = &default_cam;
  }
  mjvOption default_opt;
  if (vis_option == nullptr) {
    mjv_defaultOption(&default_opt);
    vis_option = &default_opt;
  }

  mjv_updateScene(model, data, vis_option, perturb, camera, mjCAT_ALL, &scene_);

  const bool render_to_texture = !pixels.empty();
  if (render_to_texture) {
    // mjr_readPixels reads to a RGB buffer (i.e. 3 bytes per pixel).
    if (pixels.size() != width * height * 3) {
      mju_error("Offscreen mode requires a pixel buffer of size %d.",
                width * height * 3);
    }
    if (IsClassic(gfx_)) {
      mjr_resizeOffscreen(width, height, &render_context_);
    }

    // The filament backend supports two offscreen framebuffers.
    // mjFB_OFFSCREEN renders just the mjvScene data. +1 also includes the
    // ImGui draw data.
    set_buffer_(IsClassic(gfx_) ? mjFB_OFFSCREEN : mjFB_OFFSCREEN + 1);
  }

  const mjrRect viewport = {0, 0, width, height};
  render_(viewport, &scene_);

  // The filament backend knows how to renders the ImGui draw data. For the
  // classic backend, we need to render the ImGui draw data ourselves.
  if (IsClassic(gfx_) && ImGui::GetCurrentContext()) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }

  if (render_to_texture) {
    unsigned char* ptr = reinterpret_cast<unsigned char*>(pixels.data());
    read_pixels_(ptr, viewport);
    if (IsClassic(gfx_)) {
      FlipImage(ptr, width, height, 3);
    }
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
  mjv_updateCamera(model, data, camera, &scene_);

  set_buffer_(mjFB_OFFSCREEN);
  render_(viewport, &scene_);

  unsigned char* ptr = reinterpret_cast<unsigned char*>(output);
  read_pixels_(ptr, viewport);
  if (IsClassic(gfx_)) {
    FlipImage(ptr, width, height, 3);
  }
}

int Renderer::UploadImage(int texture_id, const std::byte* pixels, int width,
                          int height, int bpp) {
  if (IsClassic(gfx_)) {
    return 0;
  } else {
    return mjrf_uploadGuiImage(texture_id,
                               reinterpret_cast<const unsigned char*>(pixels),
                               width, height, bpp, &render_context_);
  }
}

double Renderer::GetFps() { return fps_; }

void Renderer::UpdateFps() {
  if (IsClassic(gfx_)) {
    TimePoint now = std::chrono::steady_clock::now();
    TimePoint::duration delta_time = now - last_fps_update_;
    const double interval = std::chrono::duration<double>(delta_time).count();
    ++frames_;
    if (interval > 0.2) {  // only update FPS stat at most 5 times per second
      last_fps_update_ = now;
      fps_ = frames_ / interval;
      frames_ = 0;
    }
  } else {
    fps_ = mjrf_getFrameRate(&render_context_);
  }
}

}  // namespace mujoco::platform

mjPLUGIN_LIB_INIT(renderer) {
  mujoco::platform::GuiPlugin plugin;
  plugin.name = "Filament";
  plugin.update = [](mujoco::platform::GuiPlugin* self) {
    mjrf_updateGui(nullptr);
  };
  mujoco::platform::RegisterPlugin(plugin);
}
