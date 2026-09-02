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

#include "experimental/studio/hal/classic_renderer.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <span>
#include <utility>

#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <mujoco/mujoco.h>
#if !defined(__EMSCRIPTEN__) && !defined(__APPLE__)
#include "experimental/studio/hal/egl_utils.h"
#endif
#include "experimental/studio/hal/graphics_mode.h"

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

ClassicRenderer::ClassicRenderer(void* native_window, GraphicsMode gfx)
    : gfx_(gfx) {
  if (native_window == nullptr) {
#if !defined(__EMSCRIPTEN__) && !defined(__APPLE__)
    graphics_api_context_ = CreateEglContext();
#endif
  }
  if (ImGui::GetCurrentContext()) {
    ImGui_ImplOpenGL3_Init();
  }
}

ClassicRenderer::~ClassicRenderer() {
  if (ImGui::GetCurrentContext()) {
    ImGui_ImplOpenGL3_Shutdown();
  }

  Deinit();
  graphics_api_context_.reset();
}

void ClassicRenderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    mjr_defaultContext(&render_context_);
    mjr_makeContext(model, &render_context_, mjFONTSCALE_150);
    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 2000);
    initialized_ = true;
  }
}

void ClassicRenderer::Deinit() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    initialized_ = false;
  }
}

void ClassicRenderer::Render(const mjModel* model, mjData* data,
                             const mjvPerturb* perturb, mjvCamera* camera,
                             const mjvOption* vis_option, int width, int height,
                             std::span<std::byte> pixels,
                             std::span<mjvGeom> extra_geoms) {
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
  const int nextra_geoms =
      std::min<int>(extra_geoms.size(), scene_.maxgeom - scene_.ngeom);
  for (int i = 0; i < nextra_geoms; ++i) {
    scene_.geoms[scene_.ngeom++] = extra_geoms[i];
  }

  const bool render_to_texture = !pixels.empty();
  if (render_to_texture) {
    // mjr_readPixels reads to a RGB buffer (i.e. 3 bytes per pixel).
    if (pixels.size() != width * height * 3) {
      mju_error("Offscreen mode requires a pixel buffer of size %d.",
                width * height * 3);
    }
    mjr_resizeOffscreen(width, height, &render_context_);

    framebuffer_mode_ = mjFB_OFFSCREEN;
    mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  }

  const mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scene_, &render_context_);

  // The filament backend knows how to renders the ImGui draw data. For the
  // classic backend, we need to render the ImGui draw data ourselves.
  if (ImGui::GetCurrentContext()) {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }

  if (render_to_texture) {
    unsigned char* ptr = reinterpret_cast<unsigned char*>(pixels.data());
    DoReadPixels(width, height, ptr);
  }

  UpdateFps();
}

void ClassicRenderer::RenderToTexture(const mjModel* model, mjData* data,
                                      mjvCamera* camera, int width, int height,
                                      std::byte* output) {
  if (!initialized_) {
    return;
  }

  mjv_updateCamera(model, data, camera, &scene_);
  unsigned char* ptr = reinterpret_cast<unsigned char*>(output);

  framebuffer_mode_ = mjFB_OFFSCREEN;
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  const mjrRect viewport = {0, 0, width, height};
  mjr_render(viewport, &scene_, &render_context_);
  DoReadPixels(width, height, ptr);
}

int ClassicRenderer::UploadImage(int texture_id, const std::byte* pixels,
                                 int width, int height, int bpp) {
  // unsupported
  return 0;
}

void ClassicRenderer::DoReadPixels(int width, int height, unsigned char* rgb) {
  if (!rgb) {
    return;
  }
  if (framebuffer_mode_ == mjFB_WINDOW) {
    mju_warning("ReadPixels is only supported for offscreen rendering.");
    return;
  }

  const mjrRect viewport = {0, 0, width, height};
  mjr_readPixels(rgb, nullptr, viewport, &render_context_);
  mjr_setBuffer(mjFB_WINDOW, &render_context_);
  FlipImage(rgb, viewport.width, viewport.height, 3);
  framebuffer_mode_ = mjFB_WINDOW;
  mjr_setBuffer(mjFB_WINDOW, &render_context_);
}

double ClassicRenderer::GetFps() { return fps_; }

void ClassicRenderer::UpdateFps() {
  TimePoint now = std::chrono::steady_clock::now();
  TimePoint::duration delta_time = now - last_fps_update_;
  const double interval = std::chrono::duration<double>(delta_time).count();
  ++frames_;
  if (interval > 0.2) {  // only update FPS stat at most 5 times per second
    last_fps_update_ = now;
    fps_ = frames_ / interval;
    frames_ = 0;
  }
}
}  // namespace mujoco::platform
