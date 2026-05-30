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
#include <functional>
#include <memory>
#include <span>
#include <utility>

#include <backends/imgui_impl_opengl3.h>
#include <imgui.h>
#include <mujoco/mujoco.h>
#if !defined(__EMSCRIPTEN__) && !defined(__APPLE__)
#include "experimental/platform/hal/egl_utils.h"
#endif
#include "experimental/filament/compat/scene_bridge.h"
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/ux/imgui_bridge.h"
#include "experimental/platform/ux/imgui_widgets.h"
#include "experimental/platform/ux/plugin.h"

namespace mujoco::platform {

static std::function<void()> g_update_gui_callback = nullptr;
static void PluginUpdate(GuiPlugin* plugin) {
  if (g_update_gui_callback) {
    g_update_gui_callback();
  }
}

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

  g_update_gui_callback = [this]() {
    if (scene_bridge_) {
      mjrf_DEBUG_drawImguiEditor(scene_bridge_->GetScene());
    }
  };
}

Renderer::~Renderer() {
  g_update_gui_callback = nullptr;
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

    } else {
      mjrFilamentConfig cfg;
      mjrf_defaultFilamentConfig(&cfg);
      cfg.native_window = native_window_;
      cfg.width = model->vis.global.offwidth;
      cfg.height = model->vis.global.offheight;
      cfg.force_software_rendering = IsSoftware(gfx_);
      cfg.graphics_api = IsOpenGl(gfx_) || IsWebGl(gfx_)
                             ? mjGRAPHICS_API_OPENGL
                             : mjGRAPHICS_API_VULKAN;
      filament_context_ = CreateContext(cfg);
      scene_bridge_ =
          std::make_unique<SceneBridge>(filament_context_.get(), model);
      imgui_bridge_ = std::make_unique<ImguiBridge>(filament_context_.get());
      scene_bridge_->SetDrawTextFunction(DrawTextAt);
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
      scene_bridge_.reset();
      imgui_bridge_.reset();
      filament_context_.reset();
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
    DoSetBuffer(IsClassic(gfx_) ? mjFB_OFFSCREEN : mjFB_OFFSCREEN + 1);
  }

  DoRender(width, height);

  // The filament backend knows how to renders the ImGui draw data. For the
  // classic backend, we need to render the ImGui draw data ourselves.
  if (IsClassic(gfx_) && ImGui::GetCurrentContext()) {
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

void Renderer::RenderToTexture(const mjModel* model, mjData* data,
                               mjvCamera* camera, int width, int height,
                               std::byte* output) {
  if (!initialized_) {
    return;
  }

  mjv_updateCamera(model, data, camera, &scene_);
  unsigned char* ptr = reinterpret_cast<unsigned char*>(output);

  DoSetBuffer(mjFB_OFFSCREEN);
  DoRender(width, height);
  DoReadPixels(width, height, ptr);
}

int Renderer::UploadImage(int texture_id, const std::byte* pixels, int width,
                          int height, int bpp) {
  if (IsClassic(gfx_)) {
    return 0;
  } else {
    return imgui_bridge_->UploadImage(
        texture_id, reinterpret_cast<const unsigned char*>(pixels), width,
        height, bpp);
  }
}

void Renderer::DoRender(int width, int height) {
  const mjrRect viewport = {0, 0, width, height};
  if (IsClassic(gfx_)) {
    mjr_render(viewport, &scene_, &render_context_);
  } else {
    scene_bridge_->Update(viewport, &scene_);
    // Update the UX renderable entity after processing the scene in case there
    // are any elements in the scene which generate UX draw calls (e.g. labels).
    if (framebuffer_mode_ != 1) {
      imgui_bridge_->Update();
    }
    if (framebuffer_mode_ == 0) {
      mjrDrawMode draw_mode = mjDRAW_MODE_COLOR;
      if (scene_.flags[mjRND_SEGMENT]) {
        draw_mode = mjDRAW_MODE_SEGMENTATION;
      } else if (scene_.flags[mjRND_DEPTH]) {
        draw_mode = mjDRAW_MODE_DEPTH;
      } else if (scene_.flags[mjRND_WIREFRAME]) {
        draw_mode = mjDRAW_MODE_WIREFRAME;
      }

      mjrRenderRequest reqs[2];

      mjr_defaultRenderRequest(&reqs[0]);
      reqs[0].scene = scene_bridge_->GetScene();
      reqs[0].draw_mode = draw_mode;
      reqs[0].camera = scene_bridge_->GetCamera();
      reqs[0].viewport = viewport;
      reqs[0].enable_shadows = scene_.flags[mjRND_SHADOW];
      reqs[0].enable_reflections = scene_.flags[mjRND_REFLECTION];

      mjr_defaultRenderRequest(&reqs[1]);
      reqs[1].scene = imgui_bridge_->GetScene();
      reqs[1].draw_mode = mjDRAW_MODE_COLOR;
      reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
      reqs[1].viewport = viewport;
      reqs[1].enable_shadows = false;
      reqs[1].enable_reflections = false;
      reqs[1].enable_post_processing = false;

      mjrf_render(filament_context_.get(), &reqs[0], 2, nullptr, 0);
    }
  }
}

void Renderer::DoSetBuffer(int framebuffer) {
  framebuffer_mode_ = framebuffer;
  if (IsClassic(gfx_)) {
    mjr_setBuffer(framebuffer, &render_context_);
  }
}

void Renderer::DoReadPixels(int width, int height, unsigned char* rgb) {
  if (!rgb) {
    return;
  }
  if (framebuffer_mode_ == 0) {
    mju_warning("ReadPixels is only supported for offscreen rendering.");
    return;
  }

  const mjrRect viewport = {0, 0, width, height};
  if (IsClassic(gfx_)) {
    mjr_readPixels(rgb, nullptr, viewport, &render_context_);
    mjr_setBuffer(mjFB_WINDOW, &render_context_);
    FlipImage(rgb, viewport.width, viewport.height, 3);
  } else {
    mjrDrawMode draw_mode = mjDRAW_MODE_COLOR;
    if (scene_.flags[mjRND_SEGMENT]) {
      draw_mode = mjDRAW_MODE_SEGMENTATION;
    } else if (scene_.flags[mjRND_DEPTH]) {
      draw_mode = mjDRAW_MODE_DEPTH;
    } else if (scene_.flags[mjRND_WIREFRAME]) {
      draw_mode = mjDRAW_MODE_WIREFRAME;
    }

    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);
    config.width = viewport.width;
    config.height = viewport.height;
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    auto target = CreateRenderTarget(filament_context_.get(), config);

    mjrRenderRequest reqs[2];
    mjr_defaultRenderRequest(&reqs[0]);
    reqs[0].scene = scene_bridge_->GetScene();
    reqs[0].draw_mode = draw_mode;
    reqs[0].camera = scene_bridge_->GetCamera();
    reqs[0].target = target.get();
    reqs[0].viewport = viewport;
    reqs[0].enable_shadows = scene_.flags[mjRND_SHADOW];
    reqs[0].enable_reflections = scene_.flags[mjRND_REFLECTION];

    mjr_defaultRenderRequest(&reqs[1]);
    reqs[1].scene = imgui_bridge_->GetScene();
    reqs[1].draw_mode = mjDRAW_MODE_COLOR;
    reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
    reqs[1].target = target.get();
    reqs[1].viewport = viewport;
    reqs[1].enable_shadows = false;
    reqs[1].enable_reflections = false;
    reqs[1].enable_post_processing = false;

    mjrReadPixelsRequest read_request;
    mjr_defaultReadPixelsRequest(&read_request);
    read_request.target = target.get();
    read_request.output = rgb;
    read_request.num_bytes = viewport.width * viewport.height * 3;

    const int num_requests = (framebuffer_mode_ == 2) ? 2 : 1;
    const mjrFrameHandle frame = mjrf_render(
        filament_context_.get(), &reqs[0], num_requests, &read_request, 1);
    mjrf_waitForFrame(filament_context_.get(), frame);
  }
  DoSetBuffer(mjFB_WINDOW);
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
    mjrFrameStats stats;
    mjr_defaultFrameStats(&stats);
    mjrf_getFrameStats(filament_context_.get(), 0, &stats);
    fps_ = stats.frame_rate;
  }
}

}  // namespace mujoco::platform

mjPLUGIN_LIB_INIT(renderer) {
  mujoco::platform::GuiPlugin plugin;
  plugin.name = "Filament";
  plugin.update = [](mujoco::platform::GuiPlugin* self) {
    mujoco::platform::PluginUpdate(self);
  };
  mujoco::platform::RegisterPlugin(plugin);
}
