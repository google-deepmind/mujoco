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

#include "experimental/platform/hal/filament_renderer.h"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <span>

#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_bridge.h"
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/ux/imgui_bridge.h"
#include "experimental/platform/ux/imgui_widgets.h"
#include "experimental/platform/ux/plugin.h"
#include "render/filament/mjrfilament_cpp.h"

namespace mujoco::platform {

static std::function<void()> g_update_gui_callback = nullptr;
static void PluginUpdate(GuiPlugin* plugin) {
  if (g_update_gui_callback) {
    g_update_gui_callback();
  }
}

FilamentRenderer::FilamentRenderer(void* native_window, GraphicsMode gfx)
    : native_window_(native_window), gfx_(gfx) {
  g_update_gui_callback = [this]() {
    if (main_scene_) {
      mjrf_DEBUG_drawImguiEditor(main_scene_.get());
    }
  };
}

FilamentRenderer::~FilamentRenderer() {
  g_update_gui_callback = nullptr;
  Deinit();
}

void FilamentRenderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    mjrfContextConfig cfg;
    mjrf_defaultContextConfig(&cfg);
    cfg.native_window = native_window_;
    cfg.force_software_rendering = IsSoftware(gfx_);
    cfg.graphics_api = IsOpenGl(gfx_) || IsWebGl(gfx_) ? mjGRAPHICS_API_OPENGL
                                                       : mjGRAPHICS_API_VULKAN;
    filament_context_ = CreateContext(cfg);

    main_scene_ = CreateScene(filament_context_.get(), {});
    ux_scene_ = CreateScene(filament_context_.get(), {});
    scene_bridge_ = std::make_unique<SceneBridge>(filament_context_.get(),
                                                  main_scene_.get(), model);
    imgui_bridge_ =
        std::make_unique<ImguiBridge>(filament_context_.get(), ux_scene_.get());
    scene_bridge_->SetDrawTextFunction(DrawTextAt);

    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 2000);
  }
}

void FilamentRenderer::Deinit() {
  if (filament_context_) {
    mjv_freeScene(&scene_);
    scene_bridge_.reset();
    imgui_bridge_.reset();
    ux_scene_.reset();
    main_scene_.reset();
    filament_context_.reset();
  }
}

void FilamentRenderer::Render(const mjModel* model, mjData* data,
                              const mjvPerturb* perturb, mjvCamera* camera,
                              const mjvOption* vis_option, int width,
                              int height, std::span<std::byte> pixels,
                              std::span<mjvGeom> extra_geoms) {
  if (!filament_context_) {
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
    if (pixels.size() != width * height * 3) {
      mju_error("Offscreen mode requires a pixel buffer of size %d.",
                width * height * 3);
    }
    framebuffer_mode_ = mjFB_OFFSCREEN + 1;
  }

  DoRender(width, height);

  if (render_to_texture) {
    unsigned char* ptr = reinterpret_cast<unsigned char*>(pixels.data());
    DoReadPixels(width, height, ptr);
  }

  UpdateFps();
}

void FilamentRenderer::RenderToTexture(const mjModel* model, mjData* data,
                                       mjvCamera* camera, int width, int height,
                                       std::byte* output) {
  if (!filament_context_) {
    return;
  }

  mjv_updateCamera(model, data, camera, &scene_);
  unsigned char* ptr = reinterpret_cast<unsigned char*>(output);

  framebuffer_mode_ = mjFB_OFFSCREEN;
  DoRender(width, height);
  DoReadPixels(width, height, ptr);
}

int FilamentRenderer::UploadImage(int texture_id, const std::byte* pixels,
                                  int width, int height, int bpp) {
  return imgui_bridge_->UploadImage(
      texture_id, reinterpret_cast<const unsigned char*>(pixels), width, height,
      bpp);
}

void FilamentRenderer::DoRender(int width, int height) {
  const mjrRect viewport = {0, 0, width, height};
  scene_bridge_->Update(viewport, &scene_);
  // Update the UX renderable entity after processing the scene in case there
  // are any elements in the scene which generate UX draw calls (e.g. labels).
  if (framebuffer_mode_ != 1) {
    imgui_bridge_->Update();
  }
  if (framebuffer_mode_ == 0) {
    mjrDrawMode draw_mode = mjDRAW_MODE_DEFAULT;
    if (scene_.flags[mjRND_SEGMENT]) {
      if (scene_.flags[mjRND_IDCOLOR]) {
        draw_mode = mjDRAW_MODE_SEGMENTATION_BY_ID;
      } else {
        draw_mode = mjDRAW_MODE_SEGMENTATION_BY_COLOR;
      }
    } else if (scene_.flags[mjRND_DEPTH]) {
      draw_mode = mjDRAW_MODE_DEPTH;
    } else if (scene_.flags[mjRND_WIREFRAME]) {
      draw_mode = mjDRAW_MODE_WIREFRAME;
    }

    mjrfRenderRequest reqs[2];

    mjrf_defaultRenderRequest(&reqs[0]);
    reqs[0].scene = main_scene_.get();
    reqs[0].draw_mode = draw_mode;
    reqs[0].camera = scene_bridge_->GetCamera();
    reqs[0].viewport = viewport;
    reqs[0].enable_shadows = scene_.flags[mjRND_SHADOW];
    reqs[0].enable_reflections = scene_.flags[mjRND_REFLECTION];

    mjrf_defaultRenderRequest(&reqs[1]);
    reqs[1].scene = ux_scene_.get();
    reqs[1].draw_mode = mjDRAW_MODE_DEFAULT;
    reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
    reqs[1].viewport = viewport;
    reqs[1].enable_shadows = false;
    reqs[1].enable_reflections = false;
    reqs[1].enable_post_processing = false;

    mjrf_render(filament_context_.get(), &reqs[0], 2, nullptr, 0);
  }
}

void FilamentRenderer::DoReadPixels(int width, int height, unsigned char* rgb) {
  if (!rgb) {
    return;
  }
  if (framebuffer_mode_ == 0) {
    mju_warning("ReadPixels is only supported for offscreen rendering.");
    return;
  }

  const mjrRect viewport = {0, 0, width, height};
  mjrDrawMode draw_mode = mjDRAW_MODE_DEFAULT;
  if (scene_.flags[mjRND_SEGMENT]) {
    if (scene_.flags[mjRND_IDCOLOR]) {
      draw_mode = mjDRAW_MODE_SEGMENTATION_BY_ID;
    } else {
      draw_mode = mjDRAW_MODE_SEGMENTATION_BY_COLOR;
    }
  } else if (scene_.flags[mjRND_DEPTH]) {
    draw_mode = mjDRAW_MODE_DEPTH;
  } else if (scene_.flags[mjRND_WIREFRAME]) {
    draw_mode = mjDRAW_MODE_WIREFRAME;
  }

  mjrfRenderTargetConfig config;
  mjrf_defaultRenderTargetConfig(&config);
  config.width = viewport.width;
  config.height = viewport.height;
  config.color_format = mjPIXEL_FORMAT_RGB8;
  config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
  auto target = CreateRenderTarget(filament_context_.get(), config);

  mjrfRenderRequest reqs[2];
  mjrf_defaultRenderRequest(&reqs[0]);
  reqs[0].scene = main_scene_.get();
  reqs[0].draw_mode = draw_mode;
  reqs[0].camera = scene_bridge_->GetCamera();
  reqs[0].target = target.get();
  reqs[0].viewport = viewport;
  reqs[0].enable_shadows = scene_.flags[mjRND_SHADOW];
  reqs[0].enable_reflections = scene_.flags[mjRND_REFLECTION];

  mjrf_defaultRenderRequest(&reqs[1]);
  reqs[1].scene = ux_scene_.get();
  reqs[1].draw_mode = mjDRAW_MODE_DEFAULT;
  reqs[1].camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
  reqs[1].target = target.get();
  reqs[1].viewport = viewport;
  reqs[1].enable_shadows = false;
  reqs[1].enable_reflections = false;
  reqs[1].enable_post_processing = false;

  mjrfReadPixelsRequest read_request;
  mjrf_defaultReadPixelsRequest(&read_request);
  read_request.target = target.get();
  read_request.output = rgb;
  read_request.num_bytes = viewport.width * viewport.height * 3;

  const int num_requests = (framebuffer_mode_ == 2) ? 2 : 1;
  const mjrfFrameHandle frame = mjrf_render(filament_context_.get(), &reqs[0],
                                            num_requests, &read_request, 1);
  mjrf_waitForFrame(filament_context_.get(), frame);
  framebuffer_mode_ = mjFB_WINDOW;
}

double FilamentRenderer::GetFps() { return fps_; }

void FilamentRenderer::UpdateFps() {
  mjrfFrameStats stats;
  mjrf_defaultFrameStats(&stats);
  mjrf_getFrameStats(filament_context_.get(), 0, &stats);
  fps_ = stats.frame_rate;
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
