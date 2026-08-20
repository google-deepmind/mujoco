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

#include <cstddef>
#include <functional>
#include <memory>
#include <span>

#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/ux/imgui_bridge.h"
#include "experimental/platform/ux/imgui_widgets.h"
#include "experimental/platform/ux/plugin.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_decorations.h"
#include "render/filament/support/model_lights.h"
#include "render/filament/support/model_objects.h"
#include "render/filament/support/model_renderables.h"

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
    for (int i = 0; i < mjNRNDFLAG; i++) {
      render_flags_[i] = (mjRNDSTRING[i][1][0] == '1');
    }

    mjrfContextConfig cfg;
    mjrf_defaultContextConfig(&cfg);
    cfg.native_window = native_window_;
    cfg.force_software_rendering = IsSoftware(gfx_);
    cfg.graphics_api = IsOpenGl(gfx_) || IsWebGl(gfx_) ? mjGRAPHICS_API_OPENGL
                                                       : mjGRAPHICS_API_VULKAN;
    filament_context_ = CreateContext(cfg);

    float clear_color[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    const int id = mj_name2id(model, mjOBJ_NUMERIC, "filament.clearColor");
    if (id >= 0 && model->numeric_size[id] == 4) {
      const mjtNum* ptr = model->numeric_data + model->numeric_adr[id];
      for (int i = 0; i < 4; ++i) {
        clear_color[i] = static_cast<float>(ptr[i]);
      }
    }
    mjrf_setClearColor(filament_context_.get(), &clear_color[0]);

    main_scene_ = CreateScene(filament_context_.get(), {});
    mjrf_configureSceneFromModel(main_scene_.get(), model);
    model_objects_ =
        std::make_unique<ModelObjects>(model, filament_context_.get());
    model_lights_ =
        std::make_unique<ModelLights>(main_scene_.get(), model_objects_.get());
    model_renderables_ = std::make_unique<ModelRenderables>(
        main_scene_.get(), model_objects_.get());
    model_decorations_ = std::make_unique<ModelDecorations>(
        filament_context_.get(), main_scene_.get(), model);

    ux_scene_ = CreateScene(filament_context_.get(), {});
    imgui_bridge_ =
        std::make_unique<ImguiBridge>(filament_context_.get(), ux_scene_.get());

    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_RGB8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    render_target_ = CreateRenderTarget(filament_context_.get(), config);
  }
}

void FilamentRenderer::Deinit() {
  if (filament_context_) {
    model_objects_.reset();
    model_lights_.reset();
    model_renderables_.reset();
    model_decorations_.reset();
    imgui_bridge_.reset();
    render_target_.reset();
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

  const mjrRect viewport = {0, 0, width, height};

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

  mjvPerturb default_perturb;
  if (perturb == nullptr) {
    mjv_defaultPerturb(&default_perturb);
    perturb = &default_perturb;
  }

  model_lights_->Update(data);
  model_renderables_->Update(data);

  if (vis_option) {
    model_renderables_->SetOptions(*vis_option);
  }
  if (perturb->select > 0) {
    model_renderables_->MarkAsSelected(mjOBJ_BODY, perturb->select);
  } else if (perturb->flexselect >= 0) {
    model_renderables_->MarkAsSelected(mjOBJ_FLEX, perturb->flexselect);
  } else if (perturb->skinselect >= 0) {
    model_renderables_->MarkAsSelected(mjOBJ_SKIN, perturb->skinselect);
  } else {
    model_renderables_->MarkAsSelected(mjOBJ_UNKNOWN, -1);
  }

  model_decorations_->Update(data, vis_option, perturb, camera, viewport,
                             DrawTextAt, extra_geoms);

  imgui_bridge_->Update();

  mjrfRenderRequest reqs[2];
  BuildMainRenderRequest(&reqs[0], viewport,
                         mjv_camera2GLCamera(model, data, camera));
  BuildUxRenderRequest(&reqs[1], viewport);

  mjrfFrameHandle frame = 0;
  if (pixels.empty()) {
    frame = mjrf_render(filament_context_.get(), &reqs[0], 2, nullptr, 0);
  } else {
    if (pixels.size() != width * height * 3) {
      mju_error("Offscreen mode requires a pixel buffer of size %d.",
                width * height * 3);
    }

    mjrf_resizeRenderTarget(render_target_.get(), width, height);
    reqs[0].target = render_target_.get();
    reqs[1].target = render_target_.get();

    mjrfReadPixelsRequest read_request;
    mjrf_defaultReadPixelsRequest(&read_request);
    read_request.target = render_target_.get();
    read_request.output = pixels.data();
    read_request.num_bytes = viewport.width * viewport.height * 3;

    frame = mjrf_render(filament_context_.get(), &reqs[0], 2, &read_request, 1);
  }

  mjrf_waitForFrame(filament_context_.get(), frame);

  mjrfFrameStats stats;
  mjrf_defaultFrameStats(&stats);
  mjrf_getFrameStats(filament_context_.get(), frame, &stats);
  fps_ = stats.frame_rate;
}

void FilamentRenderer::RenderToTexture(const mjModel* model, mjData* data,
                                       mjvCamera* camera, int width, int height,
                                       std::byte* output) {
  if (!filament_context_) {
    return;
  }
  if (!output) {
    return;
  }

  mjrf_resizeRenderTarget(render_target_.get(), width, height);

  mjrfRenderRequest request;
  BuildMainRenderRequest(&request, {0, 0, width, height},
                         mjv_camera2GLCamera(model, data, camera));
  request.target = render_target_.get();

  mjrfReadPixelsRequest read_request;
  mjrf_defaultReadPixelsRequest(&read_request);
  read_request.target = render_target_.get();
  read_request.output = output;
  read_request.num_bytes = width * height * 3;

  const mjrfFrameHandle frame =
      mjrf_render(filament_context_.get(), &request, 1, &read_request, 1);
  mjrf_waitForFrame(filament_context_.get(), frame);
}

int FilamentRenderer::UploadImage(int texture_id, const std::byte* pixels,
                                  int width, int height, int bpp) {
  return imgui_bridge_->UploadImage(
      texture_id, reinterpret_cast<const unsigned char*>(pixels), width, height,
      bpp);
}

double FilamentRenderer::GetFps() { return fps_; }

void FilamentRenderer::BuildMainRenderRequest(mjrfRenderRequest* request,
                                              const mjrRect& viewport,
                                              const mjrCamera& camera) {
  mjrDrawMode draw_mode = mjDRAW_MODE_DEFAULT;
  if (render_flags_[mjRND_SEGMENT]) {
    if (render_flags_[mjRND_IDCOLOR]) {
      draw_mode = mjDRAW_MODE_SEGMENTATION_BY_ID;
    } else {
      draw_mode = mjDRAW_MODE_SEGMENTATION_BY_COLOR;
    }
  } else if (render_flags_[mjRND_DEPTH]) {
    draw_mode = mjDRAW_MODE_DEPTH;
  } else if (render_flags_[mjRND_WIREFRAME]) {
    draw_mode = mjDRAW_MODE_WIREFRAME;
  }

  mjrf_defaultRenderRequest(request);
  request->scene = main_scene_.get();
  request->draw_mode = draw_mode;
  request->camera = camera;
  request->viewport = viewport;
  request->enable_shadows = render_flags_[mjRND_SHADOW];
  request->enable_reflections = render_flags_[mjRND_REFLECTION];
}

void FilamentRenderer::BuildUxRenderRequest(mjrfRenderRequest* request,
                                            const mjrRect& viewport) {
  mjrf_defaultRenderRequest(request);
  request->scene = ux_scene_.get();
  request->draw_mode = mjDRAW_MODE_DEFAULT;
  request->camera = imgui_bridge_->GetCamera(viewport.width, viewport.height);
  request->viewport = viewport;
  request->enable_shadows = false;
  request->enable_reflections = false;
  request->enable_post_processing = false;
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
