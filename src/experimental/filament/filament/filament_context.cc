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

#include "experimental/filament/filament/filament_context.h"

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <utility>

#include <backend/DriverEnums.h>
#include <filament/Engine.h>
#include <filament/IndexBuffer.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/RenderTarget.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>
#include <filament/View.h>
#include <math/vec4.h>
#include <utils/FixedCapacityVector.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/imgui_editor.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture_util.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

FilamentContext::FilamentContext(const mjrFilamentConfig* config,
                                 const mjModel* model, mjrContext* con)
    : config_(*config), context_(con), model_(model) {
#if defined( __EMSCRIPTEN__)
  filament::Engine::Backend backend = filament::Engine::Backend::OPENGL;
#else
  filament::Engine::Backend backend = filament::Engine::Backend::VULKAN;
#endif

  switch (config_.graphics_api) {
    case mjGFX_DEFAULT:
      // Use the default based on the platform above.
      break;
    case mjGFX_OPENGL:
      backend = filament::Engine::Backend::OPENGL;
      break;
    case mjGFX_VULKAN:
      backend = filament::Engine::Backend::VULKAN;
      break;
    default:
      mju_error("Unsupported graphics API: %d", config_.graphics_api);
  }

  const int width = model_->vis.global.offwidth;
  const int height = model_->vis.global.offheight;

  engine_ = filament::Engine::create(backend);
  renderer_ = engine_->createRenderer();
  #ifdef __EMSCRIPTEN__
    window_swap_chain_ = engine_->createSwapChain(nullptr);
  #else
  if (config_.native_window) {
    window_swap_chain_ = engine_->createSwapChain(config_.native_window);
  } else {
    window_swap_chain_ = engine_->createSwapChain(width, height);
  }
  #endif

  offscreen_swap_chain_ = engine_->createSwapChain(width, height);
  object_manager_ = std::make_unique<ObjectManager>(model, engine_, config);

  // Set clear options.
  filament::Renderer::ClearOptions opts;
  opts.clear = true;
  opts.discard = true;
  opts.clearColor = {0, 0, 0, 1};
  renderer_->setClearOptions(opts);

  // Copy parameters from model to context.
  if (model_) {
    context_->shadowClip = model_->stat.extent * model_->vis.map.shadowclip;
    context_->shadowScale = model_->vis.map.shadowscale;
    context_->offWidth = model_->vis.global.offwidth;
    context_->offHeight = model_->vis.global.offheight;
    context_->offSamples = model_->vis.quality.offsamples;
    context_->fogStart =
        (float)(model_->stat.extent * model_->vis.map.fogstart);
    context_->fogEnd = (float)(model_->stat.extent * model_->vis.map.fogend);
    context_->fogRGBA[0] = model_->vis.rgba.fog[0];
    context_->fogRGBA[1] = model_->vis.rgba.fog[1];
    context_->fogRGBA[2] = model_->vis.rgba.fog[2];
    context_->fogRGBA[3] = model_->vis.rgba.fog[3];
    context_->lineWidth = model_->vis.global.linewidth;
    context_->shadowSize = model_->vis.quality.shadowsize;
    context_->readPixelFormat = 0x1907;  // 0x1907 = GL_RGB;
    context_->ntexture = model_->ntex;
    for (int i = 0; i < model_->ntex; ++i) {
      context_->textureType[i] = model_->tex_type[i];
    }
  }

  scene_view_ = std::make_unique<SceneView>(engine_, object_manager_.get());
  if (config_.enable_gui) {
    gui_view_ = std::make_unique<GuiView>(engine_, object_manager_.get());
  }
}

FilamentContext::~FilamentContext() {
  DestroyRenderTargets();
  gui_view_.reset();
  scene_view_.reset();
  object_manager_.reset();
  engine_->destroy(renderer_);
  engine_->destroy(window_swap_chain_);
  engine_->destroy(offscreen_swap_chain_);
  filament::Engine::destroy(engine_);
}

void FilamentContext::Render(const mjrRect& viewport, const mjvScene* scene,
                             const mjrContext* con) {
  if (con != context_) {
    mju_error("Unexpected context.");
  }

  scene_view_->SetViewport(viewport);
  scene_view_->UpdateScene(con, scene);
  // Update the UX renderable entity after processing the scene in case there
  // are any elements in the scene which generate UX draw calls (e.g. labels).
  if (gui_view_ && gui_swap_chain_target_ == scene_swap_chain_target_) {
    // Prepare the filament Renderable that contains the GUI draw commands. We
    // must call this function even if we do not plan on rendering the GUI to
    // ensure the ImGui state is updated.
    gui_view_->UpdateRenderable();
  }

  last_render_mode_ = SceneView::DrawMode::kNormal;
  if (scene->flags[mjRND_SEGMENT]) {
    last_render_mode_ = SceneView::DrawMode::kSegmentation;
  } else if (scene->flags[mjRND_DEPTH]) {
    last_render_mode_ = SceneView::DrawMode::kDepth;
  }

  // Render the frame if we're not rendering to a texture.
  if (scene_swap_chain_target_ == kWindowSwapChain) {
    #ifndef __EMSCRIPTEN__
      // Wait until previous frame is completed before requesting a new frame.
      engine_->flushAndWait();
    #endif

    if (renderer_->beginFrame(window_swap_chain_)) {
      filament::View* fview = scene_view_->PrepareRenderView(last_render_mode_);
      renderer_->render(fview);

      if (gui_swap_chain_target_ == kWindowSwapChain) {
        fview = gui_view_ ? gui_view_->PrepareRenderView() : nullptr;
        if (fview) {
          renderer_->render(fview);
        }
      }

      renderer_->endFrame();
    }

    #ifdef __EMSCRIPTEN__
      engine_->execute();
    #endif
  }
}

void FilamentContext::SetFrameBuffer(int framebuffer) {
  switch (framebuffer) {
    case mjFB_WINDOW:
      scene_swap_chain_target_ = kWindowSwapChain;
      gui_swap_chain_target_ = kWindowSwapChain;
      break;
    case mjFB_OFFSCREEN:
      scene_swap_chain_target_ = kOffscreenSwapChain;
      gui_swap_chain_target_ = kWindowSwapChain;
      break;
    case 2:  // No official constant fo this.
      scene_swap_chain_target_ = kOffscreenSwapChain;
      gui_swap_chain_target_ = kOffscreenSwapChain;
      break;
    default:
      mju_error("Invalid framebuffer mode: %d", framebuffer);
  }

  if (framebuffer == 0) {
    DestroyRenderTargets();
  }
}

void FilamentContext::PrepareRenderTargets(int width, int height) {
  for (int i = 0; i < kNumRenderTargetTextureTypes; ++i) {
    target_textures_[i] = CreateRenderTargetTexture(
        engine_, width, height, static_cast<RenderTargetTextureType>(i));
  }

  // Render target for color pass.
  filament::RenderTarget::Builder color_target_builder;
  color_target_builder.texture(filament::RenderTarget::AttachmentPoint::COLOR0,
                               target_textures_[kRenderTargetColor]);
  color_target_builder.texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                               target_textures_[kRenderTargetDepth]);
  color_target_ = color_target_builder.build(*engine_);

  // Render target for depth pass.
  filament::RenderTarget::Builder depth_target_builder;
  depth_target_builder.texture(filament::RenderTarget::AttachmentPoint::COLOR0,
                               target_textures_[kRenderTargetDepthColor]);
  depth_target_builder.texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                               target_textures_[kRenderTargetDepth]);
  depth_target_ = depth_target_builder.build(*engine_);
}

void FilamentContext::DestroyRenderTargets() {
  if (depth_target_) {
    engine_->destroy(depth_target_);
    depth_target_ = nullptr;
  }
  if (color_target_) {
    engine_->destroy(color_target_);
    color_target_ = nullptr;
  }
  for (int i = 0; i < kNumRenderTargetTextureTypes; ++i) {
    if (target_textures_[i]) {
      engine_->destroy(target_textures_[i]);
      target_textures_[i] = nullptr;
    }
  }
}

static void ReadColorPixels(filament::Renderer* renderer,
                            filament::RenderTarget* target, mjrRect viewport,
                            unsigned char* buffer, size_t num_bytes) {
  filament::backend::PixelBufferDescriptor descriptor(
      buffer, num_bytes, filament::backend::PixelDataFormat::RGB,
      filament::backend::PixelDataType::UBYTE);
  renderer->readPixels(target, viewport.left, viewport.bottom, viewport.width,
                       viewport.height, std::move(descriptor));
}

static void ReadDepthPixels(filament::Renderer* renderer,
                            filament::RenderTarget* target, mjrRect viewport,
                            float* buffer, size_t num_bytes) {
  filament::backend::PixelBufferDescriptor descriptor(
      buffer, num_bytes, filament::backend::PixelDataFormat::R,
      filament::backend::PixelDataType::FLOAT);
  renderer->readPixels(target, viewport.left, viewport.bottom, viewport.width,
                       viewport.height, std::move(descriptor));
}

void FilamentContext::ReadPixels(mjrRect viewport, unsigned char* rgb,
                                 float* depth) {
  if (scene_swap_chain_target_ != kOffscreenSwapChain) {
    mju_error("Cannot read pixels unless framebuffer is set.");
  }
  if (color_target_ == nullptr || depth_target_ == nullptr) {
    if (viewport.left != 0) {
      mju_error("Reading subpixels not supported.");
    }
    if (viewport.bottom != 0) {
      mju_error("Reading subpixels not supported.");
    }
    PrepareRenderTargets(viewport.width, viewport.height);
  }

  if (rgb) {
    if (renderer_->beginFrame(offscreen_swap_chain_)) {
      filament::View* fview =
          scene_view_->PrepareRenderView(last_render_mode_);

      // We need to disable msaa in order to render to texture.
      auto options = fview->getMultiSampleAntiAliasingOptions();
      fview->setMultiSampleAntiAliasingOptions({
          .enabled = false,
      });
      fview->setRenderTarget(color_target_);
      renderer_->render(fview);
      fview->setRenderTarget(nullptr);
      fview->setMultiSampleAntiAliasingOptions(options);

      // Render the GUI to the texture as well if requested.
      if (gui_swap_chain_target_ == kOffscreenSwapChain) {
        fview = gui_view_ ? gui_view_->PrepareRenderView() : nullptr;
        if (fview) {
          fview->setRenderTarget(color_target_);
          renderer_->render(fview);
          fview->setRenderTarget(nullptr);
        }
      }

      const size_t num_bytes = viewport.width * viewport.height * 3;
      ReadColorPixels(renderer_, color_target_, viewport, rgb, num_bytes);

      renderer_->endFrame();
    }
  }

  if (depth) {
    if (renderer_->beginFrame(offscreen_swap_chain_)) {
      filament::View* fview =
          scene_view_->PrepareRenderView(SceneView::DrawMode::kDepth);
      fview->setRenderTarget(depth_target_);
      renderer_->render(fview);
      fview->setRenderTarget(nullptr);

      const size_t num_bytes = viewport.width * viewport.height * sizeof(float);
      ReadDepthPixels(renderer_, depth_target_, viewport, depth, num_bytes);

      renderer_->endFrame();
    }
  }

  if (rgb || depth) {
    // Wait for rendering and copy back to buffer to complete.
    engine_->flushAndWait();
  }
}

void FilamentContext::UploadMesh(const mjModel* model, int id) {
  object_manager_->UploadMesh(model, id);
}

void FilamentContext::UploadTexture(const mjModel* model, int id) {
  object_manager_->UploadTexture(model, id);
}

void FilamentContext::UploadHeightField(const mjModel* model, int id) {
  object_manager_->UploadHeightField(model, id);
}

uintptr_t FilamentContext::UploadGuiImage(uintptr_t tex_id,
                                          const uint8_t* pixels, int width,
                                          int height, int bpp) {
  if (gui_view_) {
    return gui_view_->UploadImage(tex_id, pixels, width, height, bpp);
  }
  return 0;
}

double FilamentContext::GetFrameRate() const {
  utils::FixedCapacityVector<filament::Renderer::FrameInfo> frame_info =
      renderer_->getFrameInfoHistory(1);
  if (frame_info.empty()) {
    return 0;
  }
  const int64_t ns = frame_info[0].denoisedGpuFrameDuration;
  return 1.0e9 / static_cast<double>(ns);
}

void FilamentContext::UpdateGui() {
  DrawGui(scene_view_.get());
}

}  // namespace mujoco
