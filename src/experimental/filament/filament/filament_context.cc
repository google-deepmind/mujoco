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
#include <utils/compiler.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/filament_platform_factory.h"
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/imgui_editor.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/render_target_util.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

FilamentContext::FilamentContext(const mjrFilamentConfig* config)
    : config_(*config) {
  FilamentPlatformSetup setup = CreateFilamentPlatform(config_);
  platform_ = std::move(setup.platform);

  filament::Engine::Builder engine_builder;
  engine_builder.backend(setup.backend);
  engine_builder.platform(platform_.get());
  engine_builder.feature("backend.disable_parallel_shader_compile",
                         setup.disable_parallel_shader_compile);
  engine_ = engine_builder.build();

  renderer_ = engine_->createRenderer();
#ifdef __EMSCRIPTEN__
  window_swap_chain_ = engine_->createSwapChain(nullptr);
#else
  if (config_.native_window) {
    window_swap_chain_ = engine_->createSwapChain(config_.native_window);
  } else {
    window_swap_chain_ =
        engine_->createSwapChain(config_.width, config_.height);
  }
#endif
  offscreen_swap_chain_ =
      engine_->createSwapChain(config_.width, config_.height);

  object_manager_ = std::make_unique<ObjectManager>(engine_);
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

void FilamentContext::Init(const mjModel* model) {
  scene_view_ = std::make_unique<SceneView>(object_manager_.get(), model);
  gui_view_ = std::make_unique<GuiView>(
      engine_, object_manager_->GetMaterial(ObjectManager::kUnlitUi));

  // Set clear options.
  filament::Renderer::ClearOptions opts;
  opts.clear = true;
  opts.discard = true;
  opts.clearColor = ReadElement(model, "filament.clearColor",
                                filament::math::float4(0, 0, 0, 1));
  renderer_->setClearOptions(opts);
}

void FilamentContext::Render(const mjrRect& viewport, const mjvScene* scene) {
  // If we're rendering to the window, and the window size has changed, we need
  // to reacquire the swap chain.
  if (scene_swap_chain_target_ == kWindowSwapChain &&
      (viewport.width != window_width_ || viewport.height != window_height_)) {
    if (window_width_ != 0 && window_height_ != 0) {
      if constexpr (UTILS_HAS_THREADING) {
        engine_->flushAndWait();
      }
      engine_->destroy(window_swap_chain_);
      window_swap_chain_ = engine_->createSwapChain(config_.native_window);
    }
    window_width_ = viewport.width;
    window_height_ = viewport.height;
  }

  scene_view_->SetViewport(viewport);
  scene_view_->UpdateScene(scene);
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
    if constexpr (UTILS_HAS_THREADING) {
      // Wait until previous frame is completed before requesting a new frame.
      engine_->flushAndWait();
    }

    if (renderer_->beginFrame(window_swap_chain_)) {
      scene_view_->Render(renderer_, last_render_mode_);

      if (gui_view_ && gui_swap_chain_target_ == kWindowSwapChain) {
        gui_view_->Render(renderer_);
      }

      renderer_->endFrame();
    }

    if constexpr (!UTILS_HAS_THREADING) {
      engine_->execute();
    }
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
  color_target_ = std::make_unique<RenderTargetAndTextures>(
      engine_, RenderTargetTextureType::kColor,
      RenderTargetTextureType::kDepth);
  color_target_->Prepare(width, height);

  depth_target_ = std::make_unique<RenderTargetAndTextures>(
      engine_, RenderTargetTextureType::kDepthColor,
      RenderTargetTextureType::kDepth);
  depth_target_->Prepare(width, height);
}

void FilamentContext::DestroyRenderTargets() {
  depth_target_.reset();
  color_target_.reset();
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
      scene_view_->Render(renderer_, last_render_mode_,
                          color_target_->GetRenderTarget());

      // Render the GUI to the texture as well if requested.
      if (gui_view_ && gui_swap_chain_target_ == kOffscreenSwapChain) {
        gui_view_->Render(renderer_, color_target_->GetRenderTarget());
      }

      const size_t num_bytes = viewport.width * viewport.height * 3;
      ReadColorPixels(renderer_, color_target_->GetRenderTarget(), viewport,
                      rgb, num_bytes);

      renderer_->endFrame();
    }
  }

  if (depth) {
    if (renderer_->beginFrame(offscreen_swap_chain_)) {
      scene_view_->Render(renderer_, SceneView::DrawMode::kDepth,
                          depth_target_->GetRenderTarget());

      const size_t num_bytes = viewport.width * viewport.height * sizeof(float);
      ReadDepthPixels(renderer_, depth_target_->GetRenderTarget(), viewport,
                      depth, num_bytes);

      renderer_->endFrame();
    }
  }

  if (rgb || depth) {
    if constexpr (UTILS_HAS_THREADING) {
      // Wait for rendering to copy back to buffer to complete.
      engine_->flushAndWait();
    }
  }
}

void FilamentContext::UploadMesh(const mjModel* model, int id) {
  if (!scene_view_) {
    mju_error("SceneView is not initialized.");
  }
  scene_view_->UploadMesh(model, id);
}

void FilamentContext::UploadTexture(const mjModel* model, int id) {
  if (!scene_view_) {
    mju_error("SceneView is not initialized.");
  }
  scene_view_->UploadTexture(model, id);
}

void FilamentContext::UploadHeightField(const mjModel* model, int id) {
  if (!scene_view_) {
    mju_error("SceneView is not initialized.");
  }
  scene_view_->UploadHeightField(model, id);
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

void FilamentContext::UpdateGui() { DrawGui(scene_view_.get()); }

}  // namespace mujoco
