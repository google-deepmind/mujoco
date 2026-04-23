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

#include <cstdint>
#include <memory>
#include <span>
#include <utility>

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
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/filament_platform_factory.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/filament/scene_view.h"
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
  object_manager_.reset();
  engine_->destroy(renderer_);
  engine_->destroy(window_swap_chain_);
  engine_->destroy(offscreen_swap_chain_);
  filament::Engine::destroy(engine_);
}

FilamentContext::FrameHandle FilamentContext::Render(
    std::span<const RenderRequest> requests,
    std::span<const ReadPixelsRequest> read_requests) {
  if (read_requests.size() > 1) {
    mju_error("Only one read request is supported for now.");
  }

  bool render_began = false;
  RenderTarget* current_target = nullptr;
  for (const RenderRequest& request : requests) {
    if (request.target != current_target && render_began) {
      renderer_->endFrame();
      render_began = false;
    }
    current_target = request.target;

    if (current_target == nullptr) {
      if (!read_requests.empty()) {
        mju_error("Cannot read pixels from the window.");
      }

      if constexpr (UTILS_HAS_THREADING) {
        // Wait until previous frame is completed before requesting a new frame.
        engine_->flushAndWait();
      }

      // If the window size has changed, we need to reacquire the swap chain.
      if (request.width != window_width_ || request.height != window_height_) {
        if (window_width_ != 0 && window_height_ != 0) {
          engine_->destroy(window_swap_chain_);
          window_swap_chain_ = engine_->createSwapChain(config_.native_window);
        }
        window_width_ = request.width;
        window_height_ = request.height;
      }

      if (!render_began) {
        render_began = renderer_->beginFrame(window_swap_chain_);
      }
      if (render_began) {
        SceneView::RenderRequest scene_view_request;
        scene_view_request.draw_mode = request.draw_mode;
        scene_view_request.viewport = {0, 0, request.width, request.height};
        scene_view_request.camera = request.camera;
        request.scene->Render(renderer_, scene_view_request);
      }
    } else {
      if (read_requests.empty()) {
        mju_error(
            "Rendering to a render target without a read request is pointless.");
      }

      const ReadPixelsRequest& read_request = read_requests[0];
      if (read_request.num_bytes == 0) {
        mju_error("Output buffer size is zero.");
      }

      if (!render_began) {
        render_began = renderer_->beginFrame(offscreen_swap_chain_);
      }
      if (render_began) {
        SceneView::RenderRequest scene_view_request;
        scene_view_request.draw_mode = request.draw_mode;
        scene_view_request.viewport = {0, 0, request.width, request.height};
        scene_view_request.camera = request.camera;
        scene_view_request.target = request.target;
        request.scene->Render(renderer_, scene_view_request);
        request.target->ReadColorPixels(renderer_, read_request.output,
                                        read_request.num_bytes);
      }
    }
  }

  if (render_began) {
    renderer_->endFrame();
    render_began = false;
    if constexpr (!UTILS_HAS_THREADING) {
      engine_->execute();
    }
  }

  if (!read_requests.empty()) {
    engine_->flushAndWait();
    if (read_requests[0].read_completed_callback) {
      read_requests[0].read_completed_callback(read_requests[0].user_data);
    }
  }

  return ++frame_counter_;
}

void FilamentContext::WaitForFrame(FrameHandle frame_handle) {
  if (frame_counter_ < frame_handle) {
    engine_->flushAndWait();
  }
}

void FilamentContext::SetClearColor(const filament::math::float4& color) {
  filament::Renderer::ClearOptions opts;
  opts.clear = true;
  opts.discard = true;
  opts.clearColor = color;
  renderer_->setClearOptions(opts);
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

}  // namespace mujoco
