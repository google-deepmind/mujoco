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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_

#include <cstddef>
#include <cstdint>
#include <span>
#include <memory>

#include <backend/Platform.h>
#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/SwapChain.h>
#include <math/vec4.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Manages the filament renderer and provides APIs for rendering scenes.
class FilamentContext {
 public:
  explicit FilamentContext(const mjrFilamentConfig* config);
  ~FilamentContext();

  // Information needed to render a single image of a scene.
  struct RenderRequest {
    // The scene to render.
    SceneView* scene = nullptr;

    // The method (e.g. Color, Depth, Segmentation, etc.) to use for rendering.
    DrawMode draw_mode = DrawMode::Color;

    // The camera from which to render the scene.
    mjvGLCamera camera;

    // The dimensions of the output image.
    int width = 0;
    int height = 0;

    // The render target into which to render the image. If nullptr, the image
    // will be rendered to the window (as previously configured in
    // mjrFilamentConfig::native_window).
    RenderTarget* target = nullptr;
  };

  // Information needed to read pixels from a render target.
  struct ReadPixelsRequest {
    RenderTarget* target = nullptr;

    // The buffer into which the read pixels will be written.
    uint8_t* output = nullptr;

    // The number of bytes in the output buffer. This should match the size of
    // the render target texture.
    std::size_t num_bytes = 0;

    // Callback when the read pixels operation is complete. This will be called
    // during WaitForFrame() or in a subsequent call to Render(). This function
    // can optionally be used to free the output buffer if needed.
    void (*read_completed_callback)(void* user_data) = nullptr;

    // User data to pass to the completion callback.
    void* user_data = nullptr;
  };

  // Rendering is asynchronous by nature. Each render request is assigned a
  // unique Handle which can be used to query the status of the request. The
  // Handle can also be used to block until the request is completed.
  using FrameHandle = std::uint64_t;

  // Queues the given render requests for rendering. This function copies the
  // necessary data from the requests into the renderer thread and returns
  // immediately afterwards. The renderer thread will then perform the actual
  // rendering on the GPU. Callers can use WaitForFrame to block until the
  // rendering is complete.
  FrameHandle Render(std::span<const RenderRequest> render_requests,
                     std::span<const ReadPixelsRequest> read_requests = {});

  // Blocks until the given frame has completed rendering.
  void WaitForFrame(FrameHandle frame_handle);

  // Sets the clear color for the renderer.
  void SetClearColor(const filament::math::float4& color);

  // Returns the current frame rate of the renderer.
  double GetFrameRate() const;

  filament::Engine* GetEngine() const { return engine_; }

  ObjectManager* GetObjectManager() const { return object_manager_.get(); }

  FilamentContext(const FilamentContext&) = delete;
  FilamentContext& operator=(const FilamentContext&) = delete;

 private:
  mjrFilamentConfig config_;

  filament::Engine* engine_ = nullptr;
  filament::Renderer* renderer_ = nullptr;
  filament::SwapChain* window_swap_chain_ = nullptr;
  filament::SwapChain* offscreen_swap_chain_ = nullptr;
  std::unique_ptr<filament::backend::Platform> platform_;
  std::unique_ptr<ObjectManager> object_manager_;
  int window_width_ = 0;
  int window_height_ = 0;
  std::uint64_t frame_counter_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_CONTEXT_H_
