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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_

#include <cstddef>
#include <functional>

#include <mujoco/mujoco.h>

namespace mujoco::platform {

// Renders the mujoco simulation and the imgui state into the active window
// using the filament rendering backend.
class Renderer {
 public:
  // Function that creates a mjrContext for the given model. We use a function
  // to allow different mjrContext implementations to be created without
  // requiring a direct dependency on them.
  using MakeContextFn = std::function<void(const mjModel* m, mjrContext* con)>;

  explicit Renderer(MakeContextFn make_context_fn);
  ~Renderer();

  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;

  // Initializes the renderer with the given mjModel.
  void Init(const mjModel* model);

  // Renders the simulation state into the active window. Also renders the imgui
  // state, but that is obtained directly from the ImGui library.
  void Render(const mjModel* model, mjData* data, const mjvPerturb* perturb,
              mjvCamera* camera, const mjvOption* vis_option, int width,
              int height);

  // Populates the given output buffer with RGB888 pixel data. The size of the
  // output buffer must be at least width * height * 3.
  void RenderToTexture(const mjModel* model, mjData* data, mjvCamera* camera,
                       int width, int height, std::byte* output);

  // Rendering flags.
  mjtByte* GetRenderFlags() { return scene_.flags; }

  // Returns the render context.
  mjrContext& GetContext() { return render_context_; }
  const mjrContext& GetContext() const { return render_context_; }

 private:
  // Resets the renderer; no rendering will occur until Init() is called again.
  void Deinit();

  MakeContextFn make_context_fn_;
  mjrContext render_context_;
  mjvScene scene_;
  bool initialized_ = false;
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_H_
