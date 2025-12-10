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

#include "experimental/platform/renderer.h"

#include <chrono>
#include <string>

#include "experimental/platform/helpers.h"
#include <mujoco/mujoco.h>

namespace mujoco::platform {

Renderer::Renderer(MakeContextFn make_context_fn)
    : make_context_fn_(make_context_fn) {
}

Renderer::~Renderer() { Deinit(); }

void Renderer::Init(const mjModel* model) {
  Deinit();
  if (model) {
    mjr_defaultContext(&render_context_);
    make_context_fn_(model, &render_context_);
    mjv_defaultScene(&scene_);
    mjv_makeScene(model, &scene_, 2000);
    initialized_ = true;
  }
}

void Renderer::Deinit() {
  if (initialized_) {
    mjv_freeScene(&scene_);
    mjr_freeContext(&render_context_);
    initialized_ = false;
  }
}

void Renderer::Render(const mjModel* model, mjData* data,
                      const mjvPerturb* perturb, mjvCamera* camera,
                      const mjvOption* vis_option, int width, int height) {
  if (initialized_) {
    mjv_updateScene(model, data, vis_option, perturb, camera, mjCAT_ALL,
                    &scene_);
  }

  mjrRect main_viewport = {0, 0, width, height};
  mjr_render(main_viewport, data ? &scene_ : nullptr, &render_context_);
}

void Renderer::SaveScreenshot(const std::string& filename, int width,
                              int height) {
  SaveScreenshotToWebp(width, height, &render_context_, filename);
}

}  // namespace mujoco::platform
