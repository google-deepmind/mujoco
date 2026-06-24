// Copyright 2026 DeepMind Technologies Limited
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

#include "render/filament/core/reflection_manager.h"

#include <memory>

#include <filament/Engine.h>
#include <mujoco/mjrfilament.h>
#include "render/filament/core/render_target.h"

namespace mujoco {

ReflectionManager::ReflectionManager(filament::Engine* engine)
    : engine_(engine) {}

ReflectionManager::~ReflectionManager() {}

void ReflectionManager::ClearRenderables() { renderables_.clear(); }

mjrfTexture* ReflectionManager::Register(mjrfRenderable* renderable, int width,
                                         int height) {
  if (targets_.size() == renderables_.size()) {
    mjrfRenderTargetConfig config;
    mjrf_defaultRenderTargetConfig(&config);
    config.color_format = mjPIXEL_FORMAT_RGBA8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    targets_.push_back(std::make_unique<RenderTarget>(engine_, config));
  }
  RenderTarget* target = targets_[renderables_.size()].get();
  target->Prepare(width, height);
  renderables_.push_back(renderable);
  return target->GetColorTexture();
}

int ReflectionManager::GetNumRenderables() const { return renderables_.size(); }

mjrfRenderable* ReflectionManager::GetRenderable(int index) const {
  return renderables_[index];
}

const RenderTarget* ReflectionManager::GetRenderTarget(int index) const {
  return targets_[index].get();
}

}  // namespace mujoco
