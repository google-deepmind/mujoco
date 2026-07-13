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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_CORE_OUTLINER_H_
#define MUJOCO_SRC_RENDER_FILAMENT_CORE_OUTLINER_H_

#include <cstdint>
#include <memory>

#include <filament/Camera.h>
#include <filament/Renderer.h>
#include <filament/View.h>
#include <math/vec4.h>
#include <utils/Entity.h>
#include "render/filament/core/object_manager.h"
#include "render/filament/core/render_target.h"

namespace mujoco {

// Renders an outline of selected objects.
//
// This class uses the "jump flood" algorithm to create an outline of selected
// objects.
class Outliner {
 public:
  Outliner(ObjectManager* object_mgr, uint8_t layer_mask,
           filament::math::float4 color, float thickness);
  ~Outliner();

  Outliner(const Outliner&) = delete;
  Outliner& operator=(const Outliner&) = delete;

  // Renders the outline of the selected objects in the given view. The outline
  // will be rendered on top of the provided render target. This assumes that
  // any objects to be outlined have been assigned an `OutlineFlatten` material
  // instance.
  void Render(filament::Renderer* renderer, filament::View* view,
              filament::RenderTarget* render_target);

 private:
  void Prepare(int width, int height);
  void Reset();

  enum Pass {
    kPassFlatten,
    kPassJumpFlood1,
    kPassJumpFlood2,
    kPassJumpFlood3,
    kPassJumpFlood4,
    kPassJumpFlood5,
    kPassDrawOutline,
    kNumPasses,

    kNumJumpFloodPasses = kPassJumpFlood5 - kPassJumpFlood1 + 1,
  };

  ObjectManager* object_mgr_ = nullptr;
  filament::Engine* engine_ = nullptr;
  uint8_t layer_mask_ = 0xff;
  filament::math::float4 color_ = {1.0f, 1.0f, 1.0f, 1.0f};
  float thickness_ = 2.5f;

  int width_ = 0;
  int height_ = 0;

  filament::Camera* camera_ = nullptr;
  std::unique_ptr<RenderTarget> targets_[2];

  filament::View* views_[kNumPasses] = {};
  filament::Scene* scenes_[kNumPasses] = {};
  utils::Entity quads_[kNumPasses] = {};
  filament::MaterialInstance* material_instances_[kNumPasses] = {};
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_CORE_OUTLINER_H_
