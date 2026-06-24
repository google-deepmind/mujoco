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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_LIGHT_MANAGER_H_
#define MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_LIGHT_MANAGER_H_

#include <vector>

#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Manages Light entities for an mjrfScene.
class LightManager {
 public:
  LightManager(mjrfContext* ctx, mjrfScene* scene, ModelObjects* model_objects);
  ~LightManager();

  // Updates the state of the lights in the scene.
  void Update(const mjData* data);

  // Returns the light with the given index in the mjModel. Note that an extra
  // headlight is assigned of the index `nlight`.
  mjrfLight* GetLight(int index);

  LightManager(const LightManager&) = delete;
  LightManager& operator=(const LightManager&) = delete;

 private:
  void Prepare();

  mjrfContext* ctx_ = nullptr;
  mjrfScene* scene_ = nullptr;
  ModelObjects* model_objects_ = nullptr;

  UniquePtr<mjrfLight> fallback_ibl_{nullptr, nullptr};
  UniquePtr<mjrfTexture> fallback_ibl_texture_{nullptr, nullptr};
  std::vector<UniquePtr<mjrfLight>> lights_;
  int default_shadow_map_size_ = 2048;
  float default_vsm_blur_width_ = 0.0f;
  float fallback_head_light_intensity_ = 0.f;
  float fallback_scene_light_intensity_ = 80'000.f;
  float fallback_environment_light_intensity_ = 5'000.f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_LIGHT_MANAGER_H_
