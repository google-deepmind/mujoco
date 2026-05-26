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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_LIGHT_MANAGER_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_LIGHT_MANAGER_H_

#include <vector>

#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

namespace mujoco {

// Manages Light entities for an mjrScene using data from an mjvScene.
class LightManager {
 public:
  LightManager(mjrfContext* ctx, mjrScene* scene, ModelObjects* model_objects);
  ~LightManager();

  // Returns the light with the given index in the mjModel. Note that an extra
  // headlight is assigned of the index `nlight`.
  mjrLight* GetLight(int index);

  LightManager(const LightManager&) = delete;
  LightManager& operator=(const LightManager&) = delete;

 private:
  void Prepare(ModelObjects* model_objects);

  mjrfContext* ctx_ = nullptr;
  mjrScene* scene_ = nullptr;
  UniquePtr<mjrLight> fallback_ibl_{nullptr, nullptr};
  UniquePtr<mjrTexture> fallback_ibl_texture_{nullptr, nullptr};
  std::vector<UniquePtr<mjrLight>> lights_;
  int default_shadow_map_size_ = 2048;
  float default_vsm_blur_width_ = 0.0f;
  float fallback_head_light_intensity_ = 0.f;
  float fallback_scene_light_intensity_ = 80'000.f;
  float fallback_environment_light_intensity_ = 5'000.f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_LIGHT_MANAGER_H_
