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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_

#include <filament/Engine.h>
#include <filament/MaterialInstance.h>
#include <filament/Texture.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

class Material {
 public:
  // The different methods for rendering objects. Each mode uses a different
  // material, but all materials "share" the same textures and parameters
  // (unless specifically noted otherwise).
  enum DrawMode {
    kNormal,
    kDepth,
    kSegmentation,
    kNumDrawModes,
  };

  // The textures that can be assigned to the drawable's material.
  struct Textures {
    const filament::Texture* color = nullptr;
    const filament::Texture* normal = nullptr;
    const filament::Texture* metallic = nullptr;
    const filament::Texture* roughness = nullptr;
    const filament::Texture* occlusion = nullptr;
    const filament::Texture* orm = nullptr;
    const filament::Texture* emissive = nullptr;
  };

  // The parameters that can be applied to the drawable's material.
  struct Params {
    filament::math::float4 color = {1, 1, 1, 1};
    filament::math::float4 segmentation_color = {1, 1, 1, 1};
    filament::math::float2 tex_repeat = {1, 1};
    filament::math::float3 uv_scale = {1, 1, 1};
    float specular = -1.0f;
    float glossiness = -1.0f;
    float metallic = -1.0f;
    float roughness = -1.0f;
    float emissive = -1.0f;
    bool tex_uniform = false;
  };

  Material(ObjectManager* object_mgr);
  ~Material() noexcept;

  Material(const Material&) = delete;
  Material& operator=(const Material&) = delete;

  // Assigns a material to the draw mode.
  void SetNormalMaterialType(ObjectManager::MaterialType material_type);

  // Updates the material parameters of the drawable for rendering.
  void UpdateParams(const Params& params);

  // Updates the material textures of the drawable for rendering.
  void UpdateTextures(const Textures& textures);

  // Returns the material instance assigned to the draw mode.
  filament::MaterialInstance* GetMaterialInstance(DrawMode mode) {
    return instances_[mode];
  }

  // Returns the ObjectManager owning the Materials which are used to create
  // the MaterialInstances.
  ObjectManager* GetObjectManager() { return object_mgr_; }

 private:
  // Updates the material instances based on the currently set parameters and
  // textures.
  void UpdateMaterialInstances();

  ObjectManager* object_mgr_ = nullptr;
  filament::MaterialInstance* instances_[kNumDrawModes] = {nullptr};
  Params params_;
  Textures textures_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_
