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
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

// The textures that can be assigned to the drawable's material.
struct mjrMaterialTextures {
  const Texture* color;
  const Texture* normal;
  const Texture* metallic;
  const Texture* roughness;
  const Texture* occlusion;
  const Texture* orm;
  const Texture* emissive;
  const Texture* reflection;
};

void mjr_defaultMaterialTextures(mjrMaterialTextures* textures);

// The parameters that can be applied to the drawable's material.
struct mjrMaterialParams {
  float color[4];
  float segmentation_color[4];
  float tex_repeat[2];
  float uv_scale[3];
  float uv_offset[3];
  float scissor[4];
  float specular;
  float glossiness;
  float metallic;
  float roughness;
  float emissive;
  float reflectance;
  mjtByte tex_uniform;
  mjtByte reflective;
};

void mjr_defaultMaterialParams(mjrMaterialParams* params);

// Updates the material instances based on the currently set parameters and
// textures.
void UpdateMaterialInstance(filament::MaterialInstance* instance,
                            const mjrMaterialParams& params,
                            const mjrMaterialTextures& textures,
                            ObjectManager* object_mgr);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_
