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

#include "experimental/filament/filament/material.h"

#include <filament/Color.h>
#include <filament/Material.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/TextureSampler.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

Material::Material(ObjectManager* object_mgr)
    : object_mgr_(object_mgr) {
}

Material::~Material() noexcept {
  for (int i = 0; i < kNumDrawModes; ++i) {
    if (instances_[i]) {
      GetEngine()->destroy(instances_[i]);
    }
  }
}

void Material::SetMaterial(DrawMode mode, filament::Material* material) {
  const int index = static_cast<int>(mode);
  if (instances_[index]) {
    const filament::Material* current_material =
        instances_[index]->getMaterial();
    if (current_material == material) {
      return;
    }

    GetEngine()->destroy(instances_[index]);
    instances_[index] = nullptr;
  }
  if (material) {
    instances_[index] = material->createInstance();
    UpdateMaterialInstances();
  }
}

filament::MaterialInstance* Material::GetMaterialInstance(DrawMode mode) {
  return instances_[static_cast<int>(mode)];
}

void Material::UpdateParams(const Params& params) {
  params_ = params;
  UpdateMaterialInstances();
}

void Material::UpdateTextures(const Textures& textures) {
  textures_ = textures;
  UpdateMaterialInstances();
}

void Material::UpdateMaterialInstances() {
  filament::MaterialInstance* instance =
      instances_[static_cast<int>(DrawMode::Color)];
  if (instance == nullptr) {
    return;
  }

  if (params_.scissor[2] != 0 && params_.scissor[3] != 0) {
    instance->setScissor(params_.scissor[0], params_.scissor[1],
                         params_.scissor[2], params_.scissor[3]);
  }

  const filament::Material* material = instance->getMaterial();
  if (material->hasParameter("BaseColorFactor")) {
    instance->setParameter("BaseColorFactor", filament::RgbaType::sRGB,
                           params_.color);
  }
  if (material->hasParameter("EmissiveFactor")) {
    instance->setParameter("EmissiveFactor", params_.emissive);
  }
  if (material->hasParameter("SpecularFactor")) {
    instance->setParameter("SpecularFactor", params_.specular);
  }
  if (material->hasParameter("GlossinessFactor")) {
    instance->setParameter("GlossinessFactor", params_.glossiness);
  }
  if (material->hasParameter("MetallicFactor")) {
    instance->setParameter("MetallicFactor",
                           params_.metallic >= 0 ? params_.metallic : 1.0f);
  }
  if (material->hasParameter("RoughnessFactor")) {
    instance->setParameter("RoughnessFactor",
                           params_.roughness >= 0 ? params_.roughness : 1.0f);
  }
  if (material->hasParameter("UvScale")) {
    instance->setParameter("UvScale", params_.uv_scale);
  }
  if (material->hasParameter("UvOffset")) {
    instance->setParameter("UvOffset", params_.uv_offset);
  }
  if (material->hasParameter("Reflectance")) {
    instance->setParameter("Reflectance", params_.reflectance);
  }

  const int segmentation_index = static_cast<int>(DrawMode::Segmentation);
  if (instances_[segmentation_index]) {
    instances_[segmentation_index]->setParameter("BaseColorFactor",
                                                 params_.segmentation_color);
  }

  // All textures use the same default sampler.
  filament::TextureSampler sampler;
  sampler.setWrapModeR(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setWrapModeS(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setWrapModeT(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setMagFilter(filament::TextureSampler::MagFilter::LINEAR);
  sampler.setMinFilter(
      filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR);

  auto TrySetTexture = [&](const char* name, const Texture* texture,
                           mjtTextureRole role) {
    if (material->hasParameter(name)) {
      if (texture != nullptr) {
        instance->setParameter(name, texture->GetFilamentTexture(), sampler);
      } else {
        instance->setParameter(name, object_mgr_->GetFallbackTexture(role), sampler);
      }
    }
  };

  TrySetTexture("BaseColor", textures_.color, mjTEXROLE_RGB);
  TrySetTexture("Normal", textures_.normal, mjTEXROLE_NORMAL);
  TrySetTexture("Metallic", textures_.metallic, mjTEXROLE_METALLIC);
  TrySetTexture("Roughness", textures_.roughness, mjTEXROLE_ROUGHNESS);
  TrySetTexture("Occlusion", textures_.occlusion, mjTEXROLE_OCCLUSION);
  TrySetTexture("ORM", textures_.orm, mjTEXROLE_ORM);
  TrySetTexture("Emissive", textures_.emissive, mjTEXROLE_EMISSIVE);
  TrySetTexture("Reflection", textures_.reflection, mjTEXROLE_USER);
}

}  // namespace mujoco
