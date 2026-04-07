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
#include <mujoco/mjmodel.h>
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/texture.h"

namespace mujoco {

Material::Material(ObjectManager* object_mgr) : object_mgr_(object_mgr) {
  instances_[kDepth] =
      object_mgr_->GetMaterial(ObjectManager::kUnlitDepth)->createInstance();
  instances_[kSegmentation] =
      object_mgr_->GetMaterial(ObjectManager::kUnlitSegmentation)
          ->createInstance();
}

Material::~Material() noexcept {
  filament::Engine* engine = object_mgr_->GetEngine();
  for (int i = 0; i < kNumDrawModes; ++i) {
    if (instances_[i]) {
      engine->destroy(instances_[i]);
    }
  }
}

void Material::SetNormalMaterialType(
    ObjectManager::MaterialType material_type) {
  filament::Material* material = object_mgr_->GetMaterial(material_type);

  if (instances_[kNormal]) {
    const filament::Material* current_material =
        instances_[kNormal]->getMaterial();
    if (current_material == material) {
      return;
    }

    object_mgr_->GetEngine()->destroy(instances_[kNormal]);
    instances_[kNormal] = nullptr;
  }
  if (material) {
    instances_[kNormal] = material->createInstance();
    UpdateMaterialInstances();
  }
}

void Material::UpdateParams(const Params& params) {
  params_ = params;
  UpdateMaterialInstances();
}

void Material::UpdateTextures(const Textures& textures) {
  textures_ = textures;
  UpdateMaterialInstances();
}

void Material::UpdateReflectionTexture(const Texture* tex) {
  textures_.reflection = tex;
  UpdateMaterialInstances();
}

void Material::UpdateMaterialInstances() {
  filament::MaterialInstance* instance = instances_[DrawMode::kNormal];
  if (instance == nullptr) {
    return;
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

  if (instances_[DrawMode::kSegmentation]) {
    instances_[DrawMode::kSegmentation]->setParameter(
        "BaseColorFactor", params_.segmentation_color);
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
      if (texture) {
        instance->setParameter(name, texture->GetFilamentTexture(), sampler);
      } else {
        auto* fallback = object_mgr_->GetFallbackTexture(role);
        instance->setParameter(name, fallback->GetFilamentTexture(), sampler);
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
