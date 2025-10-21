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

namespace mujoco {

// Various tweakable parameters for mapping mujoco values onto filament.
static constexpr float kSpecularMultiplier = 0.6f;
static constexpr float kShininessMultiplier = 0.1f;
static constexpr float kEmissiveMultiplier = 0.3f;

Material::Material(ObjectManager* object_mgr) : object_mgr_(object_mgr) {
  instances_[kDepth] =
      object_mgr->GetMaterial(ObjectManager::kUnlitDepth)->createInstance();
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
    instance->setParameter("EmissiveFactor",
                           params_.emissive * kEmissiveMultiplier);
  }
  if (material->hasParameter("SpecularFactor")) {
    instance->setParameter("SpecularFactor",
                           params_.specular * kSpecularMultiplier);
  }
  if (material->hasParameter("GlossinessFactor")) {
    instance->setParameter("GlossinessFactor",
                           params_.glossiness * kShininessMultiplier);
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

  if (material->hasParameter("BaseColor")) {
    if (textures_.color) {
      instance->setParameter("BaseColor", textures_.color, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_RGB);
      instance->setParameter("BaseColor", fallback, sampler);
    }
  }
  if (material->hasParameter("Normal")) {
    if (textures_.normal) {
      instance->setParameter("Normal", textures_.normal, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_NORMAL);
      instance->setParameter("Normal", fallback, sampler);
    }
  }
  if (material->hasParameter("Metallic")) {
    if (textures_.metallic) {
      instance->setParameter("Metallic", textures_.metallic, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_METALLIC);
      instance->setParameter("Metallic", fallback, sampler);
    }
  }
  if (material->hasParameter("Roughness")) {
    if (textures_.roughness) {
      instance->setParameter("Roughness", textures_.roughness, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_ROUGHNESS);
      instance->setParameter("Roughness", fallback, sampler);
    }
  }
  if (material->hasParameter("Occlusion")) {
    if (textures_.occlusion) {
      instance->setParameter("Occlusion", textures_.occlusion, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_OCCLUSION);
      instance->setParameter("Occlusion", fallback, sampler);
    }
  }
  if (material->hasParameter("ORM")) {
    if (textures_.orm) {
      instance->setParameter("ORM", textures_.orm, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_ORM);
      instance->setParameter("ORM", fallback, sampler);
    }
  }
  if (material->hasParameter("Emissive")) {
    if (textures_.emissive) {
      instance->setParameter("Emissive", textures_.emissive, sampler);
    } else {
      auto* fallback = object_mgr_->GetFallbackTexture(mjTEXROLE_EMISSIVE);
      instance->setParameter("Emissive", fallback, sampler);
    }
  }
}

}  // namespace mujoco
