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
#include "experimental/filament/filament/texture.h"

namespace mujoco {

Material::Material(filament::Engine* engine)
    : engine_(engine) {
}

Material::~Material() noexcept {
  for (int i = 0; i < kNumDrawModes; ++i) {
    if (instances_[i]) {
      engine_->destroy(instances_[i]);
    }
  }
}

void Material::SetMaterial(DrawMode mode, filament::Material* material) {
  if (instances_[mode]) {
    const filament::Material* current_material =
        instances_[mode]->getMaterial();
    if (current_material == material) {
      return;
    }

    engine_->destroy(instances_[mode]);
    instances_[mode] = nullptr;
  }
  if (material) {
    instances_[mode] = material->createInstance();
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

void Material::SetFallbackTextures(const Textures* fallback_textures) {
  fallback_textures_ = fallback_textures;
}

void Material::UpdateMaterialInstances() {
  filament::MaterialInstance* instance = instances_[DrawMode::kNormal];
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
                           const Texture* fallback) {
    if (material->hasParameter(name)) {
      if (texture) {
        instance->setParameter(name, texture->GetFilamentTexture(), sampler);
      } else if (fallback) {
        instance->setParameter(name, fallback->GetFilamentTexture(), sampler);
      }
    }
  };

  TrySetTexture("BaseColor", textures_.color,
                fallback_textures_ ? fallback_textures_->color : nullptr);
  TrySetTexture("Normal", textures_.normal,
                fallback_textures_ ? fallback_textures_->normal : nullptr);
  TrySetTexture("Metallic", textures_.metallic,
                fallback_textures_ ? fallback_textures_->metallic : nullptr);
  TrySetTexture("Roughness", textures_.roughness,
                fallback_textures_ ? fallback_textures_->roughness : nullptr);
  TrySetTexture("Occlusion", textures_.occlusion,
                fallback_textures_ ? fallback_textures_->occlusion : nullptr);
  TrySetTexture("ORM", textures_.orm,
                fallback_textures_ ? fallback_textures_->orm : nullptr);
  TrySetTexture("Emissive", textures_.emissive,
                fallback_textures_ ? fallback_textures_->emissive : nullptr);
  TrySetTexture("Reflection", textures_.reflection,
                fallback_textures_ ? fallback_textures_->reflection : nullptr);
}

}  // namespace mujoco
