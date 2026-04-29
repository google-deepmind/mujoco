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
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

void UpdateMaterialInstance(filament::MaterialInstance* instance,
                            const mjrMaterialParams& params,
                            const mjrMaterialTextures& textures,
                            ObjectManager* object_mgr) {
  if (params.scissor[2] != 0 && params.scissor[3] != 0) {
    instance->setScissor(params.scissor[0], params.scissor[1],
                         params.scissor[2], params.scissor[3]);
  }

  const filament::Material* material = instance->getMaterial();
  if (material->hasParameter("BaseColorFactor")) {
    instance->setParameter("BaseColorFactor", filament::RgbaType::sRGB,
                           ReadFloat4(params.color));
  }
  if (material->hasParameter("SegmentationColor")) {
    instance->setParameter("SegmentationColor", filament::RgbaType::LINEAR,
                           ReadFloat4(params.segmentation_color));
  }
  if (material->hasParameter("EmissiveFactor")) {
    instance->setParameter("EmissiveFactor", params.emissive);
  }
  if (material->hasParameter("SpecularFactor")) {
    instance->setParameter("SpecularFactor", params.specular);
  }
  if (material->hasParameter("GlossinessFactor")) {
    instance->setParameter("GlossinessFactor", params.glossiness);
  }
  if (material->hasParameter("MetallicFactor")) {
    instance->setParameter("MetallicFactor",
                           params.metallic >= 0 ? params.metallic : 1.0f);
  }
  if (material->hasParameter("RoughnessFactor")) {
    instance->setParameter("RoughnessFactor",
                           params.roughness >= 0 ? params.roughness : 1.0f);
  }
  if (material->hasParameter("UvScale")) {
    instance->setParameter("UvScale", ReadFloat3(params.uv_scale));
  }
  if (material->hasParameter("UvOffset")) {
    instance->setParameter("UvOffset", ReadFloat3(params.uv_offset));
  }
  if (material->hasParameter("Reflectance")) {
    instance->setParameter("Reflectance", params.reflectance);
  }

  // All textures use the same default sampler.
  filament::TextureSampler sampler;
  sampler.setWrapModeR(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setWrapModeS(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setWrapModeT(filament::TextureSampler::WrapMode::REPEAT);
  sampler.setMagFilter(filament::TextureSampler::MagFilter::LINEAR);
  sampler.setMinFilter(
      filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR);

  auto TrySetTexture = [&](const char* name, const mjrTexture* texture,
                           mjtTextureRole role) {
    if (material->hasParameter(name)) {
      if (texture != nullptr) {
        instance->setParameter(
            name, Texture::downcast(texture)->GetFilamentTexture(), sampler);
      } else {
        instance->setParameter(name, object_mgr->GetFallbackTexture(role),
                               sampler);
      }
    }
  };

  TrySetTexture("BaseColor", textures.color, mjTEXROLE_RGB);
  TrySetTexture("Normal", textures.normal, mjTEXROLE_NORMAL);
  TrySetTexture("Metallic", textures.metallic, mjTEXROLE_METALLIC);
  TrySetTexture("Roughness", textures.roughness, mjTEXROLE_ROUGHNESS);
  TrySetTexture("Occlusion", textures.occlusion, mjTEXROLE_OCCLUSION);
  TrySetTexture("ORM", textures.orm, mjTEXROLE_ORM);
  TrySetTexture("Emissive", textures.emissive, mjTEXROLE_EMISSIVE);
  TrySetTexture("Reflection", textures.reflection, mjTEXROLE_USER);
}

}  // namespace mujoco
