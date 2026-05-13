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
#include "experimental/filament/filament_util.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

void UpdateMaterialInstance(filament::MaterialInstance* instance,
                            const mjrMaterial& material,
                            ObjectManager* object_mgr) {
  if (material.scissor[2] != 0 && material.scissor[3] != 0) {
    instance->setScissor(material.scissor[0], material.scissor[1],
                         material.scissor[2], material.scissor[3]);
  }

  const filament::Material* fmaterial = instance->getMaterial();
  if (fmaterial->hasParameter("BaseColorFactor")) {
    instance->setParameter("BaseColorFactor", filament::RgbaType::sRGB,
                           ReadFloat4(material.color));
  }
  if (fmaterial->hasParameter("SegmentationColor")) {
    instance->setParameter("SegmentationColor", filament::RgbaType::LINEAR,
                           ReadFloat4(material.segmentation_color));
  }
  if (fmaterial->hasParameter("EmissiveFactor")) {
    instance->setParameter("EmissiveFactor", material.emissive);
  }
  if (fmaterial->hasParameter("SpecularFactor")) {
    instance->setParameter("SpecularFactor", material.specular);
  }
  if (fmaterial->hasParameter("GlossinessFactor")) {
    instance->setParameter("GlossinessFactor", material.glossiness);
  }
  if (fmaterial->hasParameter("MetallicFactor")) {
    instance->setParameter("MetallicFactor",
                           material.metallic >= 0 ? material.metallic : 1.0f);
  }
  if (fmaterial->hasParameter("RoughnessFactor")) {
    instance->setParameter("RoughnessFactor",
                           material.roughness >= 0 ? material.roughness : 1.0f);
  }
  if (fmaterial->hasParameter("UvScale")) {
    instance->setParameter("UvScale", ReadFloat3(material.uv_scale));
  }
  if (fmaterial->hasParameter("UvOffset")) {
    instance->setParameter("UvOffset", ReadFloat3(material.uv_offset));
  }
  if (fmaterial->hasParameter("Reflectance")) {
    instance->setParameter("Reflectance", material.reflectance);
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
    if (fmaterial->hasParameter(name)) {
      if (texture != nullptr) {
        instance->setParameter(
            name, Texture::downcast(texture)->GetFilamentTexture(), sampler);
      } else {
        instance->setParameter(name, object_mgr->GetFallbackTexture(role),
                               sampler);
      }
    }
  };

  TrySetTexture("BaseColor", material.color_texture, mjTEXROLE_RGB);
  TrySetTexture("Normal", material.normal_texture, mjTEXROLE_NORMAL);
  TrySetTexture("Metallic", material.metallic_texture, mjTEXROLE_METALLIC);
  TrySetTexture("Roughness", material.roughness_texture, mjTEXROLE_ROUGHNESS);
  TrySetTexture("Occlusion", material.occlusion_texture, mjTEXROLE_OCCLUSION);
  TrySetTexture("ORM", material.orm_texture, mjTEXROLE_ORM);
  TrySetTexture("Emissive", material.emissive_texture, mjTEXROLE_EMISSIVE);
  TrySetTexture("Reflection", material.reflection_texture, mjTEXROLE_USER);
}

}  // namespace mujoco
