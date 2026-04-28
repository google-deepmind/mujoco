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

#include "experimental/filament/filament/light.h"

#include <numbers>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Scene.h>
#include <math/mat3.h>
#include <math/vec3.h>
#include <utils/Entity.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/texture.h"

namespace mujoco {

using filament::math::float3;
using filament::math::mat3f;

void mjr_defaultLightParams(mjrLightParams* params) {
  params->type = mjLIGHT_POINT;
  params->texture = nullptr;
  params->color[0] = 0;
  params->color[1] = 0;
  params->color[2] = 0;
  params->intensity = 0.0f;
  params->cast_shadows = true;
  params->range = 10.0f;
  params->spot_cone_angle = 180.f;
  params->bulb_radius = 0.0f;
  params->shadow_map_size = 2048;
  params->vsm_blur_width = 0.0f;
}

Light::Light(filament::Engine* engine, const mjrLightParams& params)
    : engine_(engine), params_(params) {
  // Filament treats image-based lights (IBLs) as separate objects (i.e.
  // filament::IndirectLight) and so we need to handle IBLs specially.
  if (params.type == mjLIGHT_IMAGE) {
    filament::IndirectLight::Builder builder;
    if (params.texture) {
      // Allow null textures for fallback lights.
      const Texture* texture = Texture::downcast(params.texture);
      builder.reflections(texture->GetFilamentTexture());
      const Texture::SphericalHarmonics* spherical_harmonics =
          texture->GetSphericalHarmonics();
      if (spherical_harmonics != nullptr) {
        builder.irradiance(3, *spherical_harmonics);
      }
    }
    builder.intensity(params.intensity);
    // Rotate the light to match mujoco's Z-up convention.
    builder.rotation(mat3f::rotation(std::numbers::pi / 2, float3{1, 0, 0}));
    ibl_ = builder.build(*engine_);
    return;
  }

  filament::LightManager::Type type;
  switch (params.type) {
    case mjLIGHT_SPOT:
      type = filament::LightManager::Type::FOCUSED_SPOT;
      break;
    case mjLIGHT_DIRECTIONAL:
      type = filament::LightManager::Type::DIRECTIONAL;
      break;
    case mjLIGHT_POINT:
      type = filament::LightManager::Type::POINT;
      break;
    default:
      mju_error("Unsupported light type: %d", params.type);
      return;
  }

  filament::LightManager::Builder builder(type);
  builder.color(ReadFloat3(params.color));
  builder.intensityCandela(params.intensity);
  builder.castShadows(params.cast_shadows);
  if (type == filament::LightManager::Type::FOCUSED_SPOT) {
    builder.spotLightCone(0,
                          params.spot_cone_angle * std::numbers::pi / 180.0f);
  }
  if (type != filament::LightManager::Type::DIRECTIONAL) {
    builder.falloff(params.range);
  }
  filament::LightManager::ShadowOptions opts;
  opts.mapSize = 4096;
  opts.shadowCascades =
      type == filament::LightManager::Type::DIRECTIONAL ? 4 : 1;
  opts.shadowBulbRadius = params.bulb_radius;
  opts.mapSize = params.shadow_map_size;
  if (params.vsm_blur_width > 0.0f) {
    opts.vsm.elvsm = true;
    opts.vsm.blurWidth = params.vsm_blur_width;
  }

  builder.shadowOptions(opts);

  entity_ = utils::EntityManager::get().create();
  if (entity_.isNull()) {
    mju_error("Failed to create light entity.");
  }
  builder.build(*engine_, entity_);
}

Light::~Light() noexcept {
  if (ibl_) {
    engine_->destroy(ibl_);
  } else {
    utils::EntityManager& em = utils::EntityManager::get();
    if (!entity_.isNull()) {
      engine_->destroy(entity_);
      em.destroy(entity_);
    }
  }
}

void Light::AddToScene(filament::Scene* scene) {
  if (ibl_) {
    scene->setIndirectLight(ibl_);
  } else {
    scene->addEntity(entity_);
  }
}

void Light::RemoveFromScene(filament::Scene* scene) {
  if (ibl_) {
    scene->setIndirectLight(nullptr);
  } else {
    scene->remove(entity_);
  }
}

void Light::SetTransform(filament::math::float3 position,
                         filament::math::float3 direction) {
  if (!ibl_) {
    filament::LightManager& lm = engine_->getLightManager();
    const filament::LightManager::Instance li = lm.getInstance(entity_);
    lm.setPosition(li, position);
    lm.setDirection(li, direction);
  }
}

void Light::SetColor(const filament::math::float3& color) {
  if (!ibl_) {
    params_.color[0] = color.r;
    params_.color[1] = color.g;
    params_.color[2] = color.b;

    filament::LightManager& lm = engine_->getLightManager();
    const filament::LightManager::Instance li = lm.getInstance(entity_);
    lm.setColor(li, color);
  }
}

void Light::SetIntensity(float intensity) {
  params_.intensity = intensity;
  if (ibl_) {
    ibl_->setIntensity(intensity);
  } else {
    filament::LightManager& lm = engine_->getLightManager();
    const filament::LightManager::Instance li = lm.getInstance(entity_);
    lm.setIntensityCandela(li, intensity);
  }
}

void Light::Enable() {
  if (!enabled_) {
    enabled_ = true;
    if (ibl_) {
      ibl_->setIntensity(params_.intensity);
    } else {
      filament::LightManager& lm = engine_->getLightManager();
      const filament::LightManager::Instance li = lm.getInstance(entity_);
      lm.setLightChannel(li, 0, enabled_);
    }
  }
}

void Light::Disable() {
  if (enabled_) {
    enabled_ = false;
    if (ibl_) {
      ibl_->setIntensity(0.f);
    } else {
      filament::LightManager& lm = engine_->getLightManager();
      const filament::LightManager::Instance li = lm.getInstance(entity_);
      lm.setLightChannel(li, 0, enabled_);
    }
  }
}

}  // namespace mujoco
