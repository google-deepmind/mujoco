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

#include <filament/Engine.h>
#include <filament/LightManager.h>
#include <filament/Scene.h>
#include <math/vec3.h>
#include <utils/Entity.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

Light::Light(ObjectManager* object_mgr, const Params& params)
    : engine_(object_mgr->GetEngine()), params_(params) {
  filament::LightManager::Type type;
  switch (params.type) {
    case mjLIGHT_SPOT:
      type = filament::LightManager::Type::FOCUSED_SPOT;
      break;
    case mjLIGHT_DIRECTIONAL:
      // We break with the spec here slightly and use a spot light for the head
      // light instead of a directional params. This is because filament only
      // supports a single directional light, and we'd rather allow a scene
      // light to be that directional params. It's also a bit odd for a
      // directional light to move with the camera.
      type = params.headlight ? filament::LightManager::Type::FOCUSED_SPOT
                             : filament::LightManager::Type::DIRECTIONAL;
      break;
    case mjLIGHT_POINT:
      type = filament::LightManager::Type::POINT;
      break;
    default:
      mju_error("Unsupported light type: %d", params.type);
      return;
  }

  filament::LightManager::Builder builder(type);
  builder.color(params.color);
  builder.intensityCandela(params.intensity);
  builder.castShadows(params.castshadow);
  if (type == filament::LightManager::Type::FOCUSED_SPOT) {
    if (params.headlight) {
      builder.spotLightCone(0, M_PI / 2.0f);
    } else {
      builder.spotLightCone(0, params.spot_cone_angle * M_PI / 180.0f);
    }
  }
  if (type != filament::LightManager::Type::DIRECTIONAL) {
    builder.falloff(params.range);
  }
  filament::LightManager::ShadowOptions opts;
  opts.mapSize = 4096;
  opts.shadowCascades =
      type == filament::LightManager::Type::DIRECTIONAL ? 4 : 1;
  opts.shadowBulbRadius = params.bulbradius;
  builder.shadowOptions(opts);

  entity_ = utils::EntityManager::get().create();
  if (entity_.isNull()) {
    mju_error("Failed to create light entity.");
  }
  builder.build(*engine_, entity_);
}

Light::~Light() noexcept {
  utils::EntityManager& em = utils::EntityManager::get();
  if (!entity_.isNull()) {
    engine_->destroy(entity_);
    em.destroy(entity_);
  }
}

void Light::AddToScene(filament::Scene* scene) { scene->addEntity(entity_); }

void Light::RemoveFromScene(filament::Scene* scene) { scene->remove(entity_); }

void Light::SetTransform(filament::math::float3 position,
                         filament::math::float3 direction) {
  filament::LightManager& lm = engine_->getLightManager();
  const filament::LightManager::Instance li = lm.getInstance(entity_);
  lm.setPosition(li, position);
  lm.setDirection(li, direction);
}

void Light::SetColor(const filament::math::float3& color) {
  filament::LightManager& lm = engine_->getLightManager();
  const filament::LightManager::Instance li = lm.getInstance(entity_);
  lm.setColor(li, color);
}

void Light::SetIntensity(float intensity) {
  filament::LightManager& lm = engine_->getLightManager();
  const filament::LightManager::Instance li = lm.getInstance(entity_);
  lm.setIntensityCandela(li, intensity);
}

void Light::Enable() {
  if (!enabled_) {
    enabled_ = true;
    filament::LightManager& lm = engine_->getLightManager();
    const filament::LightManager::Instance li = lm.getInstance(entity_);
    lm.setLightChannel(li, 0, enabled_);
  }
}

void Light::Disable() {
  if (enabled_) {
    enabled_ = false;
    filament::LightManager& lm = engine_->getLightManager();
    const filament::LightManager::Instance li = lm.getInstance(entity_);
    lm.setLightChannel(li, 0, enabled_);
  }
}

}  // namespace mujoco
