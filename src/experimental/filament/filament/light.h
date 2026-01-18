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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_LIGHT_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_LIGHT_H_

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <math/vec3.h>
#include <utils/Entity.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

// Manages the filament Entities for a single mjvLight.
class Light {
 public:
  // Configuration parameters for a light.
  struct Params {
    // The type of light (e.g. spot, point, directional, etc.)
    mjtLightType type;
    // The color of the light.
    filament::math::float3 color = {0, 0, 0};
    // The intensity of the light, in candela.
    float intensity = 0.0f;
    // Whether or not the light casts shadows.
    bool castshadow = true;
    // The range/distance in which the light is effective, in meters.
    float range = 10.0f;
    // The angle of the spot light cone, in degrees.
    float spot_cone_angle = 180.f;
    // The radius of the bulb used for soft shadows.
    float bulbradius = 0.0f;
    // Whether or not the light is a headlight.
    bool headlight = false;
  };

  Light(ObjectManager* object_mgr, const Params& params);
  ~Light() noexcept;

  Light(const Light&) = delete;
  Light& operator=(const Light&) = delete;

  // Adds the filament light Entities to the given filament Scene.
  void AddToScene(filament::Scene* scene);

  // Removes the filament light Entities from the given filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Updates the light's position/rotation.
  void SetTransform(filament::math::float3 position,
                    filament::math::float3 direction);

  // Sets the color of the light.
  void SetColor(const filament::math::float3& color);

  // Sets the intensity of the light in candela.
  void SetIntensity(float intensity);

  // Enables/disables the light in the scene.
  void Enable();
  void Disable();

  // Returns true if the light is a headlight.
  bool IsHeadlight() const { return params_.headlight; }

 private:
  filament::Engine* engine_ = nullptr;
  utils::Entity entity_;
  bool enabled_ = true;
  Params params_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_LIGHT_H_
