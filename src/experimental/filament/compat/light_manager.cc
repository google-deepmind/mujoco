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

#include "experimental/filament/compat/light_manager.h"

#include <memory>
#include <string>
#include <utility>

#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/filament_util.h"
#include "experimental/filament/render_context_filament_cpp.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3;
using filament::math::mat4;

static UniquePtr<mjrTexture> CreateFallbackIndirectLightTexture(
    mjrfContext* ctx) {
  const std::string filename = ResolveFilamentAssetPath("ibl.ktx");
  mjResource* resource =
      mju_openResource("", filename.c_str(), nullptr, nullptr, 0);
  if (!resource) {
    mju_error("Failed to open resource: %s", filename.c_str());
  }
  const void* bytes = nullptr;
  const int nbytes = mju_readResource(resource, &bytes);
  if (bytes == nullptr || nbytes <= 0) {
    mju_error("Failed to read resource: %s", filename.c_str());
  }

  mjrTextureConfig config;
  mjr_defaultTextureConfig(&config);
  config.width = 1;
  config.height = 1;
  config.sampler_type = mjTEXTURE_CUBE;
  config.format = mjPIXEL_FORMAT_KTX;
  config.color_space = mjCOLORSPACE_AUTO;

  auto texture = CreateTexture(ctx, config);

  mjrTextureData payload;
  mjr_defaultTextureData(&payload);
  payload.bytes = bytes;
  payload.nbytes = nbytes;
  payload.release_callback = +[](void* user_data) {
    mju_closeResource((mjResource*)user_data);
  };
  payload.user_data = resource;

  mjrf_setTextureData(texture.get(), &payload);
  return texture;
}

LightManager::LightManager(mjrfContext* ctx, mjrScene* scene,
                           ModelObjects* model_objects)
    : ctx_(ctx), scene_(scene) {
  const mjModel* model = model_objects->GetModel();
  default_shadow_map_size_ = ReadElement(
      model, "filament.shadows.map_size", default_shadow_map_size_);
  default_vsm_blur_width_ = ReadElement(
      model, "filament.shadows.vsm_blur_width", default_vsm_blur_width_);
  fallback_head_light_intensity_ =
      ReadElement(model, "filament.fallback.head_light_intensity",
                  fallback_head_light_intensity_);
  fallback_scene_light_intensity_ =
      ReadElement(model, "filament.fallback.scene_light_intensity",
                  fallback_scene_light_intensity_);
  fallback_environment_light_intensity_ =
      ReadElement(model, "filament.fallback.environment_light_intensity",
                  fallback_environment_light_intensity_);
  Prepare(model_objects);
}

LightManager::~LightManager() {
  for (auto& iter : lights_) {
    mjrf_removeLightFromScene(scene_, iter.get());
  }
  lights_.clear();
  if (fallback_ibl_) {
    mjrf_removeLightFromScene(scene_, fallback_ibl_.get());
  }
  fallback_ibl_.reset();
}

void LightManager::Prepare(ModelObjects* model_objects) {
  const mjModel* model = model_objects->GetModel();

  bool has_image_based_light = false;
  float total_light_intensity = 0.0f;
  for (int i = 0; i < model->nlight; ++i) {
    total_light_intensity += model->light_intensity[i];

    if (model->light_type[i] == mjLIGHT_IMAGE) {
      mjrLightParams params;
      mjr_defaultLightParams(&params);
      params.type = mjLIGHT_IMAGE;
      params.texture = model_objects->GetTexture(model->light_texid[i]);
      params.intensity = model->light_intensity[i];
      auto light_obj = CreateLight(ctx_, params);
      mjrf_addLightToScene(scene_, light_obj.get());
      lights_.emplace_back(std::move(light_obj));
      has_image_based_light = true;
    } else {
      mjrLightParams params;
      mjr_defaultLightParams(&params);
      params.color[0] = model->light_diffuse[0];
      params.color[1] = model->light_diffuse[1];
      params.color[2] = model->light_diffuse[2];
      params.type = (mjtLightType)model->light_type[i];
      params.cast_shadows = model->light_castshadow[i];
      params.bulb_radius = model->light_bulbradius[i];
      params.range = model->light_range[i];
      params.intensity = model->light_intensity[i];
      params.shadow_map_size = default_shadow_map_size_;
      params.vsm_blur_width = default_vsm_blur_width_;
      if (params.type == mjLIGHT_SPOT) {
        params.spot_cone_angle = model->light_cutoff[i];
      }

      auto light_obj = CreateLight(ctx_, params);
      mjrf_addLightToScene(scene_, light_obj.get());
      lights_.emplace_back(std::move(light_obj));
    }
  }

  // Add a placeholder (black) headlight as our last light. Going forward, we'll
  // assume lights_.back() is always the headlight.
  {
    mjrLightParams params;
    mjr_defaultLightParams(&params);
    // We break with the spec here slightly and use a spot light for the head
    // light instead of a directional params. This is because filament only
    // supports a single directional light, and we'd rather allow a scene
    // light to be that directional params. It's also a bit odd for a
    // directional light to move with the camera.
    params.type = mjLIGHT_SPOT;
    params.cast_shadows = 0;
    params.intensity = 0.0f;
    params.spot_cone_angle = 90.0f;
    auto light_obj = CreateLight(ctx_, params);
    mjrf_addLightToScene(scene_, light_obj.get());
    lights_.emplace_back(std::move(light_obj));
  }

  if (!has_image_based_light && total_light_intensity > 0.0f) {
    // Create a black indirect light to ensure that the skybox is
    // oriented to respect mujoco's Z-up convention.
    mjrLightParams params;
    mjr_defaultLightParams(&params);
    params.type = mjLIGHT_IMAGE;
    params.intensity = 10.0f;
    fallback_ibl_ = CreateLight(ctx_, params);
    mjrf_addLightToScene(scene_, fallback_ibl_.get());
  }

  // There are no "physical" lights in the scene which means we're likely
  // dealing with a "classic renderer" scene. In this case, let's add a
  // default environment light and set the light intensity ourselves.
  if (total_light_intensity == 0.0f) {
    // Create a fallback environment light.
    fallback_ibl_texture_ = CreateFallbackIndirectLightTexture(ctx_);

    mjrLightParams params;
    mjr_defaultLightParams(&params);
    params.type = mjLIGHT_IMAGE;
    params.texture = fallback_ibl_texture_.get();
    params.intensity = fallback_environment_light_intensity_;
    fallback_ibl_ = CreateLight(ctx_, params);
    mjrf_addLightToScene(scene_, fallback_ibl_.get());

    // Distribute the fallback scene light intensity among the lights.
    const float intensity = fallback_scene_light_intensity_ / lights_.size();
    for (auto& light : lights_) {
      if (light) {
        const bool is_headlight = (light == lights_.back());
        mjrf_setLightIntensity(light.get(),
                               is_headlight ? fallback_head_light_intensity_
                                            : intensity);
      }
    }
  }

  mjrf_setSceneSkybox(scene_, model_objects->GetSkyboxTexture());
}

mjrLight* LightManager::GetLight(int index) {
  if (index < 0 || index >= lights_.size()) {
    return nullptr;
  }
  return lights_[index].get();
}
}  // namespace mujoco
