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

#include "experimental/filament/compat/scene_bridge.h"

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <math/TMatHelpers.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/TVecHelpers.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/compat/scene_geom_util.h"
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

SceneBridge::SceneBridge(mjrfContext* ctx, const mjModel* model)
    : ctx_(ctx) {
  mjrSceneParams params;
  mjr_defaultSceneParams(&params);
  params.layer_mask = mjCAT_ALL;
  params.reflection_layer_mask = mjCAT_DYNAMIC | mjCAT_STATIC;
  scene_ = CreateScene(ctx_, params);
  model_objects_ = std::make_unique<ModelObjects>(model, ctx_);

  mjrf_configureSceneFromModel(scene_.get(), model);

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

  PrepareLights();
}

SceneBridge::~SceneBridge() {
  for (auto& iter : lights_) {
    mjrf_removeLightFromScene(scene_.get(), iter.get());
  }
  lights_.clear();
  if (fallback_ibl_) {
    mjrf_removeLightFromScene(scene_.get(), fallback_ibl_.get());
  }
  fallback_ibl_.reset();
  for (auto& iter : renderables_) {
    mjrf_removeRenderableFromScene(scene_.get(), iter.get());
  }
  renderables_.clear();
}

std::optional<float3> SceneBridge::ClipFromWorld(const float3& pos) const{
  const float4 clip_pos = clip_from_world_ * float4(pos, 1.0f);
  if (clip_pos.w == 0.0f) {
    return std::nullopt;
  }
  return clip_pos.xyz / clip_pos.w;
}

void SceneBridge::PrepareLights() {
  const mjModel* model = model_objects_->GetModel();

  bool has_image_based_light = false;
  float total_light_intensity = 0.0f;
  for (int i = 0; i < model->nlight; ++i) {
    total_light_intensity += model->light_intensity[i];

    if (model->light_type[i] == mjLIGHT_IMAGE) {
      mjrLightParams params;
      mjr_defaultLightParams(&params);
      params.type = mjLIGHT_IMAGE;
      params.texture = model_objects_->GetTexture(model->light_texid[i]);
      params.intensity = model->light_intensity[i];
      auto light_obj = CreateLight(ctx_, params);
      mjrf_addLightToScene(scene_.get(), light_obj.get());
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
      mjrf_addLightToScene(scene_.get(), light_obj.get());
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
    mjrf_addLightToScene(scene_.get(), light_obj.get());
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
    mjrf_addLightToScene(scene_.get(), fallback_ibl_.get());
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
    mjrf_addLightToScene(scene_.get(), fallback_ibl_.get());

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

  mjrf_setSceneSkybox(scene_.get(), model_objects_->GetSkyboxTexture());
}

mat4 CalculateClipFromWorld(const mjrRect& viewport, const mjrCamera& cam) {
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio = (float)viewport.width / (float)viewport.height;
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (cam.frustum_top - cam.frustum_bottom);
  const float left = cam.frustum_center - halfwidth;
  const float right = cam.frustum_center + halfwidth;

  mat4 projection;
  if (cam.orthographic) {
    projection = mat4::ortho(left, right, cam.frustum_bottom, cam.frustum_top,
                             cam.frustum_near, cam.frustum_far);
  } else {
    projection = mat4::frustum(left, right, cam.frustum_bottom, cam.frustum_top,
                               cam.frustum_near, cam.frustum_far);
    projection[2][2] = -1.0f;
    projection[3][2] = -2.0f * cam.frustum_near;
  }
  mat4 look_at = mat4::lookAt(cam_pos, cam_at, cam_up);
  return projection * inverse(look_at);
}

void SceneBridge::Update(const mjrRect& viewport, const mjvScene* scene) {
  mjrf_setSceneShadowsEnabled(scene_.get(), scene->flags[mjRND_SHADOW]);
  mjrf_setSceneReflectionsEnabled(scene_.get(), scene->flags[mjRND_REFLECTION]);
  if (scene->flags[mjRND_SEGMENT]) {
    draw_mode_ = mjDRAW_MODE_SEGMENTATION;
  } else if (scene->flags[mjRND_DEPTH]) {
    draw_mode_ = mjDRAW_MODE_DEPTH;
  } else {
    draw_mode_ = mjDRAW_MODE_COLOR;
  }

  mjtNum hpos[3], hfwd[3];
  float headpos[3], gazedir[3];
  mjv_cameraInModel(hpos, hfwd, nullptr, scene);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(gazedir, hfwd, 3);

  camera_ = mjv_averageCamera(scene->camera, scene->camera + 1);
  clip_from_world_ = CalculateClipFromWorld(viewport, camera_);

  // Remove all drawables from previous render and prepare new ones.
  for (auto& iter : renderables_) {
    mjrf_removeRenderableFromScene(scene_.get(), iter.get());
  }
  renderables_.clear();
  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (draw_text_callback_ && geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        draw_text_callback_(geom->label, pos->x, pos->y, pos->z);
      }
    }

    if (geom->type == mjGEOM_FLEX || geom->type == mjGEOM_SKIN) {
      model_objects_->CreateSkinFlexMesh(scene, *geom);
    }

    UniquePtr<mjrRenderable> renderable = CreateGeomRenderable(
        *geom, scene, ctx_, model_objects_.get(), headpos);

    mjrf_addRenderableToScene(scene_.get(), renderable.get());
    renderables_.push_back(std::move(renderable));
  }

  bool headlight_enabled = false;
  for (int i = 0; i < scene->nlight; ++i) {
    const mjvLight& scene_light = scene->lights[i];
    if (scene_light.id < 0 && scene_light.headlight) {
      // We position the headlight slightly behind the camera to avoid some
      // odd clipping issues.
      headlight_enabled = true;
      headpos[0] -= gazedir[0] * 0.05f;
      headpos[1] -= gazedir[1] * 0.05f;
      headpos[2] -= gazedir[2] * 0.05f;

      // The headlight is always the "back" light.
      UniquePtr<mjrLight>& light = lights_.back();
      mjrf_setLightColor(light.get(), scene_light.diffuse);
      mjrf_setLightTransform(light.get(), headpos, gazedir);
      continue;
    } else if (scene_light.id < lights_.size() - 1) {
      UniquePtr<mjrLight>& light = lights_[scene_light.id];
      if (light) {
        mjrf_setLightColor(light.get(), scene_light.diffuse);
        mjrf_setLightTransform(light.get(), scene_light.pos, scene_light.dir);
      }
    } else {
      mju_error("Unexpected light id: %d", scene_light.id);
    }
  }

  // Enable/disable the headlight based on whether or not it's in the scene.
  mjrf_setLightEnabled(lights_.back().get(), headlight_enabled);
}

void SceneBridge::UploadMesh(const mjModel* model, int id) {
  model_objects_->UploadMesh(model, id);
}

void SceneBridge::UploadTexture(const mjModel* model, int id) {
  model_objects_->UploadTexture(model, id);
}

void SceneBridge::UploadHeightField(const mjModel* model, int id) {
  model_objects_->UploadHeightField(model, id);
}

void SceneBridge::SetDrawTextFunction(DrawTextAtFn fn) {
  draw_text_callback_ = std::move(fn);
}

mjrScene* SceneBridge::GetScene() const { return scene_.get(); }

mjrCamera SceneBridge::GetCamera() const { return camera_; }

mjrDrawMode SceneBridge::GetDrawMode() const { return draw_mode_; }

}  // namespace mujoco
