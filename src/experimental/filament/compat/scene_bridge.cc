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
#include <string_view>
#include <utility>

#include <filament/Options.h>
#include <filament/View.h>
#include <math/TMatHelpers.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <math/TVecHelpers.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/imgui_bridge.h"
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/compat/scene_geom_util.h"
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/scene_view.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3;
using filament::math::mat4;

static std::unique_ptr<Texture> CreateFallbackIndirectLightTexture(
    FilamentContext* ctx, std::string_view filename = "") {
  if (filename.empty()) {
    filename = ObjectManager::kDefaultEnvironmentLight;
  }

  std::unique_ptr<ObjectManager::Asset> asset =
      ctx->GetObjectManager()->LoadAsset(filename);

  mjrTextureConfig config;
  mjr_defaultTextureConfig(&config);
  config.width = 1;
  config.height = 1;
  config.target = mjTEXTURE_CUBE;
  config.format = mjPIXEL_FORMAT_KTX;
  config.color_space = mjCOLORSPACE_AUTO;

  auto texture = std::make_unique<Texture>(ctx, config);

  mjrTextureData payload;
  mjr_defaultTextureData(&payload);
  payload.bytes = asset->GetBytes().data();
  payload.nbytes = asset->GetBytes().size();
  payload.release_callback = +[](void* user_data) {
    delete static_cast<ObjectManager::Asset*>(user_data);
  };
  payload.user_data = asset.release();

  texture->Upload(payload);
  return texture;
}

SceneBridge::SceneBridge(FilamentContext* ctx, const mjModel* model)
    : ctx_(ctx) {
  mjrSceneParams params;
  mjr_defaultSceneParams(&params);
  scene_view_ = std::make_unique<SceneView>(ctx_, params);
  model_objects_ = std::make_unique<ModelObjects>(model, ctx_);

  // Configure options for the normal view.
  auto cg = scene_view_->GetColorGradingOptions();
  cg.exposure = ReadElement(model, "filament.cg.exposure", cg.exposure);
  cg.contrast = ReadElement(model, "filament.cg.contrast", cg.contrast);
  cg.vibrance = ReadElement(model, "filament.cg.vibrance", cg.vibrance);
  cg.saturation = ReadElement(model, "filament.cg.saturation", cg.saturation);
  cg.temperature =
      ReadElement(model, "filament.cg.temperature", cg.temperature);
  cg.tint = ReadElement(model, "filament.cg.tint", cg.tint);
  cg.gamut_mapping =
      ReadElement(model, "filament.cg.gamut_mapping", cg.gamut_mapping);
  cg.luminance_scaling =
      ReadElement(model, "filament.cg.luminance_scaling", cg.luminance_scaling);
  cg.slope = ReadElement(model, "filament.cg.slope", cg.slope);
  cg.offset = ReadElement(model, "filament.cg.offset", cg.offset);
  cg.power = ReadElement(model, "filament.cg.power", cg.power);
  cg.shadow_gamma =
      ReadElement(model, "filament.cg.shadow_gamma", cg.shadow_gamma);
  cg.mid_point = ReadElement(model, "filament.cg.mid_point", cg.mid_point);
  cg.highlight_scale =
      ReadElement(model, "filament.cg.highlight_scale", cg.highlight_scale);
  cg.shadows = ReadElement(model, "filament.cg.shadows", cg.shadows);
  cg.midtones = ReadElement(model, "filament.cg.midtones", cg.midtones);
  cg.highlights = ReadElement(model, "filament.cg.highlights", cg.highlights);
  cg.tonal_ranges =
      ReadElement(model, "filament.cg.tonal_ranges", cg.tonal_ranges);

  auto tone_mapping =
      ReadElement<std::string_view>(model, "filament.cg.tone_mapping");
  if (tone_mapping == "aces") {
    cg.tone_mapper = ToneMapperType::kACES;
  } else if (tone_mapping == "aces_legacy") {
    cg.tone_mapper = ToneMapperType::kACESLegacy;
  } else if (tone_mapping == "filmic") {
    cg.tone_mapper = ToneMapperType::kFilmic;
  } else if (tone_mapping == "linear") {
    cg.tone_mapper = ToneMapperType::kLinear;
  } else if (tone_mapping == "pbr_neutral") {
    cg.tone_mapper = ToneMapperType::kPBRNeutral;
  }
  scene_view_->SetColorGradingOptions(cg);

  filament::View* fview = scene_view_->GetDefaultRenderView();
  auto ao = fview->getAmbientOcclusionOptions();
  ao.enabled = ReadElement(model, "filament.ao.enabled", true);
  ao.bentNormals = ReadElement(model, "filament.ao.bent_normals", false);
  ao.ssct.enabled = ReadElement(model, "filament.ao.ssct", ao.ssct.enabled);
  ao.quality =
      ReadElement(model, "filament.ao.quality", filament::QualityLevel::ULTRA);
  ao.lowPassFilter = ReadElement(model, "filament.ao.low_pass_filter",
                                 filament::QualityLevel::ULTRA);
  ao.upsampling = ReadElement(model, "filament.ao.upsampling",
                              filament::QualityLevel::ULTRA);
  ao.bilateralThreshold =
      ReadElement(model, "filament.ao.bilateral_threshold", 0.5f);
  fview->setAmbientOcclusionOptions(ao);

  auto bloom = fview->getBloomOptions();
  bloom.enabled = ReadElement(model, "filament.bloom.enabled", bloom.enabled);
  bloom.strength =
      ReadElement(model, "filament.bloom.strength", bloom.strength);
  bloom.dirtStrength =
      ReadElement(model, "filament.bloom.dirt_strength", bloom.dirtStrength);
  bloom.quality = ReadElement(model, "filament.bloom.quality", bloom.quality);
  bloom.resolution =
      ReadElement(model, "filament.bloom.resolution", bloom.resolution);
  bloom.levels = ReadElement(model, "filament.bloom.levels", bloom.levels);
  fview->setBloomOptions(bloom);

  auto msaa = fview->getMultiSampleAntiAliasingOptions();
  msaa.enabled = ReadElement(model, "filament.msaa.enabled", true);
  fview->setMultiSampleAntiAliasingOptions(msaa);

  default_shadow_map_size_ = ReadElement(
      model, "filament.shadows.map_size", default_shadow_map_size_);
  default_vsm_blur_width_ = ReadElement(
      model, "filament.shadows.vsm_blur_width", default_vsm_blur_width_);

  auto shadow_type = fview->getShadowType();
  shadow_type = ReadElement(model, "filament.shadows.type", shadow_type);
  fview->setShadowType(shadow_type);

  auto fog_opts = fview->getFogOptions();
  fog_opts.enabled =
      ReadElement(model, "filament.fog.enabled", fog_opts.enabled);
  fog_opts.color = ReadElement(model, "filament.fog.color", fog_opts.color);
  fog_opts.distance = ReadElement(
      model, "filament.fog.distance", fog_opts.distance);
  fog_opts.density = ReadElement(
      model, "filament.fog.density", fog_opts.density);
  fog_opts.cutOffDistance = ReadElement(
      model, "filament.fog.cutOffDistance", fog_opts.cutOffDistance);
  fog_opts.maximumOpacity = ReadElement(
      model, "filament.fog.maximumOpacity", fog_opts.maximumOpacity);
  fog_opts.height = ReadElement(model, "filament.fog.height", fog_opts.height);
  fog_opts.heightFalloff = ReadElement(
      model, "filament.fog.heightFalloff", fog_opts.heightFalloff);
  fog_opts.inScatteringStart = ReadElement(
      model, "filament.fog.inScatteringStart", fog_opts.inScatteringStart);
  fog_opts.inScatteringSize = ReadElement(
      model, "filament.fog.inScatteringSize", fog_opts.inScatteringSize);
  fview->setFogOptions(fog_opts);

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
    scene_view_->RemoveFromScene(iter.get());
  }
  lights_.clear();
  if (fallback_ibl_) {
    scene_view_->RemoveFromScene(fallback_ibl_.get());
  }
  fallback_ibl_.reset();
  for (auto& iter : renderables_) {
    scene_view_->RemoveFromScene(iter.get());
  }
  renderables_.clear();
}

void SceneBridge::SetEnvironmentLight(std::string_view filename,
                                      float intensity) {
  for (auto& light : lights_) {
    if (light->GetType() == mjLIGHT_IMAGE) {
      scene_view_->RemoveFromScene(light.get());
      light.reset();
      break;
    }
  }
  if (fallback_ibl_) {
    scene_view_->RemoveFromScene(fallback_ibl_.get());
    fallback_ibl_.reset();
  }

  fallback_ibl_texture_ = CreateFallbackIndirectLightTexture(ctx_, filename);

  mjrLightParams params;
  mjr_defaultLightParams(&params);
  params.type = mjLIGHT_IMAGE;
  params.texture = fallback_ibl_texture_.get();
  params.intensity = intensity;
  fallback_ibl_ = std::make_unique<Light>(ctx_, params);
  scene_view_->AddToScene(fallback_ibl_.get());
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
      auto light_obj = std::make_unique<Light>(ctx_, params);
      scene_view_->AddToScene(light_obj.get());
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

      auto light_obj = std::make_unique<Light>(ctx_, params);
      scene_view_->AddToScene(light_obj.get());
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
    auto light_obj = std::make_unique<Light>(ctx_, params);
    scene_view_->AddToScene(light_obj.get());
    lights_.emplace_back(std::move(light_obj));
  }

  if (!has_image_based_light && total_light_intensity > 0.0f) {
    // Create a black indirect light to ensure that the skybox is
    // oriented to respect mujoco's Z-up convention.
    mjrLightParams params;
    mjr_defaultLightParams(&params);
    params.type = mjLIGHT_IMAGE;
    params.intensity = 10.0f;
    fallback_ibl_ = std::make_unique<Light>(ctx_, params);
    scene_view_->AddToScene(fallback_ibl_.get());
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
    fallback_ibl_ = std::make_unique<Light>(ctx_, params);
    scene_view_->AddToScene(fallback_ibl_.get());

    // Distribute the fallback scene light intensity among the lights.
    const float intensity = fallback_scene_light_intensity_ / lights_.size();
    for (auto& light : lights_) {
      if (light) {
        const bool is_headlight = (light == lights_.back());
        light->SetIntensity(is_headlight ? fallback_head_light_intensity_
                                         : intensity);
      }
    }
  }

  scene_view_->SetSkybox(model_objects_->GetSkyboxTexture());
}

mat4 CalculateClipFromWorld(const mjrRect& viewport, const mjvGLCamera& cam) {
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
  if (scene->flags[mjRND_SHADOW]) {
    scene_view_->EnableShadows();
  } else {
    scene_view_->DisableShadows();
  }
  if (scene->flags[mjRND_REFLECTION]) {
    scene_view_->EnableReflections();
  } else {
    scene_view_->DisableReflections();
  }

  mjtNum hpos[3], hfwd[3];
  float headpos[3], gazedir[3];
  mjv_cameraInModel(hpos, hfwd, nullptr, scene);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(gazedir, hfwd, 3);

  const mjvGLCamera gl_camera =
      mjv_averageCamera(scene->camera, scene->camera + 1);
  clip_from_world_ = CalculateClipFromWorld(viewport, gl_camera);

  // Remove all drawables from previous render and prepare new ones.
  for (auto& iter : renderables_) {
    scene_view_->RemoveFromScene(iter.get());
  }
  renderables_.clear();
  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        DrawTextAt(geom->label, pos->x, pos->y, pos->z);
      }
    }

    if (geom->type == mjGEOM_FLEX || geom->type == mjGEOM_SKIN) {
      model_objects_->CreateSkinFlexMesh(scene, *geom);
    }

    std::unique_ptr<Renderable> renderable = CreateGeomRenderable(
        *geom, scene, ctx_, model_objects_.get(), headpos);

    scene_view_->AddToScene(renderable.get());
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
      std::unique_ptr<Light>& light = lights_.back();
      light->SetColor(ReadFloat3(scene_light.diffuse));
      light->SetTransform(ReadFloat3(headpos), ReadFloat3(gazedir));
      continue;
    } else if (scene_light.id < lights_.size() - 1) {
      std::unique_ptr<Light>& light = lights_[scene_light.id];
      if (light) {
        light->SetColor(ReadFloat3(scene_light.diffuse));
        light->SetTransform(ReadFloat3(scene_light.pos),
                            ReadFloat3(scene_light.dir));
      }
    } else {
      mju_error("Unexpected light id: %d", scene_light.id);
    }
  }

  // Enable/disable the headlight based on whether or not it's in the scene.
  if (headlight_enabled) {
    lights_.back()->Enable();
  } else {
    lights_.back()->Disable();
  }
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
}  // namespace mujoco
