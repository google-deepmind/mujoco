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

#include "experimental/filament/filament/scene_view.h"

#include <array>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include <filament/ColorGrading.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Options.h>
#include <filament/RenderableManager.h>
#include <filament/Skybox.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/drawable.h"
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

static constexpr int kNormalIndex =
    static_cast<int>(SceneView::DrawMode::kNormal);
static constexpr int kDepthIndex =
    static_cast<int>(SceneView::DrawMode::kDepth);
static constexpr int kSegmentIndex =
    static_cast<int>(SceneView::DrawMode::kSegmentation);

static filament::Viewport ReadViewport(mjrRect rect) {
  return filament::Viewport(rect.left, rect.bottom, rect.width, rect.height);
}

filament::ColorGrading::Builder ToBuilder(const ColorGradingOptions& opts) {
  return filament::ColorGrading::Builder()
      .format(opts.format)
      .dimensions(opts.dimension)
      .luminanceScaling(opts.luminance_scaling)
      .gamutMapping(opts.gamut_mapping)
      .exposure(opts.exposure)
      .nightAdaptation(opts.night_adaptation)
      .contrast(opts.contrast)
      .vibrance(opts.vibrance)
      .saturation(opts.saturation)
      .whiteBalance(opts.temperature, opts.tint)
      .channelMixer(opts.out_red, opts.out_green, opts.out_blue)
      .shadowsMidtonesHighlights(opts.shadows, opts.midtones, opts.highlights,
                                 opts.tonal_ranges)
      .slopeOffsetPower(opts.slope, opts.offset, opts.power)
      .curves(opts.shadow_gamma, opts.mid_point, opts.highlight_scale);
}

SceneView::SceneView(filament::Engine* engine, ObjectManager* object_mgr)
    : object_mgr_(object_mgr), engine_(engine) {
  scene_ = engine_->createScene();
  camera_ = engine_->createCamera(utils::EntityManager::get().create());

  for (auto& view : views_) {
    view = engine_->createView();
    view->setScene(scene_);
    view->setCamera(camera_);
  }

  const mjModel* m = object_mgr_->GetModel();

  // Configure options for the normal view.
  auto& cg = color_grading_options_;
  cg.exposure = ReadElement(m, "filament.out.exposure", cg.exposure);
  cg.contrast = ReadElement(m, "filament.out.contrast", cg.contrast);
  cg.vibrance = ReadElement(m, "filament.out.vibrance", cg.vibrance);
  cg.saturation = ReadElement(m, "filament.out.saturation", cg.saturation);
  cg.temperature = ReadElement(m, "filament.out.temperature", cg.temperature);
  cg.tint = ReadElement(m, "filament.out.tint", cg.tint);

  auto tone_mapping =
      ReadElement<std::string_view>(m, "filament.out.tone_mapping");
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
  SetColorGradingOptions(cg);

  auto ao = views_[kNormalIndex]->getAmbientOcclusionOptions();
  ao.enabled = ReadElement(m, "filament.ao.enabled", true);
  ao.bentNormals = ReadElement(m, "filament.ao.bent_normals", false);
  ao.ssct.enabled = ReadElement(m, "filament.ao.ssct", ao.ssct.enabled);
  ao.quality = filament::QualityLevel::ULTRA;
  ao.lowPassFilter = filament::QualityLevel::ULTRA;
  ao.upsampling = filament::QualityLevel::ULTRA;
  ao.bilateralThreshold = 0.5f;
  views_[kNormalIndex]->setAmbientOcclusionOptions(ao);

  auto msaa = views_[kNormalIndex]->getMultiSampleAntiAliasingOptions();
  msaa.enabled = ReadElement(m, "filament.msaa.enabled", true);
  views_[kNormalIndex]->setMultiSampleAntiAliasingOptions(msaa);

  // Disable post processing for the depth and segmentation views to preserve
  // the values.
  views_[kDepthIndex]->setPostProcessingEnabled(false);
  views_[kSegmentIndex]->setPostProcessingEnabled(false);

  // Rotate the fog to align with mujoco's +Z up space.
  auto fog = views_[kNormalIndex]->getFogEntity();
  auto& tm = engine->getTransformManager();
  tm.create(fog);
  auto rotation_axis = ReadElement(
      m, "filament.fog.rotation_axis", float3{-1, 0, 0});
  tm.setTransform(tm.getInstance(fog),
                  mat4::rotation(filament::math::f::PI / 2, rotation_axis));

  auto fog_opts = views_[kNormalIndex]->getFogOptions();
  fog_opts.enabled = ReadElement(m, "filament.fog.enabled", fog_opts.enabled);
  fog_opts.color = ReadElement(m, "filament.fog.color", fog_opts.color);
  fog_opts.distance = ReadElement(
      m, "filament.fog.distance", fog_opts.distance);
  fog_opts.density = ReadElement(
      m, "filament.fog.density", fog_opts.density);
  fog_opts.cutOffDistance = ReadElement(
      m, "filament.fog.cutOffDistance", fog_opts.cutOffDistance);
  fog_opts.maximumOpacity = ReadElement(
      m, "filament.fog.maximumOpacity", fog_opts.maximumOpacity);
  fog_opts.height = ReadElement(m, "filament.fog.height", fog_opts.height);
  fog_opts.heightFalloff = ReadElement(
      m, "filament.fog.heightFalloff", fog_opts.heightFalloff);
  fog_opts.inScatteringStart = ReadElement(
      m, "filament.fog.inScatteringStart", fog_opts.inScatteringStart);
  fog_opts.inScatteringSize = ReadElement(
      m, "filament.fog.inScatteringSize", fog_opts.inScatteringSize);
  views_[kNormalIndex]->setFogOptions(fog_opts);

  // Create an empty/black indirect light to ensure that the skybox is oriented
  // to respect mujoco's Z-up convention.
  scene_->setIndirectLight(
      object_mgr_->CreateIndirectLight(nullptr, nullptr, 100000));

  PrepareLights();
}

SceneView::~SceneView() {
  lights_.clear();
  drawables_.clear();
  engine_->destroyCameraComponent(camera_->getEntity());
  engine_->destroy(views_[kNormalIndex]->getColorGrading());
  for (auto& view : views_) {
    engine_->destroy(view);
  }
  engine_->destroy(scene_);
}

filament::View* SceneView::PrepareRenderView(DrawMode mode) {
  for (auto& iter : drawables_) {
    iter->SetDrawMode(mode);
  }
  return views_[static_cast<int>(mode)];
}

void SceneView::SetViewport(mjrRect viewport) {
  aspect_ratio_ = (float)viewport.width / (float)viewport.height;
  for (auto& view : views_) {
    view->setViewport(ReadViewport(viewport));
  }
}

void SceneView::SetColorGradingOptions(const ColorGradingOptions& opts) {
  auto tone_mapper = CreateToneMapper(opts.tone_mapper);
  auto color_grading = ToBuilder(color_grading_options_)
                           .toneMapper(tone_mapper.get())
                           .build(*engine_);
  views_[kNormalIndex]->setColorGrading(color_grading);
  engine_->destroy(color_grading_);
  color_grading_ = color_grading;
  color_grading_options_ = opts;
}

void SceneView::SetEnvironmentLight(std::string_view filename,
                                    float intensity) {
  auto* ibl = object_mgr_->LoadFallbackIndirectLight(filename, intensity);
  if (ibl) {
    scene_->setIndirectLight(ibl);
  }
}

void SceneView::SetFallbackEnvironmentLight(float intensity) {
  auto* ibl = object_mgr_->GetFallbackIndirectLight();
  if (ibl) {
    ibl->setIntensity(intensity);
    scene_->setIndirectLight(ibl);
  }
}

void SceneView::UpdateCamera(const mjvGLCamera* cameras) {
  const mjvGLCamera cam = mjv_averageCamera(cameras, cameras + 1);
  const filament::Camera::Projection type =
      cam.orthographic ? filament::Camera::Projection::ORTHO
                       : filament::Camera::Projection::PERSPECTIVE;
  float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  float3 cam_at = cam_pos + cam_fwd;
  camera_->lookAt(cam_pos, cam_at, cam_up);
  float halfwidth = cam.frustum_width ? cam.frustum_width
            : 0.5f * aspect_ratio_ * (cam.frustum_top - cam.frustum_bottom);
  camera_->setProjection(type, cam.frustum_center - halfwidth,
                         cam.frustum_center + halfwidth, cam.frustum_bottom,
                         cam.frustum_top, cam.frustum_near, cam.frustum_far);
  clip_from_world_ = camera_->getProjectionMatrix() * camera_->getViewMatrix();
}

std::optional<float3> SceneView::ClipFromWorld(const float3& pos) const{
  const float4 clip_pos = clip_from_world_ * float4(pos, 1.0f);
  if (clip_pos.w == 0.0f) {
    return std::nullopt;
  }
  return clip_pos.xyz / clip_pos.w;
}

void SceneView::PrepareLights() {
  const mjModel* model = object_mgr_->GetModel();
  filament::Skybox* skybox = object_mgr_->CreateSkybox();
  if (skybox) {
    scene_->setSkybox(skybox);
  }

  float total_light_intensity = 0.0f;

  for (int i = 0; i < model->nlight; ++i) {
    total_light_intensity += model->light_intensity[i];

    if (model->light_type[i] == mjLIGHT_IMAGE) {
      auto* indirect_light = object_mgr_->CreateIndirectLight(
          model->light_texid[i], model->light_intensity[i]);
      if (indirect_light) {
        scene_->setIndirectLight(indirect_light);
      }
      // Add an nullptr as a placeholder so that our indices still match.
      lights_.emplace_back(nullptr);
    } else {
      Light::Params params;
      params.color = ReadFloat3(model->light_diffuse);
      params.type = (mjtLightType)model->light_type[i];
      params.castshadow = model->light_castshadow[i];
      params.bulbradius = model->light_bulbradius[i];
      params.range = model->light_range[i];
      params.intensity = model->light_intensity[i];
      if (params.type == mjLIGHT_SPOT) {
        params.spot_cone_angle = model->light_cutoff[i];
      }
      auto light_obj = std::make_unique<Light>(object_mgr_, params);
#ifndef __EMSCRIPTEN__
      // TODO(b/458045799): Re-enable when lights work on glinux and chromebook.
      light_obj->AddToScene(scene_);
#endif
      lights_.emplace_back(std::move(light_obj));
    }
  }

  // Add a placeholder (black) headlight as our last light. Going forward, we'll
  // assume lights_.back() is always the headlight.
  {
    Light::Params params;
    params.color = float3(0, 0, 0);
    params.headlight = true;
    params.type = mjLIGHT_DIRECTIONAL;
    params.castshadow = 0;
    params.intensity = 0;
    auto light_obj = std::make_unique<Light>(object_mgr_, params);
#ifndef __EMSCRIPTEN__
    // TODO(b/458045799): Re-enable when lights work on glinux and chromebook.
    light_obj->AddToScene(scene_);
#endif
    lights_.emplace_back(std::move(light_obj));
  }

  // There are no "physical" lights in the scene which means we're likely
  // dealing with a "classic renderer" scene. In this case, let's add a
  // default environment light and set the light intensity ourselves.
  if (total_light_intensity == 0.0f) {
    // Headlight is not required for Filament and often confusing, disable it by
    // default.
    constexpr float kHeadlightIntensityCandela = 0.f;
    constexpr float kTotalSceneLightIntensityCandela = 100'000.f;
    constexpr float kFallbackEnvironmentLightIntensityCandela = 10'000.f;

    SetFallbackEnvironmentLight(kFallbackEnvironmentLightIntensityCandela);
    const float intensity = kTotalSceneLightIntensityCandela / lights_.size();
    for (auto& light : lights_) {
      if (light) {
        light->SetIntensity(light->IsHeadlight() ? kHeadlightIntensityCandela
                                                : intensity);
      }
    }
  }
}

void SceneView::UpdateScene(const mjrContext* context, const mjvScene* scene) {
  mjtNum hpos[3], hfwd[3];
  float headpos[3], gazedir[3];
  mjv_cameraInModel(hpos, hfwd, nullptr, scene);
  mju_n2f(headpos, hpos, 3);
  mju_n2f(gazedir, hfwd, 3);
  UpdateCamera(scene->camera);

  // Remove all drawables from previous render and prepare new ones.
  for (auto& iter : drawables_) {
    iter->RemoveFromScene(scene_);
  }
  drawables_.clear();
  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        DrawTextAt(geom->label, pos->x, pos->y, pos->z);
      }
    }

    auto drawable = std::make_unique<Drawable>(object_mgr_, *geom);
    drawable->AddToScene(scene_);
    drawable->Update(object_mgr_->GetModel(), scene, *geom);
    drawables_.push_back(std::move(drawable));
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

filament::Engine* SceneView::GetEngine() const { return engine_; }

filament::View* SceneView::GetDefaultRenderView() {
  return views_[kNormalIndex];
}

ColorGradingOptions SceneView::GetColorGradingOptions() const {
  return color_grading_options_;
}

}  // namespace mujoco
