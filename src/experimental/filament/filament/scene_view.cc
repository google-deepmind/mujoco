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
#include <cstddef>
#include <memory>
#include <optional>
#include <string_view>
#include <utility>

#include <filament/ColorGrading.h>
#include <filament/IndirectLight.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Options.h>
#include <filament/Renderer.h>
#include <filament/RenderableManager.h>
#include <filament/RenderTarget.h>
#include <filament/Skybox.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <math/TVecHelpers.h>
#include <utils/EntityManager.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/drawable.h"
#include "experimental/filament/filament/gui_view.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/model_objects.h"
#include "experimental/filament/filament/model_util.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/render_target_util.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3;
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

// Sets up the `reflection_camera`'s projection matrix so that it is a
// reflection of the `src_camera` across the plane defined by the
// `surface_xform`. The generated projection is an oblique projection so that
// the texture can be applied directly to the plane with screenspace uvs.
static void SetupReflectionCamera(const mat4& surface_xform,
                                  const filament::Camera* src_camera,
                                  filament::Camera* reflection_camera,
                                  float near = 0.01f, float far = 100.0f) {
  const mat4 src_model_matrix = src_camera->getModelMatrix();
  const mat4 src_view_matrix = src_camera->getViewMatrix();
  const mat4 src_projection = src_camera->getProjectionMatrix();

  reflection_camera->setModelMatrix(ToReflectionMatrix(surface_xform) *
                                    src_model_matrix);

  const float3 normal = surface_xform[2].xyz;
  const float3 view_pos = (src_view_matrix * surface_xform[3]).xyz;
  const float3 view_normal = (src_view_matrix * float4(normal, 0.0f)).xyz;

  const float3 plane_normal_camera = view_normal;
  const float plane_dist_camera = -dot(plane_normal_camera, view_pos);
  const float4 oblique_plane(plane_normal_camera, plane_dist_camera);
  const mat4 oblique = CalculateObliqueProjection(src_projection, oblique_plane);
  reflection_camera->setCustomProjection(oblique, near, far);
}

SceneView::SceneView(ObjectManager* object_mgr, const mjModel* model)
    : object_mgr_(object_mgr) {
  filament::Engine* engine = object_mgr_->GetEngine();
  model_objects_ = std::make_unique<ModelObjects>(model, engine);

  scene_ = engine->createScene();
  camera_ = engine->createCamera(utils::EntityManager::get().create());
  reflect_camera_ = engine->createCamera(utils::EntityManager::get().create());

  for (auto& view : views_) {
    view = engine->createView();
    view->setScene(scene_);
    view->setCamera(camera_);
  }

  reflect_view_ = engine->createView();
  reflect_view_->setScene(scene_);
  reflect_view_->setCamera(reflect_camera_);
  reflect_view_->setShadowingEnabled(false);
  reflect_view_->setPostProcessingEnabled(false);

  // Configure options for the normal view.
  auto& cg = color_grading_options_;
  cg.exposure = ReadElement(model, "filament.out.exposure", cg.exposure);
  cg.contrast = ReadElement(model, "filament.out.contrast", cg.contrast);
  cg.vibrance = ReadElement(model, "filament.out.vibrance", cg.vibrance);
  cg.saturation = ReadElement(model, "filament.out.saturation", cg.saturation);
  cg.temperature = ReadElement(model, "filament.out.temperature", cg.temperature);
  cg.tint = ReadElement(model, "filament.out.tint", cg.tint);

  auto tone_mapping =
      ReadElement<std::string_view>(model, "filament.out.tone_mapping");
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
  ao.enabled = ReadElement(model, "filament.ao.enabled", true);
  ao.bentNormals = ReadElement(model, "filament.ao.bent_normals", false);
  ao.ssct.enabled = ReadElement(model, "filament.ao.ssct", ao.ssct.enabled);
  ao.quality = filament::QualityLevel::ULTRA;
  ao.lowPassFilter = filament::QualityLevel::ULTRA;
  ao.upsampling = filament::QualityLevel::ULTRA;
  ao.bilateralThreshold = 0.5f;
  views_[kNormalIndex]->setAmbientOcclusionOptions(ao);

  auto msaa = views_[kNormalIndex]->getMultiSampleAntiAliasingOptions();
  msaa.enabled = ReadElement(model, "filament.msaa.enabled", true);
  views_[kNormalIndex]->setMultiSampleAntiAliasingOptions(msaa);

  default_shadow_map_size_ = ReadElement(
      model, "filament.shadows.map_size", default_shadow_map_size_);
  default_vsm_blur_width_ = ReadElement(
      model, "filament.shadows.vsm_blur_width", default_vsm_blur_width_);

  auto shadow_type = views_[kNormalIndex]->getShadowType();
  shadow_type = ReadElement(model, "filament.shadows.type", shadow_type);
  views_[kNormalIndex]->setShadowType(shadow_type);

  // Disable post processing for the depth and segmentation views to preserve
  // the values.
  views_[kDepthIndex]->setPostProcessingEnabled(false);
  views_[kSegmentIndex]->setPostProcessingEnabled(false);

  // Rotate the fog to align with mujoco's +Z up space.
  auto fog = views_[kNormalIndex]->getFogEntity();
  auto& tm = engine->getTransformManager();
  tm.create(fog);
  auto rotation_axis = ReadElement(
      model, "filament.fog.rotation_axis", float3{-1, 0, 0});
  tm.setTransform(tm.getInstance(fog),
                  mat4::rotation(filament::math::f::PI / 2, rotation_axis));

  auto fog_opts = views_[kNormalIndex]->getFogOptions();
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
  views_[kNormalIndex]->setFogOptions(fog_opts);

  fallback_head_light_intensity_ =
      ReadElement(model, "filament.fallback.head_light_intensity",
                  fallback_head_light_intensity_);
  fallback_scene_light_intensity_ =
      ReadElement(model, "filament.fallback.scene_light_intensity",
                  fallback_scene_light_intensity_);
  fallback_environment_light_intensity_ =
      ReadElement(model, "filament.fallback.environment_light_intensity",
                  fallback_environment_light_intensity_);

  // Create an empty/black indirect light to ensure that the skybox is oriented
  // to respect mujoco's Z-up convention.
  scene_->setIndirectLight(model_objects_->CreateIndirectLight(-1, 100000));

  PrepareLights();
}

SceneView::~SceneView() {
  lights_.clear();
  drawables_.clear();
  reflect_targets_.clear();

  filament::Engine* engine = object_mgr_->GetEngine();
  engine->destroyCameraComponent(reflect_camera_->getEntity());
  engine->destroy(reflect_view_);

  engine->destroyCameraComponent(camera_->getEntity());
  engine->destroy(views_[kNormalIndex]->getColorGrading());
  for (auto& view : views_) {
    engine->destroy(view);
  }
  engine->destroy(scene_);
}

void SceneView::Render(filament::Renderer* renderer, DrawMode draw_mode,
                       filament::RenderTarget* target) {
  filament::View* view = PrepareRenderView(draw_mode);
  filament::MultiSampleAntiAliasingOptions options =
      view->getMultiSampleAntiAliasingOptions();

  if (target) {
    // We need to disable msaa in order to render to texture.
    view->setMultiSampleAntiAliasingOptions({.enabled = false});
  }

  // Render reflection passes.
  if (draw_mode == DrawMode::kNormal) {
    for (size_t i = 0; i < reflectives_.size(); ++i) {
      Drawable* drawable = reflectives_[i];

      SetupReflectionCamera(drawable->GetTransform(), camera_, reflect_camera_);

      // Hide reflective surface from its own reflection pass.
      drawable->SetLayerMask(0x00);

      // Render the reflection to its render target.
      reflect_view_->setRenderTarget(reflect_targets_[i]->GetRenderTarget());
      renderer->render(reflect_view_);

      // Unhide the reflective surface.
      drawable->SetLayerMask(0x01);
    }
  }

  view->setRenderTarget(target);
  renderer->render(view);
  view->setRenderTarget(nullptr);

  if (target) {
    view->setMultiSampleAntiAliasingOptions(options);
  }
}

filament::View* SceneView::PrepareRenderView(DrawMode mode) {
  for (auto& iter : drawables_) {
    iter->SetDrawMode(mode);
  }
  return views_[static_cast<int>(mode)];
}

void SceneView::SetViewport(mjrRect viewport) {
  auto filament_viewport = ReadViewport(viewport);
  aspect_ratio_ = (float)viewport.width / (float)viewport.height;
  for (auto& view : views_) {
    view->setViewport(filament_viewport);
  }
  reflect_view_->setViewport(filament_viewport);
}

void SceneView::SetColorGradingOptions(const ColorGradingOptions& opts) {
  filament::Engine* engine = object_mgr_->GetEngine();

  auto tone_mapper = CreateToneMapper(opts.tone_mapper);
  auto color_grading = ToBuilder(color_grading_options_)
                           .toneMapper(tone_mapper.get())
                           .build(*engine);
  views_[kNormalIndex]->setColorGrading(color_grading);
  engine->destroy(color_grading_);
  color_grading_ = color_grading;
  color_grading_options_ = opts;
}

void SceneView::SetEnvironmentLight(std::string_view filename,
                                    float intensity) {
  scene_->setIndirectLight(nullptr);
  object_mgr_->LoadFallbackIndirectLight(filename, intensity);
  scene_->setIndirectLight(object_mgr_->GetFallbackIndirectLight());
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
  filament::Engine* engine = object_mgr_->GetEngine();
  const mjModel* model = model_objects_->GetModel();
  filament::Skybox* skybox = model_objects_->CreateSkybox();
  if (skybox) {
    scene_->setSkybox(skybox);
  }

  float total_light_intensity = 0.0f;

  for (int i = 0; i < model->nlight; ++i) {
    total_light_intensity += model->light_intensity[i];

    if (model->light_type[i] == mjLIGHT_IMAGE) {
      auto* indirect_light = model_objects_->CreateIndirectLight(
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
      params.shadow_map_size = default_shadow_map_size_;
      params.vsm_blur_width = default_vsm_blur_width_;
      if (params.type == mjLIGHT_SPOT) {
        params.spot_cone_angle = model->light_cutoff[i];
      }

      auto light_obj = std::make_unique<Light>(engine, params);
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
    auto light_obj = std::make_unique<Light>(engine, params);
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
    SetFallbackEnvironmentLight(fallback_environment_light_intensity_);
    const float intensity = fallback_scene_light_intensity_ / lights_.size();
    for (auto& light : lights_) {
      if (light) {
        light->SetIntensity(
            light->IsHeadlight() ? fallback_head_light_intensity_ : intensity);
      }
    }
  }
}

void SceneView::UpdateScene(const mjvScene* scene) {
  views_[kNormalIndex]->setShadowingEnabled(scene->flags[mjRND_SHADOW]);

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
  reflectives_.clear();
  for (int i = 0; i < scene->ngeom; ++i) {
    const mjvGeom* geom = scene->geoms + i;

    if (geom->label[0] != 0) {
      if (auto pos = ClipFromWorld(ReadFloat3(geom->pos))) {
        DrawTextAt(geom->label, pos->x, pos->y, pos->z);
      }
    }

    auto drawable =
        std::make_unique<Drawable>(object_mgr_, model_objects_.get(), *geom);
    drawable->AddToScene(scene_);
    drawable->Update(model_objects_->GetModel(), scene, *geom);
    if (drawable->IsReflective()) {
      AddReflectiveDrawable(drawable.get());
    }
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

void SceneView::AddReflectiveDrawable(Drawable* drawable) {
  const int index = reflectives_.size();
  reflectives_.push_back(drawable);

  // Ensure we have the same number of render targets as we do reflective
  // drawables.
  filament::Engine* engine = object_mgr_->GetEngine();
  while (reflect_targets_.size() < reflectives_.size()) {
    reflect_targets_.push_back(std::make_unique<RenderTargetAndTextures>(
        engine, kRenderTargetReflectionColor, kRenderTargetDepth));
  }

  // Prepare a render target for the reflective drawable.
  auto viewport = reflect_view_->getViewport();
  auto& target = reflect_targets_[index];
  target->Prepare(viewport.width, viewport.height);
  drawable->UpdateReflectionTexture(target->GetColorTexture());
}

void SceneView::UploadMesh(const mjModel* model, int id) {
  model_objects_->UploadMesh(model, id);
}

void SceneView::UploadTexture(const mjModel* model, int id) {
  model_objects_->UploadTexture(model, id);
}

void SceneView::UploadHeightField(const mjModel* model, int id) {
  model_objects_->UploadHeightField(model, id);
}

filament::Engine* SceneView::GetEngine() const {
  return object_mgr_->GetEngine();
}

filament::View* SceneView::GetDefaultRenderView() {
  return views_[kNormalIndex];
}

ColorGradingOptions SceneView::GetColorGradingOptions() const {
  return color_grading_options_;
}

}  // namespace mujoco
