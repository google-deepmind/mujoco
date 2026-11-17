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

#include "render/filament/core/scene_view.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <filament/Color.h>
#include <filament/ColorGrading.h>
#include <filament/ColorSpace.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Options.h>
#include <filament/RenderTarget.h>
#include <filament/Renderer.h>
#include <filament/Skybox.h>
#include <filament/ToneMapper.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/TVecHelpers.h>
#include <math/mat3.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/builtins.h"
#include "render/filament/core/light.h"
#include "render/filament/core/material_manager.h"
#include "render/filament/core/object_manager.h"
#include "render/filament/core/outliner.h"
#include "render/filament/core/reflection_manager.h"
#include "render/filament/core/render_target.h"
#include "render/filament/core/renderable.h"
#include "render/filament/core/texture.h"
#include "render/filament/support/filament_util.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat3f;
using filament::math::mat4;
using filament::math::mat4f;

static filament::ColorGrading* BuildFilamentColorGrading(
    filament::Engine* engine, const mjrfColorGradingOpts& opts) {
  static constexpr filament::color::ColorSpace REC709_LINEAR_D65 =
      filament::color::Rec709 - filament::color::Linear - filament::color::D65;
  static constexpr filament::color::ColorSpace REC709_SRGB_D65 =
      filament::color::Rec709 - filament::color::sRGB - filament::color::D65;

  filament::color::ColorSpace output_color_space = REC709_SRGB_D65;
  if (opts.output_color_space == mjCOLORSPACE_LINEAR) {
    output_color_space = REC709_LINEAR_D65;
  }

  const float3 out_red = ReadFloat3(opts.out_red);
  const float3 out_green = ReadFloat3(opts.out_green);
  const float3 out_blue = ReadFloat3(opts.out_blue);
  const float4 shadows = ReadFloat4(opts.shadows);
  const float4 midtones = ReadFloat4(opts.midtones);
  const float4 highlights = ReadFloat4(opts.highlights);
  const float4 ranges = ReadFloat4(opts.tonal_ranges);
  const float3 slope = ReadFloat3(opts.slope);
  const float3 offset = ReadFloat3(opts.offset);
  const float3 power = ReadFloat3(opts.power);
  const float3 shadow_gamma = ReadFloat3(opts.shadow_gamma);
  const float3 mid_point = ReadFloat3(opts.mid_point);
  const float3 highlight_scale = ReadFloat3(opts.highlight_scale);

  std::unique_ptr<filament::ToneMapper> tone_mapper;
  switch (opts.tone_mapper) {
    case mjTONE_MAPPER_PBR_NEUTRAL:
      tone_mapper = std::make_unique<filament::PBRNeutralToneMapper>();
      break;
    case mjTONE_MAPPER_ACES:
      tone_mapper = std::make_unique<filament::ACESToneMapper>();
      break;
    case mjTONE_MAPPER_ACES_LEGACY:
      tone_mapper = std::make_unique<filament::ACESLegacyToneMapper>();
      break;
    case mjTONE_MAPPER_FILMIC:
      tone_mapper = std::make_unique<filament::FilmicToneMapper>();
      break;
    case mjTONE_MAPPER_LINEAR:
      tone_mapper = std::make_unique<filament::LinearToneMapper>();
      break;
    default:
      mju_error("Unsupported tone mapper type %d", opts.tone_mapper);
      return nullptr;
  }

  return filament::ColorGrading::Builder()
      .toneMapper(tone_mapper.get())
      .outputColorSpace(output_color_space)
      .exposure(opts.exposure)
      .nightAdaptation(opts.night_adaptation)
      .contrast(opts.contrast)
      .vibrance(opts.vibrance)
      .saturation(opts.saturation)
      .whiteBalance(opts.temperature, opts.tint)
      .channelMixer(out_red, out_green, out_blue)
      .shadowsMidtonesHighlights(shadows, midtones, highlights, ranges)
      .slopeOffsetPower(slope, offset, power)
      .curves(shadow_gamma, mid_point, highlight_scale)
      .build(*engine);
}

static filament::QualityLevel ToFilamentQuality(mjrQuality quality) {
  switch (quality) {
    case mjQUALITY_LOW:
      return filament::QualityLevel::LOW;
    case mjQUALITY_MEDIUM:
      return filament::QualityLevel::MEDIUM;
    case mjQUALITY_HIGH:
      return filament::QualityLevel::HIGH;
    case mjQUALITY_ULTRA:
      return filament::QualityLevel::ULTRA;
    default:
      mju_error("Unsupported quality level");
  }
  return filament::QualityLevel::LOW;
}

static void SetupCamera(const mjrfRenderRequest& request,
                        filament::Camera* camera) {
  const mjrCamera& cam = request.camera;
  const filament::Camera::Projection type =
      cam.orthographic ? filament::Camera::Projection::ORTHO
                       : filament::Camera::Projection::PERSPECTIVE;
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio =
      (float)request.viewport.width / (float)request.viewport.height;
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (cam.frustum_top - cam.frustum_bottom);
  camera->lookAt(cam_pos, cam_at, cam_up);
  camera->setProjection(type, cam.frustum_center - halfwidth,
                        cam.frustum_center + halfwidth, cam.frustum_bottom,
                        cam.frustum_top, cam.frustum_near, cam.frustum_far);
  if (request.draw_mode == mjDRAW_MODE_DEPTH) {
    // Filament's setProjection puts the perspective far plane at infinity; use
    // the finite-far culling frustum so depth maps span [near, far].
    const mat4 proj = camera->getCullingProjectionMatrix();
    camera->setCustomProjection(proj, proj, cam.frustum_near, cam.frustum_far);
  }
}

// Sets up the `reflection_camera` as a reflection of `src_camera` across the
// plane defined by `surface_xform`, so the reflection texture can be applied to
// the plane with screen-space uvs.
static void SetupReflectionCamera(const mat4& surface_xform,
                                  const filament::Camera* src_camera,
                                  filament::Camera* reflection_camera) {
  const mat4 src_model_matrix = src_camera->getModelMatrix();
  const mat4 src_projection = src_camera->getCullingProjectionMatrix();

  reflection_camera->setModelMatrix(ToReflectionMatrix(surface_xform) *
                                    src_model_matrix);

  // Use un-skewed projection to preserve froxel lighting grid and shadow cascades.
  reflection_camera->setCustomProjection(src_projection, src_projection,
                                         src_camera->getNear(),
                                         src_camera->getCullingFar());
}

SceneView::SceneView(ObjectManager* object_mgr, MaterialManager* material_mgr,
                     const mjrfSceneParams& params)
    : object_mgr_(object_mgr), material_mgr_(material_mgr) {
  filament::Engine* engine = object_mgr_->GetEngine();
  reflection_mgr_ = std::make_unique<ReflectionManager>(engine, material_mgr);

  scene_ = engine->createScene();
  camera_ = engine->createCamera(utils::EntityManager::get().create());
  reflect_camera_ = engine->createCamera(utils::EntityManager::get().create());

  main_view_ = engine->createView();
  main_view_->setScene(scene_);
  main_view_->setCamera(camera_);
  main_view_->setVisibleLayers(0xff, kLayerMask_All);

  depth_segment_view_ = engine->createView();
  depth_segment_view_->setScene(scene_);
  depth_segment_view_->setCamera(camera_);
  depth_segment_view_->setVisibleLayers(0xff, kLayerMask_Object);
  depth_segment_view_->setShadowingEnabled(false);
  depth_segment_view_->setPostProcessingEnabled(false);

  reflect_view_ = engine->createView();
  reflect_view_->setScene(scene_);
  reflect_view_->setCamera(reflect_camera_);
  reflect_view_->setShadowingEnabled(true);
  reflect_view_->setPostProcessingEnabled(false);
  reflect_view_->setFrontFaceWindingInverted(true);
  reflect_view_->setVisibleLayers(0xff, kLayerMask_Object | kLayerMask_Skybox);
  reflect_view_->setMultiSampleAntiAliasingOptions({.enabled = false});

  // The headlight is aimed at each request's camera in Render(). It is created
  // black and disabled; requests that ask for it supply color and intensity.
  mjrfLightParams headlight_params;
  mjrf_defaultLightParams(&headlight_params);
  headlight_params.type = mjLIGHT_DIRECTIONAL;
  headlight_params.cast_shadow = 0;
  headlight_params.intensity = 0.0f;
  headlight_ = std::make_unique<Light>(engine, headlight_params);
  headlight_->Disable();
  AddToScene(headlight_.get());

  // Rotate the fog to align with mujoco's +Z up space.
  auto fog = main_view_->getFogEntity();
  auto& tm = engine->getTransformManager();
  tm.create(fog);
  tm.setTransform(tm.getInstance(fog),
                  mat4::rotation(filament::math::f::PI / 2, float3{-1, 0, 0}));

  mjrfRenderableParams haze_params;
  mjrf_defaultRenderableParams(&haze_params);
  haze_params.cast_shadow = false;
  haze_params.receive_shadow = false;
  haze_ = std::make_unique<Renderable>(engine, haze_params, material_mgr_);
  haze_->SetPriority(0);
  SetHaze(0.3f, nullptr);
  haze_->SetMesh(object_mgr_->GetBuiltins()->Haze(Builtins::Slices{28}));
  AddToScene(haze_.get());
}

SceneView::~SceneView() {
  filament::Engine* engine = object_mgr_->GetEngine();

  outliner_.reset();
  if (skybox_) {
    scene_->setSkybox(nullptr);
    engine->destroy(skybox_);
  }
  for (auto& light : lights_) {
    light->RemoveFromScene(scene_);
  }
  for (auto& renderable : renderables_) {
    renderable->RemoveFromScene(scene_);
  }
  lights_.clear();
  renderables_.clear();
  headlight_.reset();
  haze_.reset();
  engine->destroyCameraComponent(reflect_camera_->getEntity());
  engine->destroy(reflect_view_);
  engine->destroyCameraComponent(camera_->getEntity());
  if (color_grading_) {
    engine->destroy(color_grading_);
  }
  engine->destroy(scene_);
  engine->destroy(depth_segment_view_);
  engine->destroy(main_view_);
}

void SceneView::AddToScene(Light* light) {
  if (lights_.insert(light).second) {
    light->AddToScene(scene_, [this](Light* light) { RemoveFromScene(light); });
  }
}

void SceneView::RemoveFromScene(Light* light) {
  if (lights_.erase(light)) {
    light->RemoveFromScene(scene_);
  }
}

void SceneView::AddToScene(Renderable* renderable) {
  if (renderables_.insert(renderable).second) {
    renderable->AddToScene(
        scene_, [this](Renderable* renderable) { RemoveFromScene(renderable); });
  }
}

void SceneView::RemoveFromScene(Renderable* renderable) {
  if (renderables_.erase(renderable)) {
    renderable->RemoveFromScene(scene_);
  }
}

void SceneView::SetSkybox(const Texture* skybox_texture) {
  if (skybox_) {
    scene_->setSkybox(nullptr);
    GetEngine()->destroy(skybox_);
    skybox_ = nullptr;
  }
  if (skybox_texture) {
    filament::Skybox::Builder builder;
    builder.environment(skybox_texture->GetFilamentTexture());
    skybox_ = builder.build(*GetEngine());
    scene_->setSkybox(skybox_);
  }
}

void SceneView::PrepareToRender(std::span<const mjrfRenderRequest*> requests) {
  for (const mjrfRenderRequest* request : requests) {
    if (request->scene != this) {
      mju_error("Invalid scene for SceneView::PrepareToRender.");
    }
  }

  bool has_selected = false;
  reflection_mgr_->ClearRenderables();
  for (Renderable* renderable : renderables_) {
    renderable->Prepare(requests, reflection_mgr_.get());
    if (renderable->GetMaterial().selected) {
      has_selected = true;
    }
  }

  if (has_selected) {
    mjrfMaterial material;
    mjrf_defaultMaterial(&material);
    material.selected = true;
    outline_material_key_ = material_mgr_->PrepareMaterialInstance(
        material, mjDRAW_MODE_DEFAULT, mjGEOM_NONE, nullptr);
  }
}

void SceneView::Render(filament::Renderer* renderer,
                       const mjrfRenderRequest& request) {
  if (request.scene != this) {
    mju_error("Invalid scene for SceneView::Render.");
  }

  filament::Viewport viewport(request.viewport.left, request.viewport.bottom,
                              request.viewport.width, request.viewport.height);
  filament::View* view = main_view_;
  if (request.draw_mode == mjDRAW_MODE_DEPTH ||
      request.draw_mode == mjDRAW_MODE_SEGMENTATION_BY_ID ||
      request.draw_mode == mjDRAW_MODE_SEGMENTATION_BY_COLOR) {
    view = depth_segment_view_;
  } else {
    view->setShadowingEnabled(request.shadows);
    view->setPostProcessingEnabled(request.post_processing);
  }
  view->setViewport(viewport);

  filament::MultiSampleAntiAliasingOptions options =
      view->getMultiSampleAntiAliasingOptions();

  RenderTarget* render_target = RenderTarget::downcast(request.target);
  if (render_target) {
    // We need to disable msaa in order to render to texture.
    view->setMultiSampleAntiAliasingOptions({.enabled = false});
  }

  SetupCamera(request, camera_);
  camera_->setFocusDistance(focus_distance_);

  // Aim the headlight along this request's camera. Filament re-gathers the
  // scene's lights for each render() call, so requests batched into one frame
  // each see their own headlight.
  if (request.headlight) {
    const float3 forward = ReadFloat3(request.camera.forward);
    // Sit slightly behind the eye to avoid clipping artifacts.
    const float3 position = ReadFloat3(request.camera.pos) - 0.05f * forward;
    headlight_->SetTransform(position, forward);
    headlight_->SetColor(ReadFloat3(request.headlight_color));
    headlight_->SetIntensity(request.headlight_intensity);
    headlight_->Enable();
  } else {
    headlight_->Disable();
  }

  // with a skybox, center the horizon disks of infinite planes beneath the
  // camera and place the haze on the nearest plane below it
  const bool show_skybox = skybox_ != nullptr && ShowsSkybox(request);
  const float3 cam_pos = ReadFloat3(request.camera.pos);
  const float horizon = kHorizonFarRatio * request.camera.frustum_far;
  float haze_elevation = std::numeric_limits<float>::infinity();
  float3 haze_center;
  mat3f haze_rotation;

  std::vector<Renderable*> selected_renderables;
  for (Renderable* renderable : renderables_) {
    renderable->BindMaterialInstance(request);
    if (renderable->GetMaterial().selected &&
        request.draw_mode == mjDRAW_MODE_DEFAULT) {
      selected_renderables.push_back(renderable);
    }
    if (show_skybox && renderable->IsInfinitePlane()) {
      const mat4f& transform = renderable->GetTransform();
      const float3 normal = normalize(transform[2].xyz);
      const float elevation = dot(cam_pos - transform[3].xyz, normal);
      const float3 center = cam_pos - elevation * normal;
      const mat3f rotation(normalize(transform[0].xyz),
                           normalize(transform[1].xyz), normal);
      // part 1 of an infinite plane is its horizon disk
      renderable->SetPartTransform(
          1, mat4f(rotation, center) *
                 mat4f::scaling(float3{horizon, horizon, 1.0f}));
      if (elevation > 0.0f && elevation < haze_elevation) {
        haze_elevation = elevation;
        haze_center = center;
        haze_rotation = rotation;
      }
    }
  }

  const bool show_haze =
      show_skybox && request.haze &&
      haze_elevation < std::numeric_limits<float>::infinity();
  if (show_haze) {
    haze_->SetTransform(haze_center, haze_rotation);
    haze_->SetSize(float3{horizon, horizon, haze_elevation});
  }

  main_view_->setVisibleLayers(
      kLayerMask_Skybox | kLayerMask_Haze,
      (show_skybox ? kLayerMask_Skybox : 0) |
          (show_haze ? kLayerMask_Haze : 0));
  reflect_view_->setVisibleLayers(
      0xff, kLayerMask_Object | (show_skybox ? kLayerMask_Skybox : 0));

  const int num_reflective = reflection_mgr_->GetNumRenderables();
  for (int i = 0; i < num_reflective; ++i) {
    Renderable* renderable =
        Renderable::downcast(reflection_mgr_->GetRenderable(i));
    const mat4 transform(renderable->GetTransform());
    SetupReflectionCamera(transform, camera_, reflect_camera_);

    // Hide reflective surface from its own reflection pass.
    std::uint8_t prev_layer_mask = renderable->SetLayerMask(kLayerMask_None);

    // Hide geometry completely behind the mirror plane from the reflection pass.
    const float3 mirror_pos = transform[3].xyz;
    const float3 mirror_normal = normalize(transform[2].xyz);
    std::vector<std::pair<Renderable*, std::uint8_t>> hidden_behind;
    for (Renderable* other : renderables_) {
      if (other == renderable) {
        continue;
      }
      const mat4 other_xform(other->GetTransform());
      const float3 center = other_xform[3].xyz;
      // The geom's three scaled axes reach every corner, so their summed
      // lengths conservatively bound it; only hide when the whole bound is
      // behind the plane.
      const float radius = length(other_xform[0].xyz) +
                           length(other_xform[1].xyz) +
                           length(other_xform[2].xyz);
      if (dot(center - mirror_pos, mirror_normal) + radius < 0.0f) {
        hidden_behind.emplace_back(other, other->SetLayerMask(kLayerMask_None));
      }
    }

    // Render the reflection to its render target.
    viewport.left = 0;
    viewport.bottom = 0;
    reflect_view_->setViewport(viewport);
    const RenderTarget* target = reflection_mgr_->GetRenderTarget(i);
    reflect_view_->setRenderTarget(target->GetFilamentRenderTarget());
    renderer->render(reflect_view_);
    reflect_view_->setRenderTarget(nullptr);

    // Restore the behind-mirror geometry and the reflective surface.
    for (const auto& [other, mask] : hidden_behind) {
      other->SetLayerMask(mask);
    }
    renderable->SetLayerMask(prev_layer_mask);
  }
  for (int i = 0; i < num_reflective; ++i) {
    Renderable::downcast(reflection_mgr_->GetRenderable(i))
        ->SetMaterialInstance(reflection_mgr_->GetMaterialKey(i));
  }

  filament::RenderTarget* filament_render_target =
      render_target ? render_target->GetFilamentRenderTarget() : nullptr;
  view->setRenderTarget(filament_render_target);
  renderer->render(view);
  view->setRenderTarget(nullptr);

  if (!selected_renderables.empty()) {
    if (!outliner_) {
      outliner_ = std::make_unique<Outliner>(
          object_mgr_, kLayerMask_Outline, kLayerMask_Object,
          float4{0.9f, 0.9f, 0.2f, 0.7f}, 3.5f, 0.25f);
    }

    for (Renderable* renderable : selected_renderables) {
      renderable->SetMaterialInstance(outline_material_key_);
    }
    outliner_->Render(renderer, view, filament_render_target);
  }

  if (request.target) {
    view->setMultiSampleAntiAliasingOptions(options);
  }
}

void SceneView::SetColorGradingOptions(const mjrfColorGradingOpts& opts) {
  auto color_grading = BuildFilamentColorGrading(GetEngine(), opts);
  main_view_->setColorGrading(color_grading);
  if (color_grading_) {
    GetEngine()->destroy(color_grading_);
  }
  color_grading_ = color_grading;
  color_grading_options_ = opts;
}

filament::View* SceneView::GetDefaultRenderView() { return main_view_; }

mjrfColorGradingOpts SceneView::GetColorGradingOptions() const {
  return color_grading_options_;
}

void SceneView::SetMsaa(int samples) {
  samples = std::clamp(samples, 1, 255);
  auto msaa = main_view_->getMultiSampleAntiAliasingOptions();
  msaa.enabled = samples > 1;
  msaa.sampleCount = static_cast<uint8_t>(samples);
  main_view_->setMultiSampleAntiAliasingOptions(msaa);
}

void SceneView::SetHaze(float ratio, const float rgba[4]) {
  mjrfMaterial material = haze_->GetMaterial();
  material.haze_ratio = std::clamp(ratio, 0.0f, 1.0f);
  if (rgba) {
    std::copy(rgba, rgba + 4, material.color);
  }
  haze_->UpdateMaterial(material);
}

float SceneView::GetHazeRatio() const {
  return haze_->GetMaterial().haze_ratio;
}

float4 SceneView::GetHazeColor() const {
  return ReadFloat4(haze_->GetMaterial().color);
}

void SceneView::Configure(const mjModel* model) {
  mjrfRenderOpts opts;
  mjrf_modelRenderOpts(&opts, model);

  // The new RenderOpts struct uses slightly different defaults. For now, we're
  // going to preserve the old defaults if the user hasn't configured these
  // options. We will remove this logic, but it will affect rendering so we
  // need to be mindful of how we deploy it.
  if (mj_name2id(model, mjOBJ_NUMERIC, "filament.cg.shadows") < 0) {
    opts.color_grading.shadows[3] = 1.0f;
  }
  if (mj_name2id(model, mjOBJ_NUMERIC, "filament.cg.midtones") < 0) {
    opts.color_grading.midtones[3] = 1.0f;
  }
  if (mj_name2id(model, mjOBJ_NUMERIC, "filament.cg.highlights") < 0) {
    opts.color_grading.highlights[3] = 1.0f;
  }
  if (mj_name2id(model, mjOBJ_TEXT, "filament.cg.tone_mapping") < 0) {
    opts.color_grading.tone_mapper = mjTONE_MAPPER_PBR_NEUTRAL;
  }
  if (mj_name2id(model, mjOBJ_TEXT, "filament.cg.output_color_space") < 0) {
    opts.color_grading.output_color_space = mjCOLORSPACE_SRGB;
  }
  if (mj_name2id(model, mjOBJ_TEXT, "filament.ao.quality") < 0) {
    opts.ambient_occlusion.quality = mjQUALITY_ULTRA;
  }
  if (mj_name2id(model, mjOBJ_TEXT, "filament.ao.low_pass_filter") < 0) {
    opts.ambient_occlusion.low_pass_filter = mjQUALITY_ULTRA;
  }
  if (mj_name2id(model, mjOBJ_TEXT, "filament.ao.upsampling") < 0) {
    opts.ambient_occlusion.upsampling = mjQUALITY_ULTRA;
  }

  main_view_->setPostProcessingEnabled(opts.post_processing);

  Apply(opts.ambient_occlusion);
  Apply(opts.antialiasing);
  Apply(opts.bloom);
  Apply(opts.color_grading);
  Apply(opts.depth_of_field);
  Apply(opts.dithering);
  Apply(opts.fog);
  Apply(opts.haze);
  Apply(opts.headlight);
  Apply(opts.material);
  Apply(opts.shadow);
  Apply(opts.vignette);
}

void SceneView::Apply(const mjrfAmbientOcclusionOpts& opts) {
  auto ao = main_view_->getAmbientOcclusionOptions();
  ao.enabled = opts.type != mjAMBIENT_OCCLUSION_TYPE_NONE;
  ao.aoType = opts.type == mjAMBIENT_OCCLUSION_TYPE_SCALABLE
                  ? filament::AmbientOcclusionOptions::AmbientOcclusionType::SAO
                  : filament::AmbientOcclusionOptions::AmbientOcclusionType::GTAO;
  ao.radius = opts.radius;
  ao.intensity = opts.strength;
  ao.power = opts.power;
  ao.bentNormals = opts.bent_normals;
  ao.ssct.enabled = opts.ssct_enable;
  ao.quality = ToFilamentQuality((mjrQuality)opts.quality);
  ao.lowPassFilter = ToFilamentQuality((mjrQuality)opts.low_pass_filter);
  ao.upsampling = ToFilamentQuality((mjrQuality)opts.upsampling);
  main_view_->setAmbientOcclusionOptions(ao);
}

void SceneView::Apply(const mjrfAntialiasingOpts& opts) {
  main_view_->setAntiAliasing(opts.fxaa_enable ? filament::AntiAliasing::FXAA
                                               : filament::AntiAliasing::NONE);

  auto maa = main_view_->getMultiSampleAntiAliasingOptions();
  maa.enabled = opts.msaa_samples > 1;
  maa.sampleCount = std::clamp(opts.msaa_samples, 1, 255);
  main_view_->setMultiSampleAntiAliasingOptions(maa);

  auto taa = main_view_->getTemporalAntiAliasingOptions();
  taa.enabled = opts.tsaa_enable;
  main_view_->setTemporalAntiAliasingOptions(taa);
}

void SceneView::Apply(const mjrfBloomOpts& opts) {
  auto bloom = main_view_->getBloomOptions();
  bloom.enabled = opts.enable;
  bloom.strength = opts.strength;
  bloom.quality = ToFilamentQuality((mjrQuality)opts.quality);
  bloom.levels = opts.levels;
  bloom.lensFlare = opts.lens_flare;
  bloom.starburst = opts.starburst;
  main_view_->setBloomOptions(bloom);
}

void SceneView::Apply(const mjrfColorGradingOpts& opts) {
  SetColorGradingOptions(opts);
}

void SceneView::Apply(const mjrfDepthOfFieldOpts& opts) {
  auto dof = main_view_->getDepthOfFieldOptions();
  dof.enabled = opts.enable;
  dof.cocScale = opts.blur_strength;
  dof.maxApertureDiameter = opts.max_aperture_diameter;
  dof.cocAspectRatio = opts.aspect_ratio;
  focus_distance_ = opts.focus_distance;
  main_view_->setDepthOfFieldOptions(dof);
}

void SceneView::Apply(const mjrfDitheringOpts& opts) {
  auto dither = main_view_->getDithering();
  switch (opts.type) {
    case mjDITHERING_TYPE_NONE:
      dither = filament::Dithering::NONE;
      break;
    case mjDITHERING_TYPE_TEMPORAL:
      dither = filament::Dithering::TEMPORAL;
      break;
    default:
      mju_error("Unknown dithering type: %d", opts.type);
      break;
  }
  main_view_->setDithering(dither);
}

void SceneView::Apply(const mjrfFogOpts& opts) {
  auto fog = main_view_->getFogOptions();
  fog.enabled = opts.enable;
  fog.color = ReadFloat3(opts.color);
  fog.distance = opts.distance;
  fog.density = opts.density;
  fog.cutOffDistance = opts.cutoff;
  fog.maximumOpacity = opts.max_opacity;
  fog.height = opts.height;
  fog.heightFalloff = opts.height_falloff;
  main_view_->setFogOptions(fog);
}

void SceneView::Apply(const mjrfHazeOpts& opts) {
  mjrfMaterial material = haze_->GetMaterial();
  material.haze_ratio = std::clamp(opts.ratio, 0.0f, 1.0f);
  material.color[0] = opts.color[0];
  material.color[1] = opts.color[1];
  material.color[2] = opts.color[2];
  material.color[3] = opts.color[3];
  haze_->UpdateMaterial(material);
  const Builtins::Slices nslice{opts.slices};
  haze_->SetMesh(object_mgr_->GetBuiltins()->Haze(nslice));
}

void SceneView::Apply(const mjrfHeadlightOpts& opts) {
  if (opts.enable) {
    headlight_->SetColor(float3{1.0f, 1.0f, 1.0f});
    headlight_->SetIntensity(opts.intensity);
    headlight_->Enable();
  } else {
    headlight_->Disable();
  }
}

void SceneView::Apply(const mjrfMaterialOpts& opts) {
  if (opts.rgb_color_type == mjCOLORSPACE_LINEAR) {
    material_mgr_->SetBaseColorType(filament::RgbaType::LINEAR);
  } else {
    material_mgr_->SetBaseColorType(filament::RgbaType::sRGB);
  }
}

void SceneView::Apply(const mjrfShadowOpts& opts) {
  main_view_->setShadowingEnabled(opts.type != mjSHADOW_TYPE_NONE);

  auto shadow_type = main_view_->getShadowType();
  if (opts.type == mjSHADOW_TYPE_SOFT) {
    shadow_type = filament::View::ShadowType::PCSS;
  } else {
    shadow_type = filament::View::ShadowType::PCF;
  }
  main_view_->setShadowType(shadow_type);
}

void SceneView::Apply(const mjrfVignetteOpts& opts) {
  auto vignette = main_view_->getVignetteOptions();
  vignette.enabled = opts.enable;
  vignette.midPoint = opts.mid_point;
  vignette.color = ReadFloat4(opts.color);
  vignette.feather = opts.feather;
  vignette.roundness = opts.roundness;
  main_view_->setVignetteOptions(vignette);
}
}  // namespace mujoco
