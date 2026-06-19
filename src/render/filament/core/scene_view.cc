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

#include <cstdint>
#include <memory>
#include <span>
#include <string_view>
#include <vector>

#include <filament/ColorGrading.h>
#include <filament/LightManager.h>
#include <filament/Material.h>
#include <filament/Options.h>
#include <filament/RenderTarget.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Skybox.h>
#include <filament/TransformManager.h>
#include <filament/View.h>
#include <filament/Viewport.h>
#include <math/TVecHelpers.h>
#include <math/mat4.h>
#include <math/mathfwd.h>
#include <math/scalar.h>
#include <math/vec3.h>
#include <math/vec4.h>
#include <utils/EntityManager.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/core/color_grading_options.h"
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
using filament::math::mat4;

static filament::ColorGrading::Builder ToBuilder(
    const ColorGradingOptions& opts) {
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

static void SetupCamera(const mjrCamera& cam,
                        const filament::Viewport& viewport,
                        filament::Camera* camera) {
  const filament::Camera::Projection type =
      cam.orthographic ? filament::Camera::Projection::ORTHO
                       : filament::Camera::Projection::PERSPECTIVE;
  const float3 cam_pos(cam.pos[0], cam.pos[1], cam.pos[2]);
  const float3 cam_fwd(cam.forward[0], cam.forward[1], cam.forward[2]);
  const float3 cam_up(cam.up[0], cam.up[1], cam.up[2]);
  const float3 cam_at = cam_pos + cam_fwd;
  const float aspect_ratio = (float)viewport.width / (float)viewport.height;
  const float halfwidth =
      cam.frustum_width
          ? cam.frustum_width
          : 0.5f * aspect_ratio * (cam.frustum_top - cam.frustum_bottom);
  camera->lookAt(cam_pos, cam_at, cam_up);
  camera->setProjection(type, cam.frustum_center - halfwidth,
                        cam.frustum_center + halfwidth, cam.frustum_bottom,
                        cam.frustum_top, cam.frustum_near, cam.frustum_far);
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
  const mat4 oblique =
      CalculateObliqueProjection(src_projection, oblique_plane);
  reflection_camera->setCustomProjection(oblique, near, far);
}

SceneView::SceneView(ObjectManager* object_mgr, MaterialManager* material_mgr,
                     const mjrfSceneParams& params)
    : object_mgr_(object_mgr), material_mgr_(material_mgr) {
  filament::Engine* engine = object_mgr_->GetEngine();
  reflection_mgr_ = std::make_unique<ReflectionManager>(engine);

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
  depth_segment_view_->setPostProcessingEnabled(false);

  reflect_view_ = engine->createView();
  reflect_view_->setScene(scene_);
  reflect_view_->setCamera(reflect_camera_);
  reflect_view_->setShadowingEnabled(false);
  reflect_view_->setPostProcessingEnabled(false);
  reflect_view_->setFrontFaceWindingInverted(true);
  reflect_view_->setVisibleLayers(0xff, kLayerMask_Object);
  reflect_view_->setMultiSampleAntiAliasingOptions({.enabled = false});

  // Rotate the fog to align with mujoco's +Z up space.
  auto fog = main_view_->getFogEntity();
  auto& tm = engine->getTransformManager();
  tm.create(fog);
  tm.setTransform(tm.getInstance(fog),
                  mat4::rotation(filament::math::f::PI / 2, float3{-1, 0, 0}));
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
    light->AddToScene(scene_);
  }
}

void SceneView::RemoveFromScene(Light* light) {
  if (lights_.erase(light)) {
    light->RemoveFromScene(scene_);
  }
}

void SceneView::AddToScene(Renderable* renderable) {
  if (renderables_.insert(renderable).second) {
    renderable->AddToScene(scene_);
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
  }
  view->setViewport(viewport);
  view->setShadowingEnabled(request.enable_shadows);
  view->setPostProcessingEnabled(request.enable_post_processing);

  filament::MultiSampleAntiAliasingOptions options =
      view->getMultiSampleAntiAliasingOptions();

  RenderTarget* render_target = RenderTarget::downcast(request.target);
  if (render_target) {
    // We need to disable msaa in order to render to texture.
    view->setMultiSampleAntiAliasingOptions({.enabled = false});
  }

  SetupCamera(request.camera, viewport, camera_);

  std::vector<Renderable*> selected_renderables;
  for (auto& iter : renderables_) {
    iter->BindMaterialInstance(request);
    if (iter->GetMaterial().selected &&
        request.draw_mode == mjDRAW_MODE_DEFAULT) {
      selected_renderables.push_back(iter);
    }
  }

  for (int i = 0; i < reflection_mgr_->GetNumRenderables(); ++i) {
    Renderable* renderable =
        Renderable::downcast(reflection_mgr_->GetRenderable(i));
    const mat4 transform(renderable->GetTransform());
    SetupReflectionCamera(transform, camera_, reflect_camera_);

    // Hide reflective surface from its own reflection pass.
    std::uint8_t prev_layer_mask = renderable->SetLayerMask(kLayerMask_None);

    // Render the reflection to its render target.
    viewport.left = 0;
    viewport.bottom = 0;
    reflect_view_->setViewport(viewport);
    const RenderTarget* target = reflection_mgr_->GetRenderTarget(i);
    reflect_view_->setRenderTarget(target->GetFilamentRenderTarget());
    renderer->render(reflect_view_);
    reflect_view_->setRenderTarget(nullptr);

    // Unhide the reflective surface.
    renderable->SetLayerMask(prev_layer_mask);
  }

  filament::RenderTarget* filament_render_target =
      render_target ? render_target->GetFilamentRenderTarget() : nullptr;
  view->setRenderTarget(filament_render_target);
  renderer->render(view);
  view->setRenderTarget(nullptr);

  if (!selected_renderables.empty()) {
    if (!outliner_) {
      outliner_ =
          std::make_unique<Outliner>(object_mgr_, kLayerMask_Outline,
                                     float4{0.9f, 0.9f, 0.2f, 0.7f}, 3.5f);
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

void SceneView::SetColorGradingOptions(const ColorGradingOptions& opts) {
  auto tone_mapper = CreateToneMapper(opts.tone_mapper);
  auto color_grading = ToBuilder(color_grading_options_)
                           .toneMapper(tone_mapper.get())
                           .build(*GetEngine());
  main_view_->setColorGrading(color_grading);
  if (color_grading_) {
    GetEngine()->destroy(color_grading_);
  }
  color_grading_ = color_grading;
  color_grading_options_ = opts;
}

filament::View* SceneView::GetDefaultRenderView() { return main_view_; }

ColorGradingOptions SceneView::GetColorGradingOptions() const {
  return color_grading_options_;
}

void SceneView::Configure(const mjModel* model) {
  auto cg = color_grading_options_;
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
  SetColorGradingOptions(cg);

  auto ao = main_view_->getAmbientOcclusionOptions();
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
  main_view_->setAmbientOcclusionOptions(ao);

  auto msaa = main_view_->getMultiSampleAntiAliasingOptions();
  msaa.enabled = ReadElement(model, "filament.msaa.enabled", true);
  main_view_->setMultiSampleAntiAliasingOptions(msaa);

  auto shadow_type = main_view_->getShadowType();
  shadow_type = ReadElement(model, "filament.shadows.type", shadow_type);
  main_view_->setShadowType(shadow_type);

  auto fog_opts = main_view_->getFogOptions();
  fog_opts.enabled =
      ReadElement(model, "filament.fog.enabled", fog_opts.enabled);
  fog_opts.color = ReadElement(model, "filament.fog.color", fog_opts.color);
  fog_opts.distance =
      ReadElement(model, "filament.fog.distance", fog_opts.distance);
  fog_opts.density =
      ReadElement(model, "filament.fog.density", fog_opts.density);
  fog_opts.cutOffDistance = ReadElement(model, "filament.fog.cutOffDistance",
                                        fog_opts.cutOffDistance);
  fog_opts.maximumOpacity = ReadElement(model, "filament.fog.maximumOpacity",
                                        fog_opts.maximumOpacity);
  fog_opts.height = ReadElement(model, "filament.fog.height", fog_opts.height);
  fog_opts.heightFalloff =
      ReadElement(model, "filament.fog.heightFalloff", fog_opts.heightFalloff);
  fog_opts.inScatteringStart = ReadElement(
      model, "filament.fog.inScatteringStart", fog_opts.inScatteringStart);
  fog_opts.inScatteringSize = ReadElement(
      model, "filament.fog.inScatteringSize", fog_opts.inScatteringSize);
  main_view_->setFogOptions(fog_opts);

  auto bloom = main_view_->getBloomOptions();
  bloom.enabled = ReadElement(model, "filament.bloom.enabled", bloom.enabled);
  bloom.strength =
      ReadElement(model, "filament.bloom.strength", bloom.strength);
  bloom.dirtStrength =
      ReadElement(model, "filament.bloom.dirt_strength", bloom.dirtStrength);
  bloom.quality = ReadElement(model, "filament.bloom.quality", bloom.quality);
  bloom.resolution =
      ReadElement(model, "filament.bloom.resolution", bloom.resolution);
  bloom.levels = ReadElement(model, "filament.bloom.levels", bloom.levels);
  main_view_->setBloomOptions(bloom);
}
}  // namespace mujoco
