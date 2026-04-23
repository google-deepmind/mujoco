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

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include <filament/ColorGrading.h>
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
#include "experimental/filament/filament/draw_mode.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/math_util.h"
#include "experimental/filament/filament/render_target.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/texture.h"

namespace mujoco {

using filament::math::float3;
using filament::math::float4;
using filament::math::mat4;

static constexpr int kNormalIndex = static_cast<int>(DrawMode::Color);
static constexpr int kDepthIndex = static_cast<int>(DrawMode::Depth);
static constexpr int kSegmentIndex = static_cast<int>(DrawMode::Segmentation);

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

static void SetupCamera(const mjvGLCamera& cam,
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
  const mat4 oblique = CalculateObliqueProjection(src_projection, oblique_plane);
  reflection_camera->setCustomProjection(oblique, near, far);
}

SceneView::SceneView(filament::Engine* engine) : engine_(engine) {
  scene_ = engine->createScene();
  camera_ = engine->createCamera(utils::EntityManager::get().create());
  reflect_camera_ = engine->createCamera(utils::EntityManager::get().create());

  for (auto& view : views_) {
    view = engine->createView();
    view->setScene(scene_);
    view->setCamera(camera_);
    view->setVisibleLayers(0xff, mjCAT_ALL);
  }

  reflect_view_ = engine->createView();
  reflect_view_->setScene(scene_);
  reflect_view_->setCamera(reflect_camera_);
  reflect_view_->setShadowingEnabled(false);
  reflect_view_->setPostProcessingEnabled(false);
  reflect_view_->setVisibleLayers(0xff, mjCAT_DYNAMIC | mjCAT_STATIC);

  // Disable post processing for the depth and segmentation views to preserve
  // the values.
  views_[kDepthIndex]->setPostProcessingEnabled(false);
  views_[kSegmentIndex]->setPostProcessingEnabled(false);

  // Rotate the fog to align with mujoco's +Z up space.
  auto fog = views_[kNormalIndex]->getFogEntity();
  auto& tm = engine->getTransformManager();
  tm.create(fog);
  tm.setTransform(tm.getInstance(fog),
                  mat4::rotation(filament::math::f::PI / 2, float3{-1, 0, 0}));
}

SceneView::~SceneView() {
  for (auto& light : lights_) {
    light->RemoveFromScene(scene_);
  }
  for (auto& renderable : renderables_) {
    renderable->RemoveFromScene(scene_);
  }
  lights_.clear();
  renderables_.clear();
  reflect_targets_.clear();
  engine_->destroyCameraComponent(reflect_camera_->getEntity());
  engine_->destroy(reflect_view_);
  engine_->destroyCameraComponent(camera_->getEntity());
  if (color_grading_) {
    engine_->destroy(color_grading_);
  }
  engine_->destroy(scene_);
  for (auto& view : views_) {
    engine_->destroy(view);
  }
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
    if (renderable->GetMaterialParams().reflective) {
      AddReflectiveRenderable(renderable);
    }
  }
}

void SceneView::RemoveFromScene(Renderable* renderable) {
  if (renderables_.erase(renderable)) {
    auto it = std::find(reflectives_.begin(), reflectives_.end(), renderable);
    if (it != reflectives_.end()) {
      reflectives_.erase(it);
    }
    renderable->RemoveFromScene(scene_);
  }
}

void SceneView::AddToScene(filament::Skybox* skybox) {
  skybox_ = skybox;
  scene_->setSkybox(skybox);
}

void SceneView::RemoveFromScene(filament::Skybox* skybox) {
  if (skybox_ == skybox) {
    skybox_ = nullptr;
    scene_->setSkybox(nullptr);
  }
}

void SceneView::Render(filament::Renderer* renderer,
                       const RenderRequest& request) {
  filament::Viewport viewport(request.viewport.left, request.viewport.bottom,
                              request.viewport.width, request.viewport.height);
  for (auto& view : views_) {
    view->setViewport(viewport);
  }
  reflect_view_->setViewport(viewport);

  SetupCamera(request.camera, viewport, camera_);

  for (auto& iter : renderables_) {
    iter->SetDrawMode(request.draw_mode);
  }

  filament::View* view = views_[static_cast<int>(request.draw_mode)];
  filament::MultiSampleAntiAliasingOptions options =
      view->getMultiSampleAntiAliasingOptions();

  filament::RenderTarget* render_target =
      request.target ? request.target->GetFilamentRenderTarget() : nullptr;
  if (render_target) {
    // We need to disable msaa in order to render to texture.
    view->setMultiSampleAntiAliasingOptions({.enabled = false});
  }

  // Render reflection passes.
  if (request.draw_mode == DrawMode::Color && reflections_enabled_) {
    for (size_t i = 0; i < reflectives_.size(); ++i) {
      Renderable* renderable = reflectives_[i];

      // We assume the 0th entity is the reflective entity.
      mat4 transform(renderable->GetTransform());
      SetupReflectionCamera(transform, camera_, reflect_camera_);

      // Hide reflective surface from its own reflection pass.
      std::uint8_t previous_layer_mask = renderable->SetLayerMask(0x00);

      // Render the reflection to its render target.
      reflect_view_->setRenderTarget(
          reflect_targets_[i]->GetFilamentRenderTarget());
      renderer->render(reflect_view_);

      // Unhide the reflective surface.
      renderable->SetLayerMask(previous_layer_mask);
    }
  }

  view->setRenderTarget(render_target);
  renderer->render(view);
  view->setRenderTarget(nullptr);

  if (request.target) {
    view->setMultiSampleAntiAliasingOptions(options);
  }
}

void SceneView::AddReflectiveRenderable(Renderable* renderable) {
  const int index = reflectives_.size();
  reflectives_.push_back(renderable);

  // Ensure we have the same number of render targets as we do reflective
  // renderables.
  while (reflect_targets_.size() < reflectives_.size()) {
    mjrRenderTargetConfig config;
    mjr_defaultRenderTargetConfig(&config);

    config.color_format = mjPIXEL_FORMAT_RGBA8;
    config.depth_format = mjPIXEL_FORMAT_DEPTH32F;
    reflect_targets_.push_back(std::make_unique<RenderTarget>(engine_, config));
  }

  // Prepare a render target for the reflective renderable.
  auto viewport = reflect_view_->getViewport();
  auto& target = reflect_targets_[index];
  target->Prepare(viewport.width, viewport.height);

  if (reflections_enabled_) {
    mjrMaterialTextures textures = renderable->GetMaterialTextures();
    textures.reflection = target->GetColorTexture();
    renderable->UpdateMaterial(renderable->GetMaterialParams(), textures);
  }
}

void SceneView::SetColorGradingOptions(const ColorGradingOptions& opts) {
  auto tone_mapper = CreateToneMapper(opts.tone_mapper);
  auto color_grading = ToBuilder(color_grading_options_)
                           .toneMapper(tone_mapper.get())
                           .build(*engine_);
  views_[kNormalIndex]->setColorGrading(color_grading);
  if (color_grading_) {
    engine_->destroy(color_grading_);
  }
  color_grading_ = color_grading;
  color_grading_options_ = opts;
}

void SceneView::EnableShadows() {
  views_[kNormalIndex]->setShadowingEnabled(true);
}

void SceneView::DisableShadows() {
  views_[kNormalIndex]->setShadowingEnabled(false);
}

void SceneView::EnableReflections() {
  reflections_enabled_ = true;

  for (int i = 0; i < reflectives_.size(); ++i) {
    Renderable* renderable = reflectives_[i];
    mjrMaterialTextures textures = renderable->GetMaterialTextures();
    textures.reflection = reflect_targets_[i]->GetColorTexture();
    renderable->UpdateMaterial(renderable->GetMaterialParams(), textures);
  }
}

void SceneView::DisableReflections() {
  reflections_enabled_ = false;
  for (Renderable* renderable : reflectives_) {
    mjrMaterialTextures textures = renderable->GetMaterialTextures();
    textures.reflection = nullptr;
    renderable->UpdateMaterial(renderable->GetMaterialParams(), textures);
  }

}

void SceneView::EnablePostProcessing() {
  views_[kNormalIndex]->setPostProcessingEnabled(true);
}

void SceneView::DisablePostProcessing() {
  views_[kNormalIndex]->setPostProcessingEnabled(false);
}

filament::View* SceneView::GetDefaultRenderView() {
  return views_[kNormalIndex];
}

ColorGradingOptions SceneView::GetColorGradingOptions() const {
  return color_grading_options_;
}

}  // namespace mujoco
