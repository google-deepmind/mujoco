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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_SCENE_VIEW_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_SCENE_VIEW_H_

#include <array>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <math/mat4.h>
#include <math/vec3.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/drawable.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/object_manager.h"

namespace mujoco {

// Creates and owns filament Scene and View classes given a mjvScene.
//
// The filament Scene is populated with the objects (e.g. lights, geoms,
// cameras, etc.) defined by the mjvScene. Multiple Views are created to allow
// different rendering modes (e.g. normal, depth, segmentation, etc.)
class SceneView {
 public:
  SceneView(filament::Engine* engine, ObjectManager* object_mgr);
  ~SceneView();

  // Updates all views to render into the given viewport.
  void SetViewport(mjrRect viewport);

  // Updates the color grading options for the main render view.
  void SetColorGradingOptions(const ColorGradingOptions& opts);

  // Updates the environment light using the KTX image at the given path.
  void SetEnvironmentLight(std::string_view filename, float intensity);

  // Updates the environment light to the fallback light
  void SetFallbackEnvironmentLight(float intensity);

  // Updates the Entities in the filament Scene to match the current mjvScene
  // state.
  void UpdateScene(const mjrContext* context, const mjvScene* scene);

  using DrawMode = Material::DrawMode;

  // Prepares and returns the filament View for the given draw mode.
  filament::View* PrepareRenderView(DrawMode mode);

  // Accessors.
  filament::Engine* GetEngine() const;
  filament::View* GetDefaultRenderView();
  ColorGradingOptions GetColorGradingOptions() const;

  SceneView(const SceneView&) = delete;
  SceneView& operator=(const SceneView&) = delete;

 private:
  void UpdateCamera(const mjvGLCamera* cameras);

  void PrepareLights();

  // Converts a point in world space to clip space, eg. in the range [-1,-1, 0]
  // to [1, 1, 1]. Returns std::nullopt if the point is behind the camera.
  std::optional<filament::math::float3> ClipFromWorld(
      const filament::math::float3& pos) const;

  ObjectManager* object_mgr_ = nullptr;
  filament::Engine* engine_ = nullptr;
  filament::Scene* scene_ = nullptr;
  filament::Camera* camera_ = nullptr;
  filament::ColorGrading* color_grading_ = nullptr;
  std::vector<std::unique_ptr<Light>> lights_;
  std::vector<std::unique_ptr<Drawable>> drawables_;
  std::array<filament::View*, DrawMode::kNumDrawModes> views_;
  filament::math::mat4 clip_from_world_;
  ColorGradingOptions color_grading_options_;
  DrawMode active_mode_ = DrawMode::kNumDrawModes;
  float aspect_ratio_ = 1.0f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_SCENE_VIEW_H_
