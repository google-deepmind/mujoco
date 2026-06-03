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

#include <memory>
#include <span>
#include <unordered_set>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/reflection_manager.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Creates and owns the filament Scene and View (and Camera) classes.
//
// The filament Scene is populated with the objects (e.g. lights, renderables,
// skybox, etc.). It manages multiple views to support a variety of draw modes
// (e.g. normal, depth, segmentation, etc.) as well as reflective surfaces.
class SceneView : public mjrScene {
 public:
  SceneView(filament::Engine* engine, const mjrSceneParams& params);
  ~SceneView();

  SceneView(const SceneView&) = delete;
  SceneView& operator=(const SceneView&) = delete;

  // Adds/removes entities from the scene.
  void AddToScene(Light* light);
  void RemoveFromScene(Light* light);
  void AddToScene(Renderable* renderable);
  void RemoveFromScene(Renderable* renderable);
  void SetSkybox(const Texture* skybox_texture);

  // Performs necessary preparations in order to render the given requests.
  // Assumes that the Render() function will be called the same number of times
  // and in the same order with the given requests.
  void PrepareToRender(std::span<const mjrRenderRequest*> requests);

  // Fulfills the given render request using the renderer.
  void Render(filament::Renderer* renderer, const mjrRenderRequest& request);

  // Returns the filament Engine managing the scene.
  filament::Engine* GetEngine() const { return engine_; }

  // Returns the underlying filament View that is used for normal rendering.
  // Callers can update rendering settings (e.g. post processing) directly.
  filament::View* GetDefaultRenderView();

  // Helpers for managing the color grading options for the default render view.
  ColorGradingOptions GetColorGradingOptions() const;
  void SetColorGradingOptions(const ColorGradingOptions& opts);

  // Reads filament-specific settings from the mjModel and configures the
  // scene view accordingly.
  void Configure(const mjModel* model);

  static SceneView* downcast(mjrScene* scene) {
    return static_cast<SceneView*>(scene);
  }
  static const SceneView* downcast(const mjrScene* scene) {
    return static_cast<const SceneView*>(scene);
  }

 private:
  filament::Engine* engine_ = nullptr;
  filament::Scene* scene_ = nullptr;
  filament::Camera* camera_ = nullptr;
  filament::ColorGrading* color_grading_ = nullptr;
  ColorGradingOptions color_grading_options_;
  filament::View* main_view_;
  filament::View* depth_segment_view_;

  // Scene objects.
  std::unordered_set<Light*> lights_;
  std::unordered_set<Renderable*> renderables_;
  filament::Skybox* skybox_ = nullptr;

  // Custom view and camera for reflective surfaces.
  filament::View* reflect_view_ = nullptr;
  filament::Camera* reflect_camera_ = nullptr;
  std::unique_ptr<ReflectionManager> reflection_mgr_;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_SCENE_VIEW_H_
