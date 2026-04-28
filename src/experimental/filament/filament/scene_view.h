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
#include <unordered_set>
#include <vector>

#include <filament/Camera.h>
#include <filament/ColorGrading.h>
#include <filament/Engine.h>
#include <filament/Scene.h>
#include <filament/View.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/color_grading_options.h"
#include "experimental/filament/filament/light.h"
#include "experimental/filament/filament/renderable.h"
#include "experimental/filament/filament/render_target.h"
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
  SceneView(filament::Engine* engine);
  ~SceneView();

  SceneView(const SceneView&) = delete;
  SceneView& operator=(const SceneView&) = delete;

  // Adds/removes entities from the scene.
  void AddToScene(Light* light);
  void RemoveFromScene(Light* light);
  void AddToScene(Renderable* renderable);
  void RemoveFromScene(Renderable* renderable);
  void SetSkybox(const Texture* skybox_texture);

  // Parameters for rendering the scene.
  struct RenderRequest {
    // The draw mode (e.g. normal, depth, segmentation) to render.
    mjrDrawMode draw_mode = mjDRAW_MODE_COLOR;
    // The target viewport for the rendered image.
    mjrRect viewport;
    // The camera from which to render the scene.
    mjvGLCamera camera;
    // An optional render target into which the scene will be rendered.
    RenderTarget* target = nullptr;
  };

  // Renders the scene.
  void Render(filament::Renderer* renderer, const RenderRequest& request);

  // Returns the filament Engine managing the scene.
  filament::Engine* GetEngine() const { return engine_; }

  // Enables/disables shadows for the default render view.
  void EnableShadows();
  void DisableShadows();

  // Enables/disables reflections for the default render view.
  void EnableReflections();
  void DisableReflections();

  // Enables/disables post processing for the default render view.
  void EnablePostProcessing();
  void DisablePostProcessing();

  // Returns the underlying filament View that is used for normal rendering.
  // Callers can update rendering settings (e.g. post processing) directly.
  filament::View* GetDefaultRenderView();

  // Helpers for managing the color grading options for the default render view.
  ColorGradingOptions GetColorGradingOptions() const;
  void SetColorGradingOptions(const ColorGradingOptions& opts);

  static SceneView* downcast(mjrScene* scene) {
    return static_cast<SceneView*>(scene);
  }
  static const SceneView* downcast(const mjrScene* scene) {
    return static_cast<const SceneView*>(scene);
  }

 private:
  // Marks a renderable as reflective. Reflective renderables have to be
  // rendered in their own passes to create the reflective texture.
  void AddReflectiveRenderable(Renderable* renderable);

  filament::Engine* engine_ = nullptr;
  filament::Scene* scene_ = nullptr;
  filament::Camera* camera_ = nullptr;
  filament::ColorGrading* color_grading_ = nullptr;
  ColorGradingOptions color_grading_options_;
  std::array<filament::View*, mjNUM_DRAW_MODES> views_;

  // Scene objects.
  std::unordered_set<Light*> lights_;
  std::unordered_set<Renderable*> renderables_;
  filament::Skybox* skybox_ = nullptr;

  // Custom view and camera for reflective surfaces.
  filament::View* reflect_view_ = nullptr;
  filament::Camera* reflect_camera_ = nullptr;

  // The list of reflective renderables and their corresponding render targets.
  bool reflections_enabled_ = true;
  std::vector<Renderable*> reflectives_;
  std::vector<std::unique_ptr<RenderTarget>> reflect_targets_;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_SCENE_VIEW_H_
