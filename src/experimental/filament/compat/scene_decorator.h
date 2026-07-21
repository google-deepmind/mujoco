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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_DECORATOR_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_DECORATOR_H_

#include <functional>
#include <span>
#include <vector>

#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Adds decorative renderables to a scene.
//
// Internally uses mjvScene to generate decorative mjvGeoms from which the
// renderables are then created.
class SceneDecorator {
 public:
  SceneDecorator(mjrfScene* scene, ModelObjects* model_objects,
                 int num_geoms = 2000);
  ~SceneDecorator();

  // Function for drawing text at a given position in clip space.
  using DrawTextAtFn = std::function<void(const char*, float, float, float)>;

  // Creates and adds decorative renderables to the scene.
  void Update(mjData* data, const mjvOption* vis_option,
              const mjvPerturb* perturb, mjvCamera* camera,
              const mjrRect& viewport, DrawTextAtFn draw_text_at_fn = nullptr,
              std::span<const mjvGeom> extra_geoms = {});

  SceneDecorator(const SceneDecorator&) = delete;
  SceneDecorator& operator=(const SceneDecorator&) = delete;

 private:
  mjrfScene* scene_;
  ModelObjects* model_objects_;
  mjvScene mjv_scene_;
  std::vector<UniquePtr<mjrfMesh>> meshes_;
  std::vector<UniquePtr<mjrfRenderable>> decorations_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_DECORATOR_H_
