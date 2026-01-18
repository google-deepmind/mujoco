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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAWABLE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAWABLE_H_

#include <cstdint>

#include <filament/Engine.h>
#include <filament/Scene.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjvisualize.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/filament/material.h"
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/filament/renderables.h"

namespace mujoco {

// Manages the filament Entities and MaterialInstances for a single mjvGeom.
class Drawable {
 public:
  Drawable(ObjectManager* object_mgr, const mjvGeom& geom);
  ~Drawable() noexcept = default;

  Drawable(const Drawable&) = delete;
  Drawable& operator=(const Drawable&) = delete;

  // Adds the Drawable to the given filament Scene. Note that a Drawable can
  // only be assigned to a single Scene at any given time.
  void AddToScene(filament::Scene* scene);

  // Removes the Drawable from the given filament Scene.
  void RemoveFromScene(filament::Scene* scene);

  // Updates the drawable to reflect the current state (e.g. geometry,
  // transform, material, etc.) of the geom.
  void Update(const mjModel* model, const mjvScene* scene, const mjvGeom& geom);

  // Swaps the MaterialInstance that will be used to render the Drawable (e.g.
  // normal, depth, segmentation, etc.). This must be called before the filament
  // beginFrame/endFrame.
  void SetDrawMode(Material::DrawMode mode);


 private:
  void AddMesh(int data_id);
  void AddHeightField(int hfield_id);
  void AddShape(ObjectManager::ShapeType shape_type);

  // Updates the transform of the drawable for rendering.
  void SetTransform(const mjvGeom& geom);

  // Updates the material parameters of the drawable for rendering.
  void UpdateMaterial(const mjvGeom& geom, bool use_segid_color);

  Material material_;
  Renderables renderables_;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAWABLE_H_
