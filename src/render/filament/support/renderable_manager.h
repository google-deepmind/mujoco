// Copyright 2026 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_RENDERABLE_MANAGER_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_RENDERABLE_MANAGER_H_

#include <optional>
#include <vector>
#include <math/vec4.h>
#include <mujoco/mjdata.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Manages Renderable entities for an mjrfScene.
class RenderableManager {
 public:
  // Populates the mjrScene with the renderables from the model.
  RenderableManager(mjrfScene* scene, ModelObjects* model_objects);
  ~RenderableManager();

  // Updates the state of the renderables in the scene.
  void Update(const mjData* data);

  // Returns the renderable corresponding to the given model object.
  mjrfRenderable* GetRenderable(mjtObj obj_type, int obj_index);

  // Returns the default material (as defined in the mjModel) for the given
  // object. Useful if you want to "reset" the material of a renderable back
  // to its default.
  mjrfMaterial GetDefaultMaterial(mjtObj obj_type, int obj_index);

  // Marks the given object as "selected", unmarking any previously selected
  // object.
  void SelectObject(mjtObj obj_type, int obj_index);

  // Sets the visibility of all renderables of the given type. If `group` is
  // specified, only applies to renderables in that group.
  void SetVisibility(mjtObj obj_type, bool visible,
                     std::optional<int> group = std::nullopt);

  // Applies the visualization options to the renderables in the scene.
  void Apply(const mjvOption& vopts);

  RenderableManager(const RenderableManager&) = delete;
  RenderableManager& operator=(const RenderableManager&) = delete;

 private:
  void AddGeomGeoms();
  void AddSiteGeoms();
  void AddFlexGeoms();
  void AddSkinGeoms();
  void AddSliderCrankGeoms();

  void UpdateSpatialTendons(const mjData* data, int tendon_id);
  void AppendSegmentToTendon(int tendon_id);
  void RemoveSegmentFromTendon(int tendon_id);
  void UpdateSliderCranks(const mjData* data, int actuator_id, int index);

  int GetSegmentationId(mjtObj obj_type, int obj_index);

  mjrfScene* scene_;
  ModelObjects* model_objects_;

  mjvOption vopts_;

  std::vector<UniquePtr<mjrfRenderable>> geoms_;
  std::vector<UniquePtr<mjrfRenderable>> sites_;
  std::vector<UniquePtr<mjrfRenderable>> flexes_;
  std::vector<UniquePtr<mjrfRenderable>> skins_;
  std::vector<UniquePtr<mjrfRenderable>> sliders_;
  std::vector<UniquePtr<mjrfRenderable>> cranks_;
  std::vector<std::vector<UniquePtr<mjrfRenderable>>> tendons_;

  std::vector<UniquePtr<mjrfMesh>> flex_meshes_;
  std::vector<UniquePtr<mjrfMesh>> skin_meshes_;
  std::vector<filament::math::float4> point_cache_;

  mjtObj selected_obj_type_ = mjOBJ_UNKNOWN;
  int selected_obj_index_ = -1;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_RENDERABLE_MANAGER_H_
