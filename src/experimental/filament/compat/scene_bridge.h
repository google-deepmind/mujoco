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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_

#include <functional>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include <math/mat4.h>
#include <math/vec3.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

namespace mujoco {

// Manages all mjModel data and updates a SceneView using an mjvScene.
class SceneBridge {
 public:
  SceneBridge(mjrfContext* ctx, const mjModel* model);
  ~SceneBridge();

  // Updates the Entities in the filament Scene to match the current mjvScene
  // state.
  void Update(const mjrRect& viewport, const mjvScene* scene);

  // Creates the filament objects from the mjModel.
  void UploadMesh(const mjModel* model, int id);
  void UploadTexture(const mjModel* model, int id);
  void UploadHeightField(const mjModel* model, int id);

  using DrawTextAtFn = std::function<void(const char*, float, float, float)>;
  void SetDrawTextFunction(DrawTextAtFn fn);

  // Returns the managed scene.
  mjrScene* GetScene() const;
  mjrCamera GetCamera() const;
  mjrDrawMode GetDrawMode() const;

  SceneBridge(const SceneBridge&) = delete;
  SceneBridge& operator=(const SceneBridge&) = delete;

 private:
  void PrepareLights();

  // Converts a point in world space to clip space, eg. in the range [-1,-1, 0]
  // to [1, 1, 1]. Returns std::nullopt if the point is behind the camera.
  std::optional<filament::math::float3> ClipFromWorld(
      const filament::math::float3& pos) const;

  mjrfContext* ctx_ = nullptr;
  std::unique_ptr<ModelObjects> model_objects_;
  mjrCamera camera_;
  mjrDrawMode draw_mode_ = mjDRAW_MODE_COLOR;
  DrawTextAtFn draw_text_callback_;
  UniquePtr<mjrScene> scene_{nullptr, nullptr};
  UniquePtr<mjrLight> fallback_ibl_{nullptr, nullptr};
  UniquePtr<mjrTexture> fallback_ibl_texture_{nullptr, nullptr};
  std::vector<UniquePtr<mjrLight>> lights_;
  std::vector<UniquePtr<mjrRenderable>> renderables_;
  filament::math::mat4 clip_from_world_;
  int default_shadow_map_size_ = 2048;
  float default_vsm_blur_width_ = 0.0f;
  float fallback_head_light_intensity_ = 0.f;
  float fallback_scene_light_intensity_ = 80'000.f;
  float fallback_environment_light_intensity_ = 5'000.f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_BRIDGE_H_
