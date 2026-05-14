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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MODEL_OBJECTS_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MODEL_OBJECTS_H_

#include <unordered_map>

#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

namespace mujoco {

// Creates and owns various filament objects based on the mjModel.
class ModelObjects {
 public:
  ModelObjects(const mjModel* model, mjrfContext* ctx);

  void UploadMesh(const mjModel* model, int id);

  void UploadTexture(const mjModel* model, int id);

  void UploadHeightField(const mjModel* model, int id);

  void CreateSkinFlexMesh(const mjvScene* scene, const mjvGeom& geom);

  // Returns the cached instance of a filament object created from the mjModel.
  const mjrMesh* GetMesh(int data_id) const;
  const mjrMesh* GetHeightField(int hfield_id) const;
  const mjrMesh* GetFlexSkinMesh(int geom_id) const;
  const mjrTexture* GetTexture(int tex_id) const;
  const mjrTexture* GetSkyboxTexture() const;

  float GetSpecularMultiplier() const { return specular_multiplier_; }
  float GetShininessMultiplier() const { return shininess_multiplier_; }
  float GetEmissiveMultiplier() const { return emissive_multiplier_; }

  const mjModel* GetModel() const { return model_; }

  ModelObjects(const ModelObjects&) = delete;
  ModelObjects& operator=(const ModelObjects&) = delete;

 private:
  const mjModel* model_ = nullptr;
  mjrfContext* ctx_ = nullptr;
  std::unordered_map<int, UniquePtr<mjrMesh>> meshes_;
  std::unordered_map<int, UniquePtr<mjrMesh>> convex_hulls_;
  std::unordered_map<int, UniquePtr<mjrMesh>> height_fields_;
  std::unordered_map<int, UniquePtr<mjrMesh>> dynamic_meshes_;
  std::unordered_map<int, UniquePtr<mjrTexture>> textures_;
  float specular_multiplier_ = 0.2f;
  float shininess_multiplier_ = 0.1f;
  float emissive_multiplier_ = 0.3f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_MODEL_OBJECTS_H_
