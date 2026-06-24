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

#ifndef MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_MODEL_OBJECTS_H_
#define MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_MODEL_OBJECTS_H_

#include <unordered_map>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrfilament.h>
#include <mujoco/mujoco.h>
#include "render/filament/mjrfilament_cpp.h"

namespace mujoco {

// Creates and owns meshes and textures read from an mjModel.
class ModelObjects {
 public:
  ModelObjects(const mjModel* model, mjrfContext* ctx);

  // Uploads a new mesh from the model with the given id.
  void UploadMesh(const mjModel* model, int id);

  // Uploads a new texture from the model with the given id.
  void UploadTexture(const mjModel* model, int id);

  // Uploads a new height field from the model with the given id.
  void UploadHeightField(const mjModel* model, int id);

  // Returns the mjModel mesh with the given data_id. The data_id is the
  // mesh_id * 2 of the mesh in the mjModel.
  const mjrfMesh* GetMesh(int data_id) const;

  // Returns the mjModel convex hulll mesh with the given data_id. The data_id
  // is the (mesh_id * 2) + 1 of the mesh in the mjModel.
  const mjrfMesh* GetConvexHull(int data_id) const;

  // Returns the mjModel height field mesh with the given id.
  const mjrfMesh* GetHeightField(int hfield_id) const;

  // Returns the mjModel texture with the given id.
  const mjrfTexture* GetTexture(int tex_id) const;

  // Returns the skybox texture in the mjModel.
  const mjrfTexture* GetSkyboxTexture() const;

  // Returns the mjModel from which the Model Objects are created.
  const mjModel* GetModel() const { return model_; }

  // Returns the multipliers used for mapping legacy material properties to
  // filament material properties.
  float GetSpecularMultiplier() const { return specular_multiplier_; }
  float GetShininessMultiplier() const { return shininess_multiplier_; }
  float GetEmissiveMultiplier() const { return emissive_multiplier_; }

  ModelObjects(const ModelObjects&) = delete;
  ModelObjects& operator=(const ModelObjects&) = delete;

 private:
  const mjModel* model_ = nullptr;
  mjrfContext* ctx_ = nullptr;
  std::unordered_map<int, UniquePtr<mjrfMesh>> meshes_;
  std::unordered_map<int, UniquePtr<mjrfMesh>> convex_hulls_;
  std::unordered_map<int, UniquePtr<mjrfMesh>> height_fields_;
  std::unordered_map<int, UniquePtr<mjrfTexture>> textures_;
  float specular_multiplier_ = 0.2f;
  float shininess_multiplier_ = 0.1f;
  float emissive_multiplier_ = 0.3f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_RENDER_FILAMENT_SUPPORT_MODEL_OBJECTS_H_
