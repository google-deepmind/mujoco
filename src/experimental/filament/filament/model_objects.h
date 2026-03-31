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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_OBJECTS_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_OBJECTS_H_

#include <array>
#include <unordered_map>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Skybox.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>
#include "experimental/filament/filament/buffer_util.h"

namespace mujoco {

// Creates and owns various filament objects based on the data in a mjrContext.
class ModelObjects {
 public:
  ModelObjects(const mjModel* model, filament::Engine* engine);
  ~ModelObjects();

  enum ShapeType {
    kLine,
    kLineBox,
    kPlane,
    kTriangle,
    kBox,
    kSphere,
    kCone,
    kDisk,
    kDome,
    kTube,
    kNumShapes,
  };

  void UploadMesh(const mjModel* model, int id);

  void UploadTexture(const mjModel* model, int id);

  void UploadHeightField(const mjModel* model, int id);

  // Returns the filament engine used by the ModelObjects to create filament
  // objects.
  filament::Engine* GetEngine() const { return engine_; }

  // Returns the cached instance of a filament object created from the mjModel.
  const FilamentBuffers* GetShapeBuffer(ShapeType shape) const;
  const FilamentBuffers* GetMeshBuffer(int data_id) const;
  const FilamentBuffers* GetHeightFieldBuffer(int hfield_id) const;
  const filament::Texture* GetTexture(int tex_id) const;
  const filament::Texture* GetTexture(int mat_id, int role) const;

  filament::Skybox* CreateSkybox();
  filament::IndirectLight* CreateIndirectLight(int tex_id, float intensity);

  float GetSpecularMultiplier() const { return specular_multiplier_; }
  float GetShininessMultiplier() const { return shininess_multiplier_; }
  float GetEmissiveMultiplier() const { return emissive_multiplier_; }

  const mjModel* GetModel() const { return model_; }

  ModelObjects(const ModelObjects&) = delete;
  ModelObjects& operator=(const ModelObjects&) = delete;

 private:
  using SphericalHarmonics = filament::math::float3[9];

  const mjModel* model_ = nullptr;
  filament::Engine* engine_ = nullptr;
  std::vector<filament::Skybox*> skyboxes_;
  std::vector<filament::IndirectLight*> indirect_lights_;
  std::array<FilamentBuffers, kNumShapes> shapes_;
  std::unordered_map<int, FilamentBuffers> meshes_;
  std::unordered_map<int, FilamentBuffers> convex_hulls_;
  std::unordered_map<int, FilamentBuffers> height_fields_;
  std::unordered_map<int, filament::Texture*> textures_;
  std::unordered_map<int, SphericalHarmonics> spherical_harmonics_;
  float specular_multiplier_ = 0.2f;
  float shininess_multiplier_ = 0.1f;
  float emissive_multiplier_ = 0.3f;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MODEL_OBJECTS_H_
