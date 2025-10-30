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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_OBJECT_MANAGER_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_OBJECT_MANAGER_H_

#include <array>
#include <cstdint>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Skybox.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>
#include "experimental/filament/filament/buffer_util.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Creates and owns various filament objects based on the data in a mjrContext.
class ObjectManager {
 public:
  ObjectManager(const mjModel* model, filament::Engine* engine,
                const mjrFilamentConfig* config);
  ~ObjectManager();

  enum MaterialType {
    kPbr,
    kPbrPacked,
    kPhong2d,
    kPhong2dFade,
    kPhong2dUv,
    kPhong2dUvFade,
    kPhongColor,
    kPhongColorFade,
    kPhongCube,
    kPhongCubeFade,
    kUnlitSegmentation,
    kUnlitDepth,
    kUnlitLine,
    kUnlitUi,
    kNumMaterials,
  };

  enum ShapeType {
    kLine,
    kLineBox,
    kPlane,
    kBox,
    kSphere,
    kCone,
    kDisk,
    kDome,
    kTube,
    kNumShapes,
  };

  using SphericalHarmonics = filament::math::float3[9];

  void UploadMesh(const mjModel* model, int id);

  void UploadTexture(const mjModel* model, int id);

  void UploadHeightField(const mjModel* model, int id);

  void UploadFont(const uint8_t* pixels, int width, int height, int id);

  // Returns the filament engine used by the ObjectManager to create filament
  // objects.
  filament::Engine* GetEngine() const { return engine_; }

  filament::Material* GetMaterial(MaterialType type) const;

  // Returns the cached instance of a filament object created from the mjModel.
  const FilamentBuffers* GetMeshBuffer(int data_id) const;
  const FilamentBuffers* GetShapeBuffer(ShapeType shape) const;
  const FilamentBuffers* GetHeightFieldBuffer(int hfield_id) const;
  const filament::Texture* GetFont(int font_id) const;
  const filament::Texture* GetTexture(int tex_id) const;
  const filament::Texture* GetTexture(int mat_id, int role) const;
  const filament::Texture* GetTextureWithFallback(int mat_id, int role) const;
  const filament::Texture* GetFallbackTexture(int role) const;
  filament::IndirectLight* GetFallbackIndirectLight();

  // Creates and returns a new instance of a filament object. The objects are
  // owned by the ObjectManager and will be deleted in the destructor.
  filament::Skybox* CreateSkybox();
  filament::IndirectLight* CreateIndirectLight(
      filament::Texture* texture, SphericalHarmonics* spherical_harmonics,
      float intensity);
  filament::IndirectLight* CreateIndirectLight(int tex_id, float intensity);
  filament::IndirectLight* LoadFallbackIndirectLight(std::string_view filename,
                                                     float intensity);

  const mjModel* GetModel() const { return model_; }

  ObjectManager(const ObjectManager&) = delete;
  ObjectManager& operator=(const ObjectManager&) = delete;

 private:
  const mjModel* model_ = nullptr;
  filament::Engine* engine_ = nullptr;
  const mjrFilamentConfig* config_;

  std::array<FilamentBuffers, kNumShapes> shapes_;
  std::array<filament::Material*, kNumMaterials> materials_;
  std::vector<filament::Skybox*> skyboxes_;
  std::vector<filament::IndirectLight*> indirect_lights_;
  std::unordered_map<int, FilamentBuffers> meshes_;
  std::unordered_map<int, FilamentBuffers> convex_hulls_;
  std::unordered_map<int, FilamentBuffers> height_fields_;
  std::unordered_map<int, filament::Texture*> fonts_;
  std::unordered_map<int, filament::Texture*> textures_;
  std::unordered_map<int, SphericalHarmonics> spherical_harmonics_;
  std::unordered_map<int, filament::Texture*> fallback_textures_;
  filament::Texture* fallback_white_ = nullptr;
  filament::Texture* fallback_black_ = nullptr;
  filament::Texture* fallback_normal_ = nullptr;
  filament::Texture* fallback_orm_ = nullptr;
  filament::IndirectLight* fallback_indirect_light_ = nullptr;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_OBJECT_MANAGER_H_
