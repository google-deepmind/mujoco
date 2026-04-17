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
#include <cstddef>
#include <memory>
#include <span>
#include <string_view>

#include <filament/Engine.h>
#include <filament/IndirectLight.h>
#include <filament/Skybox.h>
#include <filament/Texture.h>
#include <mujoco/mujoco.h>

namespace mujoco {

// Creates and owns various filament objects based on the data in a mjrContext.
class ObjectManager {
 public:
  class Asset {
   public:
    ~Asset();

    std::span<const std::byte> GetBytes() const;

    Asset(const Asset&) = delete;
    Asset& operator=(const Asset&) = delete;
   private:
    friend class ObjectManager;
    explicit Asset(std::string_view filename);

    std::size_t size = 0;
    void* payload = nullptr;
    mjResource* resource = nullptr;
  };

  ObjectManager(filament::Engine* engine);
  ~ObjectManager();

  enum MaterialType {
    kPbr,
    kPbrPacked,
    kPhong2d,
    kPhong2dFade,
    kPhong2dReflect,
    kPhong2dUv,
    kPhong2dUvFade,
    kPhong2dUvReflect,
    kPhongColor,
    kPhongColorFade,
    kPhongColorReflect,
    kPhongCube,
    kPhongCubeFade,
    kPhongCubeReflect,
    kUnlitSegmentation,
    kUnlitDecor,
    kUnlitDepth,
    kUnlitLine,
    kUnlitUi,
    kNumMaterials,
  };

  // Returns the filament Engine that owns the assets.
  filament::Engine* GetEngine() const { return engine_; }

  // Returns the Material of the given type.
  filament::Material* GetMaterial(MaterialType type) const;

  // Returns the fallback Texture with the given role.
  const filament::Texture* GetFallbackTexture(mjtTextureRole role) const;

  // Loads the given asset from the filament resource directory.
  std::unique_ptr<Asset> LoadAsset(std::string_view filename);

  // The default environment light to use if no environment light is specified.
  static constexpr const char* kDefaultEnvironmentLight = "ibl.ktx";

  ObjectManager(const ObjectManager&) = delete;
  ObjectManager& operator=(const ObjectManager&) = delete;

 private:
  filament::Engine* engine_ = nullptr;
  std::array<filament::Material*, kNumMaterials> materials_;
  std::array<filament::Texture*, mjNTEXROLE> fallback_textures_;
  filament::Texture* fallback_white_ = nullptr;
  filament::Texture* fallback_black_ = nullptr;
  filament::Texture* fallback_normal_ = nullptr;
  filament::Texture* fallback_orm_ = nullptr;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_OBJECT_MANAGER_H_
