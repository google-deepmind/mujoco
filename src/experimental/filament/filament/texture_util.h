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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_

#include <cstdint>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>

// Functions for creating filament textures.
namespace mujoco {

// The types of textures we can create.
enum class TextureType {
  kNormal2d,
  kCube,
  kKtx,
};

// The different types of textures we can create for a render target.
enum class RenderTargetTextureType {
  kColor,
  kDepth,
  kDepthColor,
  kReflectionColor,
};

class Texture {
 public:
  // Creates a texture with the given data.
  Texture(filament::Engine* engine, TextureType texture_type,
          mjtColorSpace color_space, int width, int height, int num_channels,
          const uint8_t* data);

  // Creates a texture for use with a render target.
  Texture(filament::Engine* engine, RenderTargetTextureType type, int width,
          int height);

  ~Texture();

  filament::Texture* GetFilamentTexture() const { return texture_; }

  using SphericalHarmonics = filament::math::float3[9];

  const SphericalHarmonics* GetSphericalHarmonics() const {
    return has_spherical_harmonics_ ? &spherical_harmonics_ : nullptr;
  }

  Texture(const Texture&) = delete;
  Texture& operator=(const Texture&) = delete;

 private:

  void Create2dTexture(int width, int height, int num_channels,
                       const uint8_t* data, bool is_srgb);
  void CreateCubeTexture(int width, int height, int num_channels,
                         const uint8_t* data, bool is_srgb);
  void CreateKtxTexture(const uint8_t* data, int size);

  filament::Engine* engine_ = nullptr;
  filament::Texture* texture_ = nullptr;
  SphericalHarmonics spherical_harmonics_;
  bool has_spherical_harmonics_ = false;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_
