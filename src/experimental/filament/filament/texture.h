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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_H_

#include <filament/Engine.h>
#include <filament/Texture.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/render_context_filament.h"

// Functions for creating filament textures.
namespace mujoco {

// Wrapper around a filament::Texture.
class Texture : public mjrTexture {
 public:
  // Flags for internal use.
  struct InternalFlags {
    InternalFlags() : color_attachment(false), depth_attachment(false) {}
    bool color_attachment;
    bool depth_attachment;
  };

  // Creates a texture with the given data.
  Texture(filament::Engine* engine, const mjrTextureConfig& config,
          InternalFlags flags = InternalFlags());

  ~Texture();

  Texture(const Texture&) = delete;
  Texture& operator=(const Texture&) = delete;

  // Uploads the given data to the texture.
  void Upload(const mjrTextureData& data);

  // Returns the width of the texture.
  int GetWidth() const { return config_.width; }

  // Returns the height of the texture.
  int GetHeight() const { return config_.height; }

  // Returns the underlying filament texture.
  filament::Texture* GetFilamentTexture() const { return texture_; }

  // Returns any spherical harmonics data associated with the texture.
  using SphericalHarmonics = filament::math::float3[9];
  const SphericalHarmonics* GetSphericalHarmonics() const {
    return has_spherical_harmonics_ ? &spherical_harmonics_ : nullptr;
  }

  static Texture* downcast(mjrTexture* texture) {
    return static_cast<Texture*>(texture);
  }
  static const Texture* downcast(const mjrTexture* texture) {
    return static_cast<const Texture*>(texture);
  }

 private:
  void ReleaseData();

  filament::Engine* engine_ = nullptr;
  filament::Texture* texture_ = nullptr;
  mjrTextureConfig config_;
  SphericalHarmonics spherical_harmonics_;
  bool has_spherical_harmonics_ = false;

  void* user_data_ = nullptr;
  void (*release_callback_)(void* user_data) = nullptr;
};
}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_H_
