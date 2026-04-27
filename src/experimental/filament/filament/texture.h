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

#include <cstddef>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include <math/vec3.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>

// Functions for creating filament textures.
namespace mujoco {

// Pixel formats for textures.
typedef enum mjrPixelFormat_ {
  mjPIXEL_FORMAT_UNKNOWN = 0,
  mjPIXEL_FORMAT_R8,
  mjPIXEL_FORMAT_RGB8,
  mjPIXEL_FORMAT_RGBA8,
  mjPIXEL_FORMAT_R32F,
  mjPIXEL_FORMAT_DEPTH32F,
  mjPIXEL_FORMAT_KTX,
} mjrPixelFormat;

typedef mjtTexture mjrTextureTarget;
typedef mjtColorSpace mjrColorSpace;

// The binary contents of a texture.
struct mjrTextureData {
  // Pointer to the image data. If null, an empty texture will be created.
  const void* bytes;

  // The number of bytes in the image data.
  mjtSize nbytes;

  // Because rendering may be multithreaded, we cannot make assumptions about
  // when the image data will finish uploading to the GPU. As such, we will use
  // this callback to notify callers when it is safe to free the image data.
  void (*release_callback)(void* user_data);

  // User data to pass to the release callback.
  void* user_data;
};

// Initializes the TextureData to default values.
void mjr_defaultTextureData(mjrTextureData* data);

// Defines the basic properties of a texture.
struct mjrTextureConfig {
  // The width of the texture. For compressed textures (e.g. KTX), this is the
  // number of bytes in the compressed data.
  int width;

  // The height of the texture. For compressed textures (e.g. KTX), this should
  // be 0.
  int height;

  // The target of the texture (e.g. 2D, cube, etc.)
  mjrTextureTarget target;

  // The format of the pixels in the texture (e.g. RGB8, RGBA8, KTX, etc.)
  mjrPixelFormat format;

  // The color space of the texture (e.g. LINEAR, sRGB, etc.)
  mjrColorSpace color_space;
};

// Initializes the TextureConfig to default values.
void mjr_defaultTextureConfig(mjrTextureConfig* config);

// Wrapper around a filament::Texture.
class Texture {
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

  Texture(const Texture&) = delete;
  Texture& operator=(const Texture&) = delete;

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
