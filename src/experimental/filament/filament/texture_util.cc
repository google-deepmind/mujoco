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

#include "experimental/filament/filament/texture_util.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include <image/Ktx1Bundle.h>
#include <ktxreader/Ktx1Reader.h>
#include <math/vec3.h>
#include <mujoco/mujoco.h>


namespace mujoco {

static filament::Texture::Format GetTextureFormat(int num_channels) {
  switch (num_channels) {
    case 1:
      return filament::Texture::Format::R;
    case 3:
      return filament::Texture::Format::RGB;
    case 4:
      return filament::Texture::Format::RGBA;
    default:
      mju_error("Unsupported number of channels: %d", num_channels);
      return filament::Texture::Format::UNUSED;
    }
}

static filament::Texture::InternalFormat GetTextureInternalFormat(
    int num_channels, bool is_srgb) {
  if (is_srgb) {
    switch (num_channels) {
      case 3:
        return filament::Texture::InternalFormat::SRGB8;
      case 4:
        return filament::Texture::InternalFormat::SRGB8_A8;
      default:
        mju_error("Unsupported number of channels: %d", num_channels);
        return filament::Texture::InternalFormat::UNUSED;
    }
  } else {
    switch (num_channels) {
      case 1:
        return filament::Texture::InternalFormat::R8;
      case 3:
        return filament::Texture::InternalFormat::RGB8;
      case 4:
        return filament::Texture::InternalFormat::RGBA8;
      default:
        mju_error("Unsupported number of channels: %d", num_channels);
        return filament::Texture::InternalFormat::UNUSED;
    }
  }
}

filament::Texture* Create2dTexture(filament::Engine* engine, int width,
                                   int height, int num_channels,
                                   const uint8_t* data, bool is_srgb) {
  if (num_channels != 1 && num_channels != 3 && num_channels != 4) {
    mju_error("Unsupported number of channels: %d", num_channels);
    return nullptr;
  }

  filament::Texture::Builder builder;
  builder.width(width);
  builder.height(height);
  builder.format(GetTextureInternalFormat(num_channels, is_srgb));
  builder.sampler(filament::Texture::Sampler::SAMPLER_2D);
  // TODO: Revisit this to make it this work in WebGL
  #ifndef __EMSCRIPTEN__
  if (!is_srgb) {
    builder.usage(filament::Texture::Usage::GEN_MIPMAPPABLE |
                  filament::Texture::Usage::SAMPLEABLE |
                  filament::Texture::Usage::UPLOADABLE);
  }
  #endif
  filament::Texture* texture = builder.build(*engine);

  if (data) {
    const size_t num_bytes = width * height * sizeof(uint8_t) * num_channels;
    const filament::Texture::Format format = GetTextureFormat(num_channels);
    texture->setImage(
        *engine, 0,
        filament::Texture::PixelBufferDescriptor(
            data, num_bytes, format, filament::Texture::Type::UBYTE));
    // TODO: Revisit this to make it this work in WebGL
    #ifndef __EMSCRIPTEN__
    texture->generateMipmaps(*engine);
    #endif
  }
  return texture;
}

filament::Texture* CreateCubeTexture(filament::Engine* engine, int width,
                                     int height, int num_channels,
                                     const uint8_t* data, bool is_srgb) {
  if (num_channels != 3) {
    mju_error("Only support RGB cubemaps.");
    return nullptr;
  }

  const int kNumFacesPerCube = 6;

  int face_height = height;
  if (width != height) {
    if (width * kNumFacesPerCube != height) {
        mju_error("Cube maps must contain 6 square images.");
    }
    face_height = height / kNumFacesPerCube;
  }
  if (width != face_height) {
    mju_error("Cube map faces must be square.");
  }

  filament::Texture::Builder builder;
  builder.width(width);
  builder.height(face_height);
  builder.format(GetTextureInternalFormat(num_channels, is_srgb));
  builder.sampler(filament::Texture::Sampler::SAMPLER_CUBEMAP);
  // TODO: Revisit this to make it this work in WebGL
  #ifndef __EMSCRIPTEN__
  if (!is_srgb) {
    builder.usage(filament::Texture::Usage::GEN_MIPMAPPABLE |
                  filament::Texture::Usage::SAMPLEABLE |
                  filament::Texture::Usage::UPLOADABLE);
  }
  #endif
  filament::Texture* texture = builder.build(*engine);

  const int face_size = width * face_height * num_channels;
  const int num_bytes = face_size * kNumFacesPerCube;

  uint8_t* buffer = new uint8_t[num_bytes];
  auto callback = +[](void* buffer, size_t size, void* user) {
    delete [] reinterpret_cast<uint8_t*>(buffer);
  };

  filament::Texture::FaceOffsets offsets(face_size);
  if (width == height) {
    // Copy the image to all the faces.
    for (int i = 0; i < kNumFacesPerCube; ++i) {
      std::memcpy(buffer + (i * face_size), data, face_size);
    }
  } else {
    // Use the cubemap as is.
    std::memcpy(buffer, data, num_bytes);
  }

  if (data) {
    filament::Texture::PixelBufferDescriptor desc(
                          buffer, num_bytes, filament::Texture::Format::RGB,
                          filament::Texture::Type::UBYTE, callback);
    texture->setImage(*engine, 0, std::move(desc), offsets);
    // TODO: Revisit this to make it this work in WebGL
    #ifndef __EMSCRIPTEN__
    texture->generateMipmaps(*engine);
    #endif
  }
  return texture;
}

filament::Texture* CreateKtxTexture(
    filament::Engine* engine, const uint8_t* data, int size,
    filament::math::float3* spherical_harmonics_out) {
  image::Ktx1Bundle* bundle = new image::Ktx1Bundle(data, size);
  if (spherical_harmonics_out) {
    bundle->getSphericalHarmonics(spherical_harmonics_out);
  }
  const bool is_srgb = false;
  return ktxreader::Ktx1Reader::createTexture(engine, bundle, is_srgb);
}

filament::Texture* CreateRenderTargetTexture(filament::Engine* engine,
                                             int width, int height,
                                             RenderTargetTextureType type) {
  filament::Texture::Builder builder;
  builder.width(width);
  builder.height(height);
  switch (type) {
    case kRenderTargetColor:
      builder.usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                    filament::Texture::Usage::BLIT_SRC);
      builder.format(filament::Texture::InternalFormat::RGB8);
      break;
    case kRenderTargetDepth:
      builder.usage(filament::Texture::Usage::DEPTH_ATTACHMENT |
                    filament::Texture::Usage::SAMPLEABLE);
      builder.format(filament::Texture::InternalFormat::DEPTH32F);
      break;
    case kRenderTargetDepthColor:
      builder.usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                    filament::Texture::Usage::BLIT_SRC);
      builder.format(filament::Texture::InternalFormat::R32F);
      break;
    default:
      mju_error("Unknown type: %d", static_cast<int>(type));
  }
  return builder.build(*engine);
}

}  // namespace mujoco
