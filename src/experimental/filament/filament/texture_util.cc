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

Texture::Texture(filament::Engine* engine, TextureType texture_type,
                 mjtColorSpace color_space, int width, int height,
                 int num_channels, const uint8_t* data)
    : engine_(engine) {
  const bool is_srgb = color_space == mjCOLORSPACE_SRGB;
  if (texture_type == TextureType::kCube) {
    CreateCubeTexture(width, height, num_channels, data, is_srgb);
  } else if (texture_type == TextureType::kNormal2d) {
    Create2dTexture(width, height, num_channels, data, is_srgb);
  } else if (texture_type == TextureType::kKtx) {
    CreateKtxTexture(data, width * height * num_channels);
  } else {
    mju_error("Unsupported texture type: %d", static_cast<int>(texture_type));
  }
}

Texture::Texture(filament::Engine* engine, RenderTargetTextureType type,
                 int width, int height) : engine_(engine) {
  filament::Texture::Builder builder;
  builder.width(width);
  builder.height(height);
  switch (type) {
    case RenderTargetTextureType::kColor:
      builder.usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                    filament::Texture::Usage::BLIT_SRC);
      builder.format(filament::Texture::InternalFormat::RGB8);
      break;
    case RenderTargetTextureType::kDepth:
      builder.usage(filament::Texture::Usage::DEPTH_ATTACHMENT |
                    filament::Texture::Usage::SAMPLEABLE);
      builder.format(filament::Texture::InternalFormat::DEPTH32F);
      break;
    case RenderTargetTextureType::kDepthColor:
      builder.usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                    filament::Texture::Usage::BLIT_SRC);
      builder.format(filament::Texture::InternalFormat::R32F);
      break;
    case RenderTargetTextureType::kReflectionColor:
      builder.usage(filament::Texture::Usage::COLOR_ATTACHMENT |
                    filament::Texture::Usage::BLIT_SRC |
                    filament::Texture::Usage::SAMPLEABLE);
      builder.format(filament::Texture::InternalFormat::RGBA8);
      break;
    default:
      mju_error("Unknown type: %d", static_cast<int>(type));
  }
  texture_ = builder.build(*engine);
}

void Texture::Create2dTexture(int width, int height, int num_channels,
                              const uint8_t* data, bool is_srgb) {
  if (num_channels != 1 && num_channels != 3 && num_channels != 4) {
    mju_error("Unsupported number of channels: %d", num_channels);
    return;
  }

  filament::Texture::Builder builder;
  builder.width(width);
  builder.height(height);
  builder.format(GetTextureInternalFormat(num_channels, is_srgb));
  builder.sampler(filament::Texture::Sampler::SAMPLER_2D);
  if (!is_srgb) {
    builder.usage(filament::Texture::Usage::GEN_MIPMAPPABLE |
                  filament::Texture::Usage::SAMPLEABLE |
                  filament::Texture::Usage::UPLOADABLE);
  }
  texture_ = builder.build(*engine_);

  if (data) {
    const size_t num_bytes = width * height * sizeof(uint8_t) * num_channels;
    const filament::Texture::Format format = GetTextureFormat(num_channels);
    texture_->setImage(
        *engine_, 0,
        filament::Texture::PixelBufferDescriptor(
            data, num_bytes, format, filament::Texture::Type::UBYTE));
    if (!is_srgb) {
      texture_->generateMipmaps(*engine_);
    }
  }
}

void Texture::CreateCubeTexture(int width, int height, int num_channels,
                                const uint8_t* data, bool is_srgb) {
  if (num_channels != 3) {
    mju_error("Only support RGB cubemaps.");
    return;
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
  if (!is_srgb) {
    builder.usage(filament::Texture::Usage::GEN_MIPMAPPABLE |
                  filament::Texture::Usage::SAMPLEABLE |
                  filament::Texture::Usage::UPLOADABLE);
  }
  texture_ = builder.build(*engine_);

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
    texture_->setImage(*engine_, 0, std::move(desc), offsets);
    if (!is_srgb) {
      texture_->generateMipmaps(*engine_);
    }
  }
}

void Texture::CreateKtxTexture(const uint8_t* data, int size) {
  image::Ktx1Bundle* bundle = new image::Ktx1Bundle(data, size);
  has_spherical_harmonics_ = true;
  bundle->getSphericalHarmonics(spherical_harmonics_);
  const bool is_srgb = false;
  texture_ = ktxreader::Ktx1Reader::createTexture(engine_, bundle, is_srgb);
}

Texture::~Texture() {
  if (texture_) {
    engine_->destroy(texture_);
  }
}

}  // namespace mujoco
