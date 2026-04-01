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

#include "experimental/filament/filament/texture.h"

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

static constexpr int kNumFacesPerCube = 6;

static bool IsCompressed(const TextureConfig& config) {
  return config.format == mjPIXEL_FORMAT_KTX;
}

static bool IsCubeMap(const TextureConfig& config) {
  return config.target == mjTEXTURE_CUBE || config.target == mjTEXTURE_SKYBOX;
}

static int GetFaceHeight(const TextureConfig& config) {
  int face_height = config.height;
  if (config.width != config.height) {
    if (config.width * kNumFacesPerCube != config.height) {
      mju_error("Cube maps must contain 6 square images.");
    }
    face_height = config.height / kNumFacesPerCube;
  }
  if (config.width != face_height) {
    mju_error("Cube map faces must be square.");
  }
  return face_height;
}

static int GetNumChannels(const TextureConfig& config) {
  switch (config.format) {
    case mjPIXEL_FORMAT_R8:
      return 1;
    case mjPIXEL_FORMAT_RGB8:
      return 3;
    case mjPIXEL_FORMAT_RGBA8:
      return 4;
    default:
      mju_error("Unsupported format: %d", (int)config.format);
      return 0;
  }
}

static filament::Texture::Format GetTextureFormat(const TextureConfig& config) {
  switch (config.format) {
    case mjPIXEL_FORMAT_R8:
      return filament::Texture::Format::R;
    case mjPIXEL_FORMAT_RGB8:
      return filament::Texture::Format::RGB;
    case mjPIXEL_FORMAT_RGBA8:
      return filament::Texture::Format::RGBA;
    default:
      mju_error("Unsupported format: %d", (int)config.format);
      return filament::Texture::Format::UNUSED;
    }
}

static filament::Texture::InternalFormat GetTextureInternalFormat(
    const TextureConfig& config) {
  if (config.color_space == mjCOLORSPACE_SRGB) {
    switch (config.format) {
      case mjPIXEL_FORMAT_RGB8:
        return filament::Texture::InternalFormat::SRGB8;
      case mjPIXEL_FORMAT_RGBA8:
        return filament::Texture::InternalFormat::SRGB8_A8;
      default:
        mju_error("Unsupported format: %d", (int)config.format);
        return filament::Texture::InternalFormat::UNUSED;
    }
  } else {
    switch (config.format) {
      case mjPIXEL_FORMAT_R8:
        return filament::Texture::InternalFormat::R8;
      case mjPIXEL_FORMAT_RGB8:
        return filament::Texture::InternalFormat::RGB8;
      case mjPIXEL_FORMAT_RGBA8:
        return filament::Texture::InternalFormat::RGBA8;
      default:
        mju_error("Unsupported format: %d", (int)config.format);
        return filament::Texture::InternalFormat::UNUSED;
    }
  }
}

void DefaultTextureData(TextureData* data) {
  std::memset(data, 0, sizeof(TextureData));
}

void DefaultTextureConfig(TextureConfig* config) {
  std::memset(config, 0, sizeof(TextureConfig));
}

Texture::Texture(filament::Engine* engine, const TextureConfig& config)
    : engine_(engine), config_(config) {
  if (IsCompressed(config_)) {
    // We defer creation of compressed textures until Upload() is called. In
    // the meantime, we don't really know anything about the texture (e.g.
    // width, height, etc.).
    return;
  }

  filament::Texture::Builder builder;
  builder.width(config_.width);
  builder.height(config_.height);
  builder.format(GetTextureInternalFormat(config_));

  if (IsCubeMap(config_)) {
    if (config_.format != mjPIXEL_FORMAT_RGB8) {
      mju_error("Only support RGB cubemaps.");
      return;
    }
    builder.height(GetFaceHeight(config_));
    builder.sampler(filament::Texture::Sampler::SAMPLER_CUBEMAP);
  } else {
    builder.sampler(filament::Texture::Sampler::SAMPLER_2D);
  }

  if (config_.color_space != mjCOLORSPACE_SRGB) {
    builder.usage(filament::Texture::Usage::GEN_MIPMAPPABLE |
                  filament::Texture::Usage::SAMPLEABLE |
                  filament::Texture::Usage::UPLOADABLE);
  }
  texture_ = builder.build(*engine_);
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

Texture::~Texture() {
  ReleaseData();
  if (texture_) {
    engine_->destroy(texture_);
  }
}

void Texture::Upload(const TextureData& data) {
  user_data_ = data.user_data;
  release_callback_ = data.release_callback;

  if (data.bytes == nullptr || data.nbytes == 0) {
    ReleaseData();
    return;
  }

  if (config_.format == mjPIXEL_FORMAT_KTX) {
    image::Ktx1Bundle* bundle = new image::Ktx1Bundle(
        reinterpret_cast<const uint8_t*>(data.bytes), data.nbytes);
    has_spherical_harmonics_ = true;
    bundle->getSphericalHarmonics(spherical_harmonics_);
    const bool is_srgb = false;
    texture_ = ktxreader::Ktx1Reader::createTexture(engine_, bundle, is_srgb);
    config_.width = texture_->getWidth();
    config_.height = texture_->getHeight();
    ReleaseData();
    return;
  }

  const int num_channels = GetNumChannels(config_);
  const filament::Texture::Type type = filament::Texture::Type::UBYTE;
  const filament::Texture::Format format = GetTextureFormat(config_);

  if (!IsCubeMap(config_)) {
    if (config_.width * config_.height * num_channels != data.nbytes) {
      mju_error("Texture size does not match data size.");
    }

    auto callback = +[](void* buffer, size_t size, void* user) {
      reinterpret_cast<Texture*>(user)->ReleaseData();
    };
    filament::Texture::PixelBufferDescriptor desc(data.bytes, data.nbytes,
                                                  format, type, callback, this);
    texture_->setImage(*engine_, 0, std::move(desc));
  } else {
    const int face_size = config_.width * GetFaceHeight(config_) * num_channels;
    const int num_bytes = face_size * kNumFacesPerCube;
    filament::Texture::FaceOffsets offsets(face_size);

    if (config_.width == config_.height) {
      uint8_t* copy = new uint8_t[num_bytes];
      auto release_callback = +[](void* buffer, size_t size, void* user) {
        delete [] reinterpret_cast<uint8_t*>(buffer);
      };
      for (int i = 0; i < kNumFacesPerCube; ++i) {
        std::memcpy(copy + (i * face_size), data.bytes, face_size);
      }
      filament::Texture::PixelBufferDescriptor desc(copy, num_bytes, format,
                                                    type, release_callback);
      texture_->setImage(*engine_, 0, std::move(desc), offsets);
      ReleaseData();
    } else {
      if (num_bytes != data.nbytes) {
        mju_error("Texture size does not match data size.");
      }
      auto callback = +[](void* buffer, size_t size, void* user) {
        reinterpret_cast<Texture*>(user)->ReleaseData();
      };
      filament::Texture::PixelBufferDescriptor desc(
          data.bytes, data.nbytes, format, type, callback, this);
      texture_->setImage(*engine_, 0, std::move(desc), offsets);
    }
  }

  if (config_.color_space != mjCOLORSPACE_SRGB) {
    texture_->generateMipmaps(*engine_);
  }
}

void Texture::ReleaseData() {
  if (release_callback_) {
    release_callback_(user_data_);
    release_callback_ = nullptr;
    user_data_ = nullptr;
  }
}

}  // namespace mujoco
