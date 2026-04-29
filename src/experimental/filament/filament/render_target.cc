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

#include "experimental/filament/filament/render_target.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include <backend/DriverEnums.h>
#include <backend/PixelBufferDescriptor.h>
#include <filament/Engine.h>
#include <filament/Renderer.h>
#include <filament/RenderTarget.h>
#include <filament/Texture.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

RenderTarget::RenderTarget(FilamentContext* ctx,
                           const mjrRenderTargetConfig& config)
    : ctx_(ctx), config_(config) {
  if (config_.width > 0 && config_.height > 0) {
    Prepare(config_.width, config_.height);
  }
}

RenderTarget::~RenderTarget() noexcept {
  Destroy();
}

void RenderTarget::Prepare(int width, int height) {
  if (width == width_ && height == height_) {
    return;
  }
  Destroy();
  width_ = width;
  height_ = height;
  if (width_ <= 0 || height_ <= 0) {
    width_ = 0;
    height_ = 0;
    return;
  }

  mjrTextureConfig color_config;
  mjr_defaultTextureConfig(&color_config);
  Texture::InternalFlags color_flags;
  color_config.width = width;
  color_config.height = height;
  color_config.target = mjTEXTURE_2D;
  color_config.format = config_.color_format;
  color_config.color_space = mjCOLORSPACE_LINEAR;
  color_config.format = mjPIXEL_FORMAT_RGB8;
  color_flags.color_attachment = true;
  color_texture_ = std::make_unique<Texture>(ctx_, color_config, color_flags);

  mjrTextureConfig depth_config;
  mjr_defaultTextureConfig(&depth_config);
  Texture::InternalFlags depth_flags;
  depth_config.width = width;
  depth_config.height = height;
  depth_config.target = mjTEXTURE_2D;
  depth_config.format = config_.depth_format;
  depth_config.color_space = mjCOLORSPACE_LINEAR;
  depth_config.format = mjPIXEL_FORMAT_DEPTH32F;
  depth_flags.depth_attachment = true;
  depth_texture_ = std::make_unique<Texture>(ctx_, depth_config, depth_flags);

  filament::RenderTarget::Builder builder;
  builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                  color_texture_->GetFilamentTexture());
  builder.texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                  depth_texture_->GetFilamentTexture());
  render_target_ = builder.build(*ctx_->GetEngine());
}

void RenderTarget::ReadColorPixels(filament::Renderer* renderer, uint8_t* bytes,
                                   size_t num_bytes) {
  filament::backend::PixelDataFormat format;
  filament::backend::PixelDataType type;
  size_t expected_num_bytes = 0;
  switch (config_.color_format) {
    case mjPIXEL_FORMAT_RGB8:
      format = filament::backend::PixelDataFormat::RGB;
      type = filament::backend::PixelDataType::UBYTE;
      expected_num_bytes = width_ * height_ * 3;
      break;
    case mjPIXEL_FORMAT_R32F:
      format = filament::backend::PixelDataFormat::R;
      type = filament::backend::PixelDataType::FLOAT;
      expected_num_bytes = width_ * height_ * sizeof(float);
      break;
    default:
      mju_error("Unsupported pixel format: %d", config_.color_format);
      return;
  }
  if (num_bytes != expected_num_bytes) {
    mju_error("Invalid number of bytes.");
    return;
  }

  filament::backend::PixelBufferDescriptor desc(bytes, num_bytes, format, type);
  renderer->readPixels(render_target_, 0, 0, width_, height_, std::move(desc));
}

void RenderTarget::Destroy() {
  if (render_target_) {
    ctx_->GetEngine()->destroy(render_target_);
    render_target_ = nullptr;
  }
  color_texture_.reset();
  depth_texture_.reset();
}

Texture* RenderTarget::GetColorTexture() const {
  return color_texture_.get();
}

Texture* RenderTarget::GetDepthTexture() const {
  return depth_texture_.get();
}

filament::RenderTarget* RenderTarget::GetFilamentRenderTarget() const {
  return render_target_;
}

}  // namespace mujoco
