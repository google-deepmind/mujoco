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
#include "experimental/filament/filament/texture.h"

namespace mujoco {

RenderTarget::RenderTarget(filament::Engine* engine,
                           RenderTargetTextureType color,
                           RenderTargetTextureType depth)
    : engine_(engine), color_type_(color), depth_type_(depth) {}

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

  color_texture_ =
      std::make_unique<Texture>(engine_, color_type_, width, height);
  depth_texture_ =
      std::make_unique<Texture>(engine_, depth_type_, width, height);

  filament::RenderTarget::Builder builder;
  builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                  color_texture_->GetFilamentTexture());
  builder.texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                  depth_texture_->GetFilamentTexture());
  render_target_ = builder.build(*engine_);
}

void RenderTarget::ReadColorPixels(filament::Renderer* renderer, uint8_t* bytes,
                                   size_t num_bytes) {
  filament::backend::PixelDataFormat format;
  filament::backend::PixelDataType type;
  size_t expected_num_bytes = 0;
  switch (color_type_) {
    case RenderTargetTextureType::kColor:
      format = filament::backend::PixelDataFormat::RGB;
      type = filament::backend::PixelDataType::UBYTE;
      expected_num_bytes = width_ * height_ * 3;
      break;
    case RenderTargetTextureType::kDepthColor:
      format = filament::backend::PixelDataFormat::R;
      type = filament::backend::PixelDataType::FLOAT;
      expected_num_bytes = width_ * height_ * sizeof(float);
      break;
    default:
      mju_error("Unsupported pixel format: %d", color_type_);
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
    engine_->destroy(render_target_);
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
