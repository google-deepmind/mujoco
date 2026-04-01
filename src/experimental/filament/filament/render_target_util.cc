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

#include "experimental/filament/filament/render_target_util.h"

#include <memory>

#include <filament/Engine.h>
#include <filament/RenderTarget.h>
#include <filament/Texture.h>
#include "experimental/filament/filament/texture.h"

namespace mujoco {

RenderTargetAndTextures::RenderTargetAndTextures(filament::Engine* engine,
                                                 RenderTargetTextureType color,
                                                 RenderTargetTextureType depth)
    : engine_(engine), color_type_(color), depth_type_(depth) {}

RenderTargetAndTextures::~RenderTargetAndTextures() noexcept {
  Destroy();
}

void RenderTargetAndTextures::Prepare(int width, int height) {
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

void RenderTargetAndTextures::Destroy() {
  if (render_target_) {
    engine_->destroy(render_target_);
    render_target_ = nullptr;
  }
  color_texture_.reset();
  depth_texture_.reset();
}

Texture* RenderTargetAndTextures::GetColorTexture() const {
  return color_texture_.get();
}

Texture* RenderTargetAndTextures::GetDepthTexture() const {
  return depth_texture_.get();
}

filament::RenderTarget* RenderTargetAndTextures::GetRenderTarget() const {
  return render_target_;
}

}  // namespace mujoco
