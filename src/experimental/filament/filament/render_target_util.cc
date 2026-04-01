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

#include <filament/Engine.h>
#include <filament/RenderTarget.h>
#include <filament/Texture.h>
#include <mujoco/mujoco.h>

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
      CreateRenderTargetTexture(engine_, width, height, color_type_);
  depth_texture_ =
      CreateRenderTargetTexture(engine_, width, height, depth_type_);

  filament::RenderTarget::Builder builder;
  builder.texture(filament::RenderTarget::AttachmentPoint::COLOR,
                  color_texture_);
  builder.texture(filament::RenderTarget::AttachmentPoint::DEPTH,
                  depth_texture_);
  render_target_ = builder.build(*engine_);
}

void RenderTargetAndTextures::Destroy() {
  if (render_target_) {
    engine_->destroy(render_target_);
    render_target_ = nullptr;
  }
  if (color_texture_) {
    engine_->destroy(color_texture_);
    color_texture_ = nullptr;
  }
  if (depth_texture_) {
    engine_->destroy(depth_texture_);
    depth_texture_ = nullptr;
  }
}

}  // namespace mujoco
