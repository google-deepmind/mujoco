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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_UTIL_H_

#include <memory>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include "experimental/filament/filament/texture.h"

namespace mujoco {

// Manages a filament RenderTarget and the textures which are bound to it.
class RenderTargetAndTextures {
 public:
  // Defines the types of textures to create for the color and depth
  // attachments.
  RenderTargetAndTextures(filament::Engine* engine,
                          RenderTargetTextureType color,
                          RenderTargetTextureType depth);
  ~RenderTargetAndTextures() noexcept;

  RenderTargetAndTextures(const RenderTargetAndTextures&) = delete;
  RenderTargetAndTextures& operator=(const RenderTargetAndTextures&) = delete;

  // Creates the textures and render target if the width of height differ from
  // the last time the render target was prepared.
  void Prepare(int width, int height);

  // Returns the color texture.
  Texture* GetColorTexture() const;

  // Returns the depth texture.
  Texture* GetDepthTexture() const;

  // Returns the render target.
  filament::RenderTarget* GetRenderTarget() const;

 private:
  void Destroy();

  filament::Engine* engine_ = nullptr;
  filament::RenderTarget* render_target_ = nullptr;
  std::unique_ptr<Texture> color_texture_ = nullptr;
  std::unique_ptr<Texture> depth_texture_ = nullptr;
  RenderTargetTextureType color_type_;
  RenderTargetTextureType depth_type_;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_UTIL_H_
