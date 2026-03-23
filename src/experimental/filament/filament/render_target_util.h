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

#include <filament/Engine.h>
#include <filament/Texture.h>

namespace mujoco {

// The different types of textures we can create for a render target.
enum RenderTargetTextureType {
  kRenderTargetNone,
  kRenderTargetColor,
  kRenderTargetDepth,
  kRenderTargetDepthColor,
  kRenderTargetReflectionColor,
  kNumRenderTargetTextureTypes,
};

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
  filament::Texture* GetColorTexture() const { return color_texture_; }

  // Returns the depth texture.
  filament::Texture* GetDepthTexture() const { return depth_texture_; }

  // Returns the render target.
  filament::RenderTarget* GetRenderTarget() const { return render_target_; }

 private:
  void Destroy();

  filament::Engine* engine_ = nullptr;
  filament::Texture* color_texture_ = nullptr;
  filament::Texture* depth_texture_ = nullptr;
  filament::RenderTarget* render_target_ = nullptr;
  RenderTargetTextureType color_type_ = kRenderTargetNone;
  RenderTargetTextureType depth_type_ = kRenderTargetNone;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_UTIL_H_
