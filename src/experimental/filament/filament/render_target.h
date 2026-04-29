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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_H_

#include <cstddef>
#include <cstdint>
#include <memory>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/texture.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Manages a filament RenderTarget and the textures which are bound to it.
class RenderTarget : public mjrRenderTarget {
 public:
  // Defines the types of textures to create for the color and depth
  // attachments.
  RenderTarget(FilamentContext* ctx, const mjrRenderTargetConfig& config);
  ~RenderTarget() noexcept;

  RenderTarget(const RenderTarget&) = delete;
  RenderTarget& operator=(const RenderTarget&) = delete;

  // Creates the textures and render target if the width of height differ from
  // the last time the render target was prepared.
  void Prepare(int width, int height);

  // Reads the pixels from the render target texture.
  void ReadColorPixels(filament::Renderer* renderer, uint8_t* bytes,
                       size_t num_bytes);

  // Returns the color texture.
  Texture* GetColorTexture() const;

  // Returns the depth texture.
  Texture* GetDepthTexture() const;

  // Returns the underlying filament render target.
  filament::RenderTarget* GetFilamentRenderTarget() const;

  static RenderTarget* downcast(mjrRenderTarget* render_target) {
    return static_cast<RenderTarget*>(render_target);
  }
  static const RenderTarget* downcast(const mjrRenderTarget* render_target) {
    return static_cast<const RenderTarget*>(render_target);
  }

 private:
  void Destroy();

  FilamentContext* ctx_ = nullptr;
  mjrRenderTargetConfig config_;
  filament::RenderTarget* render_target_ = nullptr;
  std::unique_ptr<Texture> color_texture_ = nullptr;
  std::unique_ptr<Texture> depth_texture_ = nullptr;
  int width_ = 0;
  int height_ = 0;
};

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_RENDER_TARGET_H_
