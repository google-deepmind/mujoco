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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_

#include <cstdint>

#include <filament/Engine.h>
#include <filament/Texture.h>
#include <math/vec3.h>

// Functions for creating filament textures.
namespace mujoco {

// Creates a filament Texture for the given 2D texture.
filament::Texture* Create2dTexture(filament::Engine* engine, int width,
                                   int height, int num_channels,
                                   const uint8_t* data, bool is_srgb);

// Creates a filament Texture for the given cube texture.
filament::Texture* CreateCubeTexture(filament::Engine* engine, int width,
                                     int height, int num_channels,
                                     const uint8_t* data, bool is_srgb);

// Creates a filament Texture for the given KTX payload.
filament::Texture* CreateKtxTexture(
    filament::Engine* engine, const uint8_t* data, int size,
    filament::math::float3* spherical_harmonics_out);

enum RenderTargetTextureType {
  kRenderTargetColor,
  kRenderTargetDepth,
  kRenderTargetDepthColor,
  kNumRenderTargetTextureTypes,
};

// Creates a filament RenderTargetTexture for the given target type.
filament::Texture* CreateRenderTargetTexture(filament::Engine* engine,
                                             int width, int height,
                                             RenderTargetTextureType type);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_TEXTURE_UTIL_H_
