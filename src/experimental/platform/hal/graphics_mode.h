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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_GRAPHICS_MODE_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_GRAPHICS_MODE_H_

#include <string_view>

namespace mujoco::platform {

// Describes the configuration of the graphics and rendering systems.
enum class GraphicsMode {
  // The classic MuJoCo OpenGL renderer.
  ClassicOpenGl,

  // The classic MuJoCo OpenGL renderer.
  ClassicOpenGlHeadless,

  // The Filament-based renderer running on OpenGL.
  FilamentOpenGl,

  // The Filament-based renderer running on Vulkan.
  FilamentVulkan,

  // The Filament-based renderer running on Vulkan using software rendering.
  FilamentVulkanSoftware,

  // The Filament-based renderer running on WebGL.
  FilamentWebGl,

  // Similar to FilamentOpenGl, but assumes "headless" rendering.
  FilamentOpenGlHeadless,

  // Similar to FilamentOpenGl, but forces software rendering.
  FilamentOpenGlSoftware,
};

bool IsClassic(GraphicsMode gfx_mode);
bool IsFilament(GraphicsMode gfx_mode);
bool IsOpenGl(GraphicsMode gfx_mode);
bool IsVulkan(GraphicsMode gfx_mode);
bool IsWebGl(GraphicsMode gfx_mode);
bool IsHeadless(GraphicsMode gfx_mode);
bool IsSoftware(GraphicsMode gfx_mode);

GraphicsMode GraphicsModeFromString(std::string_view str,
                                    GraphicsMode default_mode);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_HAL_GRAPHICS_MODE_H_
