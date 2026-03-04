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

#include "experimental/platform/graphics_mode.h"

namespace mujoco::platform {

bool IsClassic(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::ClassicOpenGl ||
         gfx_mode == GraphicsMode::ClassicOpenGlHeadless;
}

bool IsFilament(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::FilamentOpenGl ||
         gfx_mode == GraphicsMode::FilamentVulkan ||
         gfx_mode == GraphicsMode::FilamentWebGl ||
         gfx_mode == GraphicsMode::FilamentOpenGlHeadless;
}

bool IsOpenGl(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::ClassicOpenGl ||
         gfx_mode == GraphicsMode::ClassicOpenGlHeadless ||
         gfx_mode == GraphicsMode::FilamentOpenGl ||
         gfx_mode == GraphicsMode::FilamentOpenGlHeadless;
}

bool IsVulkan(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::FilamentVulkan;
}

bool IsWebGl(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::FilamentWebGl;
}

bool IsHeadless(GraphicsMode gfx_mode) {
  return gfx_mode == GraphicsMode::ClassicOpenGlHeadless ||
         gfx_mode == GraphicsMode::FilamentOpenGlHeadless;
}

}  // namespace mujoco::platform
