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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_BACKEND_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_BACKEND_H_

namespace mujoco::platform {

// Describes the configuration of the renderer backend.
//
// This is generally a combination of two things. The first describes the
// high-level rendering engine that sits behind the MuJoCo API (e.g.
// mjr_render). The second describes the low-level graphics engine that is
// powering the rendering (e.g. OpenGL, Vulkan, etc.).
//
// More details about the backends can be found in renderer.h.
enum class RendererBackend {
  // The classic MuJoCo OpenGL renderer.
  ClassicOpenGl,

  // The Filament-based renderer running on OpenGL.
  FilamentOpenGl,

  // The Filament-based renderer running on Vulkan.
  FilamentVulkan,

  // The Filament-based renderer running on WebGL.
  FilamentWebGl,

  // Similar to FilamentOpenGl, but assumes "headless" rendering.
  FilamentOpenGlHeadless,
};

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_RENDERER_BACKEND_H_
