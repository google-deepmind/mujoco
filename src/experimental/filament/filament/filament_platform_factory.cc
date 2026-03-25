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

#include "experimental/filament/filament/filament_platform_factory.h"

#include <memory>

#include <backend/Platform.h>

#ifdef __linux__
#include <backend/platforms/PlatformEGL.h>
#include <backend/platforms/PlatformEGLHeadless.h>  // IWYU pragma: keep
#include <backend/platforms/PlatformGLX.h>
#include <backend/platforms/VulkanPlatformLinux.h>
#include "third_party/filament/libs/bluegl/include/bluegl/BlueGL.h"
#endif

#include <filament/Engine.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

static filament::Engine::Backend ResolveBackend(int graphics_api) {
#if defined(__EMSCRIPTEN__)
  filament::Engine::Backend backend = filament::Engine::Backend::OPENGL;
#else
  filament::Engine::Backend backend = filament::Engine::Backend::VULKAN;
#endif

  switch (graphics_api) {
    case mjGFX_DEFAULT:
      // Use the default based on the platform above.
      break;
    case mjGFX_OPENGL:
      backend = filament::Engine::Backend::OPENGL;
      break;
    case mjGFX_VULKAN:
      backend = filament::Engine::Backend::VULKAN;
      break;
    default:
      mju_error("Unsupported graphics API: %d", graphics_api);
  }
  return backend;
}

FilamentPlatformSetup CreateFilamentPlatform(const mjrFilamentConfig& config) {
  FilamentPlatformSetup setup;
  setup.backend = ResolveBackend(config.graphics_api);

#ifdef __linux__
  if (setup.backend == filament::Engine::Backend::OPENGL) {
    if (config.native_window == nullptr) {
      setup.platform = std::make_unique<filament::backend::PlatformEGLHeadless>();
      setup.disable_parallel_shader_compile = true;
    } else {
      setup.platform = std::make_unique<filament::backend::PlatformGLX>();
    }
  } else {
    setup.platform = std::make_unique<filament::backend::VulkanPlatformLinux>();
  }
#endif
  return setup;
}


}  // namespace mujoco
