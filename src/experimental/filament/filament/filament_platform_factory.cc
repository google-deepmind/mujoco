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

#include <dlfcn.h>
#include <memory>

#include <backend/Platform.h>
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
    case mjGRAPHICS_API_DEFAULT:
      // Use the default based on the platform above.
      break;
    case mjGRAPHICS_API_OPENGL:
      backend = filament::Engine::Backend::OPENGL;
      break;
    case mjGRAPHICS_API_VULKAN:
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
  return setup;
}
}  // namespace mujoco
