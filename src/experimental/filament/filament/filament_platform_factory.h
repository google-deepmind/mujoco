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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_PLATFORM_FACTORY_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_PLATFORM_FACTORY_H_

#include <memory>

#include "experimental/filament/render_context_filament.h"
#include <backend/Platform.h>
#include <filament/Engine.h>

namespace mujoco {

struct FilamentPlatformSetup {
  // The filament::Platform to use for the engine. May be null, in which case
  // the engine will be created with a default platform.
  std::unique_ptr<filament::backend::Platform> platform = nullptr;

  // The filament::Engine::Backend to use for the engine.
  filament::Engine::Backend backend;

  // Whether to disable parallel shader compilation.
  bool disable_parallel_shader_compile = false;
};

// Creates a filament::Platform based on the given config and defines the
// filament::Engine::Backend to use. Also returns additional information for
// setting up the filament::Engine for use with the provided Platform.
FilamentPlatformSetup CreateFilamentPlatform(const mjrFilamentConfig& config);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_FILAMENT_PLATFORM_FACTORY_H_
