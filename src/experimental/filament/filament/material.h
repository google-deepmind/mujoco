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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_

#include <filament/Engine.h>
#include <filament/MaterialInstance.h>
#include "experimental/filament/filament/object_manager.h"
#include "experimental/filament/render_context_filament.h"

namespace mujoco {

// Updates the material instances based on the currently set parameters and
// textures.
void UpdateMaterialInstance(filament::MaterialInstance* instance,
                            const mjrMaterialParams& params,
                            const mjrMaterialTextures& textures,
                            ObjectManager* object_mgr);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_MATERIAL_H_
