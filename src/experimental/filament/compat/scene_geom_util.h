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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_

#include <memory>

#include <mujoco/mjvisualize.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/filament/filament_context.h"
#include "experimental/filament/filament/renderable.h"

namespace mujoco {

// Creates a Renderable from the given mjvGeom.
std::unique_ptr<Renderable> CreateGeomRenderable(
    const mjvGeom& geom, const mjvScene* scene, FilamentContext* ctx,
    ModelObjects* model_objs, const float headpos[3]);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
