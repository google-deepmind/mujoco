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

#include <mujoco/mjvisualize.h>
#include "experimental/filament/compat/model_objects.h"
#include "experimental/filament/render_context_filament.h"
#include "experimental/filament/render_context_filament_cpp.h"

namespace mujoco {

// Creates a Renderable from the given mjvGeom.
UniquePtr<mjrRenderable> CreateGeomRenderable(
    const mjvGeom& geom, const mjvScene* scene, mjrfContext* ctx,
    ModelObjects* model_objs, const float headpos[3]);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
