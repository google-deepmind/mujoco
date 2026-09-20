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

#include <mujoco/mjrfilament.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/scene_objects.h"
#include "render/filament/mjrfilament_cpp.h"
#include "render/filament/support/model_objects.h"

namespace mujoco {

// Creates a Renderable from the given mjvGeom.
UniquePtr<mjrfRenderable> CreateGeomRenderable(const mjvGeom& geom,
                                               mjrfContext* ctx,
                                               ModelObjects* model_objs,
                                               SceneObjects* scene_objs);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_COMPAT_SCENE_GEOM_UTIL_H_
