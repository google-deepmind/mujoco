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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_IMGUI_EDITOR_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_IMGUI_EDITOR_H_

#include "experimental/filament/filament/scene_view.h"

namespace mujoco {

// Generates a ImGui Window for the given scene views.
void DrawGui(SceneView* scene_views);

}  // namespace mujoco

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_IMGUI_EDITOR_H_
