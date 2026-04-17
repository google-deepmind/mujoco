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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAW_MODE_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAW_MODE_H_

namespace mujoco {

// The different modes that can be used to render the scene.
enum class DrawMode {
  // Render the scene with "normal" colors and lighting.
  Color,
  // Render the scene as a grayscale depth map.
  Depth,
  // Render each object with a unique, uniform (flat) color regardless of
  // lighting and texture.
  Segmentation,
};

static constexpr int kNumDrawModes = 3;

}  // namespace mujoco


#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_FILAMENT_DRAW_MODE_H_
