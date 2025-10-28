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

#include "experimental/toolbox/interaction.h"
#include <algorithm>

#include "experimental/toolbox/physics.h"
#include "experimental/toolbox/renderer.h"
#include "experimental/toolbox/window.h"
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

PickResult Pick(float x, float y, Window* window, Renderer* renderer,
                Physics* physics, const mjvOption& vis_options) {
  const float w = static_cast<float>(window->GetWidth());
  const float h = static_cast<float>(window->GetHeight());
  const float aspect_ratio = w / h;

  PickResult result;
  result.body =
      mjv_select(physics->GetModel(), physics->GetData(), &vis_options,
                 aspect_ratio, x, 1.0f - y, &renderer->GetScene(), result.point,
                 &result.geom, &result.flex, &result.skin);
  return result;
}

int SetCamera(const mjModel& model, mjvCamera& camera, int request_idx) {
  // 0 = free, 1 = tracking, 2+ = fixed
  int camera_idx = std::clamp(request_idx, 0, std::max(model.ncam + 1, 0));
  if (camera_idx == 0) {
    camera.type = mjCAMERA_FREE;
  } else if (camera_idx == 1) {
    if (camera.trackbodyid >= 0) {
      camera.type = mjCAMERA_TRACKING;
      camera.fixedcamid = -1;
    } else {
      camera.type = mjCAMERA_FREE;
      camera_idx = 0;
    }
  } else {
    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = camera_idx - 2;
  }

  return camera_idx;
}
}  // namespace mujoco::toolbox
