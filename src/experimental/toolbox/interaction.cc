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

#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

PickResult Pick(const mjModel* m, const mjData* d, const mjvCamera* camera,
                float x, float y, float aspect_ratio, const mjvScene* scene,
                const mjvOption* vis_options) {
  PickResult result;
  result.body =
      mjv_select(m, d, vis_options, aspect_ratio, x, 1.0f - y, scene,
                 result.point, &result.geom, &result.flex, &result.skin);
  return result;
}

int SetCamera(const mjModel* m, mjvCamera* camera, int request_idx) {
  // 0 = free, 1 = tracking, 2+ = fixed
  int camera_idx = std::clamp(request_idx, 0, std::max(m->ncam + 1, 0));
  if (camera_idx == 0) {
    camera->type = mjCAMERA_FREE;
  } else if (camera_idx == 1) {
    if (camera->trackbodyid >= 0) {
      camera->type = mjCAMERA_TRACKING;
      camera->fixedcamid = -1;
    } else {
      camera->type = mjCAMERA_FREE;
      camera_idx = 0;
    }
  } else {
    camera->type = mjCAMERA_FIXED;
    camera->fixedcamid = camera_idx - 2;
  }

  return camera_idx;
}
}  // namespace mujoco::toolbox
