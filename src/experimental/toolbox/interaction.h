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

#ifndef MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_INTERACTION_H_
#define MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_INTERACTION_H_

#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

// The result of a pick operation.
struct PickResult {
  mjtNum point[3] = {0, 0, 0};  // World coordinates
  mjtNum dist = -1;  // Distance from the camera.
  int body = -1;
  int geom = -1;
  int flex = -1;
  int skin = -1;
};

// Returns information about the object (if any) under the mouse cursor.
PickResult Pick(const mjModel* m, const mjData* d, const mjvCamera* camera,
                float x, float y, float aspect_ratio,
                const mjvOption* vis_options);

// Updates the camera according to the requested index using this convention:
//
//  0  : selects the free camera (not defined in the model)
//  1  : selects the tracking camera (also not defined in the model)
//  2+ : selects a camera in the model; e.g. index 2 => model.cam[0];
//
// The function returns the index of the used camera following the same
// convention. Note the returned index may differ from the request if the
// request was invalid (index was out of range or tracking camera was not
// available).
int SetCamera(const mjModel* m, mjvCamera* camera, int request_idx);

// Moves the camera according to the mouse action and relative displacement.
void MoveCamera(const mjModel* m, const mjData* d, mjvCamera* cam,
                mjtMouse action, mjtNum reldx, mjtNum reldy);

void InitPerturb(const mjModel* m, const mjData* d, const mjvCamera* cam,
                 mjvPerturb* pert, mjtPertBit active);

void MovePerturb(const mjModel* m, const mjData* d, const mjvCamera* cam,
                 mjvPerturb* pert, mjtMouse action, mjtNum reldx,
                 mjtNum reldy);

}  // namespace mujoco::toolbox

#endif  // MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_INTERACTION_H_
