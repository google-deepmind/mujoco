// Copyright 2016 Svetoslav Kolev
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

// The box-box collider as it stood before the separating-axis rewrite,
// preserved as a test-only fixture. The differential tests in
// engine_collision_box_test.cc measure the rewrite against it; nothing in the
// engine links this file.

#ifndef MUJOCO_TEST_ENGINE_BOXBOX_LEGACY_H_
#define MUJOCO_TEST_ENGINE_BOXBOX_LEGACY_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtype.h>

#ifdef __cplusplus
extern "C" {
#endif

// pre-rewrite mjc_BoxBox: SAT axis search, three edge-edge candidate generators
// with count-dependent acceptance, midpoint positions, and an outside-box
// removal filter
int mjc_BoxBoxLegacy(const mjModel* m, mjData* d, mjPreContact* con, int g1,
                     int g2, mjtNum margin);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_TEST_ENGINE_BOXBOX_LEGACY_H_
