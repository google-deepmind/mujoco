// Copyright 2021 DeepMind Technologies Limited
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

#ifndef THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL3_
#define THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL3_

#include <mujoco/mjexport.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>

#if defined(__cplusplus)
extern "C" {
#endif

// 3D rendering
MJAPI void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con);

// call glFinish
MJAPI void mjr_finish(void);

// call glGetError and return result
MJAPI int mjr_getError(void);

#if defined(__cplusplus)
}
#endif
#endif  // THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL3_
