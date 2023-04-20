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

#ifndef MUJOCO_SRC_RENDER_RENDER_CONTEXT_H_
#define MUJOCO_SRC_RENDER_RENDER_CONTEXT_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjexport.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>

#if defined(__cplusplus)
extern "C" {
#endif

// builtin display list addresses (relative to base)
enum {
  mjrSPHERE = 0,
  mjrSPHERETOP,
  mjrSPHEREBOTTOM,
  mjrCYLINDER,
  mjrCYLINDEROPEN,
  mjrHAZE,
  mjrBOX,
  mjrCONE,
  mjrNUM
};


// set default mjrContext
MJAPI void mjr_defaultContext(mjrContext* con);

// allocate resources in custom OpenGL context; fontscale is mjtFontScale
MJAPI void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale);

// Change font of existing context.
MJAPI void mjr_changeFont(int fontscale, mjrContext* con);

// Add Aux buffer with given index to context; free previous Aux buffer.
MJAPI void mjr_addAux(int index, int width, int height, int samples, mjrContext* con);

// free resources in custom OpenGL context, set to default
MJAPI void mjr_freeContext(mjrContext* con);

// resize offscreen renderbuffer
MJAPI void mjr_resizeOffscreen(int offwidth, int offheight, mjrContext* con);

// (re) upload texture to GPU
MJAPI void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

// (re) upload mesh to GPU
MJAPI void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

// (re) upload height field to GPU
MJAPI void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid);

#if defined(__cplusplus)
}
#endif

#endif  // MUJOCO_SRC_RENDER_RENDER_CONTEXT_H_
