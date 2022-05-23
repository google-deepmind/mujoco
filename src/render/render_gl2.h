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

#ifndef THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL2_
#define THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL2_

#include <mujoco/mjexport.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>

#if defined(__cplusplus)
extern "C" {
#endif

// Make con->currentBuffer current again.
MJAPI void mjr_restoreBuffer(const mjrContext* con);

// actual draw text, do not initialize OpenGL options
void mjr_textActual(int font, const char* txt, const mjrContext* con,
                    float x, float y, float z, float r, float g, float b);

// set OpenGL framebuffer for rendering: mjFB_WINDOW or mjFB_OFFSCREEN
//  if only one buffer is available, set that buffer and ignore framebuffer argument
MJAPI void mjr_setBuffer(int framebuffer, mjrContext* con);

// read pixels from current OpenGL framebuffer to client buffer
//  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
MJAPI void mjr_readPixels(unsigned char* rgb, float* depth,
                          mjrRect viewport, const mjrContext* con);

// draw pixels from client buffer to current OpenGL framebuffer
//  viewport is in OpenGL framebuffer; client buffer starts at (0,0)
MJAPI void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                          mjrRect viewport, const mjrContext* con);

// blit from src viewpoint in current framebuffer to dst viewport in other framebuffer
//  if src, dst have different size and flg_depth==0, color is interpolated with GL_LINEAR
MJAPI void mjr_blitBuffer(mjrRect src, mjrRect dst,
                          int flg_color, int flg_depth, const mjrContext* con);

// Set Aux buffer for custom OpenGL rendering (call restoreBuffer when done).
MJAPI void mjr_setAux(int index, const mjrContext* con);

// Blit from Aux buffer to con->currentBuffer.
MJAPI void mjr_blitAux(int index, mjrRect src, int left, int bottom,
                       const mjrContext* con);

// draw text at (x,y) in relative coordinates; font is mjtFont
MJAPI void mjr_text(int font, const char* txt, const mjrContext* con,
                    float x, float y, float r, float g, float b);

// draw text overlay; font is mjtFont; gridpos is mjtGridPos
MJAPI void mjr_overlay(int font, int gridpos, mjrRect viewport,
                       const char* overlay, const char* overlay2, const mjrContext* con);

// get maximum viewport for active buffer
MJAPI mjrRect mjr_maxViewport(const mjrContext* con);

// draw rectangle
MJAPI void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a);

// draw rectangle with centered text
MJAPI void mjr_label(mjrRect viewport, int font, const char* txt,
                     float r, float g, float b, float a, float rt, float gt, float bt,
                     const mjrContext* con);

// draw 2D figure
MJAPI void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con);

#if defined(__cplusplus)
}
#endif
#endif  // THIRD_PARTY_MUJOCO_SRC_RENDER_RENDER_GL2_
