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

#include <mujoco/mujoco.h>

extern "C" {

void mjr_setAux(int index, const mjrContext* con) {
  mju_error("mjr_setAux not implemented.");
}
void mjr_restoreBuffer(const mjrContext* con) {
  mju_error("mjr_restoreBuffer not implemented.");
}
void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a) {
  mju_error("mjr_rectangle not implemented.");
}
void mjr_blitAux(int index, mjrRect src, int left, int bottom,
                 const mjrContext* con) {
  mju_error("mjr_blitAux not implemented.");
}
void mjr_changeFont(int fontscale, mjrContext* con) {
  mju_error("mjr_changeFont not implemented.");
}
void mjr_addAux(int index, int width, int height, int samples,
                mjrContext* con) {
  mju_error("mjr_addAux not implemented.");
}
void mjr_resizeOffscreen(int width, int height, mjrContext* con) {
  mju_error("mjr_resizeOffscreen not implemented.");
}
void mjr_drawPixels(const unsigned char* rgb, const float* depth,
                    mjrRect viewport, const mjrContext* con) {
  mju_error("mjr_drawPixels not implemented.");
}
void mjr_blitBuffer(mjrRect src, mjrRect dst, int flg_color, int flg_depth,
                    const mjrContext* con) {
  mju_error("mjr_blitBuffer not implemented.");
}
void mjr_text(int font, const char* txt, const mjrContext* con, float x,
              float y, float r, float g, float b) {
  mju_error("mjr_text not implemented.");
}
void mjr_overlay(int font, int gridpos, mjrRect viewport, const char* overlay,
                 const char* overlay2, const mjrContext* con) {
  mju_error("mjr_overlay not implemented.");
}
void mjr_label(mjrRect viewport, int font, const char* txt, float r, float g,
               float b, float a, float rt, float gt, float bt,
               const mjrContext* con) {
  mju_error("mjr_label not implemented.");
}
void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con) {
  mju_error("mjr_figure not implemented.");
}
void mjr_finish() {
  mju_error("mjr_finish not implemented.");
}
int mjr_getError() {
  mju_error("mjr_getError not implemented.");
  return 0;
}
mjrRect mjr_maxViewport(const mjrContext* con) {
  mju_error("mjr_maxViewport not implemented.");
  return mjrRect{};
}
int mjr_findRect(int x, int y, int nrect, const mjrRect* rect) {
  mju_error("mjr_findRect not implemented.");
  return 0;
}

}  // extern "C"
