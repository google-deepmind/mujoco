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

#include "experimental/filament/render_context_filament.h"

#include <cstdint>
#include <cstring>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/filament/filament_context.h"


#if defined(TLS_FILAMENT_CONTEXT)
static thread_local mujoco::FilamentContext* g_filament_context = nullptr;
#else
static mujoco::FilamentContext* g_filament_context = nullptr;
#endif

static void CheckFilamentContext() {
  if (g_filament_context == nullptr) {
    mju_error("Missing context; did you call mjr_makeFilamentContext?");
  }
}

extern "C" {

void mjr_defaultFilamentConfig(mjrFilamentConfig* config) {
  memset(config, 0, sizeof(mjrFilamentConfig));
}

void mjr_makeFilamentContext(const mjModel* m, mjrContext* con,
                             const mjrFilamentConfig* config) {
  // TODO: Support multiple contexts and multiple threads. For now, we'll just
  // assume a single, global context.
  if (g_filament_context != nullptr) {
    mju_error("Context already exists!");
  }
  g_filament_context = new mujoco::FilamentContext(config, m, con);
}

void mjr_defaultContext(mjrContext* con) { memset(con, 0, sizeof(mjrContext)); }

void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale) {
  mjr_freeContext(con);
  mjrFilamentConfig cfg;
  mjr_defaultFilamentConfig(&cfg);
  mjr_makeFilamentContext(m, con, &cfg);
}

void mjr_freeContext(mjrContext* con) {
  // mjr_freeContext may be called multiple times.
  if (g_filament_context) {
    delete g_filament_context;
    g_filament_context = nullptr;
  }
  mjr_defaultContext(con);
}

void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->Render(viewport, scn, con);
}

void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid) {
  CheckFilamentContext();
  g_filament_context->UploadMesh(m, meshid);
}

void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid) {
  CheckFilamentContext();
  g_filament_context->UploadTexture(m, texid);
}

void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid) {
  CheckFilamentContext();
  g_filament_context->UploadHeightField(m, hfieldid);
}

void mjr_setBuffer(int framebuffer, mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->SetFrameBuffer(framebuffer);
}

void mjr_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                          const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->ReadPixels(viewport, rgb, depth);
}

uintptr_t mjr_uploadGuiImage(uintptr_t tex_id, const unsigned char* pixels,
                             int width, int height, int bpp,
                             const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->UploadGuiImage(tex_id, pixels, width, height, bpp);
}

double mjr_getFrameRate(const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->GetFrameRate();
}

void mjr_updateGui(const mjrContext* con) {
  if (g_filament_context != nullptr) {
    g_filament_context->UpdateGui();
  }
}

}  // extern "C"
