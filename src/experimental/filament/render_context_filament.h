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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_

#include <cstdint>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#if defined(__cplusplus)
extern "C" {
#endif

typedef enum mjtGraphicsApi_ {  // backend graphics API to use
  mjGFX_DEFAULT        = 0,     // default based on platform
  mjGFX_OPENGL,                 // OpenGL (desktop)
  mjGFX_VULKAN                  // Vulkan
} mjtGraphicsApi;

struct mjrFilamentConfig {
  // "Loads" an asset, returning its contents in `out` and size in `out_size`.
  // Returns 0 on success and non-zero to indicate an error. The caller must
  // free `out`.
  typedef int (*load_asset_fn)(const char* path, void* user_data,
                              unsigned char** out, uint64_t* out_size);

  // Used to load filament assets (e.g. materials, image-based lights, etc.).
  load_asset_fn load_asset;
  void* load_asset_user_data;

  // The native window handle into which we can render directly.
  void* native_window;

  // The backend graphics API to use.
  int graphics_api;

  // Whether or not to enable GUI rendering.
  bool enable_gui;
};

void mjr_defaultFilamentConfig(mjrFilamentConfig* config);

void mjr_makeFilamentContext(const mjModel* m, mjrContext* con,
                             const mjrFilamentConfig* config);

MJAPI void mjr_defaultContext(mjrContext* con);

MJAPI void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale);

MJAPI void mjr_freeContext(mjrContext* con);

MJAPI void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con);

MJAPI void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

MJAPI void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

MJAPI void mjr_setBuffer(int framebuffer, mjrContext* con);

MJAPI void mjr_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                          const mjrContext* con);

double mjr_getFrameRate(const mjrContext* con);

uintptr_t mjr_uploadGuiImage(uintptr_t tex_id, const unsigned char* pixels,
                             int width, int height, int bpp,
                             const mjrContext* con);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_
