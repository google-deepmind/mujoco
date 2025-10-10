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
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <ios>

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

// Default asset loader to use when calling mjr_makeContext. This is only
// intended for basic backwards compatibility with the existing mjr_makeContext
// API. If this doesn't work as expected, you should be calling
// mjr_makeFilamentContext instead and provide your own asset loading callbacks.
// Returns 0 on success and non-zero to indicate an error.
static int DefaultLoadAsset(const char* asset_filename, void* user_data,
                             unsigned char** contents, uint64_t* out_size) {
  std::filesystem::path full_path;
  full_path.append("filament/assets/data");
  full_path.append(asset_filename);

  std::ifstream file(full_path, std::ios::binary);
  if (!file) {
    mju_error("File does not exist: %s", full_path.c_str());
    return 1;
  }

  file.seekg(0, std::ios::end);
  *out_size = static_cast<uint64_t>(file.tellg());
  if (*out_size == 0) {
    mju_error("File is empty: %s", full_path.c_str());
    return 1;
  }
  file.seekg(0, std::ios::beg);

  *contents = (unsigned char*)malloc(*out_size);
  file.read((char*)*contents, *out_size);
  file.close();
  return 0;
}

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
  cfg.load_asset = DefaultLoadAsset;
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

void mjr_uploadFont(unsigned char* pixels, int width, int height, int bpp,
                    int id, const mjrContext* con) {
  CheckFilamentContext();
  if (bpp != 4) {
    mju_error("Only 4bpp fonts are supported, got %d", bpp);
  }
  g_filament_context->UploadFont(pixels, width, height, id);
}

}  // extern "C"
