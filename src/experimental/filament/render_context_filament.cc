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

#include <array>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32) || defined(__CYGWIN__)
#define NOMINMAX
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include "experimental/filament/compat/mjr_filament_renderer.h"


#if defined(TLS_FILAMENT_CONTEXT)
static thread_local mujoco::MjrFilamentRenderer* g_filament_context = nullptr;
#else
static mujoco::MjrFilamentRenderer* g_filament_context = nullptr;
#endif

static void CheckFilamentContext() {
  if (g_filament_context == nullptr) {
    mju_error("Missing context; did you call mjrf_makeFilamentContext?");
  }
}

struct FilamentAssetResource {
  std::vector<char> data;
};

static std::once_flag g_filament_provider_once;

static std::string JoinPath(std::string_view dir, std::string_view filename) {
  if (dir.empty()) {
    return std::string(filename);
  }
  std::string path(dir);
  const char last = path.back();
  if (last != '/' && last != '\\') {
#if defined(_WIN32) || defined(__CYGWIN__)
    path.push_back('\\');
#else
    path.push_back('/');
#endif
  }
  path.append(filename);
  return path;
}

static std::string GetLibraryDir() {
  void* anchor = reinterpret_cast<void*>(
      reinterpret_cast<uintptr_t>(&mj_forward));
#if defined(_WIN32) || defined(__CYGWIN__)
  HMODULE module = nullptr;
  constexpr DWORD flags = GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                          GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT;
  if (GetModuleHandleExA(flags, reinterpret_cast<LPCSTR>(anchor), &module)) {
    char path[MAX_PATH];
    if (GetModuleFileNameA(module, path, MAX_PATH)) {
      std::string lib_path(path);
      const std::size_t last_slash = lib_path.find_last_of("\\/");
      if (last_slash != std::string::npos) {
        return lib_path.substr(0, last_slash + 1);
      }
    }
  }
#else
  Dl_info info;
  if (dladdr(anchor, &info) && info.dli_fname) {
    std::string lib_path(info.dli_fname);
    const std::size_t last_slash = lib_path.find_last_of('/');
    if (last_slash != std::string::npos) {
      return lib_path.substr(0, last_slash + 1);
    }
  }
#endif
  return "";
}

static void EnsureDefaultFilamentResourceProvider() {
  std::call_once(g_filament_provider_once, [] {
    if (mjp_getResourceProvider("filament:pbr.filamat")) {
      return;
    }

    static mjpResourceProvider provider;
    mjp_defaultResourceProvider(&provider);

    provider.open = [](mjResource* resource) {
      std::string_view name(resource->name);
      const std::size_t prefix_end = name.find(':');
      const std::string_view filename =
          prefix_end == std::string_view::npos
              ? name
              : name.substr(prefix_end + 1);

      std::vector<std::string> asset_dirs;
      if (const char* env_dir = std::getenv("MUJOCO_FILAMENT_ASSETS_DIR")) {
        asset_dirs.emplace_back(env_dir);
      }
      const std::string library_dir = GetLibraryDir();
      if (!library_dir.empty()) {
        asset_dirs.push_back(JoinPath(library_dir, "assets"));
      }
      asset_dirs.emplace_back("assets");

      for (const std::string& dir : asset_dirs) {
        std::ifstream file(JoinPath(dir, filename),
                           std::ios::binary | std::ios::ate);
        if (!file.is_open()) {
          continue;
        }
        const std::streamsize size = file.tellg();
        if (size <= 0) {
          continue;
        }
        auto* asset = new FilamentAssetResource();
        asset->data.resize(static_cast<std::size_t>(size));
        file.seekg(0, std::ios::beg);
        if (!file.read(asset->data.data(), size)) {
          delete asset;
          continue;
        }
        resource->data = asset;
        return static_cast<int>(asset->data.size());
      }
      return 0;
    };
    provider.read = [](mjResource* resource, const void** buffer) {
      auto* asset = static_cast<FilamentAssetResource*>(resource->data);
      if (!asset) {
        return -1;
      }
      *buffer = asset->data.data();
      return static_cast<int>(asset->data.size());
    };
    provider.close = [](mjResource* resource) {
      delete static_cast<FilamentAssetResource*>(resource->data);
      resource->data = nullptr;
    };
    provider.prefix = "filament";

    const int slot = mjp_registerResourceProvider(&provider);
    if (slot < 0 && !mjp_getResourceProvider("filament:pbr.filamat")) {
      mju_warning("Failed to register default Filament resource provider.");
    }
  });
}

template <int N>
static void setf(float (&arr)[N], const std::array<float, N>& values) {
  for (int i = 0; i < N; ++i) {
    arr[i] = values[i];
  }
}


extern "C" {

void mjrf_defaultFilamentConfig(mjrFilamentConfig* config) {
  memset(config, 0, sizeof(mjrFilamentConfig));
}

void mjr_defaultTextureData(mjrTextureData* data) {
  memset(data, 0, sizeof(mjrTextureData));
}

void mjr_defaultTextureConfig(mjrTextureConfig* config) {
  memset(config, 0, sizeof(mjrTextureConfig));
}

void mjr_defaultMeshData(mjrMeshData* data) {
  std::memset(data, 0, sizeof(mjrMeshData));
}

void mjr_defaultSceneParams(mjrSceneParams* params) {
  params->enable_post_processing = true;
  params->enable_reflections = true;
  params->enable_shadows = true;
  params->layer_mask = 0xff;
  params->reflection_layer_mask = 0xff;
}

void mjr_defaultLightParams(mjrLightParams* params) {
  params->type = mjLIGHT_POINT;
  params->texture = nullptr;
  params->color[0] = 0;
  params->color[1] = 0;
  params->color[2] = 0;
  params->intensity = 0.0f;
  params->cast_shadows = true;
  params->range = 10.0f;
  params->spot_cone_angle = 180.f;
  params->bulb_radius = 0.0f;
  params->shadow_map_size = 2048;
  params->vsm_blur_width = 0.0f;
}

void mjr_defaultMaterialTextures(mjrMaterialTextures* textures) {
  textures->color = nullptr;
  textures->normal = nullptr;
  textures->metallic = nullptr;
  textures->roughness = nullptr;
  textures->occlusion = nullptr;
  textures->orm = nullptr;
  textures->emissive = nullptr;
  textures->reflection = nullptr;
}

void mjr_defaultMaterialParams(mjrMaterialParams* params) {
  setf(params->color, {1.f, 1.f, 1.f, 1.f});
  setf(params->segmentation_color, {1, 1, 1, 1});
  setf(params->uv_scale, {1, 1, 1});
  setf(params->uv_offset, {0, 0, 0});
  setf(params->scissor, {0, 0, 0, 0});
  params->emissive = -1.0f;
  params->specular = -1.0f;
  params->glossiness = -1.0f;
  params->metallic = -1.0f;
  params->roughness = -1.0f;
  params->reflectance = 0.0f;
  params->tex_uniform = false;
  params->reflective = false;
}

void mjr_defaultRenderableParams(mjrRenderableParams* params) {
  params->shading_model = mjSHADING_MODEL_SCENE_OBJECT;
  params->cast_shadows = true;
  params->receive_shadows = true;
  params->layer_mask = 0x01;
  params->priority = 4;
  params->blend_order = 0;
}

void mjr_defaultRenderTargetConfig(mjrRenderTargetConfig* config) {
  memset(config, 0, sizeof(mjrRenderTargetConfig));
  config->color_format = mjPIXEL_FORMAT_RGBA8;
  config->depth_format = mjPIXEL_FORMAT_DEPTH32F;
}

void mjr_defaultRenderRequest(mjrRenderRequest* request) {
  memset(request, 0, sizeof(mjrRenderRequest));
}

void mjr_defaultReadPixelsRequest(mjrReadPixelsRequest* request) {
  memset(request, 0, sizeof(mjrReadPixelsRequest));
}

void mjrf_makeFilamentContext(const mjModel* m, mjrContext* con,
                              const mjrFilamentConfig* config) {
  // TODO: Support multiple contexts and multiple threads. For now, we'll just
  // assume a single, global context.
  if (g_filament_context != nullptr) {
    mju_error("Context already exists!");
  }
  EnsureDefaultFilamentResourceProvider();
  g_filament_context = new mujoco::MjrFilamentRenderer(config);
  g_filament_context->Init(m);
}

void mjrf_defaultContext(mjrContext* con) {
  memset(con, 0, sizeof(mjrContext));
}

void mjrf_makeContext(const mjModel* m, mjrContext* con, int fontscale) {
  mjrf_freeContext(con);
  mjrFilamentConfig cfg;
  mjrf_defaultFilamentConfig(&cfg);
  cfg.width = m->vis.global.offwidth;
  cfg.height = m->vis.global.offheight;
  mjrf_makeFilamentContext(m, con, &cfg);
}

void mjrf_freeContext(mjrContext* con) {
  // mjr_freeContext may be called multiple times.
  if (g_filament_context) {
    delete g_filament_context;
    g_filament_context = nullptr;
  }
  mjrf_defaultContext(con);
}

void mjrf_render(mjrRect viewport, mjvScene* scn, const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->Render(viewport, scn);
}

void mjrf_uploadMesh(const mjModel* m, const mjrContext* con, int meshid) {
  CheckFilamentContext();
  g_filament_context->UploadMesh(m, meshid);
}

void mjrf_uploadTexture(const mjModel* m, const mjrContext* con, int texid) {
  CheckFilamentContext();
  g_filament_context->UploadTexture(m, texid);
}

void mjrf_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid) {
  CheckFilamentContext();
  g_filament_context->UploadHeightField(m, hfieldid);
}

void mjrf_setBuffer(int framebuffer, mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->SetFrameBuffer(framebuffer);
}

void mjrf_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                          const mjrContext* con) {
  CheckFilamentContext();
  g_filament_context->ReadPixels(viewport, rgb, depth);
}

uintptr_t mjrf_uploadGuiImage(uintptr_t tex_id, const unsigned char* pixels,
                             int width, int height, int bpp,
                             const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->UploadGuiImage(tex_id, pixels, width, height, bpp);
}

double mjrf_getFrameRate(const mjrContext* con) {
  CheckFilamentContext();
  return g_filament_context->GetFrameRate();
}

void mjrf_updateGui(const mjrContext* con) {
  if (g_filament_context != nullptr) {
    g_filament_context->UpdateGui();
  }
}

}  // extern "C"
