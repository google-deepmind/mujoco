// Copyright 2022 DeepMind Technologies Limited
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

#include "user/user_resource.h"

#include <sys/types.h>
#include <sys/stat.h>

#include <cctype>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32) || defined(__CYGWIN__)
#define NOMINMAX
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <dlfcn.h>
#endif

#include <mujoco/mujoco.h>

#include <mujoco/mjplugin.h>
#include "engine/engine_plugin.h"
#include "user/user_util.h"
#include "user/user_vfs.h"

namespace {

constexpr char kMujocoAssetPrefix[] = "mujoco";
constexpr char kMujocoAssetScheme[] = "mujoco:";
constexpr char kMujocoAssetsDirEnv[] = "MUJOCO_ASSETS_DIR";
constexpr char kMujocoFilamentAssetsDirEnv[] = "MUJOCO_FILAMENT_ASSETS_DIR";

struct MujocoAssetResource {
  std::vector<char> data;
};

bool IsMujocoAssetResourceName(const char* name) {
  if (!name) {
    return false;
  }

  for (int i = 0; kMujocoAssetScheme[i]; ++i) {
    if (std::tolower(static_cast<unsigned char>(name[i])) !=
        kMujocoAssetScheme[i]) {
      return false;
    }
  }
  return true;
}

std::string JoinPath(std::string_view dir, std::string_view filename) {
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

std::string GetLibraryDir() {
#if defined(__EMSCRIPTEN__)
  return "";
#else
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
#endif
}

std::string_view ResourceFilename(std::string_view name) {
  const std::size_t prefix_end = name.find(':');
  return prefix_end == std::string_view::npos ? name
                                              : name.substr(prefix_end + 1);
}

bool IsSafeAssetFilename(std::string_view filename) {
  if (filename.empty() || filename.front() == '/' || filename.front() == '\\') {
    return false;
  }

  std::size_t segment_start = 0;
  while (segment_start <= filename.size()) {
    std::size_t segment_end = filename.find_first_of("/\\", segment_start);
    if (segment_end == std::string_view::npos) {
      segment_end = filename.size();
    }
    std::string_view segment =
        filename.substr(segment_start, segment_end - segment_start);
    if (segment == "..") {
      return false;
    }
    segment_start = segment_end + 1;
  }
  return true;
}

int OpenMujocoAssetResource(mjResource* resource) {
  const std::string_view filename = ResourceFilename(resource->name);
  if (!IsSafeAssetFilename(filename)) {
    return 0;
  }

  std::vector<std::string> asset_dirs;
  if (const char* env_dir = std::getenv(kMujocoAssetsDirEnv)) {
    asset_dirs.emplace_back(env_dir);
  }
  if (const char* env_dir = std::getenv(kMujocoFilamentAssetsDirEnv)) {
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
    auto* asset = new MujocoAssetResource();
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
}

int ReadMujocoAssetResource(mjResource* resource, const void** buffer) {
  auto* asset = static_cast<MujocoAssetResource*>(resource->data);
  if (!asset) {
    return -1;
  }
  *buffer = asset->data.data();
  return static_cast<int>(asset->data.size());
}

void CloseMujocoAssetResource(mjResource* resource) {
  delete static_cast<MujocoAssetResource*>(resource->data);
  resource->data = nullptr;
}

void EnsureDefaultMujocoAssetProvider() {
  static std::once_flag once;
  std::call_once(once, [] {
    if (mjp_getResourceProvider("mujoco:asset")) {
      return;
    }

    mjpResourceProvider provider;
    mjp_defaultResourceProvider(&provider);
    provider.prefix = kMujocoAssetPrefix;
    provider.open = OpenMujocoAssetResource;
    provider.read = ReadMujocoAssetResource;
    provider.close = CloseMujocoAssetResource;

    const int slot = mjp_registerResourceProvider(&provider);
    if (slot < 0 && !mjp_getResourceProvider("mujoco:asset")) {
      mju_warning("Failed to register default MuJoCo asset resource provider.");
    }
  });
}

}  // namespace

mjResource* mju_openResource(const char* dir, const char* name,
                             const mjVFS* vfs, char* error, size_t nerror) {
  if (IsMujocoAssetResourceName(name)) {
    EnsureDefaultMujocoAssetProvider();
  }

  // TODO: Update API to use non-const pointer. Unfortunately, while this is
  // ABI stable, it will cause compiler errors in user code that is const
  // correct.
  mjVFS* non_const_vfs = const_cast<mjVFS*>(vfs);

  // TODO: Eventually, we should make `vfs` a required argument. In the
  // meantime, when passing in a nullptr VFS, we will create a VFS dynamically
  // that self-destructs when the resource is closed (or if the resource could
  // not be opened).
  if (non_const_vfs == nullptr) {
    mjVFS* local_vfs = (mjVFS*)mju_malloc(sizeof(mjVFS));
    mj_defaultVFS(local_vfs);
    mujoco::user::VFS::Upcast(local_vfs)->SetToSelfDestruct([=]() {
      mj_deleteVFS(local_vfs);
      mju_free(local_vfs);
    });

    non_const_vfs = local_vfs;
  }

  mjResource* resource =
      mujoco::user::VFS::Upcast(non_const_vfs)->Open(dir ? dir : "", name);

  if (error) {
    if (resource) {
      error[0] = '\0';
    } else {
      std::snprintf(error, nerror, "Error opening file '%s'", name);
    }
  }

  return resource;
}

void mju_closeResource(mjResource* resource) {
  if (resource && resource->vfs) {
    mujoco::user::VFS::Upcast(resource->vfs)->Close(resource);
  }
}

int mju_readResource(mjResource* resource, const void** buffer) {
  if (resource && resource->vfs) {
    return mujoco::user::VFS::Upcast(resource->vfs)->Read(resource, buffer);
  }
  return -1;  // default (error reading bytes)
}

void mju_getResourceDir(mjResource* resource, const char** dir, int* ndir) {
  *dir = nullptr;
  *ndir = 0;

  if (resource && resource->name) {
    // ensure prefix is included even if there is no separator in the
    // resource name
    int prefix_len = 0;
    const mjpResourceProvider* provider = resource->provider;
    if (provider && provider->prefix) {
      prefix_len = strlen(provider->prefix) + 1;
    }

    *dir = resource->name;
    *ndir = prefix_len;
    for (int i = prefix_len; resource->name[i]; ++i) {
      if (resource->name[i] == '/' || resource->name[i] == '\\') {
        *ndir = i + 1;
      }
    }
  }
}

int mju_isModifiedResource(const mjResource* resource, const char* timestamp) {
  if (resource && resource->provider && resource->provider->modified) {
    return resource->provider->modified(resource, timestamp);
  }
  return 1;  // default (assume modified)
}

mjSpec* mju_decodeResource(mjResource* resource, const char* content_type, const mjVFS* vfs) {
  const mjpDecoder* decoder = nullptr;
  if (content_type) {
    decoder = mjp_findDecoder(resource, content_type);
  } else {
    decoder = mjp_findDecoder(resource, mjuu_extToContentType(resource->name).c_str());
  }
  if (!decoder) {
    mju_warning("Could not find decoder for resource '%s'", resource->name);
    return nullptr;
  }
  return decoder->decode(resource, vfs);
}
