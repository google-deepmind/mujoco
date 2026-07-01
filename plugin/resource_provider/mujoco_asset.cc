// Copyright 2026 DeepMind Technologies Limited
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

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#if defined(_WIN32) || defined(__CYGWIN__)
#define NOMINMAX
#include <windows.h>
#elif !defined(__EMSCRIPTEN__)
#include <dlfcn.h>
#endif

#include <mujoco/mjplugin.h>
#include <mujoco/mujoco.h>

namespace {

constexpr char kMujocoAssetPrefix[] = "mujoco";
constexpr char kMujocoAssetsDirEnv[] = "MUJOCO_ASSETS_DIR";
constexpr char kMujocoFilamentAssetsDirEnv[] = "MUJOCO_FILAMENT_ASSETS_DIR";

struct MujocoAssetResource {
  std::vector<char> data;
};

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

}  // namespace

mjPLUGIN_LIB_INIT(mujoco_asset_resource_provider) {
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
}
