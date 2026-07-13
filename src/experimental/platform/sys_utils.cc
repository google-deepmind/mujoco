// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "experimental/platform/sys_utils.h"

#include <filesystem>  // NOLINT(build/c++17)
#include <string>
#include <system_error>
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #include <dlfcn.h>
#endif

namespace mujoco::platform {

std::string GetModuleDir(void* addr) {
#if defined(_WIN32) || defined(__CYGWIN__)
  HMODULE hModule = NULL;
  if (GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                         GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                         (LPCSTR)addr, &hModule)) {
    char path[MAX_PATH];
    DWORD written = GetModuleFileNameA(hModule, path, sizeof(path));
    if (written > 0 && written < sizeof(path)) {
      return std::filesystem::path(path).parent_path().string();
    }
  }
  return "";
#else
  Dl_info info;
  if (dladdr(addr, &info) != 0 && info.dli_fname != nullptr) {
    std::filesystem::path p(info.dli_fname);
    if (p.is_absolute()) {
      return p.parent_path().string();
    }
    std::error_code ec;
    std::filesystem::path cwd = std::filesystem::current_path(ec);
    if (!ec) {
      return (cwd / p).parent_path().string();
    }
    return p.parent_path().string();
  }
  return "";
#endif
}

}  // namespace mujoco::platform
