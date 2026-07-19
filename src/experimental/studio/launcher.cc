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

#include "experimental/studio/launcher.h"

#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <ios>
#include <string>
#include <string_view>
#include <vector>

#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/platform/sys_utils.h"
#include "experimental/studio/app.h"

namespace mujoco::studio {
namespace {

std::string Resolve(std::string_view path) {
  std::string_view subpath = path.substr(path.find(':') + 1);
  std::filesystem::path exe_dir = mujoco::platform::GetModuleDir((void*)&Resolve);
  if (exe_dir.empty()) {
    return std::string("assets/") + std::string(subpath);
  }
  std::filesystem::path resources_dir = exe_dir.parent_path() / "Resources";
  if (std::filesystem::exists(resources_dir / "assets")) {
    return (resources_dir / "assets" / subpath).string();
  }
  return (exe_dir / "assets" / subpath).string();
}

class FileResource {
 public:
  explicit FileResource(const std::string& path)
      : file_(path, std::ios::binary | std::ios::ate) {
    if (!file_.is_open()) {
      mju_warning("Cannot open file %s", path.c_str());
      return;
    }

    size_ = file_.tellg();
    file_.seekg(0, std::ios::beg);
  }

  int Read(const void** buffer) {
    buffer_.resize(size_);
    if (!file_.read(reinterpret_cast<char*>(buffer_.data()), size_)) {
      return 0;
    }
    *buffer = buffer_.data();
    return size_;
  }

  int Size() const { return size_; }

  FileResource(const FileResource&) = delete;
  FileResource& operator=(const FileResource&) = delete;

 private:
  std::ifstream file_;
  std::vector<char> buffer_;
  int size_ = 0;
};

}  // namespace

int LaunchStudio(int argc, char** argv, LauncherConfig config) {
  const char* home = std::getenv("HOME");
  const std::string ini_path = std::string(home ? home : ".") + "/.mujoco.ini";

  mjpResourceProvider resource_provider;
  mjp_defaultResourceProvider(&resource_provider);

  resource_provider.open = [](mjResource* resource) {
    const std::string resolved_path = Resolve(resource->name);
    FileResource* f = new FileResource(resolved_path);
    if (f->Size() == 0) {
      delete f;
      return 0;
    }
    resource->data = f;
    return f->Size();
  };
  resource_provider.read = [](mjResource* resource, const void** buffer) {
    FileResource* f = static_cast<FileResource*>(resource->data);
    return f->Read(buffer);
  };
  resource_provider.close = [](mjResource* resource) {
    delete static_cast<FileResource*>(resource->data);
    resource->data = nullptr;
  };

  resource_provider.prefix = "font";
  mjp_registerResourceProvider(&resource_provider);
  resource_provider.prefix = "filament";
  mjp_registerResourceProvider(&resource_provider);

  if (config.gfx_mode.empty()) {
    const char* display = std::getenv("DISPLAY");
    if (display && strcmp(display, ":20") == 0) {
      config.gfx_mode = "opengl_headless";
    }
  }

  const char* session_type = std::getenv("XDG_SESSION_TYPE");
  const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
  if ((session_type && std::string_view(session_type) == "wayland") ||
      wayland_display) {
    if (config.gfx_mode.empty()) {
      config.gfx_mode = "opengl_headless";
    } else if (config.gfx_mode == "classic" || config.gfx_mode == "opengl") {
      mju_error(
          "Wayland does not support '%s' graphics mode. "
          "Restart with a different graphics mode, or login using X11.",
          config.gfx_mode.c_str());
    }
  }

  mujoco::platform::GraphicsMode gfx_mode =
      mujoco::platform::GraphicsModeFromString(
          config.gfx_mode, mujoco::platform::GraphicsMode::FilamentOpenGl);

  // Use config values if they are set (non-default), otherwise use flags.
  mujoco::studio::App app({
    .width = config.window_width,
    .height = config.window_height,
    .ini_path = ini_path,
    .gfx_mode = gfx_mode,
    .title = config.title,
  });

  if (config.model_file.empty()) {
    app.InitEmptyModel();
  } else {
    app.LoadModelFromFile(config.model_file);
  }

  while (app.Update()) {
    app.BuildGui();
    app.Render();
  }
  return 0;
}

}  // namespace mujoco::studio
