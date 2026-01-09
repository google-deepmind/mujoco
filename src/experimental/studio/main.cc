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

// Main entry point for the Filament-based MuJoCo renderer.

#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <ios>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <absl/flags/flag.h>
#include <mujoco/mujoco.h>
#include "experimental/studio/app.h"

ABSL_FLAG(int, window_width, 1400, "Window width");
ABSL_FLAG(int, window_height, 720, "Window height");
ABSL_FLAG(std::string, model_file, "", "MuJoCo model file.");

std::string Resolve(std::string_view path) {
  return std::string("assets/") + std::string(path);
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

int main(int argc, char** argv, char** envp) {

  const char* home = getenv("HOME");
  const std::string ini_path = std::string(home ? home : ".") + "/.mujoco.ini";

  mjpResourceProvider resource_provider;
  mjp_defaultResourceProvider(&resource_provider);

  resource_provider.open = [](mjResource* resource) {
    const std::string resolved_path = Resolve(resource->name);
    FileResource* f = new FileResource(resolved_path);
    resource->data = f;
    return f->Size();
  };
  resource_provider.read = [](mjResource* resource, const void** buffer) {
    FileResource* f = static_cast<FileResource*>(resource->data);
    return f->Read(buffer);
  };
  resource_provider.close = [](mjResource* resource) {
    delete static_cast<FileResource*>(resource->data);
  };

  resource_provider.prefix = "font";
  mjp_registerResourceProvider(&resource_provider);
  resource_provider.prefix = "filament";
  mjp_registerResourceProvider(&resource_provider);

  const int width = absl::GetFlag(FLAGS_window_width);
  const int height = absl::GetFlag(FLAGS_window_height);
  mujoco::studio::App app(width, height, ini_path);

  // If the model file is not specified, try to load it from the first argument
  std::string model_file = absl::GetFlag(FLAGS_model_file);
  if (model_file.empty() && argc > 1 && argv[1][0] != '-') model_file = argv[1];
  app.LoadModel(model_file, mujoco::studio::App::ContentType::kFilepath);
  while (app.Update()) {
    app.BuildGui();
    app.Render();
  }
  return 0;
}
