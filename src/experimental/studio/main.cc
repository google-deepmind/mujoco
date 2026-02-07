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
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include <absl/flags/flag.h>
#include "experimental/studio/app.h"

ABSL_FLAG(int, window_width, 1400, "Window width");
ABSL_FLAG(int, window_height, 720, "Window height");
ABSL_FLAG(std::string, model_file, "", "MuJoCo model file.");

static std::vector<std::byte> LoadAsset(std::string_view path) {
  std::string fullpath = std::string("assets/") + std::string(path);
  std::ifstream file(fullpath, std::ios::binary | std::ios::ate);
  if (!file.is_open()) {
    return {};
  }
  std::streampos file_size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<std::byte> buffer(file_size);
  if (!file.read(reinterpret_cast<char*>(buffer.data()), file_size)) {
    return {};
  }
  return buffer;
}

int main(int argc, char** argv, char** envp) {

  const char* home = getenv("HOME");
  const std::string ini_path = std::string(home ? home : ".") + "/.mujoco.ini";

  const int width = absl::GetFlag(FLAGS_window_width);
  const int height = absl::GetFlag(FLAGS_window_height);
  mujoco::studio::App app(width, height, ini_path, LoadAsset);

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
