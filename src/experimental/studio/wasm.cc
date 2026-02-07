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

// Main entry point for the Filament-based MuJoCo web app.

#include <cstddef>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>
#include <utility>

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "experimental/studio/app.h"

// Global app instance. Lifetime is controlled by Init/Deinit calls which are
// triggered by Javascript.
mujoco::studio::App* g_app = nullptr;

// Static registry of assets that are loaded in JSON before the main App is
// initialized.
class AssetRegistry {
 public:
  // Returns the singleton instance of the registry.
  static AssetRegistry& Instance() {
    static AssetRegistry instance;
    return instance;
  }

  // Registers asset contents with the given filename.
  void RegisterAsset(std::string filename, std::string contents) {
    assets_[filename] = std::move(contents);
  }

  // Returns the contents of the given asset by name.
  std::vector<std::byte> LoadAsset(std::string_view filename) {
    if (auto it = assets_.find(std::string(filename)); it != assets_.end()) {
      const std::byte* begin = reinterpret_cast<const std::byte*>(it->second.data());
      const std::byte* end = begin + it->second.size();
      return std::vector<std::byte>(begin, end);
    }
    return {};
  }

 private:
  std::unordered_map<std::string, std::string> assets_;
};

static std::vector<std::byte> LoadAsset(std::string_view filename) {
  return AssetRegistry::Instance().LoadAsset(filename);
}

// Javascript-facing function to register an asset.
void RegisterAsset(std::string filename, std::string contents) {
  AssetRegistry::Instance().RegisterAsset(std::move(filename),
                                        std::move(contents));
}

// Javascript-facing function to initialize the app.
void Init() {
  // Note: dimensions do not matter as window will be resized to fit canvas.
  const int width = 100;
  const int height = 100;
  const std::string ini_path = "";
  g_app = new mujoco::studio::App(width, height, ini_path, LoadAsset);
  g_app->LoadModel("", mujoco::studio::App::ContentType::kModelXml);
}

// Javascript-facing function to load a model from a MJB file.
void LoadMjb(const std::string& src) {
  if (g_app) {
    g_app->LoadModel(src, mujoco::studio::App::ContentType::kModelMjb);
  }
}

// Javascript-facing function to load a model from an XML file.
void LoadXml(const std::string& src) {
  if (g_app) {
    g_app->LoadModel(src, mujoco::studio::App::ContentType::kModelXml);
  }
}

// Javascript-facing function to render a single frame.
void RenderFrame() {
  if (g_app) {
    if (g_app->Update()) {
      g_app->BuildGui();
      g_app->Render();
    }
  }
}

// Javascript-facing function to deinitialize the app.
void Deinit() {
  delete g_app;
  g_app = nullptr;
}

EMSCRIPTEN_BINDINGS(studio_bindings) {
  emscripten::function("registerAsset", &RegisterAsset);
  emscripten::function("init", &Init);
  emscripten::function("loadMjb", &LoadMjb);
  emscripten::function("loadXml", &LoadXml);
  emscripten::function("renderFrame", &RenderFrame);
  emscripten::function("deinit", &Deinit);
}
