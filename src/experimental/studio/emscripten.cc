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

#include <emscripten.h>
#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>

#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
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
    std::filesystem::path(filename).filename().string();
    assets_[filename] = std::move(contents);
  }

  const std::string& Get(std::string_view filename) const {
    filename = filename.substr(filename.find_first_of(':') + 1);
    static std::string empty;
    auto it = assets_.find(std::string(filename));
    return it != assets_.end() ? it->second : empty;
  }

 private:
  std::unordered_map<std::string, std::string> assets_;
};

// ---------------------------------------------------------------------------
// HTTP/HTTPS resource fetching via the JS fetch API (uses ASYNCIFY to yield).
// ---------------------------------------------------------------------------

// Fetches a URL using the JS fetch API. Returns a malloc'd buffer and its size.
// The caller is responsible for freeing the buffer. Returns 0 on failure.
EM_ASYNC_JS(int, FetchUrl,
            (const char* url, char** out_data, std::int32_t* out_size), {
              try {
                const urlStr = UTF8ToString(url);
                const response = await fetch(urlStr);
                if (!response.ok) {
                  console.error('Fetch failed: ' + response.status + ' ' +
                                urlStr);
                  return 0;
                }
                const buffer = await response.arrayBuffer();
                const bytes = new Uint8Array(buffer);
                const ptr = _malloc(bytes.length);
                HEAPU8.set(bytes, ptr);
                setValue(out_data, ptr, '*');
                setValue(out_size, bytes.length, 'i32');
                return 1;
              } catch (e) {
                console.error('Fetch error:', e);
                return 0;
              }
            });

// Cache for data fetched via HTTP/HTTPS. Stores the downloaded bytes keyed by
// the resource name (URL) so that read() can return a pointer to the data.
class FetchCache {
 public:
  static FetchCache& Instance() {
    static FetchCache instance;
    return instance;
  }

  // Fetches the URL and stores the result. Returns the size (>0) on success.
  int Fetch(const char* url) {
    char* data = nullptr;
    std::int32_t size = 0;
    if (!FetchUrl(url, &data, &size)) {
      return 0;
    }
    entries_[url] = Entry{UniquePtrWasm<char[]>(data), size};
    return size;
  }

  // Returns pointer and size for a previously fetched URL.
  int Read(const char* url, const void** buffer) {
    auto it = entries_.find(url);
    if (it == entries_.end()) {
      return -1;
    }
    *buffer = it->second.data.get();
    return it->second.size;
  }

  // Frees the data for a URL.
  void Close(const char* url) { entries_.erase(url); }

 private:
  struct FreeDeleter {
    void operator()(void* p) const { std::free(p); }
  };
  template <typename T>
  using UniquePtrWasm = std::unique_ptr<T, FreeDeleter>;

  struct Entry {
    UniquePtrWasm<char[]> data;
    int size;
  };
  std::unordered_map<std::string, Entry> entries_;
};

// ---------------------------------------------------------------------------

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

  mjpResourceProvider resource_provider;
  mjp_defaultResourceProvider(&resource_provider);

  resource_provider.open = [](mjResource* resource) {
    AssetRegistry& r = AssetRegistry::Instance();
    return static_cast<int>(r.Get(resource->name).size());
  };
  resource_provider.read = [](mjResource* resource, const void** buffer) {
    AssetRegistry& r = AssetRegistry::Instance();
    const std::string& contents = r.Get(resource->name);
    *buffer = contents.data();
    return static_cast<int>(contents.size());
  };
  resource_provider.close = [](mjResource* resource) {};

  resource_provider.prefix = "font";
  mjp_registerResourceProvider(&resource_provider);
  resource_provider.prefix = "filament";
  mjp_registerResourceProvider(&resource_provider);

  // Register HTTP/HTTPS resource providers so that models loaded from URLs
  // can automatically fetch referenced assets (meshes, textures, etc.) over
  // the network.
  mjpResourceProvider http_provider;
  mjp_defaultResourceProvider(&http_provider);

  http_provider.open = [](mjResource* resource) {
    return FetchCache::Instance().Fetch(resource->name);
  };
  http_provider.read = [](mjResource* resource, const void** buffer) {
    return FetchCache::Instance().Read(resource->name, buffer);
  };
  http_provider.close = [](mjResource* resource) {
    FetchCache::Instance().Close(resource->name);
  };

  http_provider.prefix = "http";
  mjp_registerResourceProvider(&http_provider);
  http_provider.prefix = "https";
  mjp_registerResourceProvider(&http_provider);

  // Register a "github:" resource provider that resolves
  // github:org/repo/branch/path/to/file.xml to
  // https://raw.githubusercontent.com/org/repo/branch/path/to/file.xml
  mjpResourceProvider github_provider;
  mjp_defaultResourceProvider(&github_provider);

  github_provider.open = [](mjResource* resource) {
    std::string name(resource->name);
    // Strip the "github:" prefix and prepend the raw.githubusercontent URL.
    std::string url =
        "https://raw.githubusercontent.com/" + name.substr(strlen("github:"));
    return FetchCache::Instance().Fetch(url.c_str());
  };
  github_provider.read = [](mjResource* resource, const void** buffer) {
    std::string name(resource->name);
    std::string url =
        "https://raw.githubusercontent.com/" + name.substr(strlen("github:"));
    return FetchCache::Instance().Read(url.c_str(), buffer);
  };
  github_provider.close = [](mjResource* resource) {
    std::string name(resource->name);
    std::string url =
        "https://raw.githubusercontent.com/" + name.substr(strlen("github:"));
    FetchCache::Instance().Close(url.c_str());
  };

  github_provider.prefix = "github";
  mjp_registerResourceProvider(&github_provider);

  g_app = new mujoco::studio::App({
    .width = width,
    .height = height,
    .ini_path = ini_path,
    .gfx_mode = mujoco::platform::GraphicsMode::FilamentWebGl,
  });
  g_app->InitEmptyModel();
}

// Javascript-facing function to load a model from an uploaded file.
void LoadFile(const std::string& filename, const std::string& data) {
  if (!g_app) {
    return;
  }

  std::string content_type;
  if (filename.ends_with(".mjb")) {
    content_type = "application/mjb";
  } else if (filename.ends_with(".mjz")) {
    content_type = "application/zip";
  } else if (filename.ends_with(".zip")) {
    content_type = "application/zip";
  } else if (filename.ends_with(".xml")) {
    content_type = "text/xml";
  } else {
    return;
  }

  const auto ptr = reinterpret_cast<const std::byte*>(data.data());
  g_app->LoadModelFromBuffer({ptr, ptr + data.size()}, content_type, filename);
}

// Javascript-facing function to load a model from a URL.
// The URL is passed directly to LoadModelFromFile, which will use the
// registered HTTP/HTTPS resource providers to fetch the model and any
// referenced assets.
void LoadUrl(const std::string& url) {
  if (!g_app) {
    return;
  }
  g_app->LoadModelFromFile(url);
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
  emscripten::function("loadFile", &LoadFile);
  emscripten::function("loadUrl", &LoadUrl);
  emscripten::function("renderFrame", &RenderFrame);
  emscripten::function("deinit", &Deinit);
}
