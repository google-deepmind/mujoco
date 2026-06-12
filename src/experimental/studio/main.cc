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

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <ios>
#include <iosfwd>
#include <string>
#include <string_view>
#include <vector>

#include <cstdio>

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/hal/graphics_mode.h"
#include "experimental/studio/app.h"
#include "experimental/studio/llm/llm_claude.h"

ABSL_FLAG(int, window_width, 1400, "Window width");
ABSL_FLAG(int, window_height, 720, "Window height");
ABSL_FLAG(std::string, model_file, "", "MuJoCo model file.");
ABSL_FLAG(std::string, gfx, "", "Graphics API");
ABSL_FLAG(std::string, screenshot, "",
          "If set, render headless and write the framebuffer to this path as a "
          "binary PPM once the model has loaded, then exit.");
ABSL_FLAG(int, screenshot_frame, 30,
          "Frame index at which the screenshot is captured.");
ABSL_FLAG(std::string, capture_gif, "",
          "If set, render headless and write one framebuffer PPM per frame "
          "(frame_%04d.ppm) into this directory while running the scripted UI "
          "capture, then exit. Assemble into a GIF with e.g. ImageMagick.");
ABSL_FLAG(int, capture_frames, 200, "Number of frames to capture.");
ABSL_FLAG(std::string, capture_script, "tools",
          "Which scripted interaction to record: 'tools' (rail/palette window "
          "toggling) or 'llm' (ask the LLM a question in the Ctrl+P box).");
ABSL_FLAG(std::string, llm_probe, "",
          "If set, send this prompt to the real Claude provider (using "
          "ANTHROPIC_API_KEY) and print the reply, then exit. Headless probe to "
          "verify the live connection without launching the GUI.");

std::string Resolve(std::string_view path) {
  std::string_view subpath = path.substr(path.find(':') + 1);
  return std::string("assets/") + std::string(subpath);
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
  absl::ParseCommandLine(argc, argv);

  // Headless probe of the live Claude provider (no window/graphics needed).
  const std::string llm_probe = absl::GetFlag(FLAGS_llm_probe);
  if (!llm_probe.empty()) {
    std::string key = mujoco::studio::ClaudeProvider::KeyFromEnv();
    if (key.empty()) {
      std::fprintf(stderr, "[llm_probe] ANTHROPIC_API_KEY is not set.\n");
      return 1;
    }
    mujoco::studio::ClaudeProvider provider(std::move(key));
    mujoco::studio::LlmResult r = provider.Send(
        "You are a terse assistant. Answer in one short sentence.",
        {{"user", llm_probe}}, /*tools=*/{},
        [](const std::string&, const std::string&) { return std::string(); });
    if (r.ok) {
      std::fprintf(stderr, "[llm_probe] OK: %s\n", r.text.c_str());
      return 0;
    }
    std::fprintf(stderr, "[llm_probe] ERROR: %s\n", r.error.c_str());
    return 2;
  }

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

  std::string gfx = absl::GetFlag(FLAGS_gfx);

  // Screenshot capture reads the framebuffer back from CPU memory, which only
  // happens in a headless graphics mode. Default to headless OpenGL if the
  // caller requested a screenshot without specifying a graphics mode.
  const std::string screenshot = absl::GetFlag(FLAGS_screenshot);
  const std::string capture_gif = absl::GetFlag(FLAGS_capture_gif);
  if ((!screenshot.empty() || !capture_gif.empty()) && gfx.empty()) {
    gfx = "opengl_headless";
  }

  const char* session_type = std::getenv("XDG_SESSION_TYPE");
  const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
  if ((session_type && std::string_view(session_type) == "wayland") ||
      wayland_display) {
    if (gfx.empty()) {
      gfx = "opengl_headless";
    } else if (gfx == "classic" || gfx == "opengl") {
      mju_error(
          "Wayland does not support '%s' graphics mode. "
          "Restart with a different graphics mode, or login using X11.",
          gfx.c_str());
    }
  }

  mujoco::platform::GraphicsMode gfx_mode =
      mujoco::platform::GraphicsModeFromString(
          gfx, mujoco::platform::GraphicsMode::FilamentOpenGl);

  const int width = absl::GetFlag(FLAGS_window_width);
  const int height = absl::GetFlag(FLAGS_window_height);
  mujoco::studio::App app({
    .width = width,
    .height = height,
    .ini_path = ini_path,
    .gfx_mode = gfx_mode,
    .screenshot_path = screenshot,
    .screenshot_frame = absl::GetFlag(FLAGS_screenshot_frame),
  });

  // If the model file is not specified, try to load it from the first argument
  std::string model_file = absl::GetFlag(FLAGS_model_file);
  if (model_file.empty() && argc > 1 && argv[1][0] != '-') model_file = argv[1];

  if (model_file.empty()) {
    app.InitEmptyModel();
  } else {
    app.LoadModelFromFile(model_file);
  }

  // Scripted GIF capture: run the UI script headless, writing one PPM/frame.
  if (!capture_gif.empty()) {
    const std::string script = absl::GetFlag(FLAGS_capture_script);
    const auto capture_script = (script == "llm")
                                    ? mujoco::studio::CaptureScript::kLlm
                                    : mujoco::studio::CaptureScript::kTools;
    app.StartCapture(capture_gif, absl::GetFlag(FLAGS_capture_frames),
                     capture_script);
    while (app.Update() && app.capture_active()) {
      app.BuildGui();
      app.Render();
      app.SaveCaptureFrame();
    }
    return 0;
  }

  while (app.Update()) {
    app.BuildGui();
    app.Render();
  }
  return 0;
}
