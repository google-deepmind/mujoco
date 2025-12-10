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

#include "experimental/platform/window.h"

#include <string>
#include <string_view>

#include <SDL.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_hints.h>
#include <SDL_syswm.h>
#include <SDL_version.h>
#include <SDL_video.h>
#include <backends/imgui_impl_sdl2.h>
#include <imgui.h>
#include "experimental/platform/helpers.h"
#include <mujoco/mujoco.h>

// Because X11/Xlib.h defines Status.
#ifdef Status
#undef Status
#endif

#if defined(__APPLE__)
extern void* GetNativeWindowOsx(void* window);
#endif

namespace mujoco::platform {

static void InitImGui(SDL_Window* window, const LoadAssetFn& load_asset_fn,
                      bool load_fonts, bool build_fonts) {
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = nullptr;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOther(window);

  if (load_fonts) {
    // Note: fonts are stored statically because they aren't actually loaded
    // until the fonts are built.
    ImFontConfig main_cfg;
    static auto main_font = load_asset_fn("OpenSans-Regular.ttf");
    io.Fonts->AddFontFromMemoryTTF(main_font.data(), main_font.size(), 20.f,
                                  &main_cfg);

    ImFontConfig icon_cfg;
    icon_cfg.MergeMode = true;
    static auto icon_font = load_asset_fn("fontawesome-webfont.ttf");
    constexpr ImWchar icon_ranges[] = {0xf000, 0xf3ff, 0x000};
    io.Fonts->AddFontFromMemoryTTF(icon_font.data(), icon_font.size(), 14.f,
                                  &icon_cfg, icon_ranges);

    if (build_fonts) {
      io.Fonts->Build();
    }
  }
}

Window::Window(std::string_view title, int width, int height, Config config,
               const LoadAssetFn& load_asset_fn)
    : width_(width), height_(height), config_(config) {
  SDL_SetHint(SDL_HINT_FRAMEBUFFER_ACCELERATION, "1");
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
  SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 16);
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
    mju_error("Error initializing SDL: %s", SDL_GetError());
  }

  int window_flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;

  if (!config_.enable_keyboard) {
    SDL_EventState(SDL_TEXTINPUT, SDL_DISABLE);
    SDL_EventState(SDL_KEYDOWN, SDL_DISABLE);
    SDL_EventState(SDL_KEYUP, SDL_DISABLE);
  }

  RenderConfig render_config = config_.render_config;
  if (render_config == kFilamentVulkan) {
    window_flags |= SDL_WINDOW_VULKAN;
  } else if (render_config == kFilamentWebGL) {
    window_flags |= SDL_WINDOW_OPENGL;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  } else if (render_config == kClassicOpenGL || render_config == kFilamentOpenGL) {
    window_flags |= SDL_WINDOW_OPENGL;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  } else {
    mju_error("Unsupported window config: %d", render_config);
  }

  sdl_window_ =
      SDL_CreateWindow(title.data(), SDL_WINDOWPOS_UNDEFINED,
                       SDL_WINDOWPOS_UNDEFINED, width, height, window_flags);
  if (!sdl_window_) {
    mju_error("Error creating window: %s", SDL_GetError());
  }

  InitImGui(sdl_window_, load_asset_fn, config.load_fonts,
            (render_config != kClassicOpenGL));

  if (render_config == kFilamentWebGL || render_config == kClassicOpenGL) {
    SDL_GLContext gl_context = SDL_GL_CreateContext(sdl_window_);
    SDL_GL_MakeCurrent(sdl_window_, gl_context);
  }

  SDL_SysWMinfo wmi;
  SDL_VERSION(&wmi.version);
  SDL_GetWindowWMInfo(sdl_window_, &wmi);

  #if defined(__linux__)
    native_window_ = reinterpret_cast<void*>(wmi.info.x11.window);
  #elif defined(__WIN32__)
    native_window_ = reinterpret_cast<void*>(wmi.info.win.window);
  #elif defined(__APPLE__)
    native_window_ =
        GetNativeWindowOsx(reinterpret_cast<void*>(wmi.info.cocoa.window));
  #endif

  int drawable_width = width;
  int drawable_height = height;
  SDL_GL_GetDrawableSize(sdl_window_, &drawable_width, &drawable_height);
  scale_ = (float)drawable_width / (float)width_;
}

Window::~Window() {
  SDL_DestroyWindow(sdl_window_);
  SDL_Quit();
}

void Window::SetTitle(std::string_view title) {
  SDL_SetWindowTitle(sdl_window_, title.data());
}

std::string Window::GetDropFile() {
  std::string tmp;
  std::swap(tmp, drop_file_);
  return tmp;
}

Window::Status Window::NewFrame() {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);

    if (event.type == SDL_QUIT) {
      should_exit_ = true;
    } else if (event.type == SDL_APP_WILLENTERBACKGROUND) {
      should_exit_ = true;
    } else if (event.type == SDL_WINDOWEVENT) {
      if (event.window.event == SDL_WINDOWEVENT_RESIZED) {
        SDL_GetWindowSize(sdl_window_, &width_, &height_);
        int drawable_width = width_;
        int drawable_height = height_;
        SDL_GL_GetDrawableSize(sdl_window_, &drawable_width, &drawable_height);
        scale_ = (float)drawable_width / (float)width_;
      }
    } else if (event.type == SDL_DROPFILE) {
      drop_file_ = event.drop.file;
    }
  }

  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();

  return should_exit_ ? kQuitting : kRunning;
}

void Window::EndFrame() {
  // We use ImGui for input management in addition to GUI rendering so its
  // important to call ImGui::EndFrame even if we don't call ImGui::Render.
  // Note ImGui::Render internally calls ImGui::EndFrame, but so long as
  // ImGui::NewFrame has been called, ImGui::EndFrame may be called multiple
  // times; it will be a no-op.
  ImGui::EndFrame();
}

void Window::Present() {
  // Filament (with the exception of WebGL) handles the swapchain internally.
  if (config_.render_config != kFilamentVulkan
     && config_.render_config != kFilamentOpenGL) {
    SDL_GL_SwapWindow(sdl_window_);
  }
}

}  // namespace mujoco::platform
