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

#include <algorithm>
#include <cstddef>
#include <span>
#include <string>
#include <string_view>

#include <SDL.h>
#include <SDL_error.h>
#include <SDL_events.h>
#include <SDL_hints.h>
#include <SDL_render.h>
#include <SDL_surface.h>
#include <SDL_syswm.h>
#include <SDL_version.h>
#include <SDL_video.h>
#include <backends/imgui_impl_sdl2.h>
#include <imgui.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/renderer_backend.h"
#include "user/user_resource.h"

// Because X11/Xlib.h defines Status.
#ifdef Status
#undef Status
#endif

#if defined(__APPLE__)
extern void* GetNativeWindowOsx(void* window);
#endif

namespace mujoco::platform {

static void InitImGui(SDL_Window* window, float content_scale, bool load_fonts,
                      bool build_fonts) {
  ImGui::CreateContext();

  ImGuiIO& io = ImGui::GetIO();
  io.BackendFlags |= ImGuiBackendFlags_RendererHasTextures;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = nullptr;
  io.ConfigDpiScaleFonts = true;
  io.ConfigDpiScaleViewports = true;
  ImGui::StyleColorsDark();
  ImGui_ImplSDL2_InitForOther(window);

  ImGuiStyle& style = ImGui::GetStyle();
  style.ScaleAllSizes(content_scale);
  style.FontScaleDpi = content_scale;

  if (load_fonts) {
    mjResource* font = nullptr;
    int size = 0;
    void* data = nullptr;

    ImFontConfig main_cfg;
    main_cfg.FontDataOwnedByAtlas = false;
    font =
        mju_openResource("", "font:OpenSans-Regular.ttf", nullptr, nullptr, 0);
    size = mju_readResource(font, const_cast<const void**>(&data));
    io.Fonts->AddFontFromMemoryTTF(data, size, 20.f, &main_cfg);

    ImFontConfig icon_cfg;
    icon_cfg.FontDataOwnedByAtlas = false;
    icon_cfg.MergeMode = true;
    font = mju_openResource("", "font:fontawesome-webfont.ttf", nullptr,
                            nullptr, 0);
    size = mju_readResource(font, const_cast<const void**>(&data));
    constexpr ImWchar icon_ranges[] = {0xf000, 0xf3ff, 0x000};
    io.Fonts->AddFontFromMemoryTTF(data, size, 14.f, &icon_cfg, icon_ranges);

    if (build_fonts) {
      io.Fonts->Build();
    }

    // Note: we purposefully do not "close" the font resources as ImGui may
    // need them again to resize fonts.
  }
}

Window::Window(std::string_view title, int width, int height, Config config)
    : width_(width), height_(height), config_(config) {
  const RendererBackend renderer_backend = config_.renderer_backend;
  if (renderer_backend == RendererBackend::FilamentOpenGlHeadless) {
    config_.offscreen_mode = true;
  }

  if (config_.offscreen_mode) {
    SDL_SetHint(SDL_HINT_RENDER_DRIVER, "software");
    SDL_SetHint(SDL_HINT_FRAMEBUFFER_ACCELERATION, "0");
  } else {
    SDL_SetHint(SDL_HINT_FRAMEBUFFER_ACCELERATION, "1");
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 16);
  }

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
    mju_error("Error initializing SDL: %s", SDL_GetError());
  }
  if (!config_.enable_keyboard) {
    SDL_EventState(SDL_TEXTINPUT, SDL_DISABLE);
    SDL_EventState(SDL_KEYDOWN, SDL_DISABLE);
    SDL_EventState(SDL_KEYUP, SDL_DISABLE);
  }

  int window_flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
  if (renderer_backend == RendererBackend::FilamentVulkan) {
    window_flags |= SDL_WINDOW_VULKAN;
  } else if (renderer_backend == RendererBackend::FilamentWebGl) {
    window_flags |= SDL_WINDOW_OPENGL;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  } else if (renderer_backend == RendererBackend::ClassicOpenGl ||
             renderer_backend == RendererBackend::FilamentOpenGl ||
             renderer_backend == RendererBackend::FilamentOpenGlHeadless) {
    window_flags |= SDL_WINDOW_OPENGL;
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  } else {
    mju_error("Unsupported window config: %d", renderer_backend);
  }

  const float content_scale = ImGui_ImplSDL2_GetContentScaleForDisplay(0);
  sdl_window_ = SDL_CreateWindow(title.data(), SDL_WINDOWPOS_UNDEFINED,
                                 SDL_WINDOWPOS_UNDEFINED, width,
                                 height, window_flags);
  if (!sdl_window_) {
    mju_error("Error creating window: %s", SDL_GetError());
  }

  InitImGui(sdl_window_, content_scale, config.load_fonts,
            (renderer_backend != RendererBackend::ClassicOpenGl));

  if (renderer_backend == RendererBackend::FilamentWebGl ||
      (renderer_backend == RendererBackend::ClassicOpenGl &&
       !config.offscreen_mode)) {
    SDL_GLContext gl_context = SDL_GL_CreateContext(sdl_window_);
    SDL_GL_MakeCurrent(sdl_window_, gl_context);
  }

  if (config_.offscreen_mode) {
    sdl_renderer_ = SDL_CreateRenderer(sdl_window_, -1, SDL_RENDERER_SOFTWARE);
  }

  SDL_SysWMinfo wmi;
  SDL_VERSION(&wmi.version);
  SDL_GetWindowWMInfo(sdl_window_, &wmi);

  if (!config_.offscreen_mode) {
    #if defined(__linux__)
      native_window_ = reinterpret_cast<void*>(wmi.info.x11.window);
    #elif defined(__WIN32__)
      native_window_ = reinterpret_cast<void*>(wmi.info.win.window);
    #elif defined(__APPLE__)
      native_window_ =
          GetNativeWindowOsx(reinterpret_cast<void*>(wmi.info.cocoa.window));
    #endif
  }

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

void Window::DisableWindowResizing() {
  SDL_SetWindowResizable(sdl_window_, SDL_FALSE);
}
void Window::EnableWindowResizing() {
  SDL_SetWindowResizable(sdl_window_, SDL_TRUE);
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

void Window::Present(std::span<const std::byte> pixels) {
  if (config_.offscreen_mode) {
    if (pixels.size() != width_ * height_ * 3) {
      mju_error("Offscreen mode expects RGB buffer of size %d",
                width_ * height_ * 3);
    }

    SDL_Surface* surface = SDL_GetWindowSurface(sdl_window_);
    const unsigned char* src =
        reinterpret_cast<const unsigned char*>(pixels.data());
    unsigned char* dst =
        static_cast<unsigned char*>(surface->pixels);

    for (int i = 0; i < height_; ++i) {
      for (int j = 0; j < width_; ++j) {
        // Swap the RGB channels and add an alpha value of 1.0
        dst[0] = src[2];
        dst[1] = src[1];
        dst[2] = src[0];
        dst[3] = 0xff;
        dst += 4;
        src += 3;
      }
    }

    if (config_.renderer_backend == RendererBackend::ClassicOpenGl) {
      dst = static_cast<unsigned char*>(surface->pixels);
      for (int r = 0; r < height_ / 2; ++r) {
        unsigned char* top_row = &dst[4 * width_ * r];
        unsigned char* bottom_row = &dst[4 * width_ * (height_ - 1 - r)];
        std::swap_ranges(top_row, top_row + 4 * width_, bottom_row);
      }
    }

    SDL_RenderPresent(sdl_renderer_);
  } else if (config_.renderer_backend != RendererBackend::FilamentVulkan
     && config_.renderer_backend != RendererBackend::FilamentOpenGl) {
    SDL_GL_SwapWindow(sdl_window_);
  }
}

bool Window::IsOffscreenMode() const {
  return config_.offscreen_mode;
}
}  // namespace mujoco::platform
