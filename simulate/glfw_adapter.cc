// Copyright 2023 DeepMind Technologies Limited
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

#include "glfw_adapter.h"

#include <cstdlib>
#include <utility>
#include <chrono>

#include <GLFW/glfw3.h>
#include <mujoco/mjui.h>
#include <mujoco/mujoco.h>
#include "glfw_dispatch.h"

#ifdef __APPLE__
#include "glfw_corevideo.h"
#endif

namespace mujoco {
namespace {
int MaybeGlfwInit() {
  static const int is_initialized = []() {
    auto success = Glfw().glfwInit();
    if (success == GLFW_TRUE) {
      std::atexit(Glfw().glfwTerminate);
    }
    return success;
  }();
  return is_initialized;
}

GlfwAdapter& GlfwAdapterFromWindow(GLFWwindow* window) {
  return *static_cast<GlfwAdapter*>(Glfw().glfwGetWindowUserPointer(window));
}
}  // namespace

GlfwAdapter::GlfwAdapter() {
  mjr_defaultContext(&con_);

  if (MaybeGlfwInit() != GLFW_TRUE) {
    mju_error("could not initialize GLFW");
  }

  // multisampling
  Glfw().glfwWindowHint(GLFW_SAMPLES, 4);
  Glfw().glfwWindowHint(GLFW_VISIBLE, 1);

  // get video mode and save
  vidmode_ = *Glfw().glfwGetVideoMode(Glfw().glfwGetPrimaryMonitor());

  // create window
  window_ = Glfw().glfwCreateWindow((2 * vidmode_.width) / 3,
                                    (2 * vidmode_.height) / 3,
                                    "MuJoCo", nullptr, nullptr);
  if (!window_) {
    mju_error("could not create window");
  }

  // save window position and size
  Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
  Glfw().glfwGetWindowSize(window_, &window_size_.first, &window_size_.second);

  // set callbacks
  Glfw().glfwSetWindowUserPointer(window_, this);
  Glfw().glfwSetDropCallback(
      window_, +[](GLFWwindow* window, int count, const char** paths) {
        GlfwAdapterFromWindow(window).OnFilesDrop(count, paths);
      });
  Glfw().glfwSetKeyCallback(
      window_, +[](GLFWwindow* window, int key, int scancode, int act, int mods) {
        GlfwAdapterFromWindow(window).OnKey(key, scancode, act);
      });
  Glfw().glfwSetMouseButtonCallback(
      window_, +[](GLFWwindow* window, int button, int act, int mods) {
        GlfwAdapterFromWindow(window).OnMouseButton(button, act);
      });
  Glfw().glfwSetCursorPosCallback(
      window_, +[](GLFWwindow* window, double x, double y) {
        GlfwAdapterFromWindow(window).OnMouseMove(x, y);
      });
  Glfw().glfwSetScrollCallback(
      window_, +[](GLFWwindow* window, double xoffset, double yoffset) {
        GlfwAdapterFromWindow(window).OnScroll(xoffset, yoffset);
      });
  Glfw().glfwSetWindowRefreshCallback(
      window_, +[](GLFWwindow* window) {
#ifdef __APPLE__
        auto& core_video = GlfwAdapterFromWindow(window).core_video_;
        if (core_video.has_value()) {
          core_video->UpdateDisplayLink();
        }
#endif
        GlfwAdapterFromWindow(window).OnWindowRefresh();
      });
  Glfw().glfwSetWindowSizeCallback(
      window_, +[](GLFWwindow* window, int width, int height) {
        GlfwAdapterFromWindow(window).OnWindowResize(width, height);
      });

  // make context current
  Glfw().glfwMakeContextCurrent(window_);
}

GlfwAdapter::~GlfwAdapter() {
  FreeMjrContext();
  Glfw().glfwMakeContextCurrent(nullptr);
  Glfw().glfwDestroyWindow(window_);
}

void GlfwAdapter::MakeCurrent() {
    Glfw().glfwMakeContextCurrent(window_);
}

std::pair<double, double> GlfwAdapter::GetCursorPosition() const {
  double x, y;
  Glfw().glfwGetCursorPos(window_, &x, &y);
  return {x, y};
}

double GlfwAdapter::GetDisplayPixelsPerInch() const {
  int width_mm, height_mm;
  Glfw().glfwGetMonitorPhysicalSize(
      Glfw().glfwGetPrimaryMonitor(), &width_mm, &height_mm);
  return 25.4 * vidmode_.width / width_mm;
}

std::pair<int, int> GlfwAdapter::GetFramebufferSize() const {
  int width, height;
  Glfw().glfwGetFramebufferSize(window_, &width, &height);
  return {width, height};
}

std::pair<int, int> GlfwAdapter::GetWindowSize() const {
  int width, height;
  Glfw().glfwGetWindowSize(window_, &width, &height);
  return {width, height};
}

bool GlfwAdapter::IsGPUAccelerated() const {
  return true;
}

void GlfwAdapter::PollEvents() {
  Glfw().glfwPollEvents();
}

void GlfwAdapter::SetClipboardString(const char* text) {
  Glfw().glfwSetClipboardString(window_, text);
}

void GlfwAdapter::SetVSync(bool enabled){
#ifdef __APPLE__
  Glfw().glfwSwapInterval(0);
  if (enabled && !core_video_.has_value()) {
    core_video_.emplace(window_);
  } else if (!enabled && core_video_.has_value()) {
    core_video_.reset();
  }
#else
  Glfw().glfwSwapInterval(enabled);
#endif
}

void GlfwAdapter::SetWindowTitle(const char* title) {
  Glfw().glfwSetWindowTitle(window_, title);
}

bool GlfwAdapter::ShouldCloseWindow() const {
  return Glfw().glfwWindowShouldClose(window_);
}

void GlfwAdapter::SwapBuffers() {
#ifdef __APPLE__
  if (core_video_.has_value()) {
    core_video_->WaitForDisplayRefresh();
  }
#endif
  Glfw().glfwSwapBuffers(window_);
}

void GlfwAdapter::ToggleFullscreen() {
  // currently full screen: switch to windowed
  if (Glfw().glfwGetWindowMonitor(window_)) {
    // restore window from saved data
    Glfw().glfwSetWindowMonitor(window_, nullptr, window_pos_.first, window_pos_.second,
                                window_size_.first, window_size_.second, 0);
  }

  // currently windowed: switch to full screen
  else {
    // save window data
    Glfw().glfwGetWindowPos(window_, &window_pos_.first, &window_pos_.second);
    Glfw().glfwGetWindowSize(window_, &window_size_.first,
                             &window_size_.second);

    // switch
    Glfw().glfwSetWindowMonitor(window_, Glfw().glfwGetPrimaryMonitor(), 0,
                                0, vidmode_.width, vidmode_.height,
                                vidmode_.refreshRate);
  }
}

bool GlfwAdapter::IsLeftMouseButtonPressed() const {
  return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMiddleMouseButtonPressed() const {
  return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
}

bool GlfwAdapter::IsRightMouseButtonPressed() const {
  return Glfw().glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}

bool GlfwAdapter::IsAltKeyPressed() const {
  return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_ALT) == GLFW_PRESS ||
         Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_ALT) == GLFW_PRESS;
}

bool GlfwAdapter::IsCtrlKeyPressed() const {
  return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
         Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
}

bool GlfwAdapter::IsShiftKeyPressed() const {
  return Glfw().glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
         Glfw().glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
}

bool GlfwAdapter::IsMouseButtonDownEvent(int act) const {
  return act == GLFW_PRESS;
}

bool GlfwAdapter::IsKeyDownEvent(int act) const { return act == GLFW_PRESS; }

int GlfwAdapter::TranslateKeyCode(int key) const { return key; }

mjtButton GlfwAdapter::TranslateMouseButton(int button) const {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    return mjBUTTON_LEFT;
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
    return mjBUTTON_RIGHT;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
    return mjBUTTON_MIDDLE;
  }
  return mjBUTTON_NONE;
}


void GlfwAdapter::FreeMjrContext() {
  mjr_freeContext(&con_);
}

bool GlfwAdapter::RefreshMjrContext(const mjModel* m, int fontscale) {
  if (m != last_model_ || fontscale != last_fontscale_) {
    mjr_makeContext(m, &con_, fontscale);
    last_model_ = m;
    last_fontscale_ = fontscale;
    return true;
  }
  return false;
}

bool GlfwAdapter::EnsureContextSize() {
  return false;
}

void GlfwAdapter::OnFilesDrop(int count, const char** paths) {
  state_.type = mjEVENT_FILESDROP;
  state_.dropcount = count;
  state_.droppaths = paths;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }

  // remove paths pointer from mjuiState since we don't own it
  state_.dropcount = 0;
  state_.droppaths = nullptr;
}

void GlfwAdapter::OnKey(int key, int scancode, int act) {
  // translate API-specific key code
  int mj_key = TranslateKeyCode(key);

  // release: nothing to do
  if (!IsKeyDownEvent(act)) {
    return;
  }

  // update state
  UpdateMjuiState();

  // set key info
  state_.type = mjEVENT_KEY;
  state_.key = mj_key;
  state_.keytime = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }

  last_key_ = mj_key;
}

void GlfwAdapter::OnMouseButton(int button, int act)  {
  // translate API-specific mouse button code
  mjtButton mj_button = TranslateMouseButton(button);

  // update state
  UpdateMjuiState();

  // swap left and right if Alt
  if (state_.alt) {
    if (mj_button == mjBUTTON_LEFT) {
      mj_button = mjBUTTON_RIGHT;
    } else if (mj_button == mjBUTTON_RIGHT) {
      mj_button = mjBUTTON_LEFT;
    }
  }

  // press
  if (IsMouseButtonDownEvent(act)) {
    double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    // detect doubleclick: 250 ms
    if (mj_button == state_.button && now - state_.buttontime < 0.25) {
      state_.doubleclick = 1;
    } else {
      state_.doubleclick = 0;
    }

    // set info
    state_.type = mjEVENT_PRESS;
    state_.button = mj_button;
    state_.buttontime = now;

    // start dragging
    if (state_.mouserect) {
      state_.dragbutton = state_.button;
      state_.dragrect = state_.mouserect;
    }
  }

  // release
  else {
    state_.type = mjEVENT_RELEASE;
  }

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }

  // stop dragging after application processing
  if (state_.type == mjEVENT_RELEASE) {
    state_.dragrect = 0;
    state_.dragbutton = 0;
  }
}

void GlfwAdapter::OnMouseMove(double x, double y) {
  // no buttons down: nothing to do
  if (!state_.left && !state_.right && !state_.middle) {
    return;
  }

  // update state
  UpdateMjuiState();

  // set move info
  state_.type = mjEVENT_MOVE;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void GlfwAdapter::OnScroll(double xoffset, double yoffset) {
  // update state
  UpdateMjuiState();

  // set scroll info, scale by buffer-to-window ratio
  const double buffer_window_ratio =
      static_cast<double>(GetFramebufferSize().first) / GetWindowSize().first;
  state_.type = mjEVENT_SCROLL;
  state_.sx = xoffset * buffer_window_ratio;
  state_.sy = yoffset * buffer_window_ratio;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void GlfwAdapter::OnWindowRefresh() {
  state_.type = mjEVENT_REDRAW;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void GlfwAdapter::OnWindowResize(int width, int height) {
  auto [buf_width, buf_height] = GetFramebufferSize();
  state_.rect[0].width = buf_width;
  state_.rect[0].height = buf_height;
  if (state_.nrect < 1) state_.nrect = 1;

  // update window layout
  if (layout_callback_) {
    layout_callback_(&state_);
  }

  // update state
  UpdateMjuiState();

  // set resize info
  state_.type = mjEVENT_RESIZE;

  // stop dragging
  state_.dragbutton = 0;
  state_.dragrect = 0;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void GlfwAdapter::UpdateMjuiState() {
  // mouse buttons
  state_.left = IsLeftMouseButtonPressed();
  state_.right = IsRightMouseButtonPressed();
  state_.middle = IsMiddleMouseButtonPressed();

  // keyboard modifiers
  state_.control = IsCtrlKeyPressed();
  state_.shift = IsShiftKeyPressed();
  state_.alt = IsAltKeyPressed();

  // swap left and right if Alt
  if (state_.alt) {
    int tmp = state_.left;
    state_.left = state_.right;
    state_.right = tmp;
  }

  // get mouse position, scale by buffer-to-window ratio
  auto [x, y] = GetCursorPosition();
  const double buffer_window_ratio =
      static_cast<double>(GetFramebufferSize().first) / GetWindowSize().first;
  x *= buffer_window_ratio;
  y *= buffer_window_ratio;

  // invert y to match OpenGL convention
  y = state_.rect[0].height - y;

  // save
  state_.dx = x - state_.x;
  state_.dy = y - state_.y;
  state_.x = x;
  state_.y = y;

  // find mouse rectangle
  state_.mouserect = mjr_findRect(mju_round(x), mju_round(y), state_.nrect-1, state_.rect+1) + 1;
}

}  // namespace mujoco
