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

#include "platform_ui_adapter.h"

#include <chrono>

namespace mujoco {
PlatformUIAdapter::PlatformUIAdapter() {
  mjr_defaultContext(&con_);
}

void PlatformUIAdapter::FreeMjrContext() {
  mjr_freeContext(&con_);
}

bool PlatformUIAdapter::RefreshMjrContext(const mjModel* m, int fontscale) {
  if (m != last_model_ || fontscale != last_fontscale_) {
    mjr_makeContext(m, &con_, fontscale);
    last_model_ = m;
    last_fontscale_ = fontscale;
    return true;
  }
  return false;
}

bool PlatformUIAdapter::EnsureContextSize() {
  return false;
}

void PlatformUIAdapter::OnFilesDrop(int count, const char** paths) {
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

void PlatformUIAdapter::OnKey(int key, int scancode, int act) {
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

void PlatformUIAdapter::OnMouseButton(int button, int act)  {
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

void PlatformUIAdapter::OnMouseMove(double x, double y) {
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

void PlatformUIAdapter::OnScroll(double xoffset, double yoffset) {
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

void PlatformUIAdapter::OnWindowRefresh() {
  state_.type = mjEVENT_REDRAW;

  // application-specific processing
  if (event_callback_) {
    event_callback_(&state_);
  }
}

void PlatformUIAdapter::OnWindowResize(int width, int height) {
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

void PlatformUIAdapter::UpdateMjuiState() {
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
