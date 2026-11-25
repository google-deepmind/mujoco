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

#include "studio/hal/window.h"

#include <stdlib.h>

#include "testing/base/public/gunit.h"
#include <SDL_events.h>
#include <SDL_mouse.h>
#include <SDL_stdinc.h>
#include <SDL_touch.h>
#include <SDL_video.h>
#include <imgui.h>
#include "studio/hal/graphics_mode.h"
#include "studio/ux/touch_gesture.h"

namespace mujoco::studio {

struct WindowTestPeer {
  static void SetDirectTouchCheck(Window& window, bool (*check)(SDL_TouchID)) {
    window.direct_touch_check_ = check;
  }
  static Uint32 WindowId(const Window& window) { return SDL_GetWindowID(window.sdl_window_); }
};

namespace {

constexpr SDL_TouchID kDirectTouchId = 42;
constexpr SDL_TouchID kIndirectTouchId = 99;
constexpr int kWinW = 400;
constexpr int kWinH = 800;

bool DirectOnlyCheck(SDL_TouchID id) { return id == kDirectTouchId; }

void PushFingerEvent(Uint32 type, SDL_TouchID touch_id, SDL_FingerID finger_id, float x_px,
                     float y_px) {
  SDL_Event ev{};
  ev.type = type;
  ev.tfinger.touchId = touch_id;
  ev.tfinger.fingerId = finger_id;
  ev.tfinger.x = x_px / static_cast<float>(kWinW);
  ev.tfinger.y = y_px / static_cast<float>(kWinH);
  ASSERT_EQ(SDL_PushEvent(&ev), 1);
}

void PushTouchMouseMotion(Uint32 window_id, int x, int y) {
  SDL_Event ev{};
  ev.type = SDL_MOUSEMOTION;
  ev.motion.windowID = window_id;
  ev.motion.which = SDL_TOUCH_MOUSEID;
  ev.motion.x = x;
  ev.motion.y = y;
  ASSERT_EQ(SDL_PushEvent(&ev), 1);
}

void PushTouchMouseButton(Uint32 window_id, Uint32 type, int x, int y) {
  SDL_Event ev{};
  ev.type = type;
  ev.button.windowID = window_id;
  ev.button.which = SDL_TOUCH_MOUSEID;
  ev.button.button = SDL_BUTTON_LEFT;
  ev.button.state = (type == SDL_MOUSEBUTTONDOWN) ? SDL_PRESSED : SDL_RELEASED;
  ev.button.clicks = 1;
  ev.button.x = x;
  ev.button.y = y;
  ASSERT_EQ(SDL_PushEvent(&ev), 1);
}

void PushWindowFocusLost(Uint32 window_id) {
  SDL_Event ev{};
  ev.type = SDL_WINDOWEVENT;
  ev.window.windowID = window_id;
  ev.window.event = SDL_WINDOWEVENT_FOCUS_LOST;
  ASSERT_EQ(SDL_PushEvent(&ev), 1);
}

void BuildTestButtonPanel(bool* clicked) {
  ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f), ImGuiCond_Always);
  ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_NoTitleBar);
  *clicked = ImGui::Button("Action", ImVec2(120.0f, 40.0f));
  ImGui::End();
}

TEST(WindowTouchTest, TrickledFingerPressStrippedWhenSecondFingerLandsNextFrame) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);
  Window::Config cfg{.gfx_mode = GraphicsMode::FilamentOpenGlHeadless, .load_fonts = false};
  Window window("touch_test", kWinW, kWinH, cfg);
  WindowTestPeer::SetDirectTouchCheck(window, DirectOnlyCheck);
  const Uint32 win_id = WindowTestPeer::WindowId(window);

  // Warm up ImGui layout with a button covering (60, 50).
  bool clicked = false;
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Complete a tap away from the panel so ImGui records a recent LastClickedTime
  // (an unstripped press on the upcoming 1->2 pinch would double-click).
  PushTouchMouseMotion(win_id, 300, 500);
  PushTouchMouseButton(win_id, SDL_MOUSEBUTTONDOWN, 300, 500);
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 1, 300.0f, 500.0f);
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  // Frame 2 flushes the trickled press.
  window.NewFrame();
  EXPECT_TRUE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  PushTouchMouseButton(win_id, SDL_MOUSEBUTTONUP, 300, 500);
  PushFingerEvent(SDL_FINGERUP, kDirectTouchId, 1, 300.0f, 500.0f);
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  // Frame N (0 -> 1): Finger 1 lands on "Action" button at (60, 50).
  // Touch input trickling processes MousePos and holds MouseButton(Left, true)
  // in InputEventsQueue, so io.MouseDown[0] is still false.
  PushTouchMouseMotion(win_id, 60, 50);
  PushTouchMouseButton(win_id, SDL_MOUSEBUTTONDOWN, 60, 50);
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 2, 60.0f, 50.0f);
  window.NewFrame();
  EXPECT_FALSE(ImGui::GetIO().MouseDown[ImGuiMouseButton_Left]);
  EXPECT_FALSE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Frame N+1 (1 -> 2): Second finger lands at (340, 550), entering PinchPan
  // with midpoint (200, 300). Window::NewFrame must strip the queued press so
  // no click or double-click fires and MousePos moves to the pinch midpoint.
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 3, 340.0f, 550.0f);
  window.NewFrame();
  EXPECT_TRUE(window.GetTouchGesture().active);
  EXPECT_TRUE(window.GetTouchGesture().just_started);
  EXPECT_FALSE(ImGui::GetIO().MouseDown[ImGuiMouseButton_Left]);
  EXPECT_FALSE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  EXPECT_FALSE(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left));
  EXPECT_FLOAT_EQ(ImGui::GetIO().MousePos.x, 200.0f);
  EXPECT_FLOAT_EQ(ImGui::GetIO().MousePos.y, 300.0f);
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);
}

TEST(WindowTouchTest, HeldFingerCancelledOffscreenOnPinchAndFocusLoss) {
  setenv("SDL_VIDEODRIVER", "dummy", 1);
  Window::Config cfg{.gfx_mode = GraphicsMode::FilamentOpenGlHeadless, .load_fonts = false};
  Window window("touch_cancel_test", kWinW, kWinH, cfg);
  WindowTestPeer::SetDirectTouchCheck(window, DirectOnlyCheck);
  const Uint32 win_id = WindowTestPeer::WindowId(window);

  bool clicked = false;
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  // Finger 1 presses "Action" at (60, 50) and holds across two frames so
  // io.MouseDown[0] becomes true and the button is active.
  PushTouchMouseMotion(win_id, 60, 50);
  PushTouchMouseButton(win_id, SDL_MOUSEBUTTONDOWN, 60, 50);
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 1, 60.0f, 50.0f);
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  window.NewFrame();
  ASSERT_TRUE(ImGui::GetIO().MouseDown[ImGuiMouseButton_Left]);
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Second finger lands: Window::NewFrame must atomically park cursor off-screen
  // and release left button without trickling, cancelling the active button.
  PushTouchMouseMotion(win_id, 65, 55);
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 2, 300.0f, 500.0f);
  window.NewFrame();
  EXPECT_TRUE(window.GetTouchGesture().active);
  EXPECT_FALSE(ImGui::GetIO().MouseDown[ImGuiMouseButton_Left]);
  EXPECT_LT(ImGui::GetIO().MousePos.x, -1e10f);
  EXPECT_TRUE(ImGui::GetIO().ConfigInputTrickleEventQueue);
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Lift both fingers, then start a new held touch and trigger FOCUS_LOST.
  PushFingerEvent(SDL_FINGERUP, kDirectTouchId, 1, 60.0f, 50.0f);
  PushFingerEvent(SDL_FINGERUP, kDirectTouchId, 2, 300.0f, 500.0f);
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  PushTouchMouseMotion(win_id, 60, 50);
  PushTouchMouseButton(win_id, SDL_MOUSEBUTTONDOWN, 60, 50);
  PushFingerEvent(SDL_FINGERDOWN, kDirectTouchId, 4, 60.0f, 50.0f);
  window.NewFrame();
  BuildTestButtonPanel(&clicked);
  ImGui::Render();

  PushWindowFocusLost(win_id);
  window.NewFrame();
  EXPECT_FALSE(window.GetTouchGesture().active);
  EXPECT_FALSE(ImGui::GetIO().MouseDown[ImGuiMouseButton_Left]);
  EXPECT_LT(ImGui::GetIO().MousePos.x, -1e10f);
  BuildTestButtonPanel(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Indirect touch devices (e.g. trackpads) do not enter PinchPan.
  PushFingerEvent(SDL_FINGERDOWN, kIndirectTouchId, 10, 100.0f, 100.0f);
  PushFingerEvent(SDL_FINGERDOWN, kIndirectTouchId, 11, 200.0f, 200.0f);
  window.NewFrame();
  EXPECT_FALSE(window.GetTouchGesture().active);
  window.EndFrame();
}

}  // namespace
}  // namespace mujoco::studio
