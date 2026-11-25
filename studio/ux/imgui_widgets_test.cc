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

#include "studio/ux/imgui_widgets.h"

#include <algorithm>
#include <cfloat>
#include <cstddef>
#include <cstring>
#include <limits>
#include <string>

#include "testing/base/public/gunit.h"
#include <imgui.h>
#include "studio/ux/gui.h"

namespace mujoco::studio {
namespace {

int FindDrawListIndexByOwnerPrefix(const ImDrawData* draw_data, const char* prefix) {
  if (!draw_data) {
    return -1;
  }
  const size_t len = std::strlen(prefix);
  for (int i = 0; i < draw_data->CmdLists.Size; ++i) {
    const ImDrawList* list = draw_data->CmdLists[i];
    if (list->_OwnerName && std::strncmp(list->_OwnerName, prefix, len) == 0) {
      return i;
    }
  }
  return -1;
}

struct VertexBounds {
  float min_x = std::numeric_limits<float>::infinity();
  float min_y = std::numeric_limits<float>::infinity();
  float max_x = -std::numeric_limits<float>::infinity();
  float max_y = -std::numeric_limits<float>::infinity();
};

VertexBounds ComputeVertexBounds(const ImDrawList* list) {
  VertexBounds bounds;
  for (int i = 0; i < list->VtxBuffer.Size; ++i) {
    const ImVec2& p = list->VtxBuffer[i].pos;
    bounds.min_x = std::min(bounds.min_x, p.x);
    bounds.min_y = std::min(bounds.min_y, p.y);
    bounds.max_x = std::max(bounds.max_x, p.x);
    bounds.max_y = std::max(bounds.max_y, p.y);
  }
  return bounds;
}

void InitTestImGuiContext(bool enable_docking = false) {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = nullptr;
  io.DeltaTime = 1.0f / 60.0f;
  if (enable_docking) {
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }
  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
}

TEST(DrawTextAtTest, CoversFullDisplayAcrossPortraitAndLandscapeResizes) {
  InitTestImGuiContext();
  ImGuiIO& io = ImGui::GetIO();

  const auto run_frame = [&io](float w, float h) {
    io.DisplaySize = ImVec2(w, h);
    ImGui::NewFrame();
    DrawTextAt("top_left", -0.85f, 0.85f, 0.0f);
    DrawTextAt("center", 0.0f, 0.0f, 0.0f);
    DrawTextAt("bottom_right", 0.85f, -0.85f, 0.0f);
    ImGui::Render();

    const ImDrawData* draw_data = ImGui::GetDrawData();
    const int labels_idx = FindDrawListIndexByOwnerPrefix(draw_data, "labels");
    ASSERT_GE(labels_idx, 0);
    const ImDrawList* labels = draw_data->CmdLists[labels_idx];
    ASSERT_EQ(labels->CmdBuffer.Size, 1);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.x, 0.0f);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.y, 0.0f);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.z, w);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.w, h);

    ASSERT_GT(labels->VtxBuffer.Size, 0);
    const VertexBounds bounds = ComputeVertexBounds(labels);
    EXPECT_LT(bounds.min_x, 0.2f * w);
    EXPECT_LT(bounds.min_y, 0.2f * h);
    EXPECT_GT(bounds.max_x, 0.85f * w);
    EXPECT_GT(bounds.max_y, 0.85f * h);
  };

  // Run >2 portrait frames so ImGui's initial 2-frame auto-fit window expires,
  // then resize to landscape (200x600 -> 900x600), then wide -> tall portrait
  // (800x250 -> 400x800). Frame 1 must also be visible immediately.
  for (int i = 0; i < 3; ++i) {
    run_frame(200.0f, 600.0f);
  }
  for (int i = 0; i < 2; ++i) {
    run_frame(900.0f, 600.0f);
  }
  for (int i = 0; i < 3; ++i) {
    run_frame(800.0f, 250.0f);
  }
  for (int i = 0; i < 2; ++i) {
    run_frame(400.0f, 800.0f);
  }

  ImGui::DestroyContext();
}

TEST(DrawTextAtTest, HandlesNonZeroRectOffsetsMultipleRectsAndInvalidInputs) {
  InitTestImGuiContext();
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(1000.0f, 800.0f);

  const auto run_rect_frame = [](const ImVec4& rect) {
    ImGui::NewFrame();
    DrawTextAtInRect("tl", -0.8f, 0.8f, 0.0f, rect);
    DrawTextAtInRect("br", 0.8f, -0.8f, 0.0f, rect);
    ImGui::Render();

    const ImDrawData* draw_data = ImGui::GetDrawData();
    const int labels_idx = FindDrawListIndexByOwnerPrefix(draw_data, "labels");
    ASSERT_GE(labels_idx, 0);
    const ImDrawList* labels = draw_data->CmdLists[labels_idx];
    ASSERT_EQ(labels->CmdBuffer.Size, 1);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.x, rect.x);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.y, rect.y);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.z, rect.x + rect.z);
    EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.w, rect.y + rect.w);

    const VertexBounds bounds = ComputeVertexBounds(labels);
    EXPECT_GE(bounds.min_x, rect.x);
    EXPECT_GE(bounds.min_y, rect.y);
    EXPECT_GT(bounds.max_x, rect.x + 0.85f * rect.z);
    EXPECT_GT(bounds.max_y, rect.y + 0.85f * rect.w);
    EXPECT_LE(bounds.max_x, rect.x + rect.z);
    EXPECT_LE(bounds.max_y, rect.y + rect.w);
  };

  for (int i = 0; i < 3; ++i) {
    run_rect_frame(ImVec4(50.0f, 30.0f, 600.0f, 400.0f));
  }
  run_rect_frame(ImVec4(20.0f, 40.0f, 850.0f, 700.0f));

  // Two distinct sub-rects in the same frame each keep their own exact clip rect.
  ImGui::NewFrame();
  DrawTextAtInRect("left_pane", 0.0f, 0.0f, 0.0f, ImVec4(20.0f, 30.0f, 300.0f, 500.0f));
  DrawTextAtInRect("right_pane", 0.0f, 0.0f, 0.0f, ImVec4(500.0f, 60.0f, 400.0f, 600.0f));
  ImGui::Render();
  const ImDrawData* dual_data = ImGui::GetDrawData();
  const int dual_idx = FindDrawListIndexByOwnerPrefix(dual_data, "labels");
  ASSERT_GE(dual_idx, 0);
  const ImDrawList* dual = dual_data->CmdLists[dual_idx];
  ASSERT_EQ(dual->CmdBuffer.Size, 2);
  EXPECT_FLOAT_EQ(dual->CmdBuffer[0].ClipRect.x, 20.0f);
  EXPECT_FLOAT_EQ(dual->CmdBuffer[0].ClipRect.z, 320.0f);
  EXPECT_FLOAT_EQ(dual->CmdBuffer[1].ClipRect.x, 500.0f);
  EXPECT_FLOAT_EQ(dual->CmdBuffer[1].ClipRect.z, 900.0f);

  // Invalid inputs (null/empty text, out-of-frustum, NaN, non-positive rect) emit nothing.
  ImGui::NewFrame();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  DrawTextAtInRect(nullptr, 0.0f, 0.0f, 0.0f, ImVec4(0.0f, 0.0f, 500.0f, 500.0f));
  DrawTextAtInRect("", 0.0f, 0.0f, 0.0f, ImVec4(0.0f, 0.0f, 500.0f, 500.0f));
  DrawTextAtInRect("oob_x", 1.2f, 0.0f, 0.0f, ImVec4(0.0f, 0.0f, 500.0f, 500.0f));
  DrawTextAtInRect("oob_z", 0.0f, 0.0f, -1.1f, ImVec4(0.0f, 0.0f, 500.0f, 500.0f));
  DrawTextAtInRect("nan_xy", nan, 0.0f, 0.0f, ImVec4(0.0f, 0.0f, 500.0f, 500.0f));
  DrawTextAtInRect("zero_rect", 0.0f, 0.0f, 0.0f, ImVec4(0.0f, 0.0f, 0.0f, 500.0f));
  ImGui::Render();
  EXPECT_EQ(FindDrawListIndexByOwnerPrefix(ImGui::GetDrawData(), "labels"), -1);

  ImGui::DestroyContext();
}

TEST(DrawTextAtTest, IgnoresPreExistingIniEntryAndNeverPersistsLabelsWindow) {
  // 1. Pre-load a stale small [Window][labels] entry (simulating a poisoned ~/.mujoco.ini).
  InitTestImGuiContext();
  static constexpr char kStaleIni[] =
      "[Window][labels]\n"
      "Pos=60,60\n"
      "Size=120,240\n"
      "Collapsed=0\n\n";
  ImGui::LoadIniSettingsFromMemory(kStaleIni, sizeof(kStaleIni) - 1);

  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(900.0f, 600.0f);
  ImGui::NewFrame();
  DrawTextAt("top_left", -0.85f, 0.85f, 0.0f);
  DrawTextAt("far_right", 0.85f, -0.85f, 0.0f);
  ImGui::Render();

  const ImDrawData* draw_data = ImGui::GetDrawData();
  const int labels_idx = FindDrawListIndexByOwnerPrefix(draw_data, "labels");
  ASSERT_GE(labels_idx, 0);
  const ImDrawList* labels = draw_data->CmdLists[labels_idx];
  ASSERT_EQ(labels->CmdBuffer.Size, 1);
  EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.x, 0.0f);
  EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.y, 0.0f);
  EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.z, 900.0f);
  EXPECT_FLOAT_EQ(labels->CmdBuffer[0].ClipRect.w, 600.0f);
  const VertexBounds bounds = ComputeVertexBounds(labels);
  EXPECT_LT(bounds.min_x, 80.0f);
  EXPECT_GT(bounds.max_x, 750.0f);
  EXPECT_GT(bounds.max_y, 500.0f);
  ImGui::DestroyContext();

  // 2. Fresh context: SaveIniSettingsToMemory must never serialize [Window][labels].
  InitTestImGuiContext();
  ImGui::GetIO().DisplaySize = ImVec2(400.0f, 700.0f);
  ImGui::NewFrame();
  DrawTextAt("body_label", 0.0f, 0.0f, 0.0f);
  ImGui::Render();

  size_t ini_size = 0;
  const char* saved_ini = ImGui::SaveIniSettingsToMemory(&ini_size);
  ASSERT_NE(saved_ini, nullptr);
  EXPECT_EQ(std::string(saved_ini, ini_size).find("[Window][labels]"), std::string::npos);
  ImGui::DestroyContext();
}

TEST(DrawTextAtTest, RendersBehindDockedPanelsMenuAndStatusBars) {
  InitTestImGuiContext(/*enable_docking=*/true);
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(900.0f, 600.0f);

  for (int frame = 0; frame < 3; ++frame) {
    ImGui::NewFrame();
    if (ImGui::BeginMainMenuBar()) {
      ImGui::TextUnformatted("File");
      ImGui::EndMainMenuBar();
    }
    ConfigureDockingLayout(/*show_toolbar=*/true, /*show_status_bar=*/true);
    if (ImGui::Begin("ToolBar")) {
      ImGui::TextUnformatted("Toolbar");
    }
    ImGui::End();
    if (ImGui::Begin("Options")) {
      ImGui::TextUnformatted("Options Panel");
    }
    ImGui::End();
    if (ImGui::Begin("Inspector")) {
      ImGui::TextUnformatted("Inspector Panel");
    }
    ImGui::End();
    if (ImGui::Begin("StatusBar")) {
      ImGui::TextUnformatted("Running");
    }
    ImGui::End();

    // Match App::Render() order: 3D labels are emitted after BuildGui().
    DrawTextAt("scene_geom_label", 0.0f, 0.0f, 0.0f);
    ImGui::Render();

    const ImDrawData* draw_data = ImGui::GetDrawData();
    const int labels_idx = FindDrawListIndexByOwnerPrefix(draw_data, "labels");
    const int options_idx = FindDrawListIndexByOwnerPrefix(draw_data, "Options");
    const int inspector_idx = FindDrawListIndexByOwnerPrefix(draw_data, "Inspector");
    const int toolbar_idx = FindDrawListIndexByOwnerPrefix(draw_data, "ToolBar");
    const int status_idx = FindDrawListIndexByOwnerPrefix(draw_data, "StatusBar");
    const int menu_idx = FindDrawListIndexByOwnerPrefix(draw_data, "##MainMenuBar");

    ASSERT_EQ(labels_idx, 0);
    ASSERT_GT(toolbar_idx, labels_idx);
    ASSERT_GT(status_idx, labels_idx);
    ASSERT_GT(menu_idx, labels_idx);
    // Docked panels without explicit SetNextWindowSize hide on frame 0 while
    // ImGui measures their initial size, then render on all subsequent frames.
    if (frame >= 1) {
      ASSERT_GT(options_idx, labels_idx);
      ASSERT_GT(inspector_idx, labels_idx);
    }
  }

  ImGui::DestroyContext();
}

class ImGuiTouchWidgetsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctx_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx_);
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.DisplaySize = ImVec2(390.0f, 844.0f);
    unsigned char* pixels = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    io.Fonts->SetTexID(static_cast<ImTextureID>(1));
  }

  void TearDown() override {
    ImGui::DestroyContext(ctx_);
    ctx_ = nullptr;
  }

  ImGuiContext* ctx_ = nullptr;
};

TEST_F(ImGuiTouchWidgetsTest, StripUnconsumedTouchMousePressPreventsTrickledClickAndDoubleClick) {
  ImGuiIO& io = ImGui::GetIO();
  ASSERT_TRUE(io.ConfigInputTrickleEventQueue);

  // Establish a widget window at [20..220, 20..120].
  ImGui::NewFrame();
  ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f), ImGuiCond_Always);
  ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_NoTitleBar);
  ImGui::Button("Action", ImVec2(120.0f, 40.0f));
  ImGui::End();
  ImGui::Render();

  // Simulate a completed quick tap first so LastClickedTime is populated: an
  // unstripped press on the ensuing 1->2 pinch would trigger DoubleClicked!
  io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  io.AddMousePosEvent(300.0f, 500.0f);
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
  ImGui::NewFrame();
  ImGui::EndFrame();  // Trickled move.
  ImGui::NewFrame();
  EXPECT_TRUE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  ImGui::EndFrame();  // Trickled press.
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
  ImGui::NewFrame();
  ImGui::EndFrame();  // Release.

  // Frame N (0 -> 1): Finger 1 touches down at (60, 50) over the "Action"
  // button. Because TouchScreen moved from (300, 500) -> (60, 50), ImGui's
  // trickle queue processes MousePos and leaves MouseButton(0, true) queued.
  io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  io.AddMousePosEvent(60.0f, 50.0f);
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
  ImGui::NewFrame();
  EXPECT_FALSE(io.MouseDown[ImGuiMouseButton_Left]);
  EXPECT_FALSE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f), ImGuiCond_Always);
  ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_NoTitleBar);
  EXPECT_FALSE(ImGui::Button("Action", ImVec2(120.0f, 40.0f)));
  ImGui::End();
  ImGui::Render();

  // Frame N+1 (1 -> 2): Second finger lands immediately at (340, 550), so
  // midpoint origin is (200, 300). Stripping the unconsumed press prevents any
  // click or double-click and places MousePos at origin on this same frame.
  StripUnconsumedTouchMousePress();
  io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  io.AddMousePosEvent(200.0f, 300.0f);
  ImGui::NewFrame();
  EXPECT_FALSE(io.MouseDown[ImGuiMouseButton_Left]);
  EXPECT_FALSE(ImGui::IsMouseClicked(ImGuiMouseButton_Left));
  EXPECT_FALSE(ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left));
  EXPECT_FLOAT_EQ(io.MousePos.x, 200.0f);
  EXPECT_FLOAT_EQ(io.MousePos.y, 300.0f);
  ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f), ImGuiCond_Always);
  ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_NoTitleBar);
  EXPECT_FALSE(ImGui::Button("Action", ImVec2(120.0f, 40.0f)));
  ImGui::End();
  ImGui::Render();
}

TEST_F(ImGuiTouchWidgetsTest, AtomicOffscreenCancelAndHitTestImGuiWindowsRespectPanelsAndDockHole) {
  ImGuiIO& io = ImGui::GetIO();

  auto draw_ui = [](bool* button_clicked, bool open_popup = false) {
    // Full-screen passthrough dockspace (like Studio's main workspace).
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos);
    ImGui::SetNextWindowSize(vp->Size);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("Dockspace", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                     ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PopStyleVar();
    const ImGuiID dock_id = ImGui::GetID("RootDock");
    ImGui::DockSpace(dock_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::End();

    // Floating local panel at [20..220, 20..120].
    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(200.0f, 100.0f), ImGuiCond_Always);
    ImGui::Begin("DisconnectLikePanel", nullptr, ImGuiWindowFlags_NoTitleBar);
    *button_clicked = ImGui::Button("Steal", ImVec2(100.0f, 40.0f));
    if (open_popup) {
      ImGui::OpenPopup("TestPopup");
    }
    if (ImGui::BeginPopup("TestPopup")) {
      ImGui::TextUnformatted("Popup content");
      ImGui::EndPopup();
    }
    ImGui::End();
  };

  bool clicked = false;
  // Warm up layout.
  ImGui::NewFrame();
  draw_ui(&clicked);
  ImGui::Render();

  // Finger 1 presses and holds on "Steal" at (50, 45) across two frames so
  // io.MouseDown[0] is true and "Steal" is ActiveId.
  io.AddMouseSourceEvent(ImGuiMouseSource_TouchScreen);
  io.AddMousePosEvent(50.0f, 45.0f);
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
  ImGui::NewFrame();
  draw_ui(&clicked);
  ImGui::Render();
  ImGui::NewFrame();
  ASSERT_TRUE(io.MouseDown[ImGuiMouseButton_Left]);
  draw_ui(&clicked);
  ImGui::Render();
  EXPECT_FALSE(clicked);

  // Frame N+2 (1 -> 2 over the panel): park cursor at (-FLT_MAX, -FLT_MAX) and
  // release left button with trickling disabled for this frame (matching
  // Window::NewFrame).
  io.AddMouseSourceEvent(ImGuiMouseSource_Mouse);
  io.AddMousePosEvent(-FLT_MAX, -FLT_MAX);
  io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
  const bool prev_trickle = io.ConfigInputTrickleEventQueue;
  io.ConfigInputTrickleEventQueue = false;
  ImGui::NewFrame();
  io.ConfigInputTrickleEventQueue = prev_trickle;

  // Even though io.WantCaptureMouse is now false (cursor is off-screen),
  // HitTestImGuiWindows accurately identifies the panel vs passthrough hole,
  // and ignores ImGui's unused "Debug##Default" fallback window at [60..460,
  // 60..460] that NewFrame() transiently activates before EndFrame() hides it!
  EXPECT_FALSE(io.MouseDown[ImGuiMouseButton_Left]);
  EXPECT_FALSE(io.WantCaptureMouse);
  EXPECT_TRUE(HitTestImGuiWindows(ImVec2(80.0f, 60.0f)));  // Inside panel.
  EXPECT_FALSE(
      HitTestImGuiWindows(ImVec2(250.0f, 250.0f)));  // Inside Debug##Default rect, outside panels.
  EXPECT_FALSE(HitTestImGuiWindows(ImVec2(250.0f, 500.0f)));  // Dockspace hole.
  EXPECT_FALSE(HitTestImGuiWindows(ImVec2(std::numeric_limits<float>::quiet_NaN(), 50.0f)));

  draw_ui(&clicked);
  ImGui::Render();
  // Button release happened off-screen, so the click was cancelled cleanly!
  EXPECT_FALSE(clicked);

  // When a popup is open, HitTestImGuiWindows captures everywhere.
  ImGui::NewFrame();
  draw_ui(&clicked, /*open_popup=*/true);
  ImGui::Render();
  ImGui::NewFrame();
  EXPECT_TRUE(HitTestImGuiWindows(ImVec2(250.0f, 500.0f)));
  draw_ui(&clicked);
  ImGui::Render();
}

}  // namespace
}  // namespace mujoco::studio
