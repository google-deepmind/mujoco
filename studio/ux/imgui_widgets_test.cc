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

}  // namespace
}  // namespace mujoco::studio
