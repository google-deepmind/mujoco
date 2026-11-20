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

#include "studio/ux/status_bar.h"

#include <functional>

#include "testing/base/public/gunit.h"
#include <imgui.h>
#include <imgui_internal.h>

namespace mujoco::studio {
namespace {

class StatusBarTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.DisplaySize = ImVec2(800, 600);
    io.DeltaTime = 1.0f / 60.0f;
    unsigned char* pixels = nullptr;
    int width = 0;
    int height = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  }
  void TearDown() override { ImGui::DestroyContext(); }

  // Draws the status bar for a few frames and clicks at the point `click_at`
  // picks from the bar's window as laid out on the first frame. Returns the
  // resulting show_console state, starting from false.
  bool ClickBar(const std::function<ImVec2(const ImGuiWindow&)>& click_at) {
    bool show_console = false;
    ImVec2 click_pos;
    for (int frame = 0; frame < 6; ++frame) {
      ImGuiIO& io = ImGui::GetIO();
      if (frame == 1) io.AddMousePosEvent(click_pos.x, click_pos.y);
      if (frame == 3) io.AddMouseButtonEvent(ImGuiMouseButton_Left, true);
      if (frame == 4) io.AddMouseButtonEvent(ImGuiMouseButton_Left, false);
      ImGui::NewFrame();
      // The bar's window, as ConfigureDockingLayout creates it.
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      ImGui::SetNextWindowSize(ImVec2(800, 32));
      ImGui::Begin("StatusBar", nullptr,
                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar);
      ImGui::End();
      StatusBarGui("Ready", &show_console);
      if (frame == 0) {
        const ImGuiWindow* window = ImGui::FindWindowByName("StatusBar");
        EXPECT_NE(window, nullptr);
        if (window != nullptr) click_pos = click_at(*window);
      }
      ImGui::Render();
    }
    return show_console;
  }
};

TEST_F(StatusBarTest, ClickingTheConsoleButtonTogglesTheConsole) {
  // The console button sits flush with the right edge of the bar's work area.
  EXPECT_TRUE(ClickBar([](const ImGuiWindow& window) {
    return ImVec2(window.WorkRect.Max.x - 2.0f, window.InnerRect.GetCenter().y);
  }));
}

TEST_F(StatusBarTest, ClickingTheStatusTextDoesNotToggleTheConsole) {
  EXPECT_FALSE(ClickBar([](const ImGuiWindow& window) {
    return ImVec2(window.WorkRect.Min.x + 2.0f, window.InnerRect.GetCenter().y);
  }));
}

TEST_F(StatusBarTest, StatusSeparatorKeepsItemsOnOneLineWithAGap) {
  ImGui::NewFrame();
  ImGui::Begin("Bar");
  ImGui::TextUnformatted("Left");
  const ImVec2 left_min = ImGui::GetItemRectMin();
  const ImVec2 left_max = ImGui::GetItemRectMax();
  StatusSeparatorGui();
  ImGui::TextUnformatted("Right");
  const ImVec2 right_min = ImGui::GetItemRectMin();
  const float dot_width = ImGui::CalcTextSize("\xC2\xB7").x;
  ImGui::End();
  ImGui::Render();

  // Same line: "Right" starts above the bottom of "Left" (the separator may
  // align it to the frame padding, so the tops need not match exactly).
  EXPECT_LT(right_min.y, left_max.y);
  EXPECT_GE(right_min.y, left_min.y);
  // The dot and a gap on each side of it separate the two items.
  EXPECT_GT(right_min.x - left_max.x, dot_width);
}

}  // namespace
}  // namespace mujoco::studio
