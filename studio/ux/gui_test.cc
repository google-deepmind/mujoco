// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "studio/ux/gui.h"

#include "testing/base/public/gunit.h"
#include <imgui.h>
#include <imgui_internal.h>

namespace mujoco::studio {
namespace {

ImGuiContext* CreateDockTestContext(ImVec2 display_size = ImVec2(1280.0f, 720.0f)) {
  ImGuiContext* ctx = ImGui::CreateContext();
  ImGui::SetCurrentContext(ctx);
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = display_size;
  io.DeltaTime = 1.0f / 60.0f;
  io.IniFilename = nullptr;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.ConfigDpiScaleFonts = true;
  unsigned char* pixels = nullptr;
  int width = 0;
  int height = 0;
  io.Fonts->GetTexDataAsRGBA32(&pixels, &width, &height);
  return ctx;
}

struct DockWidths {
  float left = 0.0f;
  float inspector = 0.0f;
};

DockWidths StepDockFrame(ImGuiContext* ctx, float dpi_scale) {
  ImGui::SetCurrentContext(ctx);
  ImGui::GetMainViewport()->DpiScale = dpi_scale;
  ImGui::NewFrame();
  ConfigureDockingLayout(false, false);
  const ImGuiID root_id = ImGui::GetID("Root");
  ImGui::EndFrame();
  ImGuiDockNode* root = ImGui::DockBuilderGetNode(root_id);
  if (!root || !root->ChildNodes[0] || !root->ChildNodes[1] ||
      !root->ChildNodes[1]->ChildNodes[1]) {
    return {};
  }
  return {root->ChildNodes[0]->SizeRef.x, root->ChildNodes[1]->ChildNodes[1]->SizeRef.x};
}

TEST(GuiTest, ConfigureDockingLayoutAppliesPixelFloorsAt1x) {
  ImGuiContext* ctx = CreateDockTestContext(ImVec2(1280.0f, 720.0f));
  const DockWidths widths = StepDockFrame(ctx, 1.0f);
  EXPECT_NEAR(widths.left, 248.0f, 2.0f);
  EXPECT_NEAR(widths.inspector, 255.0f, 2.0f);
  ImGui::DestroyContext(ctx);
}

TEST(GuiTest, ConfigureDockingLayoutUsesWindowDpiScale) {
  ImGuiContext* ctx = CreateDockTestContext(ImVec2(1920.0f, 1080.0f));
  const DockWidths widths = StepDockFrame(ctx, 2.0f);
  EXPECT_NEAR(widths.left, 496.0f, 2.0f);
  EXPECT_NEAR(widths.inspector, 510.0f, 2.0f);
  ImGuiWindow* dock_win = ImGui::FindWindowByName("Dockspace");
  ASSERT_NE(dock_win, nullptr);
  EXPECT_NE(dock_win->Flags & ImGuiWindowFlags_NoSavedSettings, 0);
  ImGui::DestroyContext(ctx);
}

TEST(GuiTest, ConfigureDockingLayoutAppliesFloorsAtThresholdWidth) {
  ImGuiContext* ctx = CreateDockTestContext(ImVec2(1440.0f, 900.0f));
  const DockWidths widths = StepDockFrame(ctx, 2.0f);
  EXPECT_NEAR(widths.left, 496.0f, 2.0f);
  EXPECT_NEAR(widths.inspector, 510.0f, 2.0f);
  ImGui::DestroyContext(ctx);
}

TEST(GuiTest, ConfigureDockingLayoutKeepsDefaultRatiosOnNarrowViewports) {
  ImGuiContext* phone_ctx = CreateDockTestContext(ImVec2(390.0f, 844.0f));
  const DockWidths phone_widths = StepDockFrame(phone_ctx, 1.0f);
  EXPECT_NEAR(phone_widths.left, 390.0f * 0.15f, 2.0f);
  EXPECT_NEAR(phone_widths.inspector, 390.0f * 0.85f * 0.22f, 2.0f);
  ImGui::DestroyContext(phone_ctx);

  ImGuiContext* hidpi_narrow_ctx = CreateDockTestContext(ImVec2(1280.0f, 720.0f));
  const DockWidths hidpi_widths = StepDockFrame(hidpi_narrow_ctx, 2.0f);
  EXPECT_NEAR(hidpi_widths.left, 1280.0f * 0.15f, 2.0f);
  EXPECT_NEAR(hidpi_widths.inspector, 1280.0f * 0.85f * 0.22f, 2.0f);
  ImGui::DestroyContext(hidpi_narrow_ctx);
}

TEST(GuiTest, ConfigureDockingLayoutRescalesAcrossInterleavedContexts) {
  ImGuiContext* ctx_a = CreateDockTestContext();
  ImGuiContext* ctx_b = CreateDockTestContext();

  const DockWidths a_before = StepDockFrame(ctx_a, 1.0f);
  const DockWidths b_before = StepDockFrame(ctx_b, 1.0f);
  const DockWidths b_mid = StepDockFrame(ctx_b, 1.0f);
  const DockWidths a_after = StepDockFrame(ctx_a, 2.0f);
  const DockWidths b_after = StepDockFrame(ctx_b, 1.0f);

  EXPECT_NEAR(a_before.left, 248.0f, 2.0f);
  EXPECT_NEAR(a_before.inspector, 255.0f, 2.0f);
  EXPECT_NEAR(b_mid.left, b_before.left, 1.0f);
  EXPECT_NEAR(a_after.left, 2.0f * a_before.left, 1.0f);
  EXPECT_NEAR(a_after.inspector, 2.0f * a_before.inspector, 1.0f);
  EXPECT_NEAR(b_after.left, b_before.left, 1.0f);

  ImGui::DestroyContext(ctx_b);
  ImGui::DestroyContext(ctx_a);
}

}  // namespace
}  // namespace mujoco::studio
