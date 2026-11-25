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

#include "studio/ux/clipboard.h"

#include <string>
#include <vector>

#include "testing/base/public/gmock.h"
#include "testing/base/public/gunit.h"
#include <imgui.h>

namespace mujoco::studio {
namespace {

// Runs frames with one text field, which takes keyboard focus on the first.
class PasteShortcutTest : public ::testing::Test {
 protected:
  PasteShortcutTest() : context_(ImGui::CreateContext()) {
    ImGui::SetCurrentContext(context_);
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1280, 720);
    io.IniFilename = nullptr;
    unsigned char* pixels = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsAlpha8(&pixels, &w, &h);
    io.Fonts->SetTexID(static_cast<ImTextureID>(7));
    // The field becomes active a frame after asking for focus, and owns its
    // shortcuts (including paste) a frame after that.
    RunFrame(/*focus=*/true);
    RunFrame();
    RunFrame();
  }

  ~PasteShortcutTest() override { ImGui::DestroyContext(context_); }

  void RunFrame(bool focus = false) {
    ImGui::NewFrame();
    ImGui::Begin("Paste");
    if (focus) ImGui::SetKeyboardFocusHere();
    ImGui::InputText("##field", text_, sizeof(text_));
    ImGui::End();
    ImGui::Render();
  }

  ImGuiContext* context_ = nullptr;
  char text_[64] = "";
};

TEST_F(PasteShortcutTest, PressPastesAndReleaseRestoresHeldKeys) {
  ImGuiIO& io = ImGui::GetIO();
  // The user holds Shift, as for a Shift+Insert paste in the browser.
  io.AddKeyEvent(ImGuiMod_Shift, true);
  RunFrame();
  ASSERT_TRUE(io.KeyShift);
  ImGui::SetClipboardText("pasted");

  PressPasteShortcut(io);
  RunFrame();
  EXPECT_STREQ(text_, "pasted");
  EXPECT_TRUE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_TRUE(io.KeyCtrl);
  EXPECT_FALSE(io.KeyShift);  // Lifted so the chord is exactly Ctrl+V.

  HeldPasteKeys held;
  held.shift = true;
  ReleasePasteShortcut(io, held);
  RunFrame();
  EXPECT_STREQ(text_, "pasted");  // Pasted once only.
  EXPECT_FALSE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_FALSE(io.KeyCtrl);
  EXPECT_TRUE(io.KeyShift);
}

TEST_F(PasteShortcutTest, PressAndReleaseQueuedTogetherPasteOnce) {
  ImGuiIO& io = ImGui::GetIO();
  ASSERT_TRUE(io.ConfigInputTrickleEventQueue);
  ImGui::SetClipboardText("pasted");

  // Dear ImGui plays a press and a release of the same key back on consecutive
  // frames, so the text field still sees the shortcut.
  PressPasteShortcut(io);
  ReleasePasteShortcut(io, HeldPasteKeys());
  RunFrame();
  EXPECT_STREQ(text_, "pasted");
  RunFrame();
  EXPECT_STREQ(text_, "pasted");
  EXPECT_FALSE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_FALSE(io.KeyCtrl);
}

TEST_F(PasteShortcutTest, PastesWithCmdHeldUnderMacOsBehaviors) {
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigMacOSXBehaviors = true;
  // The user holds Cmd, which Dear ImGui reports as Ctrl on macOS.
  io.AddKeyEvent(ImGuiMod_Super, true);
  RunFrame();
  ASSERT_TRUE(io.KeyCtrl);
  ImGui::SetClipboardText("pasted");

  PressPasteShortcut(io);
  RunFrame();
  EXPECT_STREQ(text_, "pasted");

  HeldPasteKeys held;
  held.super = true;
  ReleasePasteShortcut(io, held);
  RunFrame();
  EXPECT_STREQ(text_, "pasted");
  EXPECT_FALSE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_TRUE(io.KeyCtrl);  // Cmd is still held.
  EXPECT_FALSE(io.KeySuper);
}

TEST_F(PasteShortcutTest, QueuePasteShortcutPastesAndKeepsHeldKeys) {
  ImGuiIO& io = ImGui::GetIO();
  // The user holds Shift, as for a Shift+Insert paste in the browser.
  io.AddKeyEvent(ImGuiMod_Shift, true);
  RunFrame();
  ASSERT_TRUE(io.KeyShift);

  ImGui::SetClipboardText("pasted");
  QueuePasteShortcut(io);
  RunFrame();
  EXPECT_STREQ(text_, "pasted");
  RunFrame();
  EXPECT_STREQ(text_, "pasted");  // Pasted once only.
  EXPECT_FALSE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_FALSE(io.KeyCtrl);
  EXPECT_TRUE(io.KeyShift);

  // With macOS behaviors the user holds Cmd, which Dear ImGui reports as Ctrl.
  io.AddKeyEvent(ImGuiMod_Shift, false);
  io.ConfigMacOSXBehaviors = true;
  io.AddKeyEvent(ImGuiMod_Super, true);
  RunFrame();
  ASSERT_TRUE(io.KeyCtrl);
  ImGui::SetClipboardText("!");
  QueuePasteShortcut(io);
  RunFrame();
  EXPECT_STREQ(text_, "pasted!");
  RunFrame();
  EXPECT_FALSE(ImGui::IsKeyDown(ImGuiKey_V));
  EXPECT_TRUE(io.KeyCtrl);
  EXPECT_FALSE(io.KeySuper);
}

using ::testing::ElementsAre;
using ::testing::IsEmpty;

// Texts the copy callback was called with, standing in for the browser's clipboard.
std::vector<std::string>& CopiedTexts() {
  static std::vector<std::string> texts;
  return texts;
}

class ClipboardSyncTest : public ::testing::Test {
 protected:
  ClipboardSyncTest() : context_(ImGui::CreateContext()) {
    ImGui::SetCurrentContext(context_);
    CopiedTexts().clear();
    SetClipboardCopyCallback([](const char* text) { CopiedTexts().push_back(text); });
  }

  ~ClipboardSyncTest() override {
    SetClipboardCopyCallback(nullptr);
    ImGui::DestroyContext(context_);
  }

  ImGuiContext* context_ = nullptr;
};

TEST_F(ClipboardSyncTest, TextCopiedInTheUiAlsoGoesToTheCallback) {
  ImGui::SetClipboardText("copied");
  EXPECT_STREQ(ImGui::GetClipboardText(), "copied");
  EXPECT_THAT(CopiedTexts(), ElementsAre("copied"));

  // Copying the same text again is written again: the browser's clipboard may have changed since.
  ImGui::SetClipboardText("copied");
  EXPECT_THAT(CopiedTexts(), ElementsAre("copied", "copied"));
}

TEST_F(ClipboardSyncTest, TextFromTheBrowserIsNotWrittenBack) {
  SetClipboardTextFromBrowser("pasted");
  EXPECT_STREQ(ImGui::GetClipboardText(), "pasted");
  EXPECT_THAT(CopiedTexts(), IsEmpty());

  // Only that one call is exempt.
  ImGui::SetClipboardText("copied");
  EXPECT_THAT(CopiedTexts(), ElementsAre("copied"));
}

TEST_F(ClipboardSyncTest, SettingTheCallbackTwiceDoesNotCallItTwice) {
  SetClipboardCopyCallback([](const char* text) { CopiedTexts().push_back(text); });
  ImGui::SetClipboardText("copied");
  EXPECT_STREQ(ImGui::GetClipboardText(), "copied");
  EXPECT_THAT(CopiedTexts(), ElementsAre("copied"));
}

}  // namespace
}  // namespace mujoco::studio
