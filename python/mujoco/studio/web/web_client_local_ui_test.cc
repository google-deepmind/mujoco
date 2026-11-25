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

#include "web_client_local_ui.h"

#include <string>
#include <string_view>

#include <gtest/gtest.h>
#include <imgui.h>
#include <imgui_internal.h>
#include "studio/ux/fonts.h"
#include "studio/ux/imgui_widgets.h"
#include "studio/ux/theme.h"
#include "role_transitions.h"
#include "web_client_session.h"

namespace mujoco::studio {
namespace {

class NoopActions : public SessionActions {
 public:
  void RequestControl() override {}
  void LeaveQueue() override {}
  void ReleaseControl() override {}
  void StealControl() override {}
  void SetCameraMode(int mode) override {}
  void SetMaxSpectators(int max_spectators) override {}
  void SetFpsCap(int fps_cap) override {}
};

class LocalUiLayoutTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ctx_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(ctx_);
    AddStudioFonts([](std::string_view file) {
      return LoadFontAsset("third_party/mujoco/studio/data", file);
    });
    ImGuiIO& io = ImGui::GetIO();
    unsigned char* pixels = nullptr;
    int w = 0, h = 0;
    io.Fonts->GetTexDataAsRGBA32(&pixels, &w, &h);
    io.Fonts->SetTexID(static_cast<ImTextureID>(1));
  }

  void TearDown() override {
    ImGui::DestroyContext(ctx_);
    ctx_ = nullptr;
  }

  void ExpectRoleBarFitsOnFirstFrame(float viewport_w,
                                     const SessionView& view) {
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(viewport_w, 780.0f);
    ImGui::NewFrame();

    RoleWindow role_window;  // Cold instance: tests Frame 1 analytic placement.
    NoopActions actions;
    GuiTheme theme = GuiTheme::kDark;
    role_window.Draw(view, actions, &theme);

    ImGuiWindow* bar = ImGui::FindWindowByName("##RoleBar");
    ASSERT_NE(bar, nullptr);
    EXPECT_FLOAT_EQ(bar->Size.x, viewport_w);
    const float max_allowed_x =
        bar->Pos.x + bar->Size.x - bar->WindowPadding.x + 0.5f;
    EXPECT_LE(bar->DC.IdealMaxPos.x, max_allowed_x)
        << "Overflow at viewport_w=" << viewport_w
        << " role=" << static_cast<int>(view.role)
        << " downloading=" << view.is_downloading
        << " retry=" << view.retry_count;
    ImGui::Render();
  }

  ImGuiContext* ctx_ = nullptr;
};

TEST_F(LocalUiLayoutTest,
       RoleBarFitsNarrowPortraitPhonesOnFirstFrameWithoutClipping) {
  for (float width : {350.0f, 360.0f, 375.0f, 390.0f, 1280.0f}) {
    // Claiming (connecting).
    SessionView claiming{};
    claiming.is_downloading = false;
    claiming.role = SessionRole::kClaiming;
    ExpectRoleBarFitsOnFirstFrame(width, claiming);

    // Spectating across all camera modes (unqueued & queued, fresh & stalled).
    for (int cam_mode : {0, 1, 2}) {
      for (int queue_pos : {0, 2}) {
        for (double staleness : {0.02, 5.0}) {
          SessionView spectating{};
          spectating.is_downloading = false;
          spectating.role = SessionRole::kSpectating;
          spectating.camera_mode = cam_mode;
          spectating.queue_pos = queue_pos;
          spectating.queue_len = queue_pos > 0 ? 3 : 0;
          spectating.seconds_since_last_payload = staleness;
          ExpectRoleBarFitsOnFirstFrame(width, spectating);
        }
      }
    }

    // Controlling (idle queue & active queue, fresh & stalled).
    for (int queue_len : {0, 5}) {
      for (double staleness : {0.02, 5.0}) {
        SessionView controlling{};
        controlling.is_downloading = false;
        controlling.role = SessionRole::kControlling;
        controlling.viewers = 4;
        controlling.max_spectators = 16;
        controlling.queue_len = queue_len;
        controlling.seconds_since_last_payload = staleness;
        ExpectRoleBarFitsOnFirstFrame(width, controlling);
      }
    }

    // Downloading (normal & chunk retry).
    for (int retry : {0, 3}) {
      SessionView downloading{};
      downloading.is_downloading = true;
      downloading.bytes_downloaded = 5 * 1024 * 1024;
      downloading.total_bytes = 12 * 1024 * 1024;
      downloading.retry_count = retry;
      ExpectRoleBarFitsOnFirstFrame(width, downloading);
    }
  }
}

TEST_F(LocalUiLayoutTest,
       DisconnectNoticeWrapsWithinNarrowPhoneViewportAndHitTestsCleanly) {
  ImGuiIO& io = ImGui::GetIO();
  io.DisplaySize = ImVec2(360.0f, 780.0f);

  // 1. Server silence notice on Frame 1.
  {
    ImGui::NewFrame();
    DisconnectNotice notice;
    notice.Draw(/*server_close_code=*/0, /*server_close_reason=*/"",
                /*seconds_since_last_payload=*/5.0, /*is_downloading=*/false,
                /*bottom_inset=*/32.0f);
    ImGuiWindow* win = ImGui::FindWindowByName("##disconnected");
    ASSERT_NE(win, nullptr);
    EXPECT_LE(win->Size.x, 360.0f - 24.0f + 0.5f);
    EXPECT_GE(win->Pos.x, 11.5f);
    EXPECT_LE(win->Pos.x + win->Size.x, 360.0f - 11.5f);
    ImGui::Render();
    ImGui::NewFrame();
    EXPECT_TRUE(HitTestImGuiWindows(ImVec2(180.0f, 374.0f)));
    ImGui::EndFrame();
  }

  // 2. Viewer error notice with an overlong unbroken traceback token.
  {
    const std::string long_reason(120, 'X');
    ImGui::NewFrame();
    DisconnectNotice notice;
    notice.Draw(kWsCloseViewerError, long_reason.c_str(),
                /*seconds_since_last_payload=*/0.1, /*is_downloading=*/false,
                /*bottom_inset=*/32.0f);
    ImGuiWindow* win = ImGui::FindWindowByName("##disconnected_by_server");
    ASSERT_NE(win, nullptr);
    EXPECT_LE(win->Size.x, 360.0f - 24.0f + 0.5f);
    ImGui::Render();
  }
}

}  // namespace
}  // namespace mujoco::studio
