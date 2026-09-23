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

#include <algorithm>
#include <cfloat>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>

#include <imgui.h>
#include "experimental/studio/ux/imgui_widgets.h"
#include "google/logging.h"
#include "web_client_session.h"

namespace mujoco::studio {
namespace {

const ImVec2 kFullWidth(-FLT_MIN, 0.0f);
const ImVec4 kSpectatingColor(1.0f, 0.75f, 0.2f, 1.0f);
const ImVec4 kControllingColor(0.3f, 0.9f, 0.4f, 1.0f);
const ImVec4 kQueueColor(1.0f, 0.62f, 0.15f, 1.0f);
const ImVec4 kConnectingColor(0.6f, 0.6f, 0.6f, 1.0f);

// Draws one screen-centered DISCONNECTED window.
void DrawDisconnectWindow(const char* window_id,
                          std::initializer_list<const char*> lines) {
  const ImGuiIO& io = ImGui::GetIO();
  ImGui::SetNextWindowPos(
      ImVec2(io.DisplaySize.x * 0.5f, io.DisplaySize.y * 0.5f),
      ImGuiCond_Always, ImVec2(0.5f, 0.5f));
  ImGui::Begin(window_id, nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_AlwaysAutoResize);
  CenteredBanner("DISCONNECTED", ImVec4(1.0f, 0.3f, 0.3f, 1.0f));
  for (const char* line : lines) {
    CenteredLine(line, nullptr);
  }
  ImGui::End();
}

}  // namespace

void DisconnectNotice::Draw(int server_close_code,
                            double seconds_since_last_payload,
                            bool is_downloading) {
  if (server_close_code != 0) {
    const char* reason = "Disconnected by the viewer.";
    if (server_close_code == kWsCloseSessionFull) {
      reason = "Session is full: too many viewers connected.";
    } else if (server_close_code == kWsCloseInactive) {
      reason = "Disconnected after inactivity.";
    }
    DrawDisconnectWindow("##disconnected_by_server",
                         {reason, "Retrying; reconnects automatically."});
  }

  // The silence notice keys on state-stream staleness rather than socket
  // state: a killed, suspended (Ctrl+Z), or unreachable server all go
  // silent, but only a killed one closes its sockets. Clears itself when
  // traffic resumes. Suppressed before the first payload (negative
  // staleness), after a deliberate server-side close (its own notice
  // above), and while a new model is downloading.
  const bool link_stale =
      seconds_since_last_payload > kServerSilenceNoticeSec &&
      server_close_code == 0 && !is_downloading;
  if (!link_stale) {
    logged_ = false;
    return;
  }
  if (!logged_) {
    LOG(Info, "Showing server-not-reachable notice");
    logged_ = true;
  }
  DrawDisconnectWindow(
      "##disconnected",
      {"Viewer server is not reachable: the Python script may",
       "have stopped. This page reconnects automatically if the",
       "viewer comes back."});
}

void RoleWindow::Draw(const SessionView& view, SessionActions& actions) {
  ComputeAnchorPositions();
  // While anchored, the window is re-pinned every frame (so it follows
  // canvas resizes) for both the collapsed pill and the expanded window
  // (same ImGui window, so one call covers whichever Begin runs this
  // frame) — except while it is being dragged.
  if (anchor_ != kAnchorFree && !dragging_) {
    ImGui::SetNextWindowPos(anchor_pos_[anchor_ - kAnchorTopLeft],
                            ImGuiCond_Always,
                            kAnchorPivot[anchor_ - kAnchorTopLeft]);
  }

  // While someone waits for control, the window background pulses toward
  // a dark orange (the queue size itself is shown in the expanded form).
  const bool pulse =
      view.role == SessionRole::kControlling && view.queue_len > 0;
  if (pulse) {
    const float phase =
        0.5f + 0.5f * sinf(static_cast<float>(ImGui::GetTime()) * 2.5f);
    ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg);
    const ImVec4 orange(0.45f, 0.22f, 0.02f, bg.w);
    const float blend = 0.6f * phase;
    bg.x += (orange.x - bg.x) * blend;
    bg.y += (orange.y - bg.y) * blend;
    bg.z += (orange.z - bg.z) * blend;
    ImGui::PushStyleColor(ImGuiCol_WindowBg, bg);
  }

  if (mode_ == Mode::kCollapsed) {
    DrawCollapsed(view);
  } else {
    DrawExpanded(view, actions);
  }

  if (pulse) {
    ImGui::PopStyleColor();
  }
}

void RoleWindow::ComputeAnchorPositions() {
  const ImVec2 canvas = ImGui::GetIO().DisplaySize;
  anchor_pos_[0] = ImVec2(kEdgeMargin, kEdgeMargin);
  anchor_pos_[1] = ImVec2(canvas.x * 0.5f, kEdgeMargin);
  anchor_pos_[2] = ImVec2(canvas.x - kEdgeMargin, kEdgeMargin);
  anchor_pos_[3] = ImVec2(kEdgeMargin, canvas.y - kEdgeMargin);
  anchor_pos_[4] = ImVec2(canvas.x * 0.5f, canvas.y - kEdgeMargin);
  anchor_pos_[5] = ImVec2(canvas.x - kEdgeMargin, canvas.y - kEdgeMargin);
}

// Tracks a drag of the window and snaps on release: if the window was
// dropped within max(20% of its size, 100px) of where an anchor would
// place it, adopt that anchor; otherwise it floats Free where it was
// dropped. Call between Begin and End of whichever form is visible.
void RoleWindow::UpdateSnap() {
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) &&
      ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
    dragging_ = true;
  } else if (dragging_ && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
    dragging_ = false;
    const ImVec2 pos = ImGui::GetWindowPos();
    const ImVec2 size = ImGui::GetWindowSize();
    const float threshold = std::max(0.2f * std::max(size.x, size.y), 100.0f);
    int best = -1;
    float best_d2 = threshold * threshold;
    for (int i = 0; i < 6; ++i) {
      // Per-axis distance to where this anchor would place the window,
      // clamped to zero on the anchor's off-screen side: a window dropped
      // past an edge or corner (mostly off-screen) still snaps to that
      // anchor instead of being left stranded outside the canvas.
      float dx = pos.x - (anchor_pos_[i].x - kAnchorPivot[i].x * size.x);
      if (kAnchorPivot[i].x == 0.0f) {
        dx = std::max(dx, 0.0f);  // Dropped past the left edge.
      } else if (kAnchorPivot[i].x == 1.0f) {
        dx = std::min(dx, 0.0f);  // Dropped past the right edge.
      }
      float dy = pos.y - (anchor_pos_[i].y - kAnchorPivot[i].y * size.y);
      if (kAnchorPivot[i].y == 0.0f) {
        dy = std::max(dy, 0.0f);  // Dropped past the top edge.
      } else {
        dy = std::min(dy, 0.0f);  // Dropped past the bottom edge.
      }
      const float d2 = dx * dx + dy * dy;
      if (d2 <= best_d2) {
        best_d2 = d2;
        best = i;
      }
    }
    anchor_ =
        best >= 0 ? static_cast<Anchor>(kAnchorTopLeft + best) : kAnchorFree;
  }
}

// Data rates, shown to both roles (spectators receive no GUI stream, so
// that line reads 0 for them). With more than one viewer, the number in
// parentheses is the host's total outgoing sim bandwidth: every connected
// browser receives the same sim stream.
void RoleWindow::DataRateLines(const SessionView& view) {
  ImGui::Text("GUI Data Rate: %" PRIu64 " KiB/s",
              static_cast<uint64_t>(view.gui_bytes_per_sec / 1024));
  if (view.role == SessionRole::kSpectating) {
    ImGui::SetItemTooltip("The UI is not streamed to spectators.");
  } else if (!view.have_remote_frame) {
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "(Waiting...)");
  }
  ImGui::Text("Sim Data Rate: %" PRIu64 " KiB/s",
              static_cast<uint64_t>(view.sim_bytes_per_sec / 1024));
  if (view.viewers > 1) {
    ImGui::SameLine();
    ImGui::Text(
        "(%" PRIu64 " KiB/s)",
        static_cast<uint64_t>(view.sim_bytes_per_sec * view.viewers / 1024));
    ImGui::SetItemTooltip("Total sim data sent across all viewers.");
  }
}

void RoleWindow::DrawCollapsed(const SessionView& view) {
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(6.0f, 6.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, ImVec2(16.0f, 16.0f));

  ImGui::Begin("Role", nullptr,
               ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_AlwaysAutoResize);
  if (view.is_downloading) {
    bool done_downloading =
        view.total_bytes > 0 && view.bytes_downloaded >= view.total_bytes;
    ImGui::TextColored(kConnectingColor,
                       done_downloading ? "PARSING..." : "DOWNLOADING...");
  } else if (view.role == SessionRole::kSpectating) {
    ImGui::TextColored(kSpectatingColor, "SPECTATING");
  } else if (view.role == SessionRole::kControlling) {
    ImGui::TextColored(kControllingColor, "CONTROLLING");
  } else {
    ImGui::TextColored(kConnectingColor, "CONNECTING...");
  }
  // Keep the window expanded while focused or being dragged, so dragging out
  // of the collapsed bounds doesn't instantly collapse it midway through
  // a move (which drops drag capture in ImGui).
  if (ImGui::IsWindowFocused() ||
      ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
    mode_ = Mode::kExpanded;
  }
  UpdateSnap();
  ImGui::End();

  ImGui::PopStyleVar(2);
}

void RoleWindow::DrawExpanded(const SessionView& view,
                              SessionActions& actions) {
  // No title bar; the window is still movable by dragging empty space
  // (ImGui's default when ConfigWindowsMoveFromTitleBarOnly is off).
  ImGui::Begin("Role", nullptr,
               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar);

  if (view.is_downloading) {
    bool done_downloading =
        view.total_bytes > 0 && view.bytes_downloaded >= view.total_bytes;
    CenteredBanner(done_downloading ? "PARSING..." : "DOWNLOADING...",
                             kConnectingColor);
    if (!done_downloading) {
      float progress = view.total_bytes > 0
                           ? static_cast<float>(view.bytes_downloaded) /
                                 static_cast<float>(view.total_bytes)
                           : 0.0f;
      char buf[128];
      if (view.total_bytes > 0) {
        snprintf(buf, sizeof(buf), "%.1f / %.1f MB (%.0f%%)",
                 view.bytes_downloaded / (1024.0 * 1024.0),
                 view.total_bytes / (1024.0 * 1024.0), progress * 100.0f);
      } else {
        snprintf(buf, sizeof(buf), "Connecting...");
      }
      ImGui::ProgressBar(progress, ImVec2(-1, 0), buf);
      if (view.retry_count > 0) {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f),
                           "Retrying chunk (%d)...", view.retry_count);
      }
    }
  } else if (view.role == SessionRole::kClaiming) {
    // The first roster or /ui claim outcome resolves this within a few
    // hundred milliseconds of page load.
    CenteredBanner("CONNECTING...", kConnectingColor);
    ImGui::Separator();
    DataRateLines(view);
  } else if (view.role == SessionRole::kSpectating) {
    DrawSpectatorContents(view, actions);
  } else {
    DrawControllerContents(view, actions);
  }

  UpdateSnap();
  UpdateCollapse();
  ImGui::End();
}

void RoleWindow::DrawSpectatorContents(const SessionView& view,
                                       SessionActions& actions) {
  // Control group.
  CenteredBanner("SPECTATING", kSpectatingColor);
  char queue_text[64];
  if (view.queue_pos > 0) {
    snprintf(queue_text, sizeof(queue_text), "Control Queue: Position %d of %d",
             view.queue_pos, view.queue_len);
  } else if (view.queue_len > 0) {
    snprintf(queue_text, sizeof(queue_text), "Control Queue: %d waiting",
             view.queue_len);
  } else {
    snprintf(queue_text, sizeof(queue_text), "Control Queue: (empty)");
  }
  CenteredLine(queue_text, nullptr);
  if (view.queue_pos == 0) {
    if (ImGui::Button("Request control", kFullWidth)) {
      actions.RequestControl();
    }
  } else {
    const float half_width =
        (ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) *
        0.5f;
    if (ImGui::Button("Leave Queue", ImVec2(half_width, 0.0f))) {
      actions.LeaveQueue();
    }
    ImGui::SetItemTooltip("Abandons the control request.");
    ImGui::SameLine();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.15f, 0.15f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                          ImVec4(0.80f, 0.20f, 0.20f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          ImVec4(0.50f, 0.10f, 0.10f, 1.0f));
    if (ImGui::Button("Steal Control", ImVec2(half_width, 0.0f))) {
      actions.StealControl();
    }
    ImGui::PopStyleColor(3);
    ImGui::SetItemTooltip("Takes control immediately, jumping the queue.");
  }

  // Data rate group.
  ImGui::Separator();
  DataRateLines(view);

  // Camera group.
  ImGui::Separator();
  // TODO(matijak): Use the studio camera-selection UI here in future, so
  // a spectator can also pick any camera defined in the model.
  // Sized to its longest option; the default width would stretch the
  // window past the CONTROLLING layout's size.
  ImGui::SetNextItemWidth(ImGui::CalcTextSize("Follow Controller").x +
                          ImGui::GetFrameHeight() * 2.0f);
  int cam_mode = view.camera_mode;
  if (ImGui::Combo("Camera", &cam_mode,
                   "Free: tumble\0Free: wasd\0Follow Controller\0")) {
    actions.SetCameraMode(cam_mode);
  }
}

void RoleWindow::DrawControllerContents(const SessionView& view,
                                        SessionActions& actions) {
  // Control group.
  CenteredBanner("CONTROLLING", kControllingColor);
  if (view.queue_len > 0) {
    char queue_text[64];
    snprintf(queue_text, sizeof(queue_text), ">> Control Queue: %d waiting <<",
             view.queue_len);
    CenteredLine(queue_text, &kQueueColor);
  } else {
    CenteredLine("Control Queue: (empty)", nullptr);
  }
  if (ImGui::Button("Release control", kFullWidth)) {
    actions.ReleaseControl();
  }

  // Data rate group.
  ImGui::Separator();
  DataRateLines(view);

  // Session settings group.
  ImGui::Separator();
  // Local edits win for a grace period (the roster round trip takes a
  // few frames and typing takes longer); afterwards the server's value
  // is the truth.
  if (max_spectators_edit_ < 0 ||
      ImGui::GetTime() - max_spectators_edit_time_ > 1.5) {
    max_spectators_edit_ = view.max_spectators;
  }
  ImGui::SetNextItemWidth(100.0f);
  if (ImGui::InputInt("Max Spectators", &max_spectators_edit_)) {
    max_spectators_edit_time_ = ImGui::GetTime();
    max_spectators_edit_ = std::clamp(max_spectators_edit_, 0, 32);
    actions.SetMaxSpectators(max_spectators_edit_);
  }
  if (ImGui::IsItemActive()) {
    max_spectators_edit_time_ = ImGui::GetTime();
  }
}

// Collapse when the mouse leaves the window, with a short grace period.
// A popup (e.g. the camera combo's dropdown) is a separate window, so
// moving the mouse into it unhovers this one; keep the window expanded
// while one of its popups is open, and give brief excursions time to
// come back before snapping shut. On page load the window instead stays
// expanded for a few seconds (the intro); the first hover ends the
// intro, so a quick swipe over the window dismisses it early.
void RoleWindow::UpdateCollapse() {
  const bool popup_open = ImGui::IsPopupOpen(
      "", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel);
  if (popup_open || dragging_ ||
      ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows) ||
      ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
    mode_ = Mode::kExpanded;  // The first hover ends the intro.
    hover_time_ = ImGui::GetTime();
  } else if (mode_ == Mode::kIntro) {
    if (intro_start_ == 0) {
      intro_start_ = ImGui::GetTime();
    } else if (ImGui::GetTime() - intro_start_ > kIntroExpandedSec) {
      mode_ = Mode::kCollapsed;
    }
  } else if (ImGui::GetTime() - hover_time_ > kCollapseGraceSec) {
    mode_ = Mode::kCollapsed;
  }
}

}  // namespace mujoco::studio
