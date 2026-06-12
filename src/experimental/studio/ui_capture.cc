// Copyright 2025 DeepMind Technologies Limited
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

// Scripted, headless UI capture used to record GIFs of the Studio UI. It drives
// the app's own state (opening/closing windows, the command palette) on a
// timeline and draws a synthetic cursor, dumping one framebuffer per frame.

#include "experimental/studio/app.h"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>

#include <imgui.h>

namespace mujoco::studio {

void App::ToggleToolWindowByName(const std::string& title) {
  for (ToolWindow& tw : tool_windows_) {
    if (tw.title == title) {
      tw.open = !tw.open;
      return;
    }
  }
}

void App::StartCapture(const std::string& out_dir, int total_frames) {
  capture_.active = true;
  capture_.out_dir = out_dir;
  capture_.frame = 0;
  capture_.total_frames = total_frames;
  capture_.cursor = ImVec2(-100.0f, -100.0f);
  capture_.click_flash = 0.0f;
}

bool App::SaveCaptureFrame() {
  if (!capture_.active) {
    return false;
  }
  const int w = static_cast<int>(window_->GetWidth());
  const int h = static_cast<int>(window_->GetHeight());
  if (pixels_.size() == static_cast<std::size_t>(w) * h * 3) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/frame_%04d.ppm",
                  capture_.out_dir.c_str(), capture_.frame);
    if (std::FILE* file = std::fopen(path, "wb")) {
      std::fprintf(file, "P6\n%d %d\n255\n", w, h);
      std::fwrite(pixels_.data(), 1, pixels_.size(), file);
      std::fclose(file);
    }
  }
  ++capture_.frame;
  if (capture_.frame >= capture_.total_frames) {
    capture_.active = false;
  }
  return capture_.active;
}

void App::CaptureStep() {
  CaptureState& c = capture_;
  const int f = c.frame;

  // Cursor targets recorded during rendering (with sane fallbacks).
  auto button = [&](const char* name) -> ImVec2 {
    auto it = rail_button_center_.find(name);
    return it != rail_button_center_.end() ? it->second : ImVec2(40.0f, 200.0f);
  };
  auto window_close = [&](const char* name) -> ImVec2 {
    auto it = tool_window_rect_.find(name);
    if (it == tool_window_rect_.end()) {
      return ImVec2(400.0f, 100.0f);
    }
    const ImVec4& r = it->second;  // (x, y, w, h)
    return ImVec2(r.x + r.z - 12.0f, r.y + 11.0f);  // the window's 'x'
  };
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const ImVec2 palette_target(vp->WorkPos.x + vp->WorkSize.x * 0.5f,
                              tmp_.top_overlay_bottom + 28.0f);

  auto move = [&](ImVec2 a, ImVec2 b, int f0, int f1) {
    float t = (f1 > f0) ? static_cast<float>(f - f0) / (f1 - f0) : 1.0f;
    t = std::clamp(t, 0.0f, 1.0f);
    t = t * t * (3.0f - 2.0f * t);  // smoothstep
    c.cursor = ImVec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
  };
  auto click = [&]() { c.click_flash = 1.0f; };

  const ImVec2 phys = button("Physics");

  // Phase 1: open + close the Physics window by clicking its rail button.
  if (f <= 24) move(ImVec2(phys.x + 240.0f, phys.y - 140.0f), phys, 4, 22);
  if (f == 25) { ToggleToolWindowByName("Physics"); click(); }
  if (f > 25 && f < 50) c.cursor = phys;
  if (f == 50) { ToggleToolWindowByName("Physics"); click(); }

  // Phase 2: toggle the Physics window open then closed via the Ctrl+P palette.
  if (f == 62) command_palette_.OpenWith(">Phys");
  if (f >= 62 && f <= 82) move(phys, palette_target, 64, 80);
  if (f == 84) { command_palette_.Close(); ToggleToolWindowByName("Physics"); click(); }
  if (f == 100) command_palette_.OpenWith(">Phys");
  if (f == 116) { command_palette_.Close(); ToggleToolWindowByName("Physics"); click(); }

  // Phase 3: open via the palette, then close with the window's 'x' button.
  if (f == 132) command_palette_.OpenWith(">Phys");
  if (f == 146) { command_palette_.Close(); ToggleToolWindowByName("Physics"); }
  if (f >= 146 && f <= 178) move(palette_target, window_close("Physics"), 150, 176);
  if (f == 180) { ToggleToolWindowByName("Physics"); click(); }

  c.click_flash = std::max(0.0f, c.click_flash - 0.15f);

  // Draw the synthetic cursor (and click ring) on the always-on-top foreground.
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  const ImVec2 p = c.cursor;
  if (c.click_flash > 0.0f) {
    const int alpha = static_cast<int>(220 * c.click_flash);
    fg->AddCircle(p, 10.0f + 14.0f * (1.0f - c.click_flash),
                  IM_COL32(255, 210, 40, alpha), 0, 3.0f);
  }
  fg->AddTriangleFilled(p, ImVec2(p.x, p.y + 18.0f), ImVec2(p.x + 13.0f, p.y + 13.0f),
                        IM_COL32(15, 15, 15, 255));
  fg->AddTriangle(p, ImVec2(p.x, p.y + 18.0f), ImVec2(p.x + 13.0f, p.y + 13.0f),
                  IM_COL32(255, 255, 255, 255), 1.5f);
}

}  // namespace mujoco::studio
