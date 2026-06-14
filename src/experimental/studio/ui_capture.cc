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
#include <memory>
#include <string>

#include <imgui.h>

#include "experimental/studio/llm/llm_mock.h"

namespace mujoco::studio {

void App::ToggleToolWindowByName(const std::string& title) {
  for (ToolWindow& tw : tool_windows_) {
    if (tw.title == title) {
      tw.open = !tw.open;
      return;
    }
  }
}

void App::StartCapture(const std::string& out_dir, int total_frames,
                       CaptureScript script, const std::string& llm_prompt) {
  capture_.active = true;
  capture_.out_dir = out_dir;
  capture_.frame = 0;
  capture_.total_frames = total_frames;
  capture_.script = script;
  capture_.llm_prompt = llm_prompt;
  capture_.cursor = ImVec2(-100.0f, -100.0f);
  capture_.click_flash = 0.0f;

  // The LLM scripts run the agent ASYNCHRONOUSLY (on a worker thread) so it can
  // call inspect_ui -- which blocks the worker until the UI thread runs a gather
  // -- without deadlocking the render loop. The capture keeps rendering frames
  // while the conversation and UI program play out, and ends once everything has
  // been idle for a while (see CaptureStepLlm). Keep whatever provider the
  // environment selected (real Claude when ANTHROPIC_API_KEY is set, else mock).
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

void App::CaptureStepLlm() {
  CaptureState& c = capture_;
  const int f = c.frame;

  const std::string kQuestion = c.llm_prompt.empty()
                                    ? std::string("open the physics menu")
                                    : c.llm_prompt;

  constexpr int kTypeStart = 12;
  constexpr int kCharsPerFrame = 2;
  const int n = static_cast<int>(kQuestion.size());
  const int type_frames = (n + kCharsPerFrame - 1) / kCharsPerFrame;
  const int submit = kTypeStart + type_frames + 6;

  // Open the palette and "type" the question a couple of chars per frame.
  if (f == 8) command_palette_.Open();
  if (f >= kTypeStart && f < kTypeStart + type_frames) {
    const int chars = std::min(n, (f - kTypeStart + 1) * kCharsPerFrame);
    command_palette_.SetText(kQuestion.substr(0, chars));
  }

  // Submit (async): the agent runs on a worker thread; its conversation and the
  // UI program it queues play out over the following frames while we keep
  // rendering. (Async so inspect_ui's blocking gather doesn't stall the loop.)
  if (f == submit) {
    command_palette_.SetText(kQuestion);
    ui_agent_.Ask(kQuestion);
    command_palette_.SetText("");
    c.asked = true;
  }

  // End the capture once the agent and test engine have been idle for a while
  // (the whole interaction has settled).
  if (c.asked && f > submit) {
    if (!ui_agent_.busy() && test_runner_.idle()) {
      ++c.idle_frames;
    } else {
      c.idle_frames = 0;
    }
    if (c.idle_frames == 12) command_palette_.Close();
    if (c.idle_frames >= 36) c.active = false;
  }

  c.cursor = ImVec2(-100.0f, -100.0f);
}

namespace {
// Escapes a string for embedding in a JSON string literal.
std::string JsonEscape(const std::string& s) {
  std::string o;
  for (char ch : s) {
    if (ch == '"' || ch == '\\') {
      o += '\\';
      o += ch;
    } else if (ch == '\n') {
      o += "\\n";
    } else {
      o += ch;
    }
  }
  return o;
}
}  // namespace

void App::CaptureStepLlmDemo() {
  CaptureState& c = capture_;
  const int f = c.frame;
  const std::string prompt =
      c.llm_prompt.empty() ? std::string("show the stats panel") : c.llm_prompt;

  constexpr int kOpen = 8;
  constexpr int kTypeStart = 16;
  constexpr int kCharsPerFrame = 2;  // cinematic typing speed
  const int n = static_cast<int>(prompt.size());
  const int type_frames = (n + kCharsPerFrame - 1) / kCharsPerFrame;
  const int submit = kTypeStart + type_frames + 8;

  // Open the command palette (the app's Ctrl+P action).
  if (f == kOpen) command_palette_.Open();

  // Type the prompt into the box a couple of characters per frame.
  if (f >= kTypeStart && f < kTypeStart + type_frames) {
    const int chars = std::min(n, (f - kTypeStart + 1) * kCharsPerFrame);
    command_palette_.SetText(prompt.substr(0, chars));
  }

  // Submit: fire the agent (synchronous, like kLlm) and clear the box. The
  // LLM's reply then drives the UI via its run_ui_program program, played back
  // by the test engine over the following frames.
  if (f == submit) {
    command_palette_.SetText(prompt);  // ensure the full prompt is shown
    ui_agent_.Ask(prompt);
    command_palette_.SetText("");
  }

  if (f == c.total_frames - 12) command_palette_.Close();
  c.cursor = ImVec2(-100.0f, -100.0f);
}

void App::CaptureStep() {
  if (capture_.script == CaptureScript::kLlmDemo) {
    CaptureStepLlmDemo();
    return;
  }
  if (capture_.script == CaptureScript::kLlm) {
    CaptureStepLlm();
    return;
  }

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
