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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_UI_CAPTURE_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_UI_CAPTURE_H_

#include <string>

#include <imgui.h>

namespace mujoco::studio {

// Drives a scripted UI interaction headless and records one framebuffer per
// frame, for assembling a GIF (see App::CaptureStep / SaveCaptureFrame, defined
// in ui_capture.cc). This is a lightweight stand-in for the Dear ImGui Test
// Engine: it scripts the app's own state + a synthetic cursor rather than
// injecting real input, which keeps it self-contained and headless-friendly.
// Which scripted interaction to record.
enum class CaptureScript {
  kTools,    // open/close tool windows via the rail and the command palette.
  kLlm,      // ask the LLM a question in the Ctrl+P box and show the reply.
  kLlmDemo,  // like kLlm but types the prompt + submits via the real input
             // path (test engine Ctrl+P / typing / Enter) for a cooler gif.
};

struct CaptureState {
  bool active = false;
  std::string out_dir;   // directory for frame_%04d.ppm
  int frame = 0;         // current capture frame index
  int total_frames = 0;  // stop after this many frames
  CaptureScript script = CaptureScript::kTools;
  std::string llm_prompt;  // question typed into the Ctrl+P box (kLlm script)

  ImVec2 cursor{-100.0f, -100.0f};  // synthetic cursor, screen space
  float click_flash = 0.0f;         // >0 draws a click ring, decays per frame

  // For the async LLM scripts: whether the question has been asked, and how many
  // consecutive frames the agent + test engine have been idle (used to end the
  // capture once the whole interaction has settled).
  bool asked = false;
  int idle_frames = 0;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_UI_CAPTURE_H_
