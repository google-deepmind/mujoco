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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PANEL_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PANEL_H_

namespace mujoco::studio {

class UiAgent;

// Renders the LLM conversation inside the current ImGui window (the command
// palette), so an "ask" exchange happens right in the Ctrl+P box. Keeps a small
// amount of presentation state: a typewriter reveal of the latest reply so
// demos/GIFs read naturally.
class LlmPanel {
 public:
  void Render(UiAgent& agent);

 private:
  int revealing_index_ = -1;  // history index of the reply being revealed.
  int reveal_chars_ = 0;
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_LLM_LLM_PANEL_H_
