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

#include "experimental/studio/llm/llm_panel.h"

#include <algorithm>
#include <string>
#include <vector>

#include <imgui.h>

#include "experimental/studio/llm/ui_agent.h"

namespace mujoco::studio {

namespace {
// Characters revealed per frame for the typewriter effect.
constexpr int kRevealSpeed = 3;
}  // namespace

void LlmPanel::Render(UiAgent& agent) {
  const std::vector<UiAgent::Turn>& history = agent.history();

  if (history.empty() && !agent.busy()) {
    ImGui::Spacing();
    ImGui::TextDisabled("Ask %s, or type  >  for commands.",
                        agent.provider_name().c_str());
    return;
  }

  // Find the latest user and assistant turns.
  int last_user = -1, last_assistant = -1;
  for (int i = 0; i < static_cast<int>(history.size()); ++i) {
    if (history[i].role == "user") last_user = i;
    else if (history[i].role == "assistant") last_assistant = i;
  }

  ImGui::Separator();

  if (last_user >= 0) {
    ImGui::TextDisabled("You");
    ImGui::TextWrapped("%s", history[last_user].text.c_str());
    ImGui::Spacing();
  }

  ImGui::TextDisabled("%s", agent.provider_name().c_str());
  if (agent.busy()) {
    // Simple animated ellipsis.
    const int dots = 1 + (static_cast<int>(ImGui::GetTime() * 3.0) % 3);
    ImGui::TextDisabled("thinking%s", std::string(dots, '.').c_str());
  } else if (last_assistant >= 0) {
    const std::string& answer = history[last_assistant].text;
    if (last_assistant != revealing_index_) {
      revealing_index_ = last_assistant;
      reveal_chars_ = 0;
    }
    reveal_chars_ =
        std::min(reveal_chars_ + kRevealSpeed, static_cast<int>(answer.size()));
    ImGui::TextWrapped("%.*s", reveal_chars_, answer.c_str());
  }
}

}  // namespace mujoco::studio
