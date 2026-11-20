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

#include <imgui.h>
#include "studio/ux/console.h"

namespace mujoco::studio {

void StatusSeparatorGui() {
  const float gap = 6.0f * ImGui::GetWindowDpiScale() * ImGui::GetIO().FontGlobalScale;
  ImGui::SameLine(0.0f, gap);
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("\xC2\xB7");
  ImGui::SameLine(0.0f, gap);
}

void StatusBarGui(const char* status_text, bool* show_console) {
  ImGui::PushStyleVar(ImGuiStyleVar_CellPadding, ImVec2(0, 0));
  if (ImGui::Begin("StatusBar")) {
    const float scale = ImGui::GetWindowDpiScale() * ImGui::GetIO().FontGlobalScale;
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(8.0f * scale, 2.0f * scale));
    if (ImGui::BeginTable("##StatusBarTable", 2)) {
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthStretch);
      ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed);
      ImGui::TableNextColumn();
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted(status_text ? status_text : "");
      ImGui::TableNextColumn();
      Console::Get().StatusGui(show_console);
      ImGui::EndTable();
    }
    ImGui::PopStyleVar();
  }
  ImGui::End();
  ImGui::PopStyleVar();
}

}  // namespace mujoco::studio
