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

#include "experimental/studio/command_palette.h"

#include <algorithm>
#include <cctype>
#include <cfloat>
#include <string>
#include <vector>

#include <imgui.h>

namespace mujoco::studio {
namespace {

// Case-insensitive substring test (empty needle matches everything).
bool ContainsCaseInsensitive(const std::string& haystack,
                             const std::string& needle) {
  if (needle.empty()) {
    return true;
  }
  auto eq = [](char a, char b) {
    return std::tolower(static_cast<unsigned char>(a)) ==
           std::tolower(static_cast<unsigned char>(b));
  };
  return std::search(haystack.begin(), haystack.end(), needle.begin(),
                     needle.end(), eq) != haystack.end();
}

}  // namespace

void CommandPalette::Open() {
  open_ = true;
  focus_input_ = true;
  selection_ = 0;
  input_[0] = '\0';
}

void CommandPalette::Close() { open_ = false; }

void CommandPalette::Toggle() {
  if (open_) {
    Close();
  } else {
    Open();
  }
}

void CommandPalette::Draw(const std::vector<Command>& commands,
                          const ImVec4& rect) {
  if (!open_) {
    return;
  }

  constexpr float kWidth = 480.0f;
  // Centered horizontally; the caller supplies the top edge (rect.y).
  ImGui::SetNextWindowPos(ImVec2(rect.x + rect.z * 0.5f, rect.y),
                          ImGuiCond_Always, ImVec2(0.5f, 0.0f));
  // Fixed width, height grows with the autocomplete list.
  ImGui::SetNextWindowSizeConstraints(ImVec2(kWidth, 0),
                                      ImVec2(kWidth, FLT_MAX));
  ImGui::SetNextWindowBgAlpha(0.97f);

  const ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDocking |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
      ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoNavInputs;

  if (ImGui::Begin("##CommandPalette", nullptr, flags)) {
    if (focus_input_) {
      ImGui::SetKeyboardFocusHere();
      focus_input_ = false;
    }
    ImGui::SetNextItemWidth(-FLT_MIN);
    const bool entered = ImGui::InputTextWithHint(
        "##cmdinput", "Type  >  for commands...", input_, sizeof(input_),
        ImGuiInputTextFlags_EnterReturnsTrue);

    // Command mode is entered by typing '>' as the first character.
    if (input_[0] == '>') {
      const std::string query(input_ + 1);

      std::vector<const Command*> matches;
      for (const Command& command : commands) {
        if (ContainsCaseInsensitive(command.name, query)) {
          matches.push_back(&command);
        }
      }

      // Keyboard navigation through the filtered list.
      if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) {
        ++selection_;
      }
      if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) {
        --selection_;
      }
      if (!matches.empty()) {
        selection_ = (selection_ % static_cast<int>(matches.size()) +
                      static_cast<int>(matches.size())) %
                     static_cast<int>(matches.size());
      } else {
        selection_ = 0;
      }

      ImGui::Separator();
      for (int i = 0; i < static_cast<int>(matches.size()); ++i) {
        const bool is_selected = (i == selection_);
        bool run = false;
        if (ImGui::Selectable(matches[i]->name.c_str(), is_selected)) {
          run = true;
        }
        if (is_selected && entered) {
          run = true;
        }
        if (run) {
          matches[i]->run();
          Close();
        }
      }
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
      Close();
    }
  }
  ImGui::End();
}

}  // namespace mujoco::studio
