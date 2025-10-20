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

#include "experimental/toolbox/imgui_widgets.h"

#include <sstream>
#include <string>
#include <unordered_map>

#include <imgui.h>
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

void AppendIniSection(std::string& ini, const std::string& section,
                      const KeyValues& key_values) {
  if (section.front() != '[' || section.back() != ']') {
    mju_error("Section must be enclosed in square brackets.");
  }
  ini += "\n" + std::string(section) + "\n";
  for (auto& [key, value] : key_values) {
    ini += key + "=" + value + "\n";
  }
}

KeyValues ReadIniSection(const std::string& contents,
                         const std::string& section) {
  if (section.front() != '[' || section.back() != ']') {
    mju_error("Section must be enclosed in square brackets.");
  }
  bool in_section = false;

  KeyValues key_values;
  std::istringstream f(contents);
  std::string line;
  while (std::getline(f, line)) {
    if (line[0] == '[') {
      in_section = (line == section);
    } else if (in_section) {
      std::string::size_type pos = line.find('=');
      if (pos != std::string::npos) {
        key_values[line.substr(0, pos)] = line.substr(pos + 1);
      }
    }
  }
  return key_values;
}

bool ImGui_Slider(const char* name, mjtNum* value, mjtNum min, mjtNum max) {
  float f = *value;
  const bool res = ImGui::SliderFloat(name, &f, min, max);
  if (res) {
    *value = f;
  }
  return res;
}

bool ImGui_FileDialog(char* buf, int len) {
  bool ok = false;
  ImGui::Text("Filename");
  ImGui::SameLine();
  ImGui::InputText("##Filename", buf, len);
  if (ImGui::Button("OK", ImVec2(120, 0))) {
    ok = true;
    ImGui::CloseCurrentPopup();
  }
  ImGui::SetItemDefaultFocus();
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(120, 0))) {
    ImGui::CloseCurrentPopup();
  }
  return ok;
}

void MaybeSaveToClipboard(const std::string& contents) {
  if (ImGui::GetIO().SetClipboardTextFn) {
    ImGui::GetIO().SetClipboardTextFn(nullptr, contents.c_str());
  }
}

}  // namespace mujoco::toolbox
