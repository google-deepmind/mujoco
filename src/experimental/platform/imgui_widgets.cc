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

#include "experimental/platform/imgui_widgets.h"

#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <imgui.h>
#include <mujoco/mujoco.h>

namespace mujoco::platform {

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

ImGui_DataPtrTable::ImGui_DataPtrTable(float w1, float w2) {
  ImGui::BeginTable("##PropertiesTable", 2);
  const float width = ImGui::GetContentRegionAvail().x;
  ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, width * w1);
  ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, width * w2);
}
ImGui_DataPtrTable::~ImGui_DataPtrTable() { ImGui::EndTable(); }

void ImGui_DataPtrTable::SetPrefix(const char* prefix) {
  prefix_ = strlen(prefix);
}

void ImGui_DataPtrTable::DataPtr(const char* label, const uintptr_t* ptr,
                                 int index, int n) {
  for (int i = 0; i < n; ++i) {
    MakeLabel(label, i, n);
    ImGui::Text("(%s)", &ptr[index + i] ? "[ptr]" : "null");
  }
}

void ImGui_DataPtrTable::DataPtr(const char* label, const char* ptr, int index,
                              int n) {
  MakeLabel(label);
  ScopedStyle style;
  style.Color(ImGuiCol_Text, ImColor(255, 0, 0, 255));
  ImGui::Text("%s", "(char not implemented, please report bug)");
}

void ImGui_DataPtrTable::DataPtr(const char* label, const mjtByte* ptr, int index,
                              int n) {
  for (int i = 0; i < n; ++i) {
    MakeLabel(label, i, n);
    ImGui::Text("%s", ptr[index + i] ? "true" : "false");
  }
}

void ImGui_DataPtrTable::DataPtr(const char* label, const mjtSize* ptr, int index,
                              int n) {
  Numeric(label, ptr, index, n);
}

void ImGui_DataPtrTable::DataPtr(const char* label, const int* ptr, int index,
                              int n) {
  Numeric(label, ptr, index, n);
}

void ImGui_DataPtrTable::DataPtr(const char* label, const float* ptr, int index,
                              int n) {
  Numeric(label, ptr, index, n);
}

void ImGui_DataPtrTable::DataPtr(const char* label, const double* ptr, int index,
                              int n) {
  Numeric(label, ptr, index, n);
}

template <typename T>
void ImGui_DataPtrTable::Numeric(const char* label, const T* ptr, int index, int n) {
  const T* addr = ptr + index * n;

  using U = std::conditional_t<std::is_floating_point_v<T>, float, int>;

  // special treatment for NaNs.
  if constexpr (std::is_same_v<U, float>) {
    if (*addr != *addr) {
      MakeLabel(label);
      ImGui::Text("nan");
      return;
    }
  }

  constexpr const char* fmt1 =
      std::is_floating_point_v<U> ? "%f" : "%d";
  constexpr const char* fmt2 =
      std::is_floating_point_v<U> ? "%f  %f" : "%d  %d";
  constexpr const char* fmt3 =
      std::is_floating_point_v<U> ? "%f  %f  %f" : "%d  %d  %d";
  constexpr const char* fmt4 =
      std::is_floating_point_v<U> ? "%f  %f  %f  %f" : "%d  %d  %d  %d";

  auto text1 = [&](int offset) {
    ImGui::Text(fmt1, (U)(addr[offset]));
  };
  auto text2 = [&](int offset) {
    ImGui::Text(fmt2, (U)(addr[offset + 0]), (U)(addr[offset + 1]));
  };
  auto text3 = [&](int offset) {
    ImGui::Text(fmt3, (U)(addr[offset + 0]), (U)(addr[offset + 1]),
                (U)(addr[offset + 2]));
  };
  auto text4 = [&](int offset) {
    ImGui::Text(fmt4, (U)(addr[offset + 0]), (U)(addr[offset + 1]),
                (U)(addr[offset + 2]), (U)(addr[offset + 3]));
  };

  if (n == 1) {
    MakeLabel(label);
    text1(0);
  } else if (n == 2) {
    MakeLabel(label);
    text2(0);
  } else if (n == 3) {
    MakeLabel(label);
    text3(0);
  } else if (n == 4) {
    MakeLabel(label);
    text4(0);
  } else if (n == 6) {
    MakeLabel(label);
    text3(0);
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
    text3(3);
  } else if (n == 9) {
    MakeLabel(label);
    text3(0);
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
    text3(3);
    ImGui::TableNextColumn();
    ImGui::TableNextColumn();
    text3(6);
  } else {
    for (int i = 0; i < n; ++i) {
      MakeLabel(label, i, n);
      text1(i);
    }
  }
}

void ImGui_DataPtrTable::MakeLabel(const char* label, int index, int total) {
  if (total == 1) {
    ImGui::TableNextColumn();
    ImGui::Text("%s", &label[prefix_]);
    ImGui::TableNextColumn();
  } else {
    const std::string tmp =
        std::string(&label[prefix_]) + "[" + std::to_string(index) + "]";
    ImGui::TableNextColumn();
    ImGui::Text("%s", tmp.c_str());
    ImGui::TableNextColumn();
  }
}

void ImGui_SpecElementTable::operator()(const char* label, mjtByte& val,
                                        const char* tooltip) {
  MakeLabel(label);
  ImGui::SetItemTooltip("%s", tooltip);
  ImGui::Text("%s", val ? "true" : "false");
}

void ImGui_SpecElementTable::operator()(const char* label, mjtSize& val,
                                        const char* tooltip) {
  Scalar(label, val, tooltip);
}

void ImGui_SpecElementTable::operator()(const char* label, int& val,
                                        const char* tooltip) {
  Scalar(label, val, tooltip);
}

void ImGui_SpecElementTable::operator()(const char* label, float& val,
                                        const char* tooltip) {
  Scalar(label, val, tooltip);
}

void ImGui_SpecElementTable::operator()(const char* label, double& val,
                                        const char* tooltip) {
  Scalar(label, val, tooltip);
}

void ImGui_SpecElementTable::operator()(const char* label, std::string* ptr,
                                        const char* tooltip) {
  MakeLabel(label);
  ImGui::SetItemTooltip("%s", tooltip);
  ImGui::Text("%s", ptr ? ptr->c_str() : "");
}

void ImGui_SpecElementTable::operator()(const char* label,
                                        std::vector<int>* ptr,
                                        const char* tooltip) {
  MakeLabel(label);
  ImGui::SetItemTooltip("%s", tooltip);
  if (ptr == nullptr || ptr->empty()) {
    ImGui::Text("[empty]");
  } else {
    ImGui::Text("[%zu values]", ptr->size());
  }
}

void ImGui_SpecElementTable::operator()(const char* label,
                                        std::vector<double>* ptr,
                                        const char* tooltip) {
  MakeLabel(label);
  ImGui::SetItemTooltip("%s", tooltip);
  if (ptr == nullptr || ptr->empty()) {
    ImGui::Text("[empty]");
  } else {
    ImGui::Text("[%zu values]", ptr->size());
  }
}

void ImGui_SpecElementTable::operator()(const char* label,
                                        std::vector<std::string>* ptr,
                                        const char* tooltip) {
  for (int i = 0; i < ptr->size(); ++i) {
    MakeLabel(label, i, ptr->size());
    ImGui::SetItemTooltip("%s", tooltip);
    ImGui::Text("%s", ptr->at(i).c_str());
  }
}

void ImGui_SpecElementTable::operator()(const char* name, double (&quat)[4],
                                        const char* alt,
                                        mjsOrientation& orientation,
                                        const char* tooltip) {
  auto alt_name = [&](const char* label) {
    return std::string(alt) + "." + label;
  };

  switch (orientation.type) {
    case mjORIENTATION_QUAT:
      (*this)(name, quat, tooltip);
      break;
    case mjORIENTATION_AXISANGLE:
      (*this)(alt_name("axisangle").c_str(), orientation.axisangle, tooltip);
      break;
    case mjORIENTATION_XYAXES:
      (*this)(alt_name("xyaxes").c_str(), orientation.xyaxes, tooltip);
      break;
    case mjORIENTATION_ZAXIS:
      (*this)(alt_name("zaxis").c_str(), orientation.zaxis, tooltip);
      break;
    case mjORIENTATION_EULER:
      (*this)(alt_name("euler").c_str(), orientation.euler, tooltip);
      break;
  }
}

bool ImGui_Slider(const char* name, mjtNum* value, mjtNum min, mjtNum max) {
  float f = *value;
  const bool res = ImGui::SliderFloat(name, &f, min, max);
  if (res) {
    *value = f;
  }
  return res;
}

void MaybeSaveToClipboard(const std::string& contents) {
  if (ImGui::GetIO().SetClipboardTextFn) {
    ImGui::GetIO().SetClipboardTextFn(nullptr, contents.c_str());
  }
}

}  // namespace mujoco::platform
