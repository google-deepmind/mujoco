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

ImGui_DataTable::ImGui_DataTable(float w1, float w2) {
  ImGui::BeginTable("##PropertiesTable", 2);
  const float width = ImGui::GetContentRegionAvail().x;
  ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, width * w1);
  ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, width * w2);
}
ImGui_DataTable::~ImGui_DataTable() { ImGui::EndTable(); }

void ImGui_DataTable::SetArrayIndex(int index) { index_ = index; }

void ImGui_DataTable::SetPrefix(const char* prefix) {
  prefix_ = strlen(prefix);
}

void ImGui_DataTable::operator()(const char* label, const uintptr_t* ptr,
                                 int n) {
  for (int i = 0; i < n; ++i) {
    MakeLabel(label, i, n);
    ImGui::Text("(%s)", &ptr[index_ + i] ? "[ptr]" : "null");
  }
}

void ImGui_DataTable::operator()(const char* label, const char* ptr, int n) {
  if (n == 1) {
    MakeLabel(label);
    ImGui::Text("%s", &ptr[index_]);
  } else {
    mju_error("char cannot be converted to a vector");
  }
}

void ImGui_DataTable::operator()(const char* label, const mjtByte* ptr, int n) {
  for (int i = 0; i < n; ++i) {
    MakeLabel(label, i, n);
    ImGui::Text("%s", ptr[index_ + i] ? "true" : "false");
  }
}

void ImGui_DataTable::operator()(const char* label, const mjtByte& val, int n) {
  MakeLabel(label, 0, 1);
  ImGui::Text("%s", val ? "true" : "false");
}

void ImGui_DataTable::operator()(const char* label, const mjtSize* ptr, int n) {
  Numeric(label, ptr, n);
}

void ImGui_DataTable::operator()(const char* label, const int* ptr, int n) {
  Numeric(label, ptr, n);
}

void ImGui_DataTable::operator()(const char* label, const float* ptr, int n) {
  Numeric(label, ptr, n);
}

void ImGui_DataTable::operator()(const char* label, const double* ptr, int n) {
  Numeric(label, ptr, n);
}

void ImGui_DataTable::operator()(const char* label, const mjtSize& val, int n) {
  Scalar(label, val, n);
}

void ImGui_DataTable::operator()(const char* label, const int& val, int n) {
  Scalar(label, val, n);
}

void ImGui_DataTable::operator()(const char* label, const float& val, int n) {
  Scalar(label, val, n);
}

void ImGui_DataTable::operator()(const char* label, const double& val, int n) {
  Scalar(label, val, n);
}

void ImGui_DataTable::operator()(const char* label, const std::string* ptr,
                                 int n) {
  for (int i = 0; i < n; ++i) {
    MakeLabel(label, i, n);
    ImGui::Text("%s", ptr[i].c_str());
  }
}

void ImGui_DataTable::operator()(const char* label,
                                 const std::vector<std::string>* ptr, int n) {
  if (n == 1) {
    for (int i = 0; i < ptr->size(); ++i) {
      MakeLabel(label, i, ptr->size());
      ImGui::Text("%s", ptr->at(i).c_str());
    }
  } else {
    mju_error("data type is vector; cannot also be an array");
  }
}

void ImGui_DataTable::operator()(const char* label, const std::vector<int>* ptr,
                                 int n) {
  if (n == 1) {
    const int size = ptr->size();
    if (size == 0) {
      (*this)(label, "[empty]", 1);
    } else {
      std::string tmp = "[" + std::to_string(size) + " values]";
      (*this)(label, tmp.c_str(), 1);
    }
  } else {
    mju_error("data type is vector; cannot also be an array");
  }
}

void ImGui_DataTable::operator()(const char* label,
                                 const std::vector<double>* ptr, int n) {
  if (n == 1) {
    const int size = ptr->size();
    if (size == 0) {
      (*this)(label, "[empty]", 1);
    } else {
      std::string tmp = "[" + std::to_string(size) + " values]";
      (*this)(label, tmp.c_str(), 1);
    }
  } else {
    mju_error("data type is vector; cannot also be an array");
  }
}

template <typename T>
void ImGui_DataTable::Scalar(const char* label, const T& value, int n) {
  if (n == 1) {
    Numeric(label, &value, n);
  } else {
    mju_error("scalar cannot be converted to a vector");
  }
}

template <typename T>
void ImGui_DataTable::Numeric(const char* label, const T* ptr, int n) {
  const T* addr = ptr + index_ * n;

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

void ImGui_DataTable::MakeLabel(const char* label, int index, int total) {
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
