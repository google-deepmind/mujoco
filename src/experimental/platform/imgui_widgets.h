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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_IMGUI_WIDGETS_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_IMGUI_WIDGETS_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <imgui.h>
#include <imgui_internal.h>
#include <mujoco/mujoco.h>
#include "experimental/platform/enum_utils.h"

namespace mujoco::platform {

// FontAwesome icon codes.
static constexpr const char ICON_FA_ADJUST[] = "\xEF\x81\x82";
static constexpr const char ICON_FA_ARROWS[] = "\xEF\x81\x87";
static constexpr const char ICON_FA_CAMERA[] = "\xEF\x80\xBD";
static constexpr const char ICON_FA_CARET_LEFT[] = "\xEF\x83\x99";
static constexpr const char ICON_FA_CARET_RIGHT[] = "\xEF\x83\x9A";
static constexpr const char ICON_FA_CHECK_SQUARE_O[] = "\xEF\x81\x9D";
static constexpr const char ICON_FA_CIRCLE[] = "\xEF\x84\x91";
static constexpr const char ICON_FA_CIRCLE_O[] = "\xEF\x84\x8C";
static constexpr const char ICON_FA_COMMENT[] = "\xEF\x83\xA5";
static constexpr const char ICON_FA_COPY[] = "\xEF\x83\x85";
static constexpr const char ICON_FA_DIAMOND[] = "\xEF\x88\x99";
static constexpr const char ICON_FA_EJECT[] = "\xEF\x81\x92";
static constexpr const char ICON_FA_FAST_FORWARD[] = "\xEF\x81\x90";
static constexpr const char ICON_FA_MOON[] = "\xEF\x86\x86";
static constexpr const char ICON_FA_MAGIC[] = "\xEF\x83\x90";
static constexpr const char ICON_FA_PAUSE[] = "\xEF\x81\x8C";
static constexpr const char ICON_FA_PLAY[] = "\xEF\x81\x8B";
static constexpr const char ICON_FA_REFRESH[] = "\xEF\x80\xA1";
static constexpr const char ICON_FA_SQUARE_O[] = "\xEF\x87\x9B";
static constexpr const char ICON_FA_SUN[] = "\xEF\x86\x85";
static constexpr const char ICON_FA_TACHOMETER[] = "\xEF\x83\xA4";
static constexpr const char ICON_FA_TRASH_CAN[] = "\xEF\x87\xB8";
static constexpr const char ICON_FA_UNDO[] = "\xEF\x83\xA2";

using KeyValues = std::unordered_map<std::string, std::string>;

// This is a workaround to fix compilation on gcc <= 12 and clang <= 16
template <typename T>
struct dependent_false : std::false_type {};

// Appends key/value pairs to an Ini file.
void AppendIniSection(std::string& ini, const std::string& section,
                      const KeyValues& key_values);

// Reads key/value pairs from an Ini file section.
KeyValues ReadIniSection(const std::string& contents,
                         const std::string& section);

template <typename T>
T ReadIniValue(const KeyValues& key_values, const std::string& key, T def) {
  auto iter = key_values.find(key);
  if (iter == key_values.end()) {
    return def;
  }
  if constexpr (std::is_same_v<T, int>) {
    return std::stoi(iter->second);
  } else if constexpr (std::is_same_v<T, float>) {
    return std::stof(iter->second);
  } else if constexpr (std::is_same_v<T, double>) {
    return std::stod(iter->second);
  } else if constexpr (std::is_same_v<T, std::string>) {
    return iter->second;
  } else if constexpr (std::is_enum_v<T>) {
    return static_cast<T>(std::stoi(iter->second));
  } else {
    static_assert(dependent_false<T>::value, "Unsupported type");
  }
}

// Helper class for setting ImGui style options; automatically resets the
// styles when going out of scope.
struct ScopedStyle {
  ScopedStyle() = default;
  ~ScopedStyle() { Reset(); }

  ScopedStyle(const ScopedStyle&) = delete;
  ScopedStyle& operator=(const ScopedStyle&) = delete;
  ScopedStyle(ScopedStyle&& other) { Swap(other); }
  ScopedStyle& operator=(ScopedStyle&& other) {
    Swap(other);
    return *this;
  }

  void Swap(ScopedStyle& other) {
    std::swap(num_colors, other.num_colors);
    std::swap(num_vars, other.num_vars);
  }

  ScopedStyle& Color(ImGuiCol col, ImColor color) {
    ImGui::PushStyleColor(col, (ImU32)color);
    ++num_colors;
    return *this;
  }

  ScopedStyle& Color(ImGuiCol col, ImGuiCol col2) {
    ImGui::PushStyleColor(col, CurrentColor(col2));
    ++num_colors;
    return *this;
  }

  ScopedStyle& Var(ImGuiStyleVar var, float value) {
    ImGui::PushStyleVar(var, value);
    ++num_vars;
    return *this;
  }

  ScopedStyle& Var(ImGuiStyleVar var, const ImVec2& value) {
    ImGui::PushStyleVar(var, value);
    ++num_vars;
    return *this;
  }

  ImVec4 CurrentColor(ImGuiCol col) {
    return ImGui::GetStyle().Colors[col];
  }

  void Reset() {
    ImGui::PopStyleVar(num_vars);
    ImGui::PopStyleColor(num_colors);
    num_colors = 0;
    num_vars = 0;
  }

  int num_colors = 0;
  int num_vars = 0;
};

// Helper for displaying rows of key/value pairs in an ImGui table.
class ImGui_DataPtrTable {
 public:
  // Starts the table (i.e. ImGui::BeginTable()) with two columns of the
  // specified widths.
  ImGui_DataPtrTable(float w1 = 0.25f, float w2 = 0.75f);

  // Ends the table (e.g. ImGui::EndTable().
  ~ImGui_DataPtrTable();

  ImGui_DataPtrTable(const ImGui_DataPtrTable& other) = delete;
  ImGui_DataPtrTable& operator=(const ImGui_DataPtrTable& other) = delete;

  // Displays a labelled value in the table. These functions are intended
  // specifically for displaying data from mjModel and mjData which store data
  // in contiguous arrays. Each value to be displayed is at a given
  // index into the array (based on the object's ID) and then has a
  // dimensionality of n.
  void DataPtr(const char* label, const char* ptr, int index, int n);
  void DataPtr(const char* label, const mjtByte* ptr, int index, int n);
  void DataPtr(const char* label, const mjtSize* ptr, int index, int n);
  void DataPtr(const char* label, const int* ptr, int index, int n);
  void DataPtr(const char* label, const float* ptr, int index, int n);
  void DataPtr(const char* label, const double* ptr, int index, int n);
  void DataPtr(const char* label, const uintptr_t* ptr, int index, int n);

  // Sets the prefix that will be removed from all labels. Note: that we simply
  // remove the first N characters of the label without actually comparing
  // against this prefix. This works well with mjModel and mjData because
  // data belonging to a given object type has a common prefix (e.g. all joints
  // properties are prefixed with "jnt_").
  void SetPrefix(const char* prefix);

 protected:
  template <typename T>
  void Numeric(const char* label, const T* ptr, int index, int n);
  void MakeLabel(const char* label, int index = 0, int total = 1);

  int prefix_ = 0;
};

// Helper for displaying mjSpec elements in an ImGui table.
class ImGui_SpecElementTable : public ImGui_DataPtrTable {
 public:
  // Scalar values used by mjSpec elements.
  void operator()(const char* label, mjtByte& val, const char* tooltip);
  void operator()(const char* label, mjtSize& val, const char* tooltip);
  void operator()(const char* label, int& val, const char* tooltip);
  void operator()(const char* label, float& val, const char* tooltip);
  void operator()(const char* label, double& val, const char* tooltip);

  // C++ container values used by mjSpec elements.
  void operator()(const char* label, std::string* ptr, const char* tooltip);
  void operator()(const char* label, std::vector<int>* ptr,
                  const char* tooltip);
  void operator()(const char* label, std::vector<double>* ptr,
                  const char* tooltip);
  void operator()(const char* label, std::vector<std::string>* ptr,
                  const char* tooltip);

  // C-style array values used by mjSpec elements.
  template <std::size_t N>
  void operator()(const char* label, char (&val)[N], const char* tooltip) {
    MakeLabel(label);
    ImGui::SetItemTooltip("%s", tooltip);
    ImGui::Text("%s", std::string(val, N).c_str());
  }
  template <std::size_t N>
  void operator()(const char* label, mjtByte (&val)[N], const char* tooltip) {
    for (int i = 0; i < N; ++i) {
      MakeLabel(label, i, N);
      ImGui::SetItemTooltip("%s", tooltip);
      ImGui::Text("%s", val[i] ? "true" : "false");
    }
  }
  template <std::size_t N>
  void operator()(const char* label, int (&val)[N], const char* tooltip) {
    Vector(label, val, N, tooltip);
  }
  template <std::size_t N>
  void operator()(const char* label, float (&val)[N], const char* tooltip) {
    Vector(label, val, N, tooltip);
  }
  template <std::size_t N>
  void operator()(const char* label, double (&val)[N], const char* tooltip) {
    Vector(label, val, N, tooltip);
  }

  // Special handling for treating enum values as integers.
  template <typename T, typename U=std::enable_if_t<std::is_enum_v<T>, T>>
  void operator()(const char* label, T& val, const char* tooltip) {
    auto v = enum_utils::enum_to_string(val);
    MakeLabel(label);
    ImGui::SetItemTooltip("%s", tooltip);
    ImGui::Text("%s", v.data());
  }

  // Special handling for quaternion/orientation pairs.
  void operator()(const char* name, double (&quat)[4], const char* alt,
                  mjsOrientation& orientation, const char* tooltip);

 private:
  template <typename T>
  void Scalar(const char* label, T& val, const char* tooltip) {
    MakeLabel(label);
    ImGui::SetItemTooltip("%s", tooltip);
    if constexpr (std::is_enum_v<T>) {
      ImGui::Text("%d", (int)val);
    } else if constexpr (std::is_integral_v<T>) {
      ImGui::Text("%d", (int)val);
    } else if constexpr (std::is_floating_point_v<T>) {
      ImGui::Text("%f", (float)val);
    }
  }

  template <typename T>
  void Vector(const char* label, T* ptr, int n, const char* tooltip) {
    for (int i = 0; i < n; ++i) {
      MakeLabel(label, i, n);
      ImGui::SetItemTooltip("%s", tooltip);
      if constexpr (std::is_enum_v<T>) {
        ImGui::Text("%d", (int)ptr[i]);
      } else if constexpr (std::is_integral_v<T>) {
        ImGui::Text("%d", (int)ptr[i]);
      } else if constexpr (std::is_floating_point_v<T>) {
        ImGui::Text("%f", (float)ptr[i]);
      }
    }
  }
};


// ImGui Slider that supports both float and double types.
bool ImGui_Slider(const char* name, mjtNum* value, mjtNum min, mjtNum max);

template <typename T>
bool ImGui_Checkbox(const char* name, T& value) {
  static_assert(std::is_integral<T>());
  bool b = (value != 0);
  const bool res = ImGui::Checkbox(name, &b);
  if (res) {
    value = b ? 1 : 0;
  }
  return res;
}

template <typename T>
bool ImGui_ButtonToggle(const char* label, T* boolean,
                        const ImVec2& size = ImVec2(0, 0)) {
  static_assert(std::is_integral_v<T>, "Toggle only supports integral types.");

  ScopedStyle style;
  const int color = *boolean ? ImGuiCol_TabSelected : ImGuiCol_WindowBg;
  style.Color(ImGuiCol_Button, ImGui::GetStyle().Colors[color]);
  style.Var(ImGuiStyleVar_ButtonTextAlign, ImVec2(0.0f, 0.5f));

  const std::string txt =
      std::string(" ") +
      std::string(*boolean ? ICON_FA_CHECK_SQUARE_O : ICON_FA_SQUARE_O) + "  " +
      label;
  if (ImGui::Button(txt.c_str(), size)) {
    *boolean = !(*boolean);
    return true;
  }
  return false;
}

template <typename T>
bool ImGui_SwitchToggle(const char* label, T* boolean,
                        const ImVec2& size = ImVec2(0, 0)) {
  static_assert(std::is_integral_v<T>, "Toggle only supports integral types.");

  int i = static_cast<int>(*boolean);
  const ImGuiSliderFlags flags = ImGuiSliderFlags_NoInput;
  if (size.x > 0) {
    ImGui::SetNextItemWidth(size.x);
  }
  const bool changed = ImGui::SliderInt(label, &i, 0, 1, label, flags);
  *boolean = (i != 0);
  return changed;
}

inline bool ImGui_BitToggle(const char* label, int* flags, int flags_value,
                            const ImVec2& size = ImVec2(0, 0)) {
  bool boolean = (*flags) & flags_value;
  const bool changed = ImGui_ButtonToggle(label, &boolean, size);
  if (changed) {
    *flags = boolean ? ((*flags) | flags_value) : ((*flags) & ~flags_value);
  }
  return changed;
}

// Options for ImGui_InputN (see below).
template <typename T>
struct ImGuiOpts {
  std::optional<T> min;
  std::optional<T> max;
  std::optional<T> step;
  std::optional<T> step_fast;
  std::optional<float> width;
  const char* format = std::is_floating_point_v<T> ? "%.3g" : "%d";
};

// A compile-time wrapper around ImGui::InputScalarN. This is useful because
// MuJoCo uses an `mjtNum` type which is an alias for float or double.
//
// Options can be used to specify step sizes, clamp ranges, and formatting.
template <typename T>
bool ImGui_InputN(const char* name, T* value, int num, ImGuiOpts<T> opts = {}) {
  bool res = false;
  if (opts.width) {
    ImGui::SetNextItemWidth(opts.width.value());
  }
  if constexpr (std::is_same_v<T, int>) {
    const int step = opts.step.value_or(1);
    const int step_fast = opts.step_fast.value_or(100);
    const char* format = opts.format;
    res = ImGui::InputScalarN(name, ImGuiDataType_S32, value, num, &step,
                              &step_fast, format);

  } else if constexpr (std::is_same_v<T, float>) {
    const float step = opts.step.value_or(0.f);
    const float step_fast = opts.step_fast.value_or(0.f);
    const float* pstep = opts.step.has_value() ? &step : nullptr;
    const float* pstep_fast = opts.step_fast.has_value() ? &step_fast : nullptr;
    const char* format = opts.format ? opts.format : "%.3f";
    res = ImGui::InputScalarN(name, ImGuiDataType_Float, value, num, pstep,
                              pstep_fast, format);

  } else if constexpr (std::is_same_v<T, double>) {
    const double step = opts.step.value_or(0.0);
    const double step_fast = opts.step_fast.value_or(0.0);
    const double* pstep = opts.step.has_value() ? &step : nullptr;
    const double* pstep_fast =
        opts.step_fast.has_value() ? &step_fast : nullptr;
    const char* format = opts.format ? opts.format : "%.3f";
    res = ImGui::InputScalarN(name, ImGuiDataType_Double, value, num, pstep,
                              pstep_fast, format);
  } else {
    static_assert(dependent_false<T>::value, "Unsupported type");
  }

  if (opts.min.has_value()) {
    if (*value < *opts.min) *value = *opts.min;
  }
  if (opts.max.has_value()) {
    if (*value > *opts.max) *value = *opts.max;
  }
  return res;
}

template <typename T>
bool ImGui_Input(const char* name, T* value, ImGuiOpts<T> opts = {}) {
  return ImGui_InputN(name, value, 1, opts);
}

// Returns true if the given chord is has _just_ been pressed in this frame.
// (This is opposed to "Pressed" which means the chord is active, i.e. the user
// is holding down the keys.)
inline bool ImGui_IsChordJustPressed(ImGuiKeyChord chord) {
  return ImGui::IsKeyChordPressed(chord, 0);
}

// Stateful button that displays the given color when active, and shows a
// semi-transparent hover color (controlled by hover_alpha) when inactive.
inline bool ImGui_ColorButton(const char* label, bool active, ImColor color,
                              const ImVec2& size = ImVec2(0, 0),
                              float hover_alpha = 0.5f) {
  ScopedStyle style;
  const ImColor hover(color.Value.x, color.Value.y, color.Value.z,
                      color.Value.w * hover_alpha);
  if (active) {
    style.Color(ImGuiCol_Button, color);
    style.Color(ImGuiCol_ButtonHovered, color);
  } else {
    style.Color(ImGuiCol_ButtonHovered, hover);
  }
  style.Color(ImGuiCol_ButtonActive, color);
  return ImGui::Button(label, size);
}

// Begin a boxed section with outer borders - use EndBoxSection to close.
inline bool BeginBoxSection(const char* id, ImGuiTableFlags extra_flags = 0) {
  ImGuiTableFlags flags = ImGuiTableFlags_BordersOuter | extra_flags;
  if (ImGui::BeginTable(id, 1, flags)) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    return true;
  }
  return false;
}

inline void EndBoxSection() { ImGui::EndTable(); }

// Saves the given contents to the clipboard if the clipboard is available.
void MaybeSaveToClipboard(const std::string& contents);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_IMGUI_WIDGETS_H_
