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

#include <optional>
#include <utility>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <imgui.h>
#include <imgui_internal.h>
#include <mujoco/mujoco.h>

namespace mujoco::platform {

// FontAwesome icon codes.
static constexpr const char ICON_FA_ARROWS[] = "\xEF\x81\x87";
static constexpr const char ICON_FA_CAMERA[] = "\xEF\x80\xBD";
static constexpr const char ICON_FA_CARET_LEFT[] = "\xEF\x83\x99";
static constexpr const char ICON_FA_CARET_RIGHT[] = "\xEF\x83\x9A";
static constexpr const char ICON_FA_CHECK_SQUARE_O[] = "\xEF\x81\x9D";
static constexpr const char ICON_FA_COMMENT[] = "\xEF\x83\xA5";
static constexpr const char ICON_FA_COPY[] = "\xEF\x83\x85";
static constexpr const char ICON_FA_DIAMOND[] = "\xEF\x88\x99";
static constexpr const char ICON_FA_EJECT[] = "\xEF\x81\x92";
static constexpr const char ICON_FA_FAST_FORWARD[] = "\xEF\x81\x90";
static constexpr const char ICON_FA_MOON[] = "\xEF\x86\x86";
static constexpr const char ICON_FA_PAUSE[] = "\xEF\x81\x8C";
static constexpr const char ICON_FA_PLAY[] = "\xEF\x81\x8B";
static constexpr const char ICON_FA_REFRESH[] = "\xEF\x80\xA1";
static constexpr const char ICON_FA_SQUARE_O[] = "\xEF\x87\x9B";
static constexpr const char ICON_FA_SUN[] = "\xEF\x86\x85";
static constexpr const char ICON_FA_TACHOMETER[] = "\xEF\x83\xA4";
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
  ~ScopedStyle() {
    Reset();
  }

  ScopedStyle(const ScopedStyle&) = delete;
  ScopedStyle& operator=(const ScopedStyle&) = delete;
  ScopedStyle(ScopedStyle&& other) { Swap(other); }
  ScopedStyle& operator=(ScopedStyle&& other) { Swap(other); return *this; }

  void Swap(ScopedStyle& other) {
    std::swap(num_colors, other.num_colors);
    std::swap(num_vars, other.num_vars);
  }

  ScopedStyle& Color(ImGuiCol col, ImColor color) {
    ImGui::PushStyleColor(col, (ImU32)color);
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

  void Reset() {
    ImGui::PopStyleVar(num_vars);
    ImGui::PopStyleColor(num_colors);
    num_colors = 0;
    num_vars = 0;
  }

  int num_colors = 0;
  int num_vars = 0;
};

// ImGui file dialog.
bool ImGui_FileDialog(char* buf, int len);

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

// Saves the given contents to the clipboard if the clipboard is available.
void MaybeSaveToClipboard(const std::string& contents);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_IMGUI_WIDGETS_H_
