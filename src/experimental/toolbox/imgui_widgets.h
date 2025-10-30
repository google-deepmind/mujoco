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

#ifndef MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_IMGUI_WIDGETS_H_
#define MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_IMGUI_WIDGETS_H_

#include <cstring>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <imgui.h>
#include <imgui_internal.h>  // For ButtonEx and PressedOnClick
#include <mujoco/mujoco.h>

namespace mujoco::toolbox {

using KeyValues = std::unordered_map<std::string, std::string>;

// Appends key/value pairs to an Ini file.
void AppendIniSection(std::string& ini, const std::string& section,
                      const KeyValues& key_values);

// Reads key/value pairs from an Ini file section.
KeyValues ReadIniSection(const std::string& contents,
                         const std::string& section);

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

enum class ToggleKind {
  // Solid when ON and transparent when OFF.
  kButton,

  // Slider which is right when ON and left when OFF.
  kSlider,
};

template <typename T>
bool Toggle(const char* label, T& boolean,
            ToggleKind kind = ToggleKind::kButton, bool set_width = true) {
  static_assert(std::is_integral_v<T>, "Toggle only supports integral types.");

  // Compute this width once and cache it. Only used when set_width is true.
  static int toggle_width = []() {
    int longest = 0;
    const char* longest_label = "";
    for (int i = 0; i < mjNVISFLAG; ++i) {
      int length = static_cast<int>(strlen(mjVISSTRING[i][0]));
      if (length > longest) {
        longest_label = mjVISSTRING[i][0];
        longest = length;
      }
    }
    return ImGui::CalcTextSize(longest_label).x + 5;
  }();

  ImGui::PushID(label);
  bool changed = false;
  switch (kind) {
    case ToggleKind::kButton: {
      bool b = (boolean != 0);
      bool transparent = !b;

      // NOTE(matijak): Its nice to have the button trigger on click but this
      // requires using the currently internal PressedOnClick flag and ButtonEx
      // function. It looks like this API has been stable for a long time, but
      // in case it changes in a way which breaks and is annoying to maintain we
      // can revert to the else clause and remove the imgui_internal.h include.
      // Note that the else clause overrides different style colors since the UI
      // is more intuitive with different settings.
      if constexpr (true) {
        ImColor button = ImGui::GetStyle().Colors[ImGuiCol_Button];
        if (transparent) button.Value.w = 0.0f;
        ImGui::PushStyleColor(ImGuiCol_Button, (ImU32)button);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImU32)button);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImU32)button);

        // Button width is set via an explicit size parameter, not via the
        // SetNextItemWidth function.
        ImVec2 size = set_width ? ImVec2(toggle_width, 0) : ImVec2(0, 0);
        changed = ImGui::ButtonEx(label, size, ImGuiButtonFlags_PressedOnClick);

        if (changed) {
          b = !b;
        }
        boolean = b;

        ImGui::PopStyleColor(3);
      } else {
        if (transparent) {
          ImColor button = ImGui::GetStyle().Colors[ImGuiCol_Button];
          button.Value.w = 0.0f;
          ImGui::PushStyleColor(ImGuiCol_Button, (ImU32)button);
        }

        // Button width is set via an explicit size parameter, not via the
        // SetNextItemWidth function.
        ImVec2 size = set_width ? ImVec2(toggle_width, 0) : ImVec2(0, 0);
        changed = ImGui::Button(label, size);

        if (changed) {
          b = !b;
        }
        boolean = b;

        if (transparent) {
          ImGui::PopStyleColor(1);
        }
      }
    } break;

    case ToggleKind::kSlider: {
      int i = (int)boolean;
      const char* labels[2] = {label, label};
      const ImGuiSliderFlags flags = ImGuiSliderFlags_NoInput;
      if (set_width) ImGui::SetNextItemWidth(toggle_width);
      changed = ImGui::SliderInt("", &i, 0, 1, labels[i], flags);
      boolean = (i != 0);
    } break;
  }
  ImGui::PopID();
  return changed;
}

inline bool ToggleBit(const char* label, int& flags, int flags_value,
                      ToggleKind kind = ToggleKind::kButton,
                      bool set_width = true) {
  bool boolean = flags & flags_value;
  bool changed = Toggle(label, boolean, kind, set_width);
  if (changed) {
    flags = boolean ? (flags | flags_value) : (flags & ~flags_value);
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
    static_assert(false, "Unsupported type");
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

}  // namespace mujoco::toolbox

#endif  // MUJOCO_SRC_EXPERIMENTAL_TOOLBOX_IMGUI_WIDGETS_H_
