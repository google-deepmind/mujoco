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

#ifndef MUJOCO_SRC_EXPERIMENTAL_STUDIO_COMMAND_PALETTE_H_
#define MUJOCO_SRC_EXPERIMENTAL_STUDIO_COMMAND_PALETTE_H_

#include <functional>
#include <string>
#include <vector>

#include <imgui.h>

namespace mujoco::studio {

// A VS Code-style command palette. The owner opens it (e.g. on Ctrl+P) and
// passes the available commands to Draw() each frame. The palette starts as a
// single-line input; typing '>' switches to command mode, which grows the box
// and shows a filtered, keyboard-navigable list. Selecting an entry runs its
// callback.
//
// This widget is intentionally self-contained and knows nothing about the app:
// commands are just {name, callback} pairs, so it is easy to move to its own
// module or reuse elsewhere.
class CommandPalette {
 public:
  struct Command {
    std::string name;
    std::function<void()> run;
  };

  void Open();
  // Opens the palette pre-filled with `text` (e.g. ">Physics"); used by the
  // capture script to show command-mode interactions.
  void OpenWith(const std::string& text);
  // Replaces the input text without opening/closing (used by the capture script
  // to "type" a question one character at a time).
  void SetText(const std::string& text);
  void Close();
  void Toggle();
  bool is_open() const { return open_; }
  // Center of the palette window from the last Draw (for the capture cursor).
  ImVec2 window_center() const { return center_; }

  // Draws the palette (if open), horizontally centered near the top of `rect`
  // (x, y, width, height). `commands` is searched in '>' command mode.
  //
  // Input that does NOT start with '>' is "ask" mode: pressing Enter calls
  // `on_submit_plain(text)` (and clears the box, keeping it open), and
  // `render_below` is invoked inside the palette window to draw the LLM
  // conversation right in the Ctrl+P box. Both callbacks are optional.
  void Draw(const std::vector<Command>& commands, const ImVec4& rect,
            const std::function<void()>& render_below = {},
            const std::function<void(const std::string&)>& on_submit_plain = {});

 private:
  bool open_ = false;
  bool focus_input_ = false;
  int selection_ = 0;
  char input_[256] = "";
  ImVec2 center_{0.0f, 0.0f};
};

}  // namespace mujoco::studio

#endif  // MUJOCO_SRC_EXPERIMENTAL_STUDIO_COMMAND_PALETTE_H_
