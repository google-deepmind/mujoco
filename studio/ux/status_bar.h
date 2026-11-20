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

// Shared status bar drawing helpers for Studio and the web viewer's browser bar.

#ifndef MUJOCO_STUDIO_UX_STATUS_BAR_H_
#define MUJOCO_STUDIO_UX_STATUS_BAR_H_

namespace mujoco::studio {

// The dot between two status bar items, on the current line.
void StatusSeparatorGui();

// Draws Studio's "StatusBar" window with `status_text` on the left and the
// console summary button on the right, toggling `*show_console` on click.
void StatusBarGui(const char* status_text, bool* show_console);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_STATUS_BAR_H_
