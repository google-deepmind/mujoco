// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Clipboard helpers for Studio's UI.

#ifndef MUJOCO_STUDIO_UX_CLIPBOARD_H_
#define MUJOCO_STUDIO_UX_CLIPBOARD_H_

#include <string>

namespace mujoco::studio {

// Saves the given contents to the clipboard if the clipboard is available.
void MaybeSaveToClipboard(const std::string& contents);

}  // namespace mujoco::studio

#endif  // MUJOCO_STUDIO_UX_CLIPBOARD_H_
