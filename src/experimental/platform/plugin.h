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

#ifndef MUJOCO_SRC_EXPERIMENTAL_PLATFORM_PLUGIN_H_
#define MUJOCO_SRC_EXPERIMENTAL_PLATFORM_PLUGIN_H_

#include <functional>

namespace mujoco::platform {

// Important: Do not inherit from these plugin structs. They are copied by value
// and therefore any derived classes will be sliced. We assume plugins are
// effectively globals and so any pointers will be valid for the lifetime
// of the process.

// Plugin for processing custom UI windows. The plugin will be listed in the
// "Plugins" main menu and, when selected, an ImGui window will be opened with
// the name of the plugin as the title. The `update` function can then be used
// to process the GUI. All functions will be called by passing `this` as the
// first argument.
struct GuiPlugin final {
  using UpdateFn = void (*)(GuiPlugin* self);

  // Whether or not to display the plugin window.
  bool active = false;

  // The name of the plugin; must be unique.
  const char* name = "";

  // The function that will update the plugin's window. Plugin GUI updates
  // happen when the window is `active` and after all other Studio GUI updates.
  UpdateFn update = nullptr;

  // Optional data pointer.
  void* data = nullptr;
};

// Registers a plugin with a global registry. The plugin must have a
// case-insensitive unique name.
void RegisterGuiPlugin(const GuiPlugin* plugin);

// Executes the given function for each registered plugin.
void ForEachGuiPlugin(const std::function<void(GuiPlugin*)>& fn);

}  // namespace mujoco::platform

#endif  // MUJOCO_SRC_EXPERIMENTAL_PLATFORM_PLUGIN_H_
