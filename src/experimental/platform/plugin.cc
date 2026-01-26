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

#include <functional>
#include <string_view>

#include <mujoco/mujoco.h>
#include "experimental/platform/plugin.h"
#include "engine/engine_global_table.h"

namespace mujoco::platform {

void RegisterGuiPlugin(const GuiPlugin* plugin) {
  if (plugin->name == nullptr || plugin->name[0] == '\0') {
    mju_error("Plugin name must not be empty or null.");
  }
  GlobalTable<GuiPlugin>::GetSingleton().AppendIfUnique(*plugin);
}

void ForEachGuiPlugin(const std::function<void(GuiPlugin*)>& fn) {
  auto& table = GlobalTable<GuiPlugin>::GetSingleton();
  for (int i = 0; i < table.count(); ++i) {
    const GuiPlugin* plugin = table.GetAtSlot(i);
    fn(const_cast<GuiPlugin*>(plugin));
  }
}

void RegisterModelPlugin(const ModelPlugin* plugin) {
  if (plugin->name == nullptr || plugin->name[0] == '\0') {
    mju_error("Plugin name must not be empty or null.");
  }
  GlobalTable<ModelPlugin>::GetSingleton().AppendIfUnique(*plugin);
}

void ForEachModelPlugin(const std::function<void(ModelPlugin*)>& fn) {
  auto& table = GlobalTable<ModelPlugin>::GetSingleton();
  for (int i = 0; i < table.count(); ++i) {
    const ModelPlugin* plugin = table.GetAtSlot(i);
    fn(const_cast<ModelPlugin*>(plugin));
  }
}

}  // namespace mujoco::platform

using mujoco::GlobalTable;
using GuiPlugin = mujoco::platform::GuiPlugin;
using ModelPlugin = mujoco::platform::ModelPlugin;

template <>
const char* GlobalTable<GuiPlugin>::HumanReadableTypeName() {
  return "gui plugin";
}

template <>
std::string_view GlobalTable<GuiPlugin>::ObjectKey(const GuiPlugin& plugin) {
  return std::string_view(plugin.name);
}

template <>
bool GlobalTable<GuiPlugin>::ObjectEqual(const GuiPlugin& p1, const GuiPlugin& p2) {
  return CaseInsensitiveEqual(p1.name, p2.name);
}

template <>
bool GlobalTable<GuiPlugin>::CopyObject(GuiPlugin& dst, const GuiPlugin& src, ErrorMessage& err) {
  dst = src;
  return true;
}

template <>
const char* GlobalTable<ModelPlugin>::HumanReadableTypeName() {
  return "model plugin";
}

template <>
std::string_view GlobalTable<ModelPlugin>::ObjectKey(const ModelPlugin& plugin) {
  return std::string_view(plugin.name);
}

template <>
bool GlobalTable<ModelPlugin>::ObjectEqual(const ModelPlugin& p1, const ModelPlugin& p2) {
  return CaseInsensitiveEqual(p1.name, p2.name);
}

template <>
bool GlobalTable<ModelPlugin>::CopyObject(ModelPlugin& dst, const ModelPlugin& src, ErrorMessage& err) {
  dst = src;
  return true;
}
