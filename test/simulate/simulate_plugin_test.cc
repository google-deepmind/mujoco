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

#include <filesystem>
#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#define main mujoco_simulate_main
#include "simulate/main.cc"
#undef main

namespace {

TEST(SimulatePluginTest, ScansDefaultAndConfiguredPluginDirectories) {
  const std::filesystem::path executable_dir(getExecutableDir());
  ASSERT_FALSE(executable_dir.empty());

  const std::filesystem::path default_plugin_dir = executable_dir / MUJOCO_PLUGIN_DIR;
  const std::filesystem::path default_plugin_library =
      default_plugin_dir / std::filesystem::path(MUJOCO_DEFAULT_PLUGIN_LIBRARY).filename();
  std::filesystem::remove(default_plugin_library);
  ASSERT_FALSE(std::filesystem::exists(default_plugin_library));
  std::filesystem::create_directories(default_plugin_dir);
  ASSERT_TRUE(std::filesystem::copy_file(
      MUJOCO_DEFAULT_PLUGIN_LIBRARY, default_plugin_library,
      std::filesystem::copy_options::overwrite_existing));

  const std::filesystem::path configured_plugin_dir(MUJOCO_CONFIGURED_PLUGIN_DIR);
  const std::filesystem::path configured_plugin_library =
      configured_plugin_dir / std::filesystem::path(MUJOCO_CONFIGURED_PLUGIN_LIBRARY).filename();
  std::filesystem::remove(configured_plugin_library);
  ASSERT_FALSE(std::filesystem::exists(configured_plugin_library));
  std::filesystem::create_directories(configured_plugin_dir);
  ASSERT_TRUE(std::filesystem::copy_file(
      MUJOCO_CONFIGURED_PLUGIN_LIBRARY, configured_plugin_library,
      std::filesystem::copy_options::overwrite_existing));

  scanPluginLibraries();

  bool loaded_default_plugin = false;
  bool loaded_configured_plugin = false;
  for (int i = 0; i < mjp_pluginCount(); ++i) {
    const std::string plugin_name(mjp_getPluginAtSlot(i)->name);
    loaded_default_plugin |= plugin_name == "mujoco.sdf.bolt";
    loaded_configured_plugin |= plugin_name == "mujoco.elasticity.cable";
  }
  EXPECT_TRUE(loaded_default_plugin);
  EXPECT_TRUE(loaded_configured_plugin);
}

}  // namespace
