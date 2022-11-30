// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_ENGINE_ENGINE_PLUGIN_H_
#define MUJOCO_SRC_ENGINE_ENGINE_PLUGIN_H_

#include <mujoco/mjexport.h>
#include <mujoco/mjplugin.h>

#ifdef __cplusplus
extern "C" {
#endif

// set default plugin definition
MJAPI void mjp_defaultPlugin(mjpPlugin* plugin);

// globally register a plugin (thread-safe), return new slot id
MJAPI int mjp_registerPlugin(const mjpPlugin* plugin);

// return the number of globally registered plugins
MJAPI int mjp_pluginCount();

// look up a plugin by name, optionally also get its registered slot number
MJAPI const mjpPlugin* mjp_getPlugin(const char* name, int* slot);

// look up a plugin by slot number
MJAPI const mjpPlugin* mjp_getPluginAtSlot(int slot);

// return a config attribute of a plugin instance
// NULL: invalid plugin instance ID or attribute name
MJAPI const char* mj_getPluginConfig(const mjModel* m, int plugin_id, const char* attrib);

// load plugins from a dynamic library
MJAPI void mj_loadPluginLibrary(const char* path);

// scan a directory and load all dynamic libraries
MJAPI void mj_loadAllPluginLibraries(const char* directory, mjfPluginLibraryLoadCallback callback);

// =================================================================================================
// MuJoCo-internal functions beyond this point.
// "Unsafe" suffix indicates that improper use of these functions may result in data races.
//
// The unsafe functions assume that called mjp_pluginCount has already been called, and that it is
// safe to assume that all plugins up to `count` have been completely written into the global table.
// =================================================================================================

// look up a plugin by name, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginUnsafe(const char* name, int* slot, int nslot);

// look up a plugin by slot number, assuming that mjp_pluginCount has already been called
const mjpPlugin* mjp_getPluginAtSlotUnsafe(int slot, int nslot);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_SRC_ENGINE_ENGINE_PLUGIN_H_
