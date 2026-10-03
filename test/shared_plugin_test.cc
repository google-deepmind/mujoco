// Copyright 2021 DeepMind Technologies Limited
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

#include <array>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include "rules_cc/cc/runfiles/runfiles.h"

int main(int argc, char** argv) {
  using rules_cc::cc::runfiles::Runfiles;
  if (argc != 5) {
    std::fprintf(stderr,
                 "Expected actuator, elasticity, sensor, and SDF libraries\n");
    return EXIT_FAILURE;
  }
  std::string error;
  std::unique_ptr<Runfiles> runfiles(
      Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY, &error));
  if (!runfiles) {
    std::fprintf(stderr, "Cannot locate runfiles: %s\n", error.c_str());
    return EXIT_FAILURE;
  }
  for (int i = 1; i < argc; ++i) {
    const std::string library = runfiles->Rlocation(argv[i]);
    if (library.empty()) {
      std::fprintf(stderr, "Cannot locate plugin: %s\n", argv[i]);
      return EXIT_FAILURE;
    }
    mj_loadPluginLibrary(library.c_str());
  }
  constexpr std::array names = {
      "mujoco.pid",      "mujoco.elasticity.cable", "mujoco.sensor.touch_grid",
      "mujoco.sdf.bolt", "mujoco.sdf.bowl",         "mujoco.sdf.gear",
      "mujoco.sdf.nut",  "mujoco.sdf.torus",
  };
  for (const char* name : names) {
    if (!mjp_getPlugin(name, nullptr)) {
      std::fprintf(stderr, "Plugin did not register: %s\n", name);
      return EXIT_FAILURE;
    }
  }
  if (mjp_pluginCount() != static_cast<int>(names.size())) {
    std::fprintf(stderr, "Unexpected plugin count: %d\n", mjp_pluginCount());
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
