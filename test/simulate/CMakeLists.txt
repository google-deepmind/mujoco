# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

mujoco_test(
  simulate_plugin_test
  ADDITIONAL_LINK_LIBRARIES
  libmujoco_simulate
  glfw
  Threads::Threads
)
target_compile_definitions(
  simulate_plugin_test
  PRIVATE
  "MUJOCO_CONFIGURED_PLUGIN_DIR=\"${CMAKE_CURRENT_BINARY_DIR}/configured_plugins\""
  "MUJOCO_CONFIGURED_PLUGIN_LIBRARY=\"$<TARGET_FILE:elasticity>\""
  "MUJOCO_DEFAULT_PLUGIN_LIBRARY=\"$<TARGET_FILE:sdf_plugin>\""
)
simulate_add_plugin_directories(simulate_plugin_test "${CMAKE_CURRENT_BINARY_DIR}/configured_plugins")
add_dependencies(simulate_plugin_test elasticity sdf_plugin)
