# Copyright 2021 DeepMind Technologies Limited
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

mujoco_test(user_model_test)
target_link_libraries(user_model_test fixture gmock absl::str_format)

mujoco_test(user_objects_test)
target_link_libraries(user_objects_test fixture gmock)

mujoco_test(
  user_api_test
  PROPERTIES
  ENVIRONMENT
  "MUJOCO_PLUGIN_DIR=$<TARGET_FILE_DIR:elasticity>"
)
target_link_libraries(user_api_test fixture gmock)

mujoco_test(user_flex_test)
target_link_libraries(user_flex_test fixture gmock)

mujoco_test(user_mesh_test)
target_link_libraries(
  user_mesh_test
  fixture
  gmock
  absl::str_format
)

mujoco_test(user_composite_test)
target_link_libraries(user_composite_test fixture gmock)

mujoco_test(user_resource_test)
target_link_libraries(user_resource_test fixture gmock)

mujoco_test(user_vfs_test)
target_link_libraries(user_vfs_test fixture gmock)
