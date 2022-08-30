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

mujoco_test(engine_collision_convex_test)
target_link_libraries(engine_collision_convex_test fixture gmock)

mujoco_test(engine_collision_driver_test)
target_link_libraries(engine_collision_driver_test fixture gmock)

mujoco_test(engine_core_constraint_test)
target_link_libraries(engine_core_constraint_test fixture gmock)

mujoco_test(engine_core_smooth_test)
target_link_libraries(engine_core_smooth_test fixture gmock)

mujoco_test(engine_derivative_test)
target_link_libraries(engine_derivative_test fixture gmock)

mujoco_test(engine_forward_test)
target_link_libraries(engine_forward_test fixture gmock)

mujoco_test(engine_io_test)
target_link_libraries(
  engine_io_test
  fixture
  gmock
  absl::str_format
)

mujoco_test(engine_print_test)
target_link_libraries(engine_print_test fixture gmock)

mujoco_test(engine_ray_test)
target_link_libraries(engine_ray_test fixture gmock)

mujoco_test(engine_sensor_test)
target_link_libraries(engine_sensor_test fixture gmock)

mujoco_test(engine_support_test)
target_link_libraries(engine_support_test fixture gmock)

mujoco_test(engine_util_blas_test)
target_link_libraries(engine_util_blas_test fixture gmock)

mujoco_test(engine_util_errmem_test)
target_link_libraries(engine_util_errmem_test fixture gmock)

mujoco_test(engine_util_solve_test)
target_link_libraries(engine_util_solve_test fixture gmock)

mujoco_test(engine_util_spatial_test)
target_link_libraries(engine_util_spatial_test fixture gmock)
