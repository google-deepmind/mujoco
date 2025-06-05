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

mujoco_test(engine_collision_box_test)

mujoco_test(engine_collision_convex_test)

mujoco_test(engine_collision_driver_test)

mujoco_test(engine_collision_gjk_test ADDITIONAL_LINK_LIBRARIES ccd)

mujoco_test(engine_core_constraint_test)

mujoco_test(engine_core_smooth_test ADDITIONAL_LINK_LIBRARIES absl::span)

mujoco_test(engine_derivative_test)

mujoco_test(engine_forward_test)

mujoco_test(engine_inverse_test)

mujoco_test(engine_island_test)

mujoco_test(engine_io_test ADDITIONAL_LINK_LIBRARIES absl::str_format)

mujoco_test(
  engine_plugin_test
  ADDITIONAL_LINK_LIBRARIES
  absl::str_format
  PROPERTIES
  ENVIRONMENT
  "MUJOCO_PLUGIN_DIR=$<TARGET_FILE_DIR:elasticity>"
)

mujoco_test(engine_passive_test)

mujoco_test(engine_print_test)

mujoco_test(engine_ray_test)

mujoco_test(engine_sensor_test)

mujoco_test(engine_solver_test)

mujoco_test(engine_sort_test)

mujoco_test(engine_support_test)

mujoco_test(engine_thread_test)

mujoco_test(engine_util_blas_test)

mujoco_test(engine_util_errmem_test)

mujoco_test(engine_util_solve_test)

mujoco_test(engine_util_spatial_test)

mujoco_test(engine_vis_visualize_test)
