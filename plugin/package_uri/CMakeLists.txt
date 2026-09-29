# Copyright 2026 PickNik Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(MUJOCO_PACKAGE_URI_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../..)

add_library(package_uri SHARED)
target_sources(package_uri PRIVATE package_uri.cc)
target_include_directories(package_uri PRIVATE ${MUJOCO_PACKAGE_URI_INCLUDE})
target_link_libraries(package_uri PRIVATE mujoco)
target_compile_options(
  package_uri
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)
target_link_options(
  package_uri
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)
