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

add_library(lodepng OBJECT)

set_target_properties(lodepng PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(lodepng
  PUBLIC
    $<BUILD_INTERFACE:${lodepng_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_sources(lodepng
  PRIVATE
    ${lodepng_SOURCE_DIR}/lodepng.cpp
  PUBLIC
    $<BUILD_INTERFACE:${lodepng_SOURCE_DIR}/lodepng.h>
)

target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
