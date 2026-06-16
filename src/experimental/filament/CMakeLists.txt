# Copyright 2025 DeepMind Technologies Limited
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

cmake_minimum_required(VERSION 3.16)

set(MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME mujoco_filament_experimental)

add_library(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME} STATIC)
target_compile_definitions(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME} PRIVATE MJ_STATIC)

target_sources(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME}
    PUBLIC
        compat/scene_bridge.cc
        compat/scene_bridge.h
        compat/scene_geom_util.cc
        compat/scene_geom_util.h
        compat/scene_objects.cc
        compat/scene_objects.h
)
if(MUJOCO_USE_FILAMENT_MJR_COMPAT)
    target_sources(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME}
        PUBLIC
            mjr_compat.cc
    )
endif()

target_include_directories(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

target_link_libraries(${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME}
    mujoco_filament
)

add_library(mujoco::filament_experimental ALIAS ${MUJOCO_FILAMENT_EXPERIMENTAL_TARGET_NAME})
