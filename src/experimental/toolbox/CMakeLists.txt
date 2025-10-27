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

set(MUJOCO_TOOLBOX_TARGET_NAME mujoco_toolbox)

add_library(${MUJOCO_TOOLBOX_TARGET_NAME} STATIC)

target_sources(${MUJOCO_TOOLBOX_TARGET_NAME}
    PUBLIC
        helpers.cc
        helpers.h
        imgui_widgets.cc
        imgui_widgets.h
        interaction.cc
        interaction.h
        physics.cc
        physics.h
        renderer.cc
        renderer.h
        step_control.cc
        step_control.h
        window.cc
        window.h
)
target_include_directories(${MUJOCO_TOOLBOX_TARGET_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

include(third_party_deps/dear_imgui)
include(third_party_deps/implot)
include(third_party_deps/sdl2)
include(third_party_deps/libwebp)

target_link_libraries(${MUJOCO_TOOLBOX_TARGET_NAME}
    dear_imgui
    implot
    webp
    SDL2::SDL2-static
)

add_library(mujoco::toolbox ALIAS ${MUJOCO_TOOLBOX_TARGET_NAME})
