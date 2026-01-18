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

set(MUJOCO_STUDIO_TARGET_NAME mujoco_studio)

add_executable(${MUJOCO_STUDIO_TARGET_NAME})

target_sources(${MUJOCO_STUDIO_TARGET_NAME}
    PRIVATE
        app.cc
        app.h
        main.cc
)

target_include_directories(${MUJOCO_STUDIO_TARGET_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

if (WIN32)
    target_compile_definitions(${MUJOCO_STUDIO_TARGET_NAME}
        PRIVATE
        -D_USE_MATH_DEFINES
    )
    set_target_properties(${MUJOCO_STUDIO_TARGET_NAME}
        PROPERTIES
            VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )
endif()

include(third_party_deps/dear_imgui)
include(third_party_deps/implot)
include(third_party_deps/opensans)
include(third_party_deps/font_awesome)

target_link_libraries(${MUJOCO_STUDIO_TARGET_NAME}
    PRIVATE
        absl::flags
        dear_imgui
        implot
        mujoco::mujoco
        mujoco::platform
)

file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets)

add_custom_command(
    TARGET ${MUJOCO_STUDIO_TARGET_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -E copy
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../_deps/opensans-src/fonts/ttf/OpenSans-Regular.ttf
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
    COMMENT "Copying OpenSans-Regular.ttf assets to build directory"
)
add_custom_command(
    TARGET ${MUJOCO_STUDIO_TARGET_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND}
        -E copy
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../_deps/font_awesome-src/fonts/fontawesome-webfont.ttf
            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
    COMMENT "Copying fontawesome-webfont.ttf to build directory"
)

# Filament backend requires additional files to be copied into an "assets" folder.
if(MUJOCO_USE_FILAMENT)
    add_custom_command(
        TARGET ${MUJOCO_STUDIO_TARGET_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -E copy_directory
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../src/experimental/filament/assets
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
        COMMENT "Copying Filament assets to build directory"
    )
endif()
