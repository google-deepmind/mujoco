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

# Determine the render configuration and dependencies based options.
if(MUJOCO_USE_FILAMENT AND MUJOCO_USE_FILAMENT_VULKAN)
    set(MUJOCO_STUDIO_RENDER_CONFIG "USE_FILAMENT_VULKAN")
    set(MUJOCO_STUDIO_RENDER_DEPS "mujoco::filament")
elseif(MUJOCO_USE_FILAMENT)
    set(MUJOCO_STUDIO_RENDER_CONFIG "USE_FILAMENT_OPENGL")
    set(MUJOCO_STUDIO_RENDER_DEPS "mujoco::filament")
else()
    set(MUJOCO_STUDIO_RENDER_CONFIG "USE_CLASSIC_OPENGL")
endif()

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

target_compile_definitions(${MUJOCO_STUDIO_TARGET_NAME}
    PRIVATE
        -D${MUJOCO_STUDIO_RENDER_CONFIG}
)

include(third_party_deps/dear_imgui)
include(third_party_deps/implot)
target_link_libraries(${MUJOCO_STUDIO_TARGET_NAME}
    PRIVATE
        ${MUJOCO_STUDIO_RENDER_DEPS}
        absl::flags
        dear_imgui
        dear_imgui_OpenGL3
        dear_imgui_SDL2
        implot
        mujoco::mujoco
        mujoco::toolbox
)

# Filament backend requires additional files to be copied into an "assets" folder.
if(MUJOCO_USE_FILAMENT)
    add_custom_command(
        TARGET ${MUJOCO_STUDIO_TARGET_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -E create_symlink
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../src/experimental/filament/assets
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
        COMMENT "Copying Filament assets to build directory"
    )
endif()
