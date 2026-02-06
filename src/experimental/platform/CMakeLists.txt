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

set(MUJOCO_PLATFORM_TARGET_NAME mujoco_platform)

# Determine the render configuration and dependencies based options.
if(MUJOCO_USE_FILAMENT AND MUJOCO_USE_FILAMENT_VULKAN)
    set(MUJOCO_PLATFORM_RENDER_CONFIG "MUJOCO_RENDERER_FILAMENT_VULKAN")
    set(MUJOCO_PLATFORM_RENDER_DEPS "mujoco::filament")
elseif(MUJOCO_USE_FILAMENT)
    set(MUJOCO_PLATFORM_RENDER_CONFIG "MUJOCO_RENDERER_FILAMENT_OPENGL")
    set(MUJOCO_PLATFORM_RENDER_DEPS "mujoco::filament")
else()
    set(MUJOCO_PLATFORM_RENDER_CONFIG "MUJOCO_RENDERER_CLASSIC_OPENGL")
    set(MUJOCO_PLATFORM_RENDER_DEPS "dear_imgui_OpenGL3")
endif()

add_library(${MUJOCO_PLATFORM_TARGET_NAME} STATIC)

target_compile_definitions(${MUJOCO_PLATFORM_TARGET_NAME}
    PUBLIC
        -D${MUJOCO_PLATFORM_RENDER_CONFIG}
)

target_sources(${MUJOCO_PLATFORM_TARGET_NAME}
    PUBLIC
        egl_utils.cc
        egl_utils.h
        file_dialog.h
        gui.cc
        gui.h
        helpers.cc
        helpers.h
        imgui_widgets.cc
        imgui_widgets.h
        interaction.cc
        interaction.h
        model_holder.cc
        model_holder.h
        picture_gui.h
        picture_gui.cc
        plugin.cc
        plugin.h
        renderer.cc
        renderer.h
        renderer_backend.h
        sim_history.cc
        sim_history.h
        sim_profiler.cc
        sim_profiler.h
        step_control.cc
        step_control.h
        window.cc
        window.h
)

if(APPLE)
    set_source_files_properties(window_osx.mm PROPERTIES
        COMPILE_FLAGS "-x objective-c++")
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        window_osx.mm
        file_dialog_cocoa.mm
    )
elseif(UNIX AND NOT APPLE)
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        file_dialog_zenity.cc
    )
elseif(WINDOWS)
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        file_dialog_win.cc
    )
else()
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        file_dialog.cc
    )
endif()

target_include_directories(${MUJOCO_PLATFORM_TARGET_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

include(third_party_deps/dear_imgui)
include(third_party_deps/implot)
include(third_party_deps/sdl2)
include(third_party_deps/libwebp)

target_link_libraries(${MUJOCO_PLATFORM_TARGET_NAME}
    dear_imgui
    dear_imgui_SDL2
    implot
    webp
    SDL2::SDL2-static
    ${MUJOCO_PLATFORM_RENDER_DEPS}
)

add_library(mujoco::platform ALIAS ${MUJOCO_PLATFORM_TARGET_NAME})
