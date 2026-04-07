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

add_library(${MUJOCO_PLATFORM_TARGET_NAME} STATIC)

target_sources(${MUJOCO_PLATFORM_TARGET_NAME}
    PUBLIC
        helpers.cc
        helpers.h
        hal/egl_utils.cc
        hal/egl_utils.h
        hal/graphics_mode.cc
        hal/graphics_mode.h
        hal/renderer.cc
        hal/renderer.h
        hal/window.cc
        hal/window.h
        sim/model_holder.cc
        sim/model_holder.h
        sim/sim_history.cc
        sim/sim_history.h
        sim/sim_profiler.cc
        sim/sim_profiler.h
        sim/step_control.cc
        sim/step_control.h
        ux/enum_utils.h
        ux/file_dialog.h
        ux/gui.cc
        ux/gui.h
        ux/gui_spec.cc
        ux/gui_spec.h
        ux/imgui_widgets.cc
        ux/imgui_widgets.h
        ux/interaction.cc
        ux/interaction.h
        ux/picture_gui.h
        ux/picture_gui.cc
        ux/plugin.cc
        ux/plugin.h
        ux/spec_editor.cc
        ux/spec_editor.h
)

if(NOT WIN32)
  target_sources(${MUJOCO_PLATFORM_TARGET_NAME}
      PUBLIC
          ux/object_launcher_plugin.cc
  )
endif()

if(APPLE)
    set_source_files_properties(hal/window_osx.mm PROPERTIES
        COMPILE_FLAGS "-x objective-c++")
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        hal/window_osx.mm
        ux/file_dialog_cocoa.mm
    )
elseif(UNIX AND NOT APPLE)
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        ux/file_dialog_zenity.cc
    )
elseif(WIN32)
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        ux/file_dialog_win.cc
    )
else()
    target_sources(${MUJOCO_PLATFORM_TARGET_NAME} PUBLIC
        ux/file_dialog.cc
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
    dear_imgui_OpenGL3
    dear_imgui_stdlib
    implot
    mujoco::filament
    webp
    SDL2::SDL2-static
    ${MUJOCO_PLATFORM_RENDER_DEPS}
)

add_library(mujoco::platform ALIAS ${MUJOCO_PLATFORM_TARGET_NAME})
