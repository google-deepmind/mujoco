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

include(third_party_deps/dear_imgui)
include(third_party_deps/implot)
include(third_party_deps/opensans)
include(third_party_deps/font_awesome)

# Common configuration shared between mujoco_studio and mujoco_live.
function(configure_studio_target TARGET_NAME)
    target_sources(${TARGET_NAME}
        PRIVATE
            app.cc
            app.h
    )

    target_include_directories(${TARGET_NAME}
        PUBLIC
            ${PROJECT_SOURCE_DIR}/include
            ${PROJECT_SOURCE_DIR}/src
    )

    if (WIN32)
        target_compile_definitions(${TARGET_NAME}
            PRIVATE
            -D_USE_MATH_DEFINES
        )
        set_target_properties(${TARGET_NAME}
            PROPERTIES
                VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
        )
    endif()

    target_link_libraries(${TARGET_NAME}
        PRIVATE
            absl::flags
            absl::flags_parse
            dear_imgui
            implot
            mujoco::mujoco
            mujoco::platform
    )

    file(MAKE_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets)

    add_custom_command(
        TARGET ${TARGET_NAME}
        POST_BUILD
        COMMAND ${CMAKE_COMMAND}
            -E copy
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../_deps/opensans-src/fonts/ttf/OpenSans-Regular.ttf
                ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
        COMMENT "Copying OpenSans-Regular.ttf assets to build directory"
    )
    add_custom_command(
        TARGET ${TARGET_NAME}
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
            TARGET ${TARGET_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND}
                -E copy_directory
                    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../src/experimental/filament/assets
                    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/assets
            COMMENT "Copying Filament assets to build directory"
        )
    endif()
endfunction()

# Desktop Studio target.
if(NOT EMSCRIPTEN)
    add_executable(mujoco_studio)
    target_sources(mujoco_studio PRIVATE main.cc)
    configure_studio_target(mujoco_studio)
endif()

# WASM Live target.
if(EMSCRIPTEN)
    add_executable(mujoco_studio)
    target_sources(mujoco_studio PRIVATE emscripten.cc)
    target_compile_options(mujoco_studio PRIVATE -g)
    configure_studio_target(mujoco_studio)
    target_link_libraries(mujoco_studio PRIVATE mujoco::render_noop)

    # Ensure resource decoder plugins are linked into the WASM binary.
    target_link_options(mujoco_studio PRIVATE
        -Wl,--whole-archive $<TARGET_FILE:mujoco> -Wl,--no-whole-archive
    )

    # Filament's OpenGL backend requires WebGL2 / full ES3 bindings.
    target_link_options(mujoco_studio PRIVATE
        --bind
        -sUSE_WEBGL2=1
        -sFULL_ES3
        -sMIN_WEBGL_VERSION=2
        -sMAX_WEBGL_VERSION=2
        -sALLOW_MEMORY_GROWTH=1
        -sASYNCIFY=1
        -sFETCH=1
        -sGL_PREINITIALIZED_CONTEXT=1
        -sSTACK_SIZE=512mb
        -sINITIAL_MEMORY=1024mb
        -sASSERTIONS=1
        -fexceptions
        -g
    )
endif()
