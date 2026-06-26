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

set(MUJOCO_FILAMENT_TARGET_NAME mujoco_filament)

add_library(${MUJOCO_FILAMENT_TARGET_NAME} STATIC)
target_compile_definitions(${MUJOCO_FILAMENT_TARGET_NAME} PRIVATE MJ_STATIC)

target_sources(${MUJOCO_FILAMENT_TARGET_NAME}
    PUBLIC
        mjrfilament.cc
        mjrfilament_cpp.h
        core/builtins.cc
        core/builtins.h
        core/color_grading_options.cc
        core/color_grading_options.h
        core/filament_context.cc
        core/filament_context.h
        core/filament_platform_factory.cc
        core/filament_platform_factory.h
        core/imgui_editor.cc
        core/light.cc
        core/light.h
        core/material_manager.cc
        core/material_manager.h
        core/mesh.cc
        core/mesh.h
        core/object_manager.cc
        core/object_manager.h
        core/outliner.cc
        core/outliner.h
        core/reflection_manager.cc
        core/reflection_manager.h
        core/render_target.cc
        core/render_target.h
        core/renderable.cc
        core/renderable.h
        core/scene_view.cc
        core/scene_view.h
        core/texture.cc
        core/texture.h
        core/types.h
        support/filament_util.h
        support/filament_util.cc
        support/light_manager.h
        support/light_manager.cc
        support/model_objects.h
        support/model_objects.cc
)

target_include_directories(${MUJOCO_FILAMENT_TARGET_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
)

include(third_party_deps/dear_imgui)
include(third_party_deps/filament)
target_link_libraries(${MUJOCO_FILAMENT_TARGET_NAME}
    dear_imgui
    filament
    image       # provided by filament
    ktxreader   # provided by filament
)

add_library(mujoco::filament ALIAS ${MUJOCO_FILAMENT_TARGET_NAME})

# The "mujoco::filament" library needs access to some assets (e.g. filament
# material shaders) in order to do rendering. As such, you must also include
# the assets under MUJOCO_FILAMENT_ASSETS_DIR in your project. As part of
# the filament library configuration, you will provide a function that the
# library will use to load these assets.
set(ASSETS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/assets")
set(OUTPUT_ASSETS_DIR "${CMAKE_CURRENT_BINARY_DIR}/assets")
file(MAKE_DIRECTORY ${OUTPUT_ASSETS_DIR})
file(COPY
    "${ASSETS_DIR}/ibl.ktx"
    DESTINATION ${OUTPUT_ASSETS_DIR}
)

set(MATC_EXECUTABLE matc)
set(MATERIAL_FILES
    pbr.mat
    pbr_transparent.mat
    pbr_packed.mat
    pbr_packed_transparent.mat
    phong_2d_fade.mat
    phong_2d.mat
    phong_2d_reflect.mat
    phong_2d_uv_fade.mat
    phong_2d_uv.mat
    phong_2d_uv_reflect.mat
    phong_color_fade.mat
    phong_color.mat
    phong_color_reflect.mat
    phong_cube_fade.mat
    phong_cube.mat
    phong_cube_reflect.mat
    outline_composite.mat
    outline_flatten.mat
    outline_jumpflood.mat
    decor.mat
    unlit_depth.mat
    unlit_segmentation.mat
    unlit_ui.mat
)

foreach(MATERIAL_FILE ${MATERIAL_FILES})
    get_filename_component(MATERIAL_NAME ${MATERIAL_FILE} NAME_WE)
    set(INPUT_FILE "${ASSETS_DIR}/${MATERIAL_FILE}")
    set(OUTPUT_FILE "${OUTPUT_ASSETS_DIR}/${MATERIAL_NAME}.filamat")

    if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        set(PRECOMPILED_FILE "${MUJOCO_NATIVE_BUILD_DIR}/src/render/filament/assets/${MATERIAL_NAME}.filamat")
        add_custom_command(
            OUTPUT ${OUTPUT_FILE}
            COMMAND ${CMAKE_COMMAND} -E copy
                ${PRECOMPILED_FILE}
                ${OUTPUT_FILE}
            DEPENDS ${PRECOMPILED_FILE}
            COMMENT "Copying precompiled material ${MATERIAL_NAME}.filamat"
        )
    else()
        add_custom_command(
            OUTPUT ${OUTPUT_FILE}
            COMMAND ${MATC_EXECUTABLE}
                --platform=all
                --api=vulkan
                --api=opengl
                --variant-filter skinning
                --optimize-size
                --output ${OUTPUT_FILE}
                ${INPUT_FILE}
            DEPENDS ${INPUT_FILE}
            DEPENDS matc
            COMMENT "Compiling ${MATERIAL_FILE}"
        )
    endif()

    list(APPEND MUJOCO_FILAMENT_ASSET_FILES ${OUTPUT_FILE})
endforeach()

list(APPEND MUJOCO_FILAMENT_ASSET_FILES "${OUTPUT_ASSETS_DIR}/ibl.ktx")

add_custom_target(mujoco_filament_assets DEPENDS ${MUJOCO_FILAMENT_ASSET_FILES})
add_dependencies(${MUJOCO_FILAMENT_TARGET_NAME} mujoco_filament_assets)

set(MUJOCO_FILAMENT_ASSETS_DIR ${OUTPUT_ASSETS_DIR})
