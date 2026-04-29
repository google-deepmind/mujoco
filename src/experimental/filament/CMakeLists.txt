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
        render_context_filament.h
        render_context_filament.cc
        filament/builtins.cc
        filament/builtins.h
        filament/color_grading_options.cc
        filament/color_grading_options.h
        filament/filament_context.cc
        filament/filament_context.h
        filament/filament_platform_factory.cc
        filament/filament_platform_factory.h
        filament/light.cc
        filament/light.h
        filament/material.cc
        filament/material.h
        filament/math_util.cc
        filament/math_util.h
        filament/mesh.cc
        filament/mesh.h
        filament/model_util.h
        filament/object_manager.cc
        filament/object_manager.h
        filament/render_target.cc
        filament/render_target.h
        filament/renderable.cc
        filament/renderable.h
        filament/scene_view.cc
        filament/scene_view.h
        filament/texture.cc
        filament/texture.h
        compat/imgui_bridge.cc
        compat/imgui_bridge.h
        compat/imgui_editor.cc
        compat/imgui_editor.h
        compat/mjr_filament_renderer.cc
        compat/mjr_filament_renderer.h
        compat/model_objects.cc
        compat/model_objects.h
        compat/scene_bridge.cc
        compat/scene_bridge.h
        compat/scene_geom_util.cc
        compat/scene_geom_util.h
)
if(MUJOCO_USE_FILAMENT_MJR_COMPAT)
    target_sources(${MUJOCO_FILAMENT_TARGET_NAME}
        PUBLIC
            mjr_compat.cc
    )
endif()

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
    pbr_packed.mat
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
    unlit_decor.mat
    unlit_depth.mat
    unlit_line.mat
    unlit_segmentation.mat
    unlit_ui.mat
)

foreach(MATERIAL_FILE ${MATERIAL_FILES})
    get_filename_component(MATERIAL_NAME ${MATERIAL_FILE} NAME_WE)
    set(INPUT_FILE "${ASSETS_DIR}/${MATERIAL_FILE}")
    set(OUTPUT_FILE "${OUTPUT_ASSETS_DIR}/${MATERIAL_NAME}.filamat")

    if(CMAKE_SYSTEM_NAME STREQUAL "Emscripten")
        set(PRECOMPILED_FILE "${MUJOCO_NATIVE_BUILD_DIR}/src/experimental/filament/assets/${MATERIAL_NAME}.filamat")
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
