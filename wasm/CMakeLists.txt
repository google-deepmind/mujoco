# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/wasm/dist")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/wasm)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${CMAKE_BINARY_DIR}/wasm/filament/assets)

link_directories(${CMAKE_BINARY_DIR}/lib)

option(MUJOCO_BUILD_WASM_FILAMENT "Build MuJoCo WASM with Filament support" ON)

file(GLOB MUJOCO_WASM_FILES
    "codegen/generated/*.cc"
    "unpack.cc"
)

if(MUJOCO_BUILD_WASM_FILAMENT)
    # Add dependencies for Filament rendering
    list(APPEND MUJOCO_WASM_FILES
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/hal/renderer.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/hal/window.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/hal/graphics_mode.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/sim/step_control.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/sim/model_holder.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/sim/sim_history.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/sim/sim_profiler.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/platform/helpers.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/render_context_filament.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/builtins.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/color_grading_options.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/filament_context.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/filament_platform_factory.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/imgui_bridge.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/imgui_editor.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/light.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/material.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/math_util.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/mesh.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/model_objects.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/object_manager.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/render_target.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/renderable.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/scene_bridge.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/scene_geom_util.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/scene_view.cc"
        "${PROJECT_SOURCE_DIR}/src/experimental/filament/filament/texture.cc"
    )

    # Filament host tools for asset compilation
    set(FILAMENT_HOST_TOOLS_PATH "" CACHE PATH "Path to Filament host tools (containing matc)")

    # Asset generation logic
    set(ASSETS_DIR "${PROJECT_SOURCE_DIR}/src/experimental/filament/assets")
    set(WASM_FILAMENT_ASSETS_DIR "${CMAKE_BINARY_DIR}/wasm/filament/assets")
    file(MAKE_DIRECTORY ${WASM_FILAMENT_ASSETS_DIR})

    # Compile materials using matc
    set(MATC_EXECUTABLE matc)
    set(MATERIAL_FILES
        pbr.mat pbr_packed.mat phong_2d_fade.mat phong_2d.mat
        phong_2d_reflect.mat phong_2d_uv_fade.mat phong_2d_uv.mat
        phong_2d_uv_reflect.mat phong_color_fade.mat phong_color.mat
        phong_color_reflect.mat phong_cube_fade.mat phong_cube.mat
        phong_cube_reflect.mat unlit_decor.mat unlit_depth.mat
        unlit_line.mat unlit_segmentation.mat unlit_ui.mat
    )

    foreach(MATERIAL_FILE ${MATERIAL_FILES})
        get_filename_component(MATERIAL_NAME ${MATERIAL_FILE} NAME_WE)
        set(INPUT_FILE "${ASSETS_DIR}/${MATERIAL_FILE}")
        set(OUTPUT_FILE "${WASM_FILAMENT_ASSETS_DIR}/${MATERIAL_NAME}.filamat")

        add_custom_command(
            OUTPUT ${OUTPUT_FILE}
            COMMAND ${MATC_EXECUTABLE}
                --platform=all --api=opengl --variant-filter skinning --optimize-size
                --output ${OUTPUT_FILE} ${INPUT_FILE}
            DEPENDS ${INPUT_FILE}
            COMMENT "Compiling ${MATERIAL_FILE} to .filamat"
        )
        list(APPEND FILAMENT_COMPILED_ASSETS ${OUTPUT_FILE})
    endforeach()

    # Asset embedding logic
    set(EMBED_HEADER "${WASM_FILAMENT_ASSETS_DIR}/filament_assets_embed.h")
    set(EMBED_CC "${WASM_FILAMENT_ASSETS_DIR}/filament_assets_embed.cc")
    add_custom_command(
        OUTPUT ${EMBED_HEADER} ${EMBED_CC}
        COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/codegen/embed_assets.py"
            ${EMBED_HEADER} ${EMBED_CC} ${FILAMENT_COMPILED_ASSETS} "${ASSETS_DIR}/ibl.ktx"
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/codegen/embed_assets.py" ${FILAMENT_COMPILED_ASSETS} "${ASSETS_DIR}/ibl.ktx"
        COMMENT "Embedding Filament assets into C++"
    )
    list(APPEND MUJOCO_WASM_FILES ${EMBED_CC})
    add_definitions(-DMUJOCO_WASM_FILAMENT)
endif()

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found in codegen/generated/")
endif()

# Set Emscripten linker flags
set(EMCC_LINKER_FLAGS
    "--bind"
    "-s ASSERTIONS=1"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s EXPORT_ES6=1"
    "-s MODULARIZE=1"
    "-s FORCE_FILESYSTEM=1"
    "-s EXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','MEMFS']"
    "-s EXPORT_NAME=loadMujoco"
    "-s DISABLE_EXCEPTION_CATCHING=0"
    "-gsource-map"
    "-g"
    "--emit-tsd mujoco.d.ts"
)

if(MUJOCO_BUILD_WASM_FILAMENT)
    list(APPEND EMCC_LINKER_FLAGS
        "-s USE_WEBGL2=1"
        "-s FULL_ES3=1"
    )
endif()

if(MUJOCO_WASM_THREADS)
    list(APPEND EMCC_LINKER_FLAGS
        "-pthread"
        "-s PTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
    )
    add_definitions(-DMUJOCO_WASM_THREADS)
endif()
string (REPLACE ";" " " EMCC_LINKER_FLAGS_STR "${EMCC_LINKER_FLAGS}")

add_executable(mujoco_wasm ${MUJOCO_WASM_FILES})

# Keep the internal target name distinct to avoid colliding with the native
# `mujoco` library target, but emit artifacts named `mujoco.*` by setting the
# output name. Also apply the emscripten linker flags to the wasm target.
set_target_properties(mujoco_wasm PROPERTIES
    LINK_FLAGS "${EMCC_LINKER_FLAGS_STR}"
    OUTPUT_NAME "mujoco"
)

# Link the mujoco library as a whole archive to avoid losing plugin
# registration such as obj_decoder and stl_decoder.
target_link_libraries(mujoco_wasm PRIVATE
    -Wl,--whole-archive mujoco -Wl,--no-whole-archive
    ccd lodepng tinyxml2 qhullstatic_r
)

if(MUJOCO_BUILD_WASM_FILAMENT)
    include(third_party_deps/dear_imgui)
    include(third_party_deps/implot)
    include(third_party_deps/libwebp)
    include(third_party_deps/filament)
    target_link_libraries(mujoco_wasm PRIVATE
        filament image ktxreader
        dear_imgui
        dear_imgui_SDL2
        dear_imgui_OpenGL3
        dear_imgui_stdlib
        implot
        webp
    )
endif()

install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
