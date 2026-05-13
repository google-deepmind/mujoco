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

link_directories(${CMAKE_BINARY_DIR}/lib)

# Set Emscripten compile flags.
# -fexceptions is required for val::throw_() (used by ThrowMujocoErrorToJS) to
# actually throw a JS exception. Without it, Emscripten compiles throw as a
# no-op. In the MT build, -pthread implicitly enables exception support, but
# the ST build needs it set explicitly here.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fexceptions")

# Set Emscripten linker flags shared by all WASM targets.
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
    "-fexceptions"
    "-gsource-map"
    "-g"
)
if(MUJOCO_WASM_THREADS)
    list(APPEND EMCC_LINKER_FLAGS
        "-pthread"
        "-s PTHREAD_POOL_SIZE=navigator.hardwareConcurrency"
    )
    add_definitions(-DMUJOCO_WASM_THREADS)
endif()

# Common link libraries for all WASM targets. The mujoco library is linked as a
# whole archive to avoid losing plugin registration such as obj_decoder and
# stl_decoder.
set(MUJOCO_WASM_LINK_LIBRARIES
    -Wl,--whole-archive mujoco -Wl,--no-whole-archive
    ccd lodepng tinyxml2 qhullstatic_r
)

# --- Main WASM bindings target ---

file(GLOB MUJOCO_WASM_FILES
    "codegen/generated/*.cc"
    "unpack.cc"
)

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found in codegen/generated/")
endif()

set(MUJOCO_WASM_LINKER_FLAGS ${EMCC_LINKER_FLAGS} "--emit-tsd mujoco.d.ts")
string(REPLACE ";" " " MUJOCO_WASM_LINKER_FLAGS_STR "${MUJOCO_WASM_LINKER_FLAGS}")

add_executable(mujoco_wasm ${MUJOCO_WASM_FILES})

# Keep the internal target name distinct to avoid colliding with the native
# `mujoco` library target, but emit artifacts named `mujoco.*` by setting the
# output name. Also apply the emscripten linker flags to the wasm target.
set_target_properties(mujoco_wasm PROPERTIES
    LINK_FLAGS "${MUJOCO_WASM_LINKER_FLAGS_STR}"
    OUTPUT_NAME "mujoco"
)

target_link_libraries(mujoco_wasm PRIVATE ${MUJOCO_WASM_LINK_LIBRARIES})

install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})

# --- Benchmark target ---

if(MUJOCO_BUILD_TESTS_WASM)
    file(GLOB MUJOCO_WASM_BENCHMARK_FILES
        "tests/benchmark_test.cc"
        "unpack.cc"
    )

    if(NOT MUJOCO_WASM_BENCHMARK_FILES)
        message(FATAL_ERROR "No benchmark source files found")
    endif()

    set(BENCHMARK_LINKER_FLAGS ${EMCC_LINKER_FLAGS} "--emit-tsd mujoco_wasm_benchmark.d.ts")
    string(REPLACE ";" " " BENCHMARK_LINKER_FLAGS_STR "${BENCHMARK_LINKER_FLAGS}")

    add_executable(mujoco_wasm_benchmark ${MUJOCO_WASM_BENCHMARK_FILES})

    set_target_properties(mujoco_wasm_benchmark PROPERTIES
        LINK_FLAGS "${BENCHMARK_LINKER_FLAGS_STR}"
    )

    target_link_libraries(mujoco_wasm_benchmark PRIVATE ${MUJOCO_WASM_LINK_LIBRARIES})

    install(TARGETS mujoco_wasm_benchmark DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
endif()
