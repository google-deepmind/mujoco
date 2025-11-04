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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -fexceptions")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/wasm)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR})

link_directories(${CMAKE_BINARY_DIR}/lib)

file(GLOB MUJOCO_WASM_FILES
    "codegen/generated/*.cc"
    "unpack.cc"
)

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found in codegen/generated/")
endif()

add_compile_options(-pthread)
add_compile_options(-fexceptions)

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
    "--emit-tsd mujoco_wasm.d.ts"
)
string (REPLACE ";" " " EMCC_LINKER_FLAGS_STR "${EMCC_LINKER_FLAGS}")

add_executable(mujoco_wasm ${MUJOCO_WASM_FILES})

set_target_properties(mujoco_wasm PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS_STR}")

target_link_libraries(mujoco_wasm ccd lodepng mujoco tinyxml2 qhullstatic_r)

install(TARGETS mujoco_wasm DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
