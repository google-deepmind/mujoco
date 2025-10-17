set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/dist")

project(mujoco_wasm_benchmark)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3")

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

include_directories(${PROJECT_SOURCE_DIR}/../../include)
include_directories(${PROJECT_SOURCE_DIR}/../../src)
link_directories(${PROJECT_SOURCE_DIR}/../../build/lib)

file(GLOB MUJOCO_WASM_FILES
    "benchmark_test.cc"
    "../unpack.cc"
)

if(NOT MUJOCO_WASM_FILES)
    message(FATAL_ERROR "No source files found")
endif()

add_compile_options(-pthread)

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
    "-gsource-map"
    "-g"
    "--emit-tsd mujoco_wasm_benchmark.d.ts"
)
string (REPLACE ";" " " EMCC_LINKER_FLAGS_STR "${EMCC_LINKER_FLAGS}")

add_executable(mujoco_wasm_benchmark ${MUJOCO_WASM_FILES})

set_target_properties(mujoco_wasm_benchmark PROPERTIES LINK_FLAGS "${EMCC_LINKER_FLAGS_STR}")

target_link_libraries(mujoco_wasm_benchmark ccd lodepng mujoco tinyxml2 qhullstatic_r)

install(TARGETS mujoco_wasm_benchmark DESTINATION ${DIVISIBLE_INSTALL_BIN_DIR})
