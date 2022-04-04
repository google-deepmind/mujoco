# Copyright 2022 DeepMind Technologies Limited
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

project(mujoco_python)
cmake_minimum_required(VERSION 3.15)

# Support new IN_LIST if() operator.
set(CMAKE_POLICY_DEFAULT_CMP0057 NEW)
# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

enable_language(C)
enable_language(CXX)

if(MSVC AND MSVC_VERSION GREATER_EQUAL 1927)
  set(CMAKE_CXX_STANDARD 20) # For forceinline lambdas.
else()
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

# TODO(stunya) Figure out why this is required.
separate_arguments(CMDLINE_LINK_OPTIONS UNIX_COMMAND ${CMAKE_SHARED_LINKER_FLAGS})
add_link_options(${CMDLINE_LINK_OPTIONS})

include(MujocoLinkOptions)
get_mujoco_extra_link_options(EXTRA_LINK_OPTIONS)
add_link_options(${EXTRA_LINK_OPTIONS})

if(APPLE)
  add_compile_options(-Werror=partial-availability -Werror=unguarded-availability)
  add_link_options(-Wl,-no_weak_imports)
endif()

find_package(Python3 COMPONENTS Interpreter Development)

include(FindOrFetch)

# ==================== MUJOCO LIBRARY ==========================================
if(NOT TARGET mujoco)
  find_library(MUJOCO_LIBRARY mujoco mujoco.2.1.4 HINTS ${MUJOCO_LIBRARY_DIR} REQUIRED)
  find_path(MUJOCO_INCLUDE mujoco.h HINTS ${MUJOCO_INCLUDE_DIR} REQUIRED)
  message("MuJoCo is at ${MUJOCO_LIBRARY}")
  message("MuJoCo headers are at ${MUJOCO_INCLUDE}")
  add_library(mujoco SHARED IMPORTED)
  if(WIN32)
    set_target_properties(mujoco PROPERTIES IMPORTED_IMPLIB "${MUJOCO_LIBRARY}")
  else()
    set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}")
  endif()
  target_include_directories(mujoco INTERFACE "${MUJOCO_INCLUDE}")
  if(APPLE)
    execute_process(
      COMMAND otool -XD ${MUJOCO_LIBRARY}
      COMMAND head -n 1
      COMMAND xargs dirname
      COMMAND xargs echo -n
      OUTPUT_VARIABLE MUJOCO_INSTALL_NAME_DIR
    )
    set_target_properties(mujoco PROPERTIES INSTALL_NAME_DIR "${MUJOCO_INSTALL_NAME_DIR}")
  elseif(UNIX)
    execute_process(
      COMMAND objdump -p ${MUJOCO_LIBRARY}
      COMMAND grep SONAME
      COMMAND grep -Po [^\\s]+$
      COMMAND xargs echo -n
      OUTPUT_VARIABLE MUJOCO_SONAME
    )
    set_target_properties(mujoco PROPERTIES IMPORTED_SONAME "${MUJOCO_SONAME}")
  endif()
endif()

# ==================== ABSEIL ==================================================
if(APPLE)
  set(ABSL_EXTRA_FETCH_ARGS
      PATCH_COMMAND
      "sed"
      "-i"
      " "
      "s/-march=armv8-a+crypto/-mcpu=apple-m1+crypto/g"
      "${CMAKE_BINARY_DIR}/_deps/abseil-cpp-src/absl/copts/GENERATED_AbseilCopts.cmake"
  )
else()
  set(ABSL_EXTRA_FETCH_ARGS "")
endif()
findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  absl
  LIBRARY_NAME
  abseil-cpp
  GIT_REPO
  https://github.com/abseil/abseil-cpp
  GIT_TAG
  215105818dfde3174fe799600bb0f3cae233d0bf # 20211102.0
  TARGETS
  absl::core_headers
  absl::flat_hash_map
  absl::span
  ${ABSL_EXTRA_FETCH_ARGS}
  EXCLUDE_FROM_ALL
)

# ==================== EIGEN ===================================================
add_compile_definitions(EIGEN_MPL2_ONLY)
findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  Eigen3
  LIBRARY_NAME
  eigen
  GIT_REPO
  https://gitlab.com/libeigen/eigen
  GIT_TAG
  3147391d946bb4b6c68edd901f2add6ac1f31f8c # 3.4.0
  TARGETS
  Eigen3::Eigen
  EXCLUDE_FROM_ALL
)

# ==================== PYBIND11 ================================================
if(MUJOCO_PYBIND11_DIR)
  FetchContent_Declare(pybind11 SOURCE_DIR ${MUJOCO_PYBIND11_DIR} EXCLUDE_FROM_ALL)
  FetchContent_MakeAvailable(pybind11)
else()
  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    pybind11
    LIBRARY_NAME
    pybind11
    GIT_REPO
    https://github.com/pybind/pybind11
    GIT_TAG
    a8f1a5567608f346bdba293b3d062a288ee16cd4
    TARGETS
    pybind11::pybind11_headers
    EXCLUDE_FROM_ALL
  )
endif()

# ==================== MUJOCO PYTHON BINDINGS ==================================

add_subdirectory(util)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/enum_traits.h)
  add_library(enum_traits INTERFACE)
  target_sources(enum_traits INTERFACE enum_traits.h)
else()
  add_custom_command(
    OUTPUT enum_traits.h
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${mujoco_SOURCE_DIR}/mujoco ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_enum_traits.py > enum_traits.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_enum_traits.py
  )
  add_library(enum_traits INTERFACE)
  target_sources(enum_traits INTERFACE enum_traits.h)
  target_include_directories(
    enum_traits INTERFACE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
target_link_libraries(enum_traits INTERFACE mujoco absl::core_headers)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/function_traits.h)
  add_library(function_traits INTERFACE)
  target_sources(function_traits INTERFACE function_traits.h)
else()
  add_custom_command(
    OUTPUT function_traits.h
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${mujoco_SOURCE_DIR}/mujoco ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_function_traits.py > function_traits.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_function_traits.py
  )
  add_library(function_traits INTERFACE)
  target_sources(function_traits INTERFACE function_traits.h)
  target_include_directories(
    function_traits INTERFACE ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()
target_link_libraries(function_traits INTERFACE mujoco absl::core_headers)

add_library(errors_header INTERFACE)
target_sources(errors_header INTERFACE errors.h)
set_target_properties(errors_header PROPERTIES PUBLIC_HEADER errors.h)
target_link_libraries(errors_header INTERFACE crossplatform func_wrap mujoco)

add_library(raw INTERFACE)
target_sources(raw INTERFACE raw.h)
set_target_properties(raw PROPERTIES PUBLIC_HEADER raw.h)
target_link_libraries(raw INTERFACE mujoco)

add_library(structs_header INTERFACE)
target_sources(
  structs_header
  INTERFACE indexer_xmacro.h
            indexers.h
            mjdata_meta.h
            structs.h
)
set_target_properties(structs_header PROPERTIES PUBLIC_HEADER structs.h)
target_link_libraries(
  structs_header
  INTERFACE absl::flat_hash_map
            absl::span
            mujoco
            raw
)

add_library(functions_header INTERFACE)
target_sources(functions_header INTERFACE functions.h)
set_target_properties(functions_header PROPERTIES PUBLIC_HEADER functions.h)
target_link_libraries(
  functions_header
  INTERFACE array_traits
            crossplatform
            Eigen3::Eigen
            errors_header
            func_wrap
            structs_header
            tuple_tools
)

include(CheckAvxSupport)
get_avx_compile_options(AVX_COMPILE_OPTIONS)

macro(mujoco_pybind11_module name)
  pybind11_add_module(${name} ${ARGN})
  target_compile_options(${name} PRIVATE ${AVX_COMPILE_OPTIONS})
  set_target_properties(${name} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  if(APPLE)
    add_custom_command(
      TARGET ${name}
      POST_BUILD
      COMMAND
        install_name_tool -change
        $<TARGET_PROPERTY:mujoco,INSTALL_NAME_DIR>/$<TARGET_FILE_NAME:mujoco>
        @rpath/$<TARGET_FILE_NAME:mujoco> -add_rpath @loader_path $<TARGET_FILE:${name}>
    )
  elseif(NOT WIN32)
    set_target_properties(${name} PROPERTIES INSTALL_RPATH $ORIGIN BUILD_WITH_INSTALL_RPATH TRUE)
  endif()
endmacro()

mujoco_pybind11_module(_callbacks callbacks.cc)
target_link_libraries(
  _callbacks
  PRIVATE errors_header
          mujoco
          raw
          structs_header
)

mujoco_pybind11_module(_constants constants.cc)
target_link_libraries(_constants PRIVATE mujoco)

mujoco_pybind11_module(_enums enums.cc)
target_link_libraries(
  _enums
  PRIVATE crossplatform
          enum_traits
          mujoco
          tuple_tools
)

mujoco_pybind11_module(_errors errors.cc)
target_link_libraries(_errors PRIVATE errors_header)

mujoco_pybind11_module(_functions functions.cc)
target_link_libraries(
  _functions
  PRIVATE Eigen3::Eigen
          functions_header
          function_traits
          mujoco
          raw
)
if(APPLE)
  # C++17 aligned allocation is not available until macOS 10.14.
  target_compile_options(_functions PRIVATE -fno-aligned-allocation)
endif()

mujoco_pybind11_module(_render render.cc)
target_link_libraries(
  _render
  PRIVATE Eigen3::Eigen
          errors_header
          functions_header
          function_traits
          mujoco
          raw
          structs_header
)

mujoco_pybind11_module(_rollout rollout.cc)
target_link_libraries(_rollout PRIVATE functions_header mujoco raw)

mujoco_pybind11_module(
  _structs
  indexers.cc
  serialization.h
  structs.cc
)
target_link_libraries(
  _structs
  PRIVATE absl::flat_hash_map
          mujoco
          raw
          errors_header
          func_wrap
          function_traits
          structs_header
)

set(LIBRARIES_FOR_WHEEL
    "$<TARGET_FILE:_callbacks>"
    "$<TARGET_FILE:_constants>"
    "$<TARGET_FILE:_enums>"
    "$<TARGET_FILE:_errors>"
    "$<TARGET_FILE:_functions>"
    "$<TARGET_FILE:_render>"
    "$<TARGET_FILE:_rollout>"
    "$<TARGET_FILE:_structs>"
    "$<TARGET_FILE:mujoco>"
)

if(MUJOCO_PYTHON_MAKE_WHEEL)
  add_custom_target(
    wheel ALL
    COMMAND "${CMAKE_COMMAND}" -E rm -rf "${CMAKE_CURRENT_BINARY_DIR}/dist"
    COMMAND "${CMAKE_COMMAND}" -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/.."
            "${CMAKE_CURRENT_BINARY_DIR}/dist"
    COMMAND "${CMAKE_COMMAND}" -E copy "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
            "${CMAKE_CURRENT_BINARY_DIR}/dist/LICENSE"
    COMMAND "${CMAKE_COMMAND}" -E copy ${LIBRARIES_FOR_WHEEL}
            "${CMAKE_CURRENT_BINARY_DIR}/dist/mujoco"
    COMMAND "${Python3_EXECUTABLE}" -m pip wheel --wheel-dir "${CMAKE_BINARY_DIR}" --no-deps -vvv
            "${CMAKE_CURRENT_BINARY_DIR}/dist"
  )

  add_dependencies(
    wheel
    _callbacks
    _constants
    _enums
    _errors
    _functions
    _render
    _rollout
    _structs
    mujoco
  )
endif()
