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
if(APPLE)
  enable_language(OBJCXX)
endif()

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

if(MSVC)
  add_compile_options(/Gy /Gw /Oi)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fdata-sections -ffunction-sections)
endif()

include(MujocoLinkOptions)
get_mujoco_extra_link_options(EXTRA_LINK_OPTIONS)
add_link_options(${EXTRA_LINK_OPTIONS})

include(MujocoMacOS)
enforce_mujoco_macosx_min_version()

if(WIN32)
  add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

include(MujocoHarden)
add_compile_options("${MUJOCO_HARDEN_COMPILE_OPTIONS}")
add_link_options("${MUJOCO_HARDEN_LINK_OPTIONS}")

find_package(Python3 COMPONENTS Interpreter Development)

include(FindOrFetch)

# ==================== MUJOCO LIBRARY ==========================================
if(NOT TARGET mujoco)
  add_library(
    mujoco
    SHARED
    IMPORTED
    GLOBAL
  )
  if(APPLE)
    # On macOS, check if we are using mujoco.framework first.
    # Framework headers are searched differently from normal headers.
    # We need to use -F instead of the usual target_include_directories.
    find_path(MUJOCO_FRAMEWORK mujoco.Framework HINTS ${MUJOCO_FRAMEWORK_DIR})
    if(MUJOCO_FRAMEWORK)
      message("MuJoCo framework is at ${MUJOCO_FRAMEWORK}/mujoco.framework")
      set(MUJOCO_LIBRARY
          ${MUJOCO_FRAMEWORK}/mujoco.framework/Versions/A/libmujoco.3.3.4.dylib
      )
      target_compile_options(mujoco INTERFACE -F${MUJOCO_FRAMEWORK})
    endif()
  endif()

  if(NOT MUJOCO_FRAMEWORK)
    find_library(
      MUJOCO_LIBRARY mujoco mujoco.3.3.4 HINTS ${MUJOCO_LIBRARY_DIR} REQUIRED
    )
    find_path(MUJOCO_INCLUDE mujoco/mujoco.h HINTS ${MUJOCO_INCLUDE_DIR} REQUIRED)
    message("MuJoCo is at ${MUJOCO_LIBRARY}")
    message("MuJoCo headers are at ${MUJOCO_INCLUDE}")
    target_include_directories(mujoco INTERFACE "${MUJOCO_INCLUDE}")
  endif()

  if(WIN32)
    set_target_properties(mujoco PROPERTIES IMPORTED_IMPLIB "${MUJOCO_LIBRARY}")
  else()
    set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}")
  endif()

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
  add_library(mujoco::mujoco ALIAS mujoco)
endif()

# ==================== ABSEIL ==================================================
set(MUJOCO_PYTHON_ABSL_TARGETS absl::core_headers absl::flat_hash_map absl::span)
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
  bc257a88f7c1939f24e0379f14a3589e926c950c # LTS 20250512.0
  TARGETS
  ${MUJOCO_PYTHON_ABSL_TARGETS}
  EXCLUDE_FROM_ALL
)
foreach(absl_target IN ITEMS ${MUJOCO_PYTHON_ABSL_TARGETS})
  get_target_property(absl_target_aliased ${absl_target} ALIASED_TARGET)
  if(absl_target_aliased)
    set(absl_target ${absl_target_aliased})
  endif()
  get_target_property(absl_target_type ${absl_target} TYPE)
  if(NOT
     ${absl_target_type}
     STREQUAL
     "INTERFACE_LIBRARY"
  )
    target_compile_options(${absl_target} PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(${absl_target} PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
  endif()
endforeach()

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
  d0b490ee091629068e0c11953419eb089f9e6bb2
  TARGETS
  Eigen3::Eigen
  EXCLUDE_FROM_ALL
)

# ==================== PYBIND11 ================================================
option(MUJOCO_PYTHON_USE_SYSTEM_PYBIND11 "Use installed pybind11 version." OFF)
findorfetch(
  USE_SYSTEM_PACKAGE
  MUJOCO_PYTHON_USE_SYSTEM_PYBIND11
  PACKAGE_NAME
  pybind11
  LIBRARY_NAME
  pybind11
  GIT_REPO
  https://github.com/pybind/pybind11
  GIT_TAG
  a2e59f0e7065404b44dfe92a28aca47ba1378dc4 # v2.13.6
  TARGETS
  pybind11::pybind11_headers
  EXCLUDE_FROM_ALL
)

# ==================== MUJOCO PYTHON BINDINGS ==================================
set(SIMULATE_BUILD_EXECUTABLE OFF)
set(SIMULATE_GLFW_DYNAMIC_SYMBOLS ON)
add_subdirectory(simulate)

add_subdirectory(util)

add_library(mujoco_private_api_header INTERFACE)
target_sources(mujoco_private_api_header INTERFACE private.h)
set_target_properties(mujoco_private_api_header PROPERTIES PUBLIC_HEADER private.h)
target_link_libraries(mujoco_private_api_header INTERFACE mujoco)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/enum_traits.h)
  add_library(enum_traits INTERFACE)
  target_sources(enum_traits INTERFACE enum_traits.h)
else()
  add_custom_command(
    OUTPUT enum_traits.h
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${mujoco_SOURCE_DIR}/mujoco/python/mujoco ${Python3_EXECUTABLE}
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
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${mujoco_SOURCE_DIR}/mujoco/python/mujoco ${Python3_EXECUTABLE}
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
target_link_libraries(
  errors_header
  INTERFACE crossplatform
            func_wrap
            mujoco
            mujoco_private_api_header
)

add_library(raw INTERFACE)
target_sources(raw INTERFACE raw.h)
set_target_properties(raw PROPERTIES PUBLIC_HEADER raw.h)
target_link_libraries(raw INTERFACE mujoco)

add_library(structs_header INTERFACE)
target_sources(
  structs_header
  INTERFACE indexer_xmacro.h
            indexers.h
            structs.h
)
set_target_properties(structs_header PROPERTIES PUBLIC_HEADER structs.h)
target_link_libraries(
  structs_header
  INTERFACE absl::flat_hash_map
            absl::span
            crossplatform
            mujoco
            raw
)

add_library(structs_wrappers STATIC
  structs_wrappers.cc
  serialization.h
  indexers.cc
)
target_include_directories(structs_wrappers PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(
  structs_wrappers
  PRIVATE absl::flat_hash_map
          absl::span
          crossplatform
          mujoco
          raw
          structs_header
          pybind11::headers
          Eigen3::Eigen
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
  if(NOT MSVC)
    target_compile_options(
      ${name}
      PRIVATE -Werror
              -Wall
              -Wimplicit-fallthrough
              -Wunused
              -Wno-int-in-bool-context
              -Wno-sign-compare
    )
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      # Only allow fallthrough annotation via __attribute__.
      target_compile_options(${name} PRIVATE -Wimplicit-fallthrough=5)

      # https://gcc.gnu.org/bugzilla/show_bug.cgi?id=109561
      if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 11)
        target_compile_options(${name} PRIVATE -Wno-maybe-uninitialized)
      endif()
    endif()
  endif()
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

mujoco_pybind11_module(_rollout rollout.cc threadpool.cc)
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
          crossplatform
          mujoco
          mujoco_private_api_header
          raw
          errors_header
          func_wrap
          function_traits
          structs_header
          structs_wrappers
)

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/specs.cc.inc)
  add_custom_command(
    OUTPUT specs.cc.inc
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${mujoco_SOURCE_DIR}/mujoco/python ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_spec_bindings.py > specs.cc.inc
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/codegen/generate_spec_bindings.py
  )
endif()

mujoco_pybind11_module(
  _specs
  specs.cc
  specs_wrapper.h
  specs_wrapper.cc
  specs.cc.inc
)
target_link_libraries(
  _specs
  PRIVATE mujoco
          Eigen3::Eigen
          errors_header
          raw
          structs_header
)

mujoco_pybind11_module(_simulate simulate.cc)
target_link_libraries(
  _simulate
  PRIVATE mujoco
          mujoco::libsimulate
          errors_header
          raw
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
    "$<TARGET_FILE:_simulate>"
    "$<TARGET_FILE:_specs>"
    "$<TARGET_FILE:_structs>"
    "$<TARGET_FILE:mujoco>"
)

if(APPLE)
  add_executable(mjpython mjpython/mjpython.mm)
  target_include_directories(mjpython PRIVATE "${Python3_INCLUDE_DIRS}")
  target_link_libraries(mjpython PRIVATE "-framework Cocoa")
  set(LIBRARIES_FOR_WHEEL "${LIBRARIES_FOR_WHEEL}" "$<TARGET_FILE:mjpython>")
endif()

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
    _simulate
    _specs
    _structs
    mujoco
  )
  if(APPLE)
    add_dependencies(wheel mjpython)
  endif()
endif()
