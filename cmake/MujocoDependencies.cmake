# Copyright 2021 DeepMind Technologies Limited
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

# Build configuration for third party libraries used in MuJoCo.

set(MUJOCO_DEP_VERSION_lodepng
    17d08dd26cac4d63f43af217ebd70318bfb8189c
    CACHE STRING "Version of `lodepng` to be fetched."
)
set(MUJOCO_DEP_VERSION_tinyxml2
    9a89766acc42ddfa9e7133c7d81a5bda108a0ade
    CACHE STRING "Version of `tinyxml2` to be fetched."
)
set(MUJOCO_DEP_VERSION_tinyobjloader
    1421a10d6ed9742f5b2c1766d22faa6cfbc56248
    CACHE STRING "Version of `tinyobjloader` to be fetched."
)
set(MUJOCO_DEP_VERSION_MarchingCubeCpp
    f03a1b3ec29b1d7d865691ca8aea4f1eb2c2873d
    CACHE STRING "Version of `MarchingCubeCpp` to be fetched."
)
set(MUJOCO_DEP_VERSION_ccd
    7931e764a19ef6b21b443376c699bbc9c6d4fba8 # v2.1
    CACHE STRING "Version of `ccd` to be fetched."
)
set(MUJOCO_DEP_VERSION_qhull
    c7bee59d068a69f427b1273e71cdc5bc455a5bdd
    CACHE STRING "Version of `qhull` to be fetched."
)
set(MUJOCO_DEP_VERSION_Eigen3
    d0b490ee091629068e0c11953419eb089f9e6bb2
    CACHE STRING "Version of `Eigen3` to be fetched."
)

set(MUJOCO_DEP_VERSION_abseil
    bc257a88f7c1939f24e0379f14a3589e926c950c # LTS 20250512.0
    CACHE STRING "Version of `abseil` to be fetched."
)

set(MUJOCO_DEP_VERSION_gtest
    52eb8108c5bdec04579160ae17225d66034bd723 # v1.17.0
    CACHE STRING "Version of `gtest` to be fetched."
)

set(MUJOCO_DEP_VERSION_benchmark
    049f6e79cc3e8636cec21bbd94ed185b4a5f2653
    CACHE STRING "Version of `benchmark` to be fetched."
)

set(MUJOCO_DEP_VERSION_sdflib
    1927bee6bb8225258a39c8cbf14e18a4d50409ae
    CACHE STRING "Version of `SdfLib` to be fetched."
)

mark_as_advanced(MUJOCO_DEP_VERSION_lodepng)
mark_as_advanced(MUJOCO_DEP_VERSION_MarchingCubeCpp)
mark_as_advanced(MUJOCO_DEP_VERSION_tinyxml2)
mark_as_advanced(MUJOCO_DEP_VERSION_tinyobjloader)
mark_as_advanced(MUJOCO_DEP_VERSION_ccd)
mark_as_advanced(MUJOCO_DEP_VERSION_qhull)
mark_as_advanced(MUJOCO_DEP_VERSION_Eigen3)
mark_as_advanced(MUJOCO_DEP_VERSION_abseil)
mark_as_advanced(MUJOCO_DEP_VERSION_gtest)
mark_as_advanced(MUJOCO_DEP_VERSION_benchmark)
mark_as_advanced(MUJOCO_DEP_VERSION_sdflib)

include(FetchContent)
include(FindOrFetch)

# Override the BUILD_SHARED_LIBS setting, just for building third party libs (since we always want
# static libraries). The ccd CMakeLists.txt doesn't expose an option to build a static ccd library,
# unless BUILD_SHARED_LIBS is set.

# We force all the dependencies to be compiled as static libraries.
# TODO(fraromano) Revisit this choice when adding support for install.
set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS
    OFF
    CACHE INTERNAL "Build SHARED libraries"
)

if(NOT TARGET lodepng)
  FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )

  FetchContent_GetProperties(lodepng)
  if(NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif()
endif()

if(NOT TARGET marchingcubecpp)
  FetchContent_Declare(
    marchingcubecpp
    GIT_REPOSITORY https://github.com/aparis69/MarchingCubeCpp.git
    GIT_TAG ${MUJOCO_DEP_VERSION_MarchingCubeCpp}
  )

  FetchContent_GetProperties(marchingcubecpp)
  if(NOT marchingcubecpp_POPULATED)
    FetchContent_Populate(marchingcubecpp)
    include_directories(${marchingcubecpp_SOURCE_DIR})
  endif()
endif()

set(QHULL_ENABLE_TESTING OFF)

findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  qhull
  LIBRARY_NAME
  qhull
  GIT_REPO
  https://github.com/qhull/qhull.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_qhull}
  TARGETS
  qhull
  EXCLUDE_FROM_ALL
)
# MuJoCo includes a file from libqhull_r which is not exported by the qhull include directories.
# Add it to the target.
target_include_directories(
  qhullstatic_r INTERFACE $<BUILD_INTERFACE:${qhull_SOURCE_DIR}/src/libqhull_r>
)
target_compile_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
target_link_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})

set(tinyxml2_BUILD_TESTING OFF)
findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  tinyxml2
  LIBRARY_NAME
  tinyxml2
  GIT_REPO
  https://github.com/leethomason/tinyxml2.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_tinyxml2}
  TARGETS
  tinyxml2
  EXCLUDE_FROM_ALL
)
target_compile_options(tinyxml2 PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
target_link_options(tinyxml2 PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})

findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  tinyobjloader
  LIBRARY_NAME
  tinyobjloader
  GIT_REPO
  https://github.com/tinyobjloader/tinyobjloader.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_tinyobjloader}
  TARGETS
  tinyobjloader
  EXCLUDE_FROM_ALL
)

option(SDFLIB_USE_ASSIMP OFF)
option(SDFLIB_USE_OPENMP OFF)
option(SDFLIB_USE_ENOKI OFF)
findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  sdflib
  LIBRARY_NAME
  sdflib
  GIT_REPO
  https://github.com/UPC-ViRVIG/SdfLib.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_sdflib}
  TARGETS
  SdfLib
  EXCLUDE_FROM_ALL
)
target_compile_options(SdfLib PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
target_link_options(SdfLib PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})

set(ENABLE_DOUBLE_PRECISION ON)
set(CCD_HIDE_ALL_SYMBOLS ON)
findorfetch(
  USE_SYSTEM_PACKAGE
  OFF
  PACKAGE_NAME
  ccd
  LIBRARY_NAME
  ccd
  GIT_REPO
  https://github.com/danfis/libccd.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_ccd}
  TARGETS
  ccd
  EXCLUDE_FROM_ALL
)
target_compile_options(ccd PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
target_link_options(ccd PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})

# libCCD has an unconditional `#define _CRT_SECURE_NO_WARNINGS` on Windows.
# TODO(stunya): Remove this after https://github.com/danfis/libccd/pull/77 is merged.
if(WIN32)
  if(MSVC)
    # C4005 is the MSVC equivalent of -Wmacro-redefined.
    target_compile_options(ccd PRIVATE /wd4005)
  else()
    target_compile_options(ccd PRIVATE -Wno-macro-redefined)
  endif()
endif()

if(MUJOCO_BUILD_TESTS)
  set(ABSL_PROPAGATE_CXX_STD ON)

  # This specific version of Abseil does not have the following variable. We need to work with BUILD_TESTING
  set(BUILD_TESTING_OLD ${BUILD_TESTING})
  set(BUILD_TESTING
      OFF
      CACHE INTERNAL "Build tests."
  )

  set(ABSL_BUILD_TESTING OFF)
  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    absl
    LIBRARY_NAME
    abseil-cpp
    GIT_REPO
    https://github.com/abseil/abseil-cpp.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_abseil}
    TARGETS
    absl::core_headers
    EXCLUDE_FROM_ALL
  )

  set(BUILD_TESTING
      ${BUILD_TESTING_OLD}
      CACHE BOOL "Build tests." FORCE
  )

  # Avoid linking errors on Windows by dynamically linking to the C runtime.
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE
  )

  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    GTest
    LIBRARY_NAME
    googletest
    GIT_REPO
    https://github.com/google/googletest.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_gtest}
    TARGETS
    gtest
    gmock
    gtest_main
    EXCLUDE_FROM_ALL
  )

  set(BENCHMARK_EXTRA_FETCH_ARGS "")
  if(WIN32 AND NOT MSVC)
    set(BENCHMARK_EXTRA_FETCH_ARGS
        PATCH_COMMAND
        "sed"
        "-i"
        "-e"
        "s/-std=c++11/-std=c++14/g"
        "-e"
        "s/HAVE_CXX_FLAG_STD_CXX11/HAVE_CXX_FLAG_STD_CXX14/g"
        "${CMAKE_BINARY_DIR}/_deps/benchmark-src/CMakeLists.txt"
    )
  endif()

  set(BENCHMARK_ENABLE_TESTING OFF)

  findorfetch(
    USE_SYSTEM_PACKAGE
    OFF
    PACKAGE_NAME
    benchmark
    LIBRARY_NAME
    benchmark
    GIT_REPO
    https://github.com/google/benchmark.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_benchmark}
    TARGETS
    benchmark::benchmark
    benchmark::benchmark_main
    ${BENCHMARK_EXTRA_FETCH_ARGS}
    EXCLUDE_FROM_ALL
  )
endif()

if(MUJOCO_TEST_PYTHON_UTIL)
  add_compile_definitions(EIGEN_MPL2_ONLY)
  if(NOT TARGET eigen)
    # Support new IN_LIST if() operator.
    set(CMAKE_POLICY_DEFAULT_CMP0057 NEW)

    FetchContent_Declare(
      Eigen3
      GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
      GIT_TAG ${MUJOCO_DEP_VERSION_Eigen3}
    )

    FetchContent_GetProperties(Eigen3)
    if(NOT Eigen3_POPULATED)
      FetchContent_Populate(Eigen3)

      # Mark the library as IMPORTED as a workaround for https://gitlab.kitware.com/cmake/cmake/-/issues/15415
      add_library(Eigen3::Eigen INTERFACE IMPORTED)
      set_target_properties(
        Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
      )
    endif()
  endif()
endif()

# Reset BUILD_SHARED_LIBS to its previous value
set(BUILD_SHARED_LIBS
    ${BUILD_SHARED_LIBS_OLD}
    CACHE BOOL "Build MuJoCo as a shared library" FORCE
)
