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

# Override the BUILD_SHARED_LIBS setting, just for building third party libs (since we always want
# static libraries). The ccd CMakeLists.txt doesn't expose an option to build a static ccd library,
# unless BUILD_SHARED_LIBS is set.

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)
set(MUJOCO_DEP_VERSION_tinyxml2
    1dee28e51f9175a31955b9791c74c430fe13dc82 # 9.0.0
    CACHE STRING "Version of `tinyxml2` to be fetched."
)
set(MUJOCO_DEP_VERSION_tinyobjloader
    1421a10d6ed9742f5b2c1766d22faa6cfbc56248
    CACHE STRING "Version of `tinyobjloader` to be fetched."
)
set(MUJOCO_DEP_VERSION_ccd
    7931e764a19ef6b21b443376c699bbc9c6d4fba8 # v2.1
    CACHE STRING "Version of `ccd` to be fetched."
)
set(MUJOCO_DEP_VERSION_qhull
    3df027b91202cf179f3fba3c46eebe65bbac3790
    CACHE STRING "Version of `qhull` to be fetched."
)
set(MUJOCO_DEP_VERSION_Eigen3
    34780d8bd13d0af0cf17a22789ef286e8512594d
    CACHE STRING "Version of `Eigen3` to be fetched."
)

set(MUJOCO_DEP_VERSION_abseil
    8c0b94e793a66495e0b1f34a5eb26bd7dc672db0 # LTS 20220623.1
    CACHE STRING "Version of `abseil` to be fetched."
)

set(MUJOCO_DEP_VERSION_gtest
    58d77fa8070e8cec2dc1ed015d66b454c8d78850 # release-1.12.0
    CACHE STRING "Version of `gtest` to be fetched."
)

set(MUJOCO_DEP_VERSION_benchmark
    d845b7b3a27d54ad96280a29d61fa8988d4fddcf # v1.6.2
    CACHE STRING "Version of `benchmark` to be fetched."
)

mark_as_advanced(MUJOCO_DEP_VERSION_lodepng)
mark_as_advanced(MUJOCO_DEP_VERSION_tinyxml2)
mark_as_advanced(MUJOCO_DEP_VERSION_tinyobjloader)
mark_as_advanced(MUJOCO_DEP_VERSION_ccd)
mark_as_advanced(MUJOCO_DEP_VERSION_qhull)
mark_as_advanced(MUJOCO_DEP_VERSION_Eigen3)
mark_as_advanced(MUJOCO_DEP_VERSION_abseil)
mark_as_advanced(MUJOCO_DEP_VERSION_gtest)
mark_as_advanced(MUJOCO_DEP_VERSION_benchmark)

include(FetchContent)
include(FindOrFetch)

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

# TODO(fraromano) We fetch qhull before the other libraries as it needs to go before until https://github.com/qhull/qhull/pull/111 is merged.
set(QHULL_ENABLE_TESTING OFF)
# We need Git to apply the patch using git apply.
find_package(Git REQUIRED)

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
  # TODO(fraromano) Remove when https://github.com/qhull/qhull/pull/112 is merged.
  # Do not fail if patch fails. This will happen the second time we run CMake as the sources will be already patched.
  PATCH_COMMAND
  "${GIT_EXECUTABLE}"
  "apply"
  "-q"
  "${PROJECT_SOURCE_DIR}/cmake/qhull_fix_testing.patch"
  "||"
  "${CMAKE_COMMAND}"
  "-E"
  "true"
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
