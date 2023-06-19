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
    9a89766acc42ddfa9e7133c7d81a5bda108a0ade
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
    0c8fc90d2037588024d9964515c1e684f6007ecc
    CACHE STRING "Version of `qhull` to be fetched."
)
set(MUJOCO_DEP_VERSION_Eigen3
    b378014fef017a829fb42c7fad15f3764bfb8ef9
    CACHE STRING "Version of `Eigen3` to be fetched."
)

set(MUJOCO_DEP_VERSION_abseil
    b971ac5250ea8de900eae9f95e06548d14cd95fe # LTS 20230125.2
    CACHE STRING "Version of `abseil` to be fetched."
)

set(MUJOCO_DEP_VERSION_gtest
    b796f7d44681514f58a683a3a71ff17c94edb0c1 # v1.13.0
    CACHE STRING "Version of `gtest` to be fetched."
)

set(MUJOCO_DEP_VERSION_benchmark
    d572f4777349d43653b21d6c2fc63020ab326db2 # v1.7.1
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

option(MUJOCO_USE_SYSTEM_lodepng "Use installed lodepng version." OFF)
mark_as_advanced(MUJOCO_USE_SYSTEM_lodepng)

if(NOT TARGET lodepng)
  if(NOT MUJOCO_USE_SYSTEM_lodepng)
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
  else()
    find_package(lodepng REQUIRED)
  endif()
endif()


option(MUJOCO_USE_SYSTEM_qhull "Use installed qhull version." OFF)
mark_as_advanced(MUJOCO_USE_SYSTEM_qhull)

set(QHULL_ENABLE_TESTING OFF)

findorfetch(
  USE_SYSTEM_PACKAGE
  ${MUJOCO_USE_SYSTEM_qhull}
  PACKAGE_NAME
  Qhull
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

if(NOT MUJOCO_USE_SYSTEM_qhull)
  # MuJoCo includes a file from libqhull_r which is not exported by the qhull include directories.
  # Add it to the target.
  target_include_directories(
    qhullstatic_r INTERFACE $<BUILD_INTERFACE:${qhull_SOURCE_DIR}/src/libqhull_r>
  )
  target_compile_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
  target_link_options(qhullstatic_r PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
else()
  if(NOT TARGET qhullstatic_r)
    add_library(qhullstatic_r INTERFACE)
    set_target_properties(qhullstatic_r PROPERTIES INTERFACE_LINK_LIBRARIES Qhull::qhull_r)

    # Workaround as headers are installed in <prefix>/include/libqhull_r/something.h
    # but mujoco include them as #include <something.h>
    get_property(qhull_include_dirs TARGET Qhull::qhull_r PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    foreach(qhull_include_dir IN LISTS qhull_include_dirs)
      target_include_directories(qhullstatic_r INTERFACE ${qhull_include_dirs}/libqhull_r)
    endforeach()
    target_include_directories(qhullstatic_r INTERFACE )
  endif()
endif()

option(MUJOCO_USE_SYSTEM_tinyxml2 "Use installed tinyxml2 version." OFF)
mark_as_advanced(MUJOCO_USE_SYSTEM_tinyxml2)

set(tinyxml2_BUILD_TESTING OFF)
findorfetch(
  USE_SYSTEM_PACKAGE
  ${MUJOCO_USE_SYSTEM_tinyxml2}
  PACKAGE_NAME
  tinyxml2
  LIBRARY_NAME
  tinyxml2
  GIT_REPO
  https://github.com/leethomason/tinyxml2.git
  GIT_TAG
  ${MUJOCO_DEP_VERSION_tinyxml2}
  TARGETS
  tinyxml2::tinyxml2
  EXCLUDE_FROM_ALL
)

if(NOT MUJOCO_USE_SYSTEM_tinyxml2)
  target_compile_options(tinyxml2 PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
  target_link_options(tinyxml2 PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
endif()

option(MUJOCO_USE_SYSTEM_tinyobjloader "Use installed tinyobjloader version." OFF)
mark_as_advanced(MUJOCO_USE_SYSTEM_tinyobjloader)

findorfetch(
  USE_SYSTEM_PACKAGE
  ${MUJOCO_USE_SYSTEM_tinyobjloader}
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

if(MUJOCO_USE_SYSTEM_tinyobjloader)
  # As of tinyobjloader v2.0.0rc10, the tinyobjloader target is named tinyobjloader in the build,
  # but tinyobjloader::tinyobjloader when it is installed. To deal with this, if tinyobjloader is
  # found in the system, we create an ALIAS
  # The following is equivalent to add_library(tinyobjloader ALIAS tinyobjloader::tinyobjloader), 
  # but compatible with CMake 3.16 . Once the minimum CMake is bumped to CMake 3.18, we can use
  # the simpler version
  add_library(tinyobjloader INTERFACE IMPORTED)
  set_target_properties(tinyobjloader PROPERTIES INTERFACE_LINK_LIBRARIES tinyobjloader::tinyobjloader)
endif()

option(MUJOCO_USE_SYSTEM_ccd "Use installed ccd version." OFF)
mark_as_advanced(MUJOCO_USE_SYSTEM_ccd)

set(ENABLE_DOUBLE_PRECISION ON)
set(CCD_HIDE_ALL_SYMBOLS ON)
findorfetch(
  USE_SYSTEM_PACKAGE
  ${MUJOCO_USE_SYSTEM_ccd}
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

if(NOT MUJOCO_USE_SYSTEM_ccd)
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
endif()

if(MUJOCO_BUILD_TESTS)
  option(MUJOCO_USE_SYSTEM_abseil "Use installed abseil version." OFF)
  mark_as_advanced(MUJOCO_USE_SYSTEM_abseil)

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
    ${MUJOCO_USE_SYSTEM_abseil}
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

  option(MUJOCO_USE_SYSTEM_gtest "Use installed gtest version." OFF)
  mark_as_advanced(MUJOCO_USE_SYSTEM_gtest)

  # Avoid linking errors on Windows by dynamically linking to the C runtime.
  set(gtest_force_shared_crt
      ON
      CACHE BOOL "" FORCE
  )

  findorfetch(
    USE_SYSTEM_PACKAGE
    ${MUJOCO_USE_SYSTEM_gtest}
    PACKAGE_NAME
    GTest
    LIBRARY_NAME
    googletest
    GIT_REPO
    https://github.com/google/googletest.git
    GIT_TAG
    ${MUJOCO_DEP_VERSION_gtest}
    TARGETS
    GTest::gmock
    GTest::gtest_main
    EXCLUDE_FROM_ALL
  )

  option(MUJOCO_USE_SYSTEM_benchmark "Use installed benchmark version." OFF)
  mark_as_advanced(MUJOCO_USE_SYSTEM_benchmark)

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
    ${MUJOCO_USE_SYSTEM_benchmark}
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
  option(MUJOCO_USE_SYSTEM_Eigen3 "Use installed Eigen3 version." OFF)
  mark_as_advanced(MUJOCO_USE_SYSTEM_Eigen3)

  add_compile_definitions(EIGEN_MPL2_ONLY)
  if(NOT TARGET Eigen3::Eigen)
    if(NOT MUJOCO_USE_SYSTEM_Eigen3)
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
    else()
      find_package(Eigen3 REQUIRED)
    endif()
  endif()
endif()

# Reset BUILD_SHARED_LIBS to its previous value
set(BUILD_SHARED_LIBS
    ${BUILD_SHARED_LIBS_OLD}
    CACHE BOOL "Build MuJoCo as a shared library" FORCE
)
