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

# Global compilation settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # For LLVM tooling

if(NOT CMAKE_CONFIGURATION_TYPES)
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE
        "Release"
        CACHE STRING "Choose the type of build, recommanded options are: Debug or Release" FORCE
    )
  endif()
  set(BUILD_TYPES
      "Debug"
      "Release"
      "MinSizeRel"
      "RelWithDebInfo"
  )
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${BUILD_TYPES})
endif()

include(GNUInstallDirs)

# Change the default output directory in the build structure. This is not stricly needed, but helps
# running in Windows, such that all built executables have DLLs in the same folder as the .exe
# files.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

set(OpenGL_GL_PREFERENCE GLVND)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_VISIBILITY_PRESET hidden)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

if(MSVC)
  add_compile_options(/Gy /Gw /Oi)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-fdata-sections -ffunction-sections)
endif()

# We default to shared library.
set(BUILD_SHARED_LIBS
    ON
    CACHE BOOL "Build Mujoco as shared library."
)

option(MUJOCO_ENABLE_AVX "Build binaries that require AVX instructions, if possible." ON)
option(MUJOCO_ENABLE_AVX_INTRINSICS "Make use of hand-written AVX intrinsics, if possible." ON)
option(MUJOCO_ENABLE_RPATH "Enable RPath support when installing Mujoco." ON)
mark_as_advanced(MUJOCO_ENABLE_RPATH)

if(MUJOCO_ENABLE_AVX)
  include(CheckAvxSupport)
  get_avx_compile_options(AVX_COMPILE_OPTIONS)
else()
  set(AVX_COMPILE_OPTIONS)
endif()

option(MUJOCO_BUILD_MACOS_FRAMEWORKS "Build libraries as macOS Frameworks" OFF)
option(MUJOCO_INSTALL_PLUGINS "Install MuJoCo plugins" ON)
mark_as_advanced(MUJOCO_INSTALL_PLUGINS)

# Define function to simplify installations of plugins
function(mujoco_install_plugin plugin_target_name)
  # Do not install plugin on macOS when using macOS Frameworks
  if(MUJOCO_INSTALL_PLUGINS AND NOT MUJOCO_BUILD_MACOS_FRAMEWORKS)
    if(WIN32)
      install(TARGETS ${plugin_target_name}
              # consistently with the official mujoco binary format, we do
              # not want to install .lib import libraries on Windows, and
              # only install .dll . To do so, we install non-.dll files to
              # a fake install directory that is not actually in the install path
              ARCHIVE DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/fake_plugin_install"
              LIBRARY DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/fake_plugin_install"
              RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin")
    else()
      install(TARGETS ${plugin_target_name}
              ARCHIVE DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin"
              LIBRARY DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin"
              RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin")
    endif()
  endif()
endfunction()

# Get some extra link options.
include(MujocoLinkOptions)
get_mujoco_extra_link_options(EXTRA_LINK_OPTIONS)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT MSVC))
  set(EXTRA_COMPILE_OPTIONS
      -Werror
      -Wall
      -Wpedantic
      -Wimplicit-fallthrough
      -Wunused
      -Wvla
      -Wno-int-in-bool-context
      -Wno-sign-compare
      -Wno-unknown-pragmas
  )
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    # Set -Wimplicit-fallthrough=5 to only allow fallthrough annotation via __attribute__.
    set(EXTRA_COMPILE_OPTIONS ${EXTRA_COMPILE_OPTIONS} -Wimplicit-fallthrough=5
                              -Wno-maybe-uninitialized
    )
  endif()
endif()

include(MujocoHarden)
set(EXTRA_COMPILE_OPTIONS ${EXTRA_COMPILE_OPTIONS} ${MUJOCO_HARDEN_COMPILE_OPTIONS})
set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} ${MUJOCO_HARDEN_LINK_OPTIONS})

if(WIN32)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE)
endif()
