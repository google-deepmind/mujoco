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

cmake_minimum_required(VERSION 3.16)

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# Default to GLVND if available.
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

# This line has to appear before 'PROJECT' in order to be able to disable incremental linking
set(MSVC_INCREMENTAL_DEFAULT ON)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

project(
  mujoco_simulate
  VERSION 3.3.4
  DESCRIPTION "MuJoCo simulate binaries"
  HOMEPAGE_URL "https://mujoco.org"
)

enable_language(C)
enable_language(CXX)
if(APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()

option(SIMULATE_BUILD_EXECUTABLE "Build the simulate executable binary." ON)
option(SIMULATE_GLFW_DYNAMIC_SYMBOLS "Whether to resolve GLFW symbols dynamically." OFF)

# Check if we are building as standalone project.
set(SIMULATE_STANDALONE OFF)
set(_INSTALL_SIMULATE ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SIMULATE_STANDALONE ON)
  # If standalone, do not install the samples.
  set(_INSTALL_SIMULATE OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../cmake")

if(SIMULATE_STANDALONE)
  include(SimulateOptions)
else()
  enforce_mujoco_macosx_min_version()
endif()
include(SimulateDependencies)

set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SIMULATE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SIMULATE_COMPILE_OPTIONS "${MUJOCO_SIMULATE_COMPILE_OPTIONS}" -fPIE -fPIC)
    if(APPLE)
      set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -Wl,-pie)
    else()
      set(MUJOCO_SIMULATE_LINK_OPTIONS "${MUJOCO_SIMULATE_LINK_OPTIONS}" -pie)
    endif()
  endif()
endif()

# Fetch lodepng dependency.
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

# Simulate library
add_library(platform_ui_adapter OBJECT)
target_sources(
  platform_ui_adapter
  PUBLIC glfw_adapter.h glfw_dispatch.h platform_ui_adapter.h
  PRIVATE glfw_adapter.cc glfw_dispatch.cc platform_ui_adapter.cc
)
target_compile_options(platform_ui_adapter PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
if(APPLE)
  target_sources(platform_ui_adapter PUBLIC glfw_corevideo.h PRIVATE glfw_corevideo.mm)
  set_source_files_properties(glfw_corevideo.mm PROPERTIES COMPILE_FLAGS -Wno-deprecated-declarations)
  target_link_libraries(platform_ui_adapter PUBLIC "-framework CoreVideo")
endif()
target_include_directories(
  platform_ui_adapter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
                             $<TARGET_PROPERTY:glfw,INTERFACE_INCLUDE_DIRECTORIES>
)
target_link_libraries(platform_ui_adapter PUBLIC mujoco::mujoco)
if(SIMULATE_GLFW_DYNAMIC_SYMBOLS)
  target_compile_definitions(platform_ui_adapter PUBLIC mjGLFW_DYNAMIC_SYMBOLS)
endif()
add_library(mujoco::platform_ui_adapter ALIAS platform_ui_adapter)

add_library(libsimulate STATIC $<TARGET_OBJECTS:platform_ui_adapter>)
set_target_properties(libsimulate PROPERTIES OUTPUT_NAME simulate)
add_library(mujoco::libsimulate ALIAS libsimulate)

target_sources(
  libsimulate
  PUBLIC simulate.h
  PRIVATE simulate.cc array_safety.h
)
target_include_directories(libsimulate PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_link_libraries(libsimulate PUBLIC lodepng mujoco::platform_ui_adapter mujoco::mujoco)
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

if(APPLE)
  target_sources(libsimulate PRIVATE macos_gui.mm)
  target_link_libraries(libsimulate PUBLIC "-framework Cocoa")
endif()

# Build simulate executable
if(SIMULATE_BUILD_EXECUTABLE)
  if(APPLE)
    set(SIMULATE_RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../dist/mujoco.icns)
  elseif(WIN32)
    set(SIMULATE_RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../dist/simulate.rc)
  else()
    set(SIMULATE_RESOURCE_FILES "")
  endif()

  add_executable(simulate main.cc array_safety.h ${SIMULATE_RESOURCE_FILES})
  target_compile_options(simulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
  if(WIN32)
    add_custom_command(
      TARGET simulate
      PRE_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/../dist/mujoco.ico
              ${CMAKE_CURRENT_SOURCE_DIR}
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E rm ${CMAKE_CURRENT_SOURCE_DIR}/mujoco.ico
    )
  endif()

  target_link_libraries(
    simulate
    libsimulate
    mujoco::mujoco
    glfw
    Threads::Threads
    lodepng
  )

  target_link_options(simulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})

  if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
    set_target_properties(
      simulate
      PROPERTIES INSTALL_RPATH @executable_path/../Frameworks
                 BUILD_WITH_INSTALL_RPATH TRUE
                 RESOURCE ${SIMULATE_RESOURCE_FILES}
                 MACOSX_BUNDLE TRUE
                 MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/../dist/Info.plist.simulate.in
                 MACOSX_BUNDLE_BUNDLE_NAME "MuJoCo"
                 MACOSX_BUNDLE_GUI_IDENTIFIER "org.mujoco.mujoco"
                 MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
                 MACOSX_BUNDLE_INFO_STRING ${PROJECT_VERSION}
                 MACOSX_BUNDLE_LONG_VERSION_STRING ${PROJECT_VERSION}
                 MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
                 MACOSX_BUNDLE_ICON_FILE "mujoco.icns"
                 MACOSX_BUNDLE_COPYRIGHT "Copyright 2021 DeepMind Technologies Limited."
    )

    macro(embed_in_bundle target)
      add_dependencies(${target} simulate)
      set_target_properties(
        ${target}
        PROPERTIES INSTALL_RPATH @executable_path/../Frameworks
                   BUILD_WITH_INSTALL_RPATH TRUE
                   RUNTIME_OUTPUT_DIRECTORY $<TARGET_FILE_DIR:simulate>
      )
    endmacro()

    # Embed mujoco.framework inside the App bundle ane move the icon file over too.
    add_custom_command(
      TARGET simulate
      POST_BUILD
      COMMAND mkdir -p $<TARGET_FILE_DIR:simulate>/../Frameworks
      COMMAND rm -rf $<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework
      COMMAND cp -a $<TARGET_FILE_DIR:mujoco::mujoco>/../../../mujoco.framework
              $<TARGET_FILE_DIR:simulate>/../Frameworks/
      # Delete the symlink and the TBD, otherwise we can't sign and notarize.
      COMMAND rm -rf $<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework/mujoco.tbd
      COMMAND rm -rf
              $<TARGET_FILE_DIR:simulate>/../Frameworks/mujoco.framework/Versions/A/libmujoco.dylib
    )
  endif()

  # Do not install if macOS Bundles are created as RPATH is managed manually there.
  if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
    set(_INSTALL_SIMULATE OFF)
  endif()

  if(_INSTALL_SIMULATE)

    include(TargetAddRpath)

    # Add support to RPATH for the samples.
    target_add_rpath(
      TARGETS
      simulate
      INSTALL_DIRECTORY
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
      LIB_DIRS
      "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
      DEPENDS
      MUJOCO_ENABLE_RPATH
    )

    install(
      TARGETS simulate
      EXPORT ${PROJECT_NAME}
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
      BUNDLE DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
      PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
    )

    if(NOT MUJOCO_SIMULATE_USE_SYSTEM_GLFW)
      # We downloaded GLFW. Depending if it is a static or shared LIBRARY we might
      # need to install it.
      get_target_property(MJ_GLFW_LIBRARY_TYPE glfw TYPE)
      if(MJ_GLFW_LIBRARY_TYPE STREQUAL SHARED_LIBRARY)
        install(
          TARGETS glfw
          EXPORT ${PROJECT_NAME}
          RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT simulate
          LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
          ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT simulate
          PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT simulate
        )
      endif()
    endif()
  endif()
endif()
