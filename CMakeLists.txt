cmake_minimum_required(VERSION 3.15)
# ~~~
# These sources helped a lot:
# * CMake imported targets: https://tinyurl.com/rrt9v5u8
# * Fix for RPATH issue (build): https://tinyurl.com/wfnffk8e
# ~~~

project(
  MuJoCo
  VERSION 0.1.0
  DESCRIPTION
    "Multi-Joint dynamics with Contact. A general purpose physics simulator"
  HOMEPAGE_URL "https://github.com/deepmind/mujoco"
  LANGUAGES C CXX)

# @todo(wilbert): do we need these for installation?
# ~~~
# set(CMAKE_BUILD_RPATH_USE_ORIGIN TRUE)
# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
# set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# ~~~

include(FetchContent)

# Get MuJoCo library from project's page
set(mujoco_url "https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz")
set(mujoco_hash
    a436ca2f4144c38b837205635bbd60ffe1162d5b44c87df22232795978d7d012)
# cmake-format: off
FetchContent_Declare(
  mujocolib
  URL ${mujoco_url}
  URL_HASH SHA256=${mujoco_hash}
  USES_TERMINAL_DOWNLOAD TRUE
  PREFIX "${CMAKE_SOURCE_DIR}/third_party/mujoco"
  DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/third_party/mujoco"
  SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/mujoco/source"
  BINARY_DIR "${CMAKE_BINARY_DIR}/third_party/mujoco/build"
  STAMP_DIR "${CMAKE_BINARY_DIR}/third_party/mujoco/stamp"
  TMP_DIR "${CMAKE_BINARY_DIR}/third_party/mujoco/tmp")
# cmake-format: on
FetchContent_MakeAvailable(mujocolib)

# Expose a target that we can use ##############################################
set(CMAKE_USE_PTHREADS_INIT TRUE)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# Target for the MuJoCo physics library itself
add_library(mujoco::mujoco SHARED IMPORTED)
set_property(
  TARGET mujoco::mujoco
  PROPERTY IMPORTED_LOCATION
           "${CMAKE_SOURCE_DIR}/third_party/mujoco/source/bin/libmujoco210.so")
set_property(TARGET mujoco::mujoco PROPERTY IMPORTED_NO_SONAME TRUE)
target_include_directories(
  mujoco::mujoco
  INTERFACE "${CMAKE_SOURCE_DIR}/third_party/mujoco/source/include")
target_compile_features(mujoco::mujoco INTERFACE cxx_std_11)
target_compile_options(mujoco::mujoco INTERFACE -mavx)
target_link_libraries(mujoco::mujoco INTERFACE Threads::Threads)

# Target for the provided GLFW library
add_library(mujoco::glfw SHARED IMPORTED)
set_property(
  TARGET mujoco::glfw
  PROPERTY IMPORTED_LOCATION
           "${CMAKE_SOURCE_DIR}/third_party/mujoco/source/bin/libglfw.so.3")
set_property(TARGET mujoco::glfw PROPERTY IMPORTED_NO_SONAME TRUE)
target_include_directories(
  mujoco::glfw
  INTERFACE "${CMAKE_SOURCE_DIR}/third_party/mujoco/source/include")

# Target for the provided GLEW library
add_library(mujoco::glew SHARED IMPORTED)
set_property(
  TARGET mujoco::glew
  PROPERTY IMPORTED_LOCATION
           "${CMAKE_SOURCE_DIR}/third_party/mujoco/source/bin/libglew.so")
set_property(TARGET mujoco::glew PROPERTY IMPORTED_NO_SONAME TRUE)
target_link_libraries(mujoco::glew INTERFACE OpenGL::OpenGL OpenGL::GLX)
# ##############################################################################

# Build the simulation example
add_executable(
  AppSimulate
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/source/include/uitools.c
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mujoco/source/sample/simulate.cc)
target_link_libraries(AppSimulate PRIVATE mujoco::glew mujoco::glfw
                                          mujoco::mujoco)
