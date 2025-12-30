# Copyright 2025 DeepMind Technologies Limited
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
cmake_minimum_required(VERSION "3.16")

option(BUILD_USD OFF "Build OpenUSD")
project(openusd-dependency)

if (BUILD_USD)
  include(ExternalProject)

  set(DEPS_DIR ${CMAKE_BINARY_DIR}/../../../../build/_deps)

  ExternalProject_Add(openusd
    GIT_REPOSITORY https://github.com/PixarAnimationStudios/USD.git
    GIT_TAG v25.11
    PREFIX openusd
    BUILD_IN_SOURCE 0
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    INSTALL_COMMAND ""
    SOURCE_DIR ${DEPS_DIR}/openusd-src
    BINARY_DIR ${DEPS_DIR}/openusd-build
    INSTALL_DIR ${DEPS_DIR}/openusd-build
    BUILD_COMMAND python3 ${DEPS_DIR}/openusd-src/build_scripts/build_usd.py
        --build-shared
        --no-examples
        --no-tools
        --no-ptex
        --no-prman
        --no-openimageio
        --no-opencolorio
        --no-alembic
        --no-draco
        --no-materialx
        --no-tutorials
        --no-tests
        --no-docs
        --no-imaging
        --no-python
        --no-usdValidation
        <INSTALL_DIR>
  )
endif()
