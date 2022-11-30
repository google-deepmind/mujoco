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

#include (GenerateExportHeader)          

set(MUJOCO_ELASTICITY_INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src
)

set(MUJOCO_ELASTICITY_SRCS
    cable.cc
    cable.h
)

if (WIN32)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/plugin/ )   #change output directory of plugin to be found by simulate.exe
    #set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib/ )          #change output directory of static plugin to be at the right place at debug time
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)                                 #export also .lib along the .dll for static linking
    set(BUILD_SHARED_LIBS TRUE)
    
    #add_library(elasticity SHARED ${MUJOCO_ELASTICITY_SRCS})           
    
    #target_sources(mujoco PRIVATE ${MUJOCO_ELASTICITY_SRCS})
    
    #add_link_options("LINKER:-v")

endif()

add_library(elasticity SHARED)
target_sources(elasticity PRIVATE ${MUJOCO_ELASTICITY_SRCS})
target_include_directories(elasticity PRIVATE ${MUJOCO_ELASTICITY_INCLUDE})
target_link_libraries(elasticity PRIVATE mujoco)


target_compile_options(
  elasticity
  PRIVATE ${AVX_COMPILE_OPTIONS}
          ${MUJOCO_MACOS_COMPILE_OPTIONS}
          ${EXTRA_COMPILE_OPTIONS}
          ${MUJOCO_CXX_FLAGS}
)

target_link_options(
  elasticity
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)


