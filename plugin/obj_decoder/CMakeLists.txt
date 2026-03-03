# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(EMSCRIPTEN)
  add_library(obj_decoder OBJECT obj_decoder.cc)
else()
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")

  add_library(obj_decoder SHARED obj_decoder.cc)
endif()

target_include_directories(obj_decoder PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)

if(EMSCRIPTEN)
  target_link_libraries(obj_decoder PRIVATE
    tinyobjloader
  )
else()
  target_link_libraries(obj_decoder PRIVATE
    mujoco
    tinyobjloader
  )
endif()

target_compile_definitions(obj_decoder PRIVATE TINYOBJLOADER_IMPLEMENTATION)

target_compile_options(obj_decoder PRIVATE
  ${AVX_COMPILE_OPTIONS}
  ${MUJOCO_MACOS_COMPILE_OPTIONS}
  ${EXTRA_COMPILE_OPTIONS}
  ${MUJOCO_CXX_FLAGS}
)

if(NOT EMSCRIPTEN)
  target_link_options(obj_decoder PRIVATE
    ${MUJOCO_MACOS_LINK_OPTIONS}
    ${EXTRA_LINK_OPTIONS}
  )

  install(
    TARGETS obj_decoder
    LIBRARY DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin"
  )
endif()
