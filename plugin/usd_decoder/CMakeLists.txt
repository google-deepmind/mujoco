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
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_LIBDIR}")
set(USD_DECODER_SRCS
    usd_decoder.cc
    kinematic_tree.cc
    kinematic_tree.h
    material_parsing.cc
    material_parsing.h
)

add_library(usd_decoder_plugin SHARED ${USD_DECODER_SRCS})
target_include_directories(usd_decoder_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_link_libraries(usd_decoder_plugin PRIVATE
  mujoco
  mjcPhysics
)
target_compile_options(usd_decoder_plugin PRIVATE
  ${AVX_COMPILE_OPTIONS}
  ${MUJOCO_MACOS_COMPILE_OPTIONS}
  ${EXTRA_COMPILE_OPTIONS}
  ${MUJOCO_CXX_FLAGS}
  -Wno-deprecated # pxr Tf lib uses deprecated header
)
target_link_options(usd_decoder_plugin PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}
  ${EXTRA_LINK_OPTIONS}
)

# Install to mujoco_plugin directory in bin location so that it is picked up by simulate
# on startup.
install(
  TARGETS usd_decoder_plugin
  LIBRARY DESTINATION "${CMAKE_INSTALL_BINDIR}/mujoco_plugin"
)

target_link_libraries(usd_decoder_plugin PRIVATE usd usdGeom usdPhysics usdShade gf tf ar vt kind)
