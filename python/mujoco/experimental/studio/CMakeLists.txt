# Copyright 2026 DeepMind Technologies Limited
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

if(NOT TARGET mujoco::platform)
  message(WARNING "mujoco::platform target not found. experimental/studio bindings will not be built.")
  return()
endif()

mujoco_pybind11_module(parser parser.cc)
target_link_libraries(parser PRIVATE mujoco::platform structs_header structs_wrappers)

mujoco_pybind11_module(native_viewer_cc native_viewer.cc)
target_link_libraries(native_viewer_cc PRIVATE
  mujoco::platform
  structs_header
)

mujoco_pybind11_module(renderer renderer.cc)
target_link_libraries(renderer PRIVATE mujoco::platform structs_header)

mujoco_pybind11_module(ux ux.cc)
target_link_libraries(ux PRIVATE
  mujoco::platform
  structs_header
)

mujoco_pybind11_module(sim sim.cc)
target_link_libraries(sim PRIVATE mujoco::platform structs_header)

set(MUJOCO_STUDIO_TARGETS
    parser
    native_viewer_cc
    renderer
    ux
    sim
    PARENT_SCOPE
)

# dear_imgui bindings
mujoco_pybind11_module(dear_imgui ../dear_imgui/dear_imgui.cc)
target_link_libraries(dear_imgui PRIVATE
  mujoco::dep::dear_imgui
  mujoco::dep::dear_imgui_stdlib
)
target_include_directories(dear_imgui PRIVATE ../dear_imgui)
set_target_properties(dear_imgui PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../dear_imgui"
)

# implot bindings
mujoco_pybind11_module(implot ../implot/implot.cc)
target_link_libraries(implot PRIVATE
  mujoco::dep::implot
  mujoco::dep::dear_imgui
)
target_include_directories(implot PRIVATE ../dear_imgui)
set_target_properties(implot PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/../implot"
)
