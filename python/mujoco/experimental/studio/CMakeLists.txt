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

set(STUDIO_COMMON_LIBS
  mujoco::platform
  structs_header
  structs_wrappers
)

mujoco_pybind11_module(parser parser.cc)
target_link_libraries(parser PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(native_viewer_cc native_viewer.cc)
target_link_libraries(native_viewer_cc PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(renderer renderer.cc)
target_link_libraries(renderer PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(ux ux.cc)
target_link_libraries(ux PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(sim sim.cc)
target_link_libraries(sim PRIVATE ${STUDIO_COMMON_LIBS})

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

# Workaround for duplicate symbol error (wp_fractional_scale_manager_v1_interface)
# when linking both GLFW (via Filament) and SDL2 statically on Linux.
if(UNIX AND NOT APPLE)
  set(STUDIO_PY_MODULES
      parser
      native_viewer_cc
      renderer
      ux
      sim
  )
  foreach(target ${STUDIO_PY_MODULES})
    target_link_options(${target} PRIVATE "-Wl,--allow-multiple-definition")
  endforeach()
endif()

if(APPLE)
  set(STUDIO_PY_MODULES
      parser
      native_viewer_cc
      renderer
      ux
      sim
      dear_imgui
      implot
  )
  foreach(target ${STUDIO_PY_MODULES})
    if(TARGET ${target})
      add_custom_command(
        TARGET ${target}
        POST_BUILD
        COMMAND
          install_name_tool -add_rpath @loader_path/../../ $<TARGET_FILE:${target}>
      )
    endif()
  endforeach()
endif()

