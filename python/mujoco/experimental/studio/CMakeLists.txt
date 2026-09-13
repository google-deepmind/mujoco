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

mujoco_pybind11_module(native_viewer_cc native_viewer.cc)
target_link_libraries(native_viewer_cc PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(renderer renderer.cc)
target_link_libraries(renderer PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(ux ux.cc)
target_link_libraries(ux PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(sim sim.cc)
target_link_libraries(sim PRIVATE ${STUDIO_COMMON_LIBS})

mujoco_pybind11_module(window window.cc)
target_link_libraries(window PRIVATE ${STUDIO_COMMON_LIBS})

add_subdirectory(web)

set(MUJOCO_STUDIO_TARGETS
    native_viewer_cc
    renderer
    ux
    sim
    window
    ${MUJOCO_WEB_VIEWER_TARGETS}
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
      native_viewer_cc
      renderer
      ux
      sim
      window
      headless_ui
  )
  foreach(target ${STUDIO_PY_MODULES})
    if(TARGET ${target})
      target_link_options(${target} PRIVATE "-Wl,--allow-multiple-definition")
    endif()
  endforeach()
endif()

if(APPLE)
  # A module records where to look for the libraries it needs as a list of
  # rpaths baked into the binary at link time, where @loader_path means "the
  # directory this module is in". mujoco_pybind11_module (../../CMakeLists.txt)
  # gives every module it creates exactly one: @loader_path. That is correct for
  # the modules at the mujoco/ package root, which sit next to libmujoco, and
  # wrong for these, which are installed below it -- in
  # mujoco/experimental/studio/, mujoco/experimental/dear_imgui/ and so on --
  # while libmujoco stays at the root. So append the hop back up to the root.
  #
  # Append to BUILD_RPATH rather than calling install_name_tool from an
  # add_custom_command(TARGET ...) POST_BUILD hook the way
  # mujoco_pybind11_module does: that form only accepts targets created in the
  # same directory, which excludes the web/ modules below.
  foreach(target native_viewer_cc renderer ux sim window dear_imgui implot)
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "@loader_path/../..")
  endforeach()

  # One @loader_path level deeper: these are installed in
  # mujoco/experimental/studio/web/. web/CMakeLists.txt sets this list only on
  # the branch that builds the two modules natively, and leaves it unset under
  # Emscripten and when implot or netimgui are missing, so the foreach doubles
  # as the check for whether they exist.
  foreach(target ${MUJOCO_WEB_VIEWER_TARGETS})
    set_property(TARGET ${target} APPEND PROPERTY BUILD_RPATH "@loader_path/../../..")
  endforeach()
endif()

