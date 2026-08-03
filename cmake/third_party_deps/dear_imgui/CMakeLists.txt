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

add_library(dear_imgui STATIC)
set(MUJOCO_DEAR_IMGUI_SOURCE_DIR "${dear_imgui_SOURCE_DIR}" CACHE INTERNAL "")
target_include_directories(dear_imgui
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}
)
target_sources(dear_imgui
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui.h
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui.cpp
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui_internal.h
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui_draw.cpp
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui_tables.cpp
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui_widgets.cpp
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/imgui_demo.cpp
)

function(mujoco_add_dear_imgui_sdl2_backend)
  if(TARGET dear_imgui_SDL2 OR NOT TARGET SDL2::SDL2-static)
    return()
  endif()

  add_library(dear_imgui_SDL2 STATIC)
  target_include_directories(dear_imgui_SDL2
    PUBLIC
      ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}
  )
  target_sources(dear_imgui_SDL2
    PUBLIC
      ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  )
  target_link_libraries(dear_imgui_SDL2
    SDL2::SDL2-static
  )
endfunction()

# SDL2 backend. Filament uses base dear_imgui without SDL2; platform calls this
# helper again after SDL2 is pulled in.
mujoco_add_dear_imgui_sdl2_backend()

# OpenGL3 backend
add_library(dear_imgui_OpenGL3 STATIC)
target_include_directories(dear_imgui_OpenGL3
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}
)
target_sources(dear_imgui_OpenGL3
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.h
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

# stdlib
add_library(dear_imgui_stdlib STATIC)
target_include_directories(dear_imgui_stdlib
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}
)
target_sources(dear_imgui_stdlib
  PUBLIC
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/misc/cpp/imgui_stdlib.h
    ${MUJOCO_DEAR_IMGUI_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
)
