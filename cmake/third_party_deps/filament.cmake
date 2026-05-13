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

set(MUJOCO_DEP_VERSION_filament
    06793c4a80dd467025b2db1b3b7ea63bf1a865bb
    CACHE STRING "Tag/version of `filament` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_filament)

include(FindOrFetch)

set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)

# Filament's ShaderMinifier.cpp uses strlen without including <cstring>, and
# PostProcessManager.h uses std::optional without including <optional>.
set(CMAKE_CXX_FLAGS_OLD "${CMAKE_CXX_FLAGS}")
if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FI cstring /FI optional")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -include cstring -include optional")
endif()

set(FILAMENT_ENABLE_EXPERIMENTAL_GCC_SUPPORT ON)
set(FILAMENT_SKIP_SDL2 ON)
set(FILAMENT_USE_EXTERNAL_ABSL ON)
set(FILAMENT_USE_EXTERNAL_BENCHMARK ON)
set(FILAMENT_USE_EXTERNAL_GTEST ON)
if(WIN32)
    set(USE_STATIC_CRT OFF)
endif()

fetchpackage(
    PACKAGE_NAME  filament
    GIT_REPO      https://github.com/google/filament.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_filament}
)

set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_OLD})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_OLD}")
