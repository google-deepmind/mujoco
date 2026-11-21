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

set(MUJOCO_DEP_VERSION_sdl2
    e64034952b2a8f8356895b56aa06645382395a18
    CACHE STRING "Version of `SDL2` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_sdl2)

include(FindOrFetch)

set(SDL_SHARED_ENABLED_BY_DEFAULT OFF)
if(DEFINED SDL_AUDIO)
    set(_OLD_SDL_AUDIO "${SDL_AUDIO}")
    set(_SDL_AUDIO_WAS_DEFINED TRUE)
else()
    set(_SDL_AUDIO_WAS_DEFINED FALSE)
endif()
set(SDL_AUDIO OFF)
if(APPLE)
    set(_SDL2_CMAKE_C_FLAGS_OLD "${CMAKE_C_FLAGS}")
    set(_SDL2_CMAKE_OBJC_FLAGS_OLD "${CMAKE_OBJC_FLAGS}")
    if(DEFINED MUJOCO_MACOSX_VERSION_MIN)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=${MUJOCO_MACOSX_VERSION_MIN}")
        set(CMAKE_OBJC_FLAGS "${CMAKE_OBJC_FLAGS} -mmacosx-version-min=${MUJOCO_MACOSX_VERSION_MIN}")
    endif()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-deprecated-declarations")
    set(CMAKE_OBJC_FLAGS "${CMAKE_OBJC_FLAGS} -Wno-deprecated-declarations")
endif()
# update cmake_minimum_required version for compatibility with newer version of cmake
if(NOT DEFINED CMAKE_POLICY_VERSION_MINIMUM)
    set(CMAKE_POLICY_VERSION_MINIMUM ${MUJOCO_CMAKE_MIN_REQ})
    set(CMAKE_POLICY_VERSION_MINIMUM_LOCALLY_DEFINED ON)
endif()
fetchpackage(
    PACKAGE_NAME  sdl2
    GIT_REPO      https://github.com/libsdl-org/SDL.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_sdl2}
)
if(CMAKE_POLICY_VERSION_MINIMUM_LOCALLY_DEFINED)
    unset(CMAKE_POLICY_VERSION_MINIMUM)
    unset(CMAKE_POLICY_VERSION_MINIMUM_LOCALLY_DEFINED)
endif()
if(APPLE)
    set(CMAKE_C_FLAGS "${_SDL2_CMAKE_C_FLAGS_OLD}")
    set(CMAKE_OBJC_FLAGS "${_SDL2_CMAKE_OBJC_FLAGS_OLD}")
    unset(_SDL2_CMAKE_C_FLAGS_OLD)
    unset(_SDL2_CMAKE_OBJC_FLAGS_OLD)
endif()
if(_SDL_AUDIO_WAS_DEFINED)
    set(SDL_AUDIO "${_OLD_SDL_AUDIO}")
else()
    unset(SDL_AUDIO)
endif()
unset(_SDL_AUDIO_WAS_DEFINED)
unset(_OLD_SDL_AUDIO)
