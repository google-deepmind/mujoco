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

set(MUJOCO_DEP_VERSION_libwebp
    v1.6.0
    CACHE STRING "Tag/version of `libwebp` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_libwebp)

include(FindOrFetch)

set(LIBWEBP_PATCH_COMMAND
  git --git-dir=. -c core.autocrlf=false -c core.whitespace=cr-at-eol apply --verbose --whitespace=fix --ignore-space-change ${mujoco_SOURCE_DIR}/cmake/libwebp-apple-float16.patch
)

set(BUILD_SHARED_LIBS_OLD ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
if(APPLE)
    set(_LIBWEBP_CMAKE_C_FLAGS_OLD "${CMAKE_C_FLAGS}")
    set(_LIBWEBP_CMAKE_CXX_FLAGS_OLD "${CMAKE_CXX_FLAGS}")
    if(DEFINED MUJOCO_MACOSX_VERSION_MIN)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mmacosx-version-min=${MUJOCO_MACOSX_VERSION_MIN}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmacosx-version-min=${MUJOCO_MACOSX_VERSION_MIN}")
    endif()
endif()

fetchpackage(
    PACKAGE_NAME  libwebp
    GIT_REPO      https://github.com/webmproject/libwebp.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_libwebp}
    TARGETS       webp
    PATCH_COMMAND ${LIBWEBP_PATCH_COMMAND}
)

if(APPLE)
    set(CMAKE_C_FLAGS "${_LIBWEBP_CMAKE_C_FLAGS_OLD}")
    set(CMAKE_CXX_FLAGS "${_LIBWEBP_CMAKE_CXX_FLAGS_OLD}")
    unset(_LIBWEBP_CMAKE_C_FLAGS_OLD)
    unset(_LIBWEBP_CMAKE_CXX_FLAGS_OLD)
endif()
set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_OLD})
