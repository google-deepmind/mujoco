# Copyright 2021 DeepMind Technologies Limited
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

# Prefer the CMake package installed by a full MuJoCo build.
find_package(mujoco CONFIG QUIET)
if(mujoco_FOUND)
  return()
endif()

# Binary release archives may contain headers and libraries but no package
# configuration. Only search the explicitly supplied archive root in that case.
if(mujoco_ROOT)
  find_path(MUJOCO_ARCHIVE_INCLUDE_DIR
            NAMES mujoco/mujoco.h
            PATHS "${mujoco_ROOT}/include"
            NO_DEFAULT_PATH)
  find_library(MUJOCO_ARCHIVE_LIBRARY
               NAMES mujoco
               PATHS "${mujoco_ROOT}/lib" "${mujoco_ROOT}/bin"
               NO_DEFAULT_PATH)
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mujoco
  REQUIRED_VARS MUJOCO_ARCHIVE_INCLUDE_DIR MUJOCO_ARCHIVE_LIBRARY)
if(mujoco_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_ARCHIVE_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ARCHIVE_INCLUDE_DIR}")
endif()
mark_as_advanced(MUJOCO_ARCHIVE_INCLUDE_DIR MUJOCO_ARCHIVE_LIBRARY)
