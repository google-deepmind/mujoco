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

set(MUJOCO_DEP_VERSION_lodepng
    17d08dd26cac4d63f43af217ebd70318bfb8189c
    CACHE STRING "Version of `lodepng` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_lodepng)

include(FindOrFetch)

fetchpackage(
    PACKAGE_NAME  lodepng
    GIT_REPO      https://github.com/lvandeve/lodepng.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_lodepng}
    CUSTOM_CMAKE  "${CMAKE_CURRENT_LIST_DIR}/lodepng/CMakeLists.txt"
)
