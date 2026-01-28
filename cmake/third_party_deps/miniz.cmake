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

set(MUJOCO_DEP_VERSION_miniz
    2.1.0
    CACHE STRING "Tag/version of `miniz` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_miniz)

include(FindOrFetch)

fetchpackage(
    PACKAGE_NAME  miniz
    GIT_REPO      https://github.com/richgel999/miniz.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_miniz}
    TARGETS       miniz
    CUSTOM_CMAKE  "${CMAKE_SOURCE_DIR}/cmake/third_party_deps/miniz/CMakeLists.txt"
)
