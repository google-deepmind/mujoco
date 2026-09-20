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

set(MUJOCO_DEP_VERSION_atkinson_hyperlegible_mono
    154d50362016cc3e873eb21d242cd0772384c8f9
    CACHE STRING "Tag/version of `atkinson_hyperlegible_mono` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_atkinson_hyperlegible_mono)

include(FindOrFetch)

fetchpackage(
    PACKAGE_NAME  atkinson_hyperlegible_mono
    GIT_REPO      https://github.com/googlefonts/atkinson-hyperlegible-next-mono.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_atkinson_hyperlegible_mono}
    CUSTOM_CMAKE  "${CMAKE_SOURCE_DIR}/cmake/third_party_deps/atkinson_hyperlegible_mono/CMakeLists.txt"
)
