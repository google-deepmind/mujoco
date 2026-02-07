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

set(MUJOCO_DEP_VERSION_dear_imgui
    3109131a882daec56a530aff540416983c240443
    CACHE STRING "Tag/version of `dear_imgui` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_dear_imgui)

include(FindOrFetch)

fetchpackage(
    PACKAGE_NAME  dear_imgui
    GIT_REPO      https://github.com/ocornut/imgui.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_dear_imgui}
    CUSTOM_CMAKE  "${CMAKE_SOURCE_DIR}/cmake/third_party_deps/dear_imgui/CMakeLists.txt"
)
