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

# Dear ImGui Test Engine: lets us script the UI (ItemClick/MenuClick/...) so the
# LLM can drive Studio via a JSON op-program. Requires dear_imgui to be fetched
# first (it links against it and is built with IMGUI_ENABLE_TEST_ENGINE).

set(MUJOCO_DEP_VERSION_imgui_test_engine
    main
    CACHE STRING "Tag/version of `imgui_test_engine` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_imgui_test_engine)

include(FindOrFetch)

fetchpackage(
    PACKAGE_NAME  imgui_test_engine
    GIT_REPO      https://github.com/ocornut/imgui_test_engine.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_imgui_test_engine}
    CUSTOM_CMAKE  "${CMAKE_SOURCE_DIR}/cmake/third_party_deps/imgui_test_engine/CMakeLists.txt"
)
