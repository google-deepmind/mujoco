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
    98d1f3a45aae568ccd6ed5fec179330f47d4d356
    CACHE STRING "Version of `SDL2` to be fetched."
)
mark_as_advanced(MUJOCO_DEP_VERSION_sdl2)

include(FindOrFetch)

set(SDL_SHARED_ENABLED_BY_DEFAULT OFF)
fetchpackage(
    PACKAGE_NAME  sdl2
    GIT_REPO      https://github.com/libsdl-org/SDL.git
    GIT_TAG       ${MUJOCO_DEP_VERSION_sdl2}
)
