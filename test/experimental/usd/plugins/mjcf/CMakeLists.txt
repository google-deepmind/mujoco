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

find_package(pxr REQUIRED)

add_library(usd_fixture STATIC fixture.h fixture.cc)
target_include_directories(usd_fixture PUBLIC ${MUJOCO_TEST_INCLUDE})
target_compile_definitions(usd_fixture PUBLIC MJSTATIC)

target_link_libraries(
  usd_fixture
  PUBLIC usd
         tf
         gtest
         gmock
         mujoco
)

mujoco_test(mjcf_file_format_test ADDITIONAL_LINK_LIBRARIES usd tf usdGeom usdImaging usdPhysics usdShade usd_fixture)
