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

# Only support testing when building against a standalone version of USD.
if(MUJOCO_WITH_USD)
  add_library(usd_test_utils STATIC usd/test_utils.h usd/test_utils.cc)
  target_include_directories(usd_test_utils PUBLIC ${MUJOCO_TEST_INCLUDE})
  target_compile_definitions(usd_test_utils PUBLIC MJSTATIC)
  target_link_libraries(
    usd_test_utils
    PUBLIC usd
           tf
           gtest
           gmock
           mujoco
  )

  add_subdirectory(usd/plugins/mjcf)
endif()


