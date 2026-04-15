# Copyright 2022 DeepMind Technologies Limited
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

# INTERPROCEDURAL_OPTIMIZATION is enforced when enabled.
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

add_library(crossplatform INTERFACE)
target_sources(crossplatform INTERFACE crossplatform.h)
target_compile_features(crossplatform INTERFACE ${MUJOCO_PYTHON_CXX_FEATURE})
set_target_properties(crossplatform PROPERTIES PUBLIC_HEADER crossplatform.h CXX_EXTENSIONS OFF)
target_include_directories(crossplatform INTERFACE ${mujoco_SOURCE_DIR}/mujoco)

add_library(array_traits INTERFACE)
target_sources(array_traits INTERFACE array_traits.h)
target_compile_features(array_traits INTERFACE ${MUJOCO_PYTHON_CXX_FEATURE})
set_target_properties(array_traits PROPERTIES PUBLIC_HEADER array_traits.h CXX_EXTENSIONS OFF)
target_include_directories(array_traits INTERFACE ${mujoco_SOURCE_DIR}/mujoco)
target_link_libraries(array_traits INTERFACE crossplatform Eigen3::Eigen)

add_library(func_traits INTERFACE)
target_sources(func_traits INTERFACE func_traits.h)
target_compile_features(func_traits INTERFACE ${MUJOCO_PYTHON_CXX_FEATURE})
set_target_properties(func_traits PROPERTIES PUBLIC_HEADER func_traits.h CXX_EXTENSIONS OFF)
target_include_directories(func_traits INTERFACE ${mujoco_SOURCE_DIR}/mujoco)

add_library(tuple_tools INTERFACE)
target_sources(tuple_tools INTERFACE tuple_tools.h)
target_compile_features(tuple_tools INTERFACE ${MUJOCO_PYTHON_CXX_FEATURE})
set_target_properties(tuple_tools PROPERTIES PUBLIC_HEADER tuple_tools.h CXX_EXTENSIONS OFF)
target_include_directories(tuple_tools INTERFACE ${mujoco_SOURCE_DIR}/mujoco)
target_link_libraries(tuple_tools INTERFACE crossplatform)

add_library(func_wrap INTERFACE)
target_sources(func_wrap INTERFACE func_wrap.h)
target_compile_features(func_wrap INTERFACE ${MUJOCO_PYTHON_CXX_FEATURE})
set_target_properties(func_wrap PROPERTIES PUBLIC_HEADER func_wrap.h CXX_EXTENSIONS OFF)
target_include_directories(func_wrap INTERFACE ${mujoco_SOURCE_DIR}/mujoco)
target_link_libraries(
  func_wrap
  INTERFACE crossplatform
            Eigen3::Eigen
            array_traits
            func_traits
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(array_traits_test array_traits_test.cc)
  target_link_libraries(
    array_traits_test
    array_traits
    gmock
    gtest_main
  )
  gtest_add_tests(TARGET array_traits_test SOURCES array_traits_test.cc)

  add_executable(func_traits_test func_traits_test.cc)
  target_link_libraries(
    func_traits_test
    func_traits
    gmock
    gtest_main
  )
  gtest_add_tests(TARGET func_traits_test SOURCES func_traits_test.cc)

  add_executable(func_wrap_test func_wrap_test.cc)
  target_link_libraries(
    func_wrap_test
    func_wrap
    gmock
    gtest_main
  )
  gtest_add_tests(TARGET func_wrap_test SOURCES func_wrap_test.cc)

  add_executable(tuple_tools_test tuple_tools_test.cc)
  target_link_libraries(
    tuple_tools_test
    func_wrap
    gmock
    gtest_main
  )
  gtest_add_tests(TARGET tuple_tools_test SOURCES tuple_tools_test.cc)
endif()
