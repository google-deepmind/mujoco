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

# Macro for benchmarks that don't use GL (same as mujoco_api_test, but uses
# benchmark::benchmark_main).

macro(mujoco_benchmark_test name)
  add_executable(${name} ${name}.cc)
  target_link_libraries(
    ${name}
    benchmark::benchmark_main
    mujoco
    absl::core_headers
  )
  target_include_directories(${name} PRIVATE ${MUJOCO_TEST_INCLUDE})
  # TODO(fraromano) Check RPATH settings
  set_target_properties(${name} PROPERTIES BUILD_RPATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  # gtest_discover_tests is recommended over gtest_add_tests, but has some issues in Windows.
  gtest_add_tests(
    TARGET ${name}
    SOURCES ${name}.cc
    WORKING_DIRECTORY ${MUJOCO_TEST_WORKING_DIR}
    TEST_LIST testList
  )
  if(WIN32)
    set_tests_properties(
      ${testList} PROPERTIES ENVIRONMENT "PATH=$<TARGET_FILE_DIR:mujoco>;$ENV{PATH}"
    )
  endif()
endmacro()

mujoco_benchmark_test(step_benchmark_test)
target_link_libraries(
  step_benchmark_test
  fixture
  gmock
  benchmark::benchmark
)

mujoco_benchmark_test(parse_benchmark_test)
target_link_libraries(
  parse_benchmark_test
  fixture
  gmock
  benchmark::benchmark
)

mujoco_benchmark_test(engine_util_spatial_benchmark_test)
target_link_libraries(
  engine_util_spatial_benchmark_test
  fixture
  gmock
  benchmark::benchmark
)
