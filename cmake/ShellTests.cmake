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

function(add_mujoco_shell_test TEST_NAME TARGET_BINARY)
  find_program(BASH_PROGRAM bash)
  if(BASH_PROGRAM)
    # Set up the test directory
    set(TEST_TMPDIR "${CMAKE_CURRENT_BINARY_DIR}/${TEST_NAME}")
    add_test(NAME ${TEST_NAME}_setup
             COMMAND ${BASH_PROGRAM} "${PROJECT_SOURCE_DIR}/cmake/setup_test_dir.sh" ${TEST_TMPDIR}
    )
    set_tests_properties(${TEST_NAME}_setup PROPERTIES FIXTURES_SETUP ${TEST_NAME}_fixture)
    add_test(NAME ${TEST_NAME}_cleanup
             COMMAND ${BASH_PROGRAM} "${PROJECT_SOURCE_DIR}/cmake/cleanup_test_dir.sh"
                     ${TEST_TMPDIR}
    )
    set_tests_properties(${TEST_NAME}_cleanup PROPERTIES FIXTURES_CLEANUP ${TEST_NAME}_fixture)
    add_test(
      NAME ${TEST_NAME}
      COMMAND ${BASH_PROGRAM} "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.sh"
      WORKING_DIRECTORY $<TARGET_FILE_DIR:${TARGET_BINARY}>
    )
    set_tests_properties(${TEST_NAME} PROPERTIES FIXTURES_REQUIRED ${TEST_NAME}_fixture)
    set_property(
      TEST "${TEST_NAME}"
      PROPERTY ENVIRONMENT
               "CMAKE_SOURCE_DIR=${CMAKE_SOURCE_DIR}"
               "TARGET_BINARY=$<TARGET_FILE:${TARGET_BINARY}>"
               "TEST_TMPDIR=${TEST_TMPDIR}"
    )
    if(WIN32)
      # Define the directory containing the mujoco DLL library so that it can be added to the PATH.
      # We modify the PATH in the script as it is more reliable.
      set_property(
        TEST "${TEST_NAME}"
        APPEND
        PROPERTY ENVIRONMENT "MUJOCO_DLL_DIR=$<TARGET_FILE_DIR:mujoco>"
      )
    endif()
  endif()
endfunction()
