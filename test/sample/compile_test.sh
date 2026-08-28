#!/bin/bash
# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

MODEL="${CMAKE_SOURCE_DIR}/model/humanoid/humanoid100.xml"
OUTPUT_FILE="${TEST_TMPDIR}/compiled.mjb"


die() { echo "$*" 1>&2 ; exit 1; }

if [ -z "$TARGET_BINARY" ]; then
  die "Expecting environment variable TARGET_BINARY."
fi

if [ -z "$MUJOCO_DLL_DIR" ]; then
  # Extend PATH to include the directory containing the mujoco DLL.
  # This is needed on Windows.
  PATH=$PATH:$MUJOCO_DLL_DIR
fi

"$TARGET_BINARY" "$MODEL" "$OUTPUT_FILE" || die "compile failed"

if [ ! -s "$OUTPUT_FILE" ]; then
  die "Output file empty or missing (${OUTPUT_FILE})."
fi

# Test invalid arguments (expect failure)
"$TARGET_BINARY" > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure with no arguments"
fi

# Test illegal combination (MJB -> XML, expect failure)
"$TARGET_BINARY" "$OUTPUT_FILE" "${TEST_TMPDIR}/failed.xml" > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for MJB -> XML"
fi

# Test compile with 1 argument (expect success)
"$TARGET_BINARY" "$MODEL" > /dev/null || die "compile with 1 argument failed"

# Test XML -> MJZ (expect success)
OUTPUT_MJZ="${TEST_TMPDIR}/compiled.mjz"
"$TARGET_BINARY" "$MODEL" "$OUTPUT_MJZ" || die "compile to MJZ failed"

if [ ! -s "$OUTPUT_MJZ" ]; then
  die "MJZ output file empty or missing (${OUTPUT_MJZ})."
fi

# Test MJZ -> MJB (expect success)
OUTPUT_MJB_FROM_MJZ="${TEST_TMPDIR}/compiled_from_mjz.mjb"
"$TARGET_BINARY" "$OUTPUT_MJZ" "$OUTPUT_MJB_FROM_MJZ" || die "compile from MJZ to MJB failed"

if [ ! -s "$OUTPUT_MJB_FROM_MJZ" ]; then
  die "MJB output from MJZ empty or missing (${OUTPUT_MJB_FROM_MJZ})."
fi

echo "PASS"
