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

echo "PASS"
