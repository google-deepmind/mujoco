#!/bin/bash
# Copyright 2026 DeepMind Technologies Limited
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

MODEL="${CMAKE_SOURCE_DIR}/model/humanoid/humanoid.xml"

die() { echo "$*" 1>&2 ; exit 1; }

if [ -z "$TARGET_BINARY" ]; then
  die "Expecting environment variable TARGET_BINARY."
fi

if [ -n "$MUJOCO_DLL_DIR" ]; then
  # Extend PATH to include the directory containing the mujoco DLL.
  # This is needed on Windows.
  PATH=$PATH:$MUJOCO_DLL_DIR
fi

# Test help with no arguments (expect success and usage message)
OUTPUT=$("$TARGET_BINARY") || die "render failed with no arguments"
echo "$OUTPUT" | grep "Usage:" > /dev/null || die "Expected usage message with no arguments"

# Test help with --help (expect success and usage message)
OUTPUT=$("$TARGET_BINARY" --help) || die "render failed with --help"
echo "$OUTPUT" | grep "Usage:" > /dev/null || die "Expected usage message with --help"

# Test help with -h (expect success and usage message)
OUTPUT=$("$TARGET_BINARY" -h) || die "render failed with -h"
echo "$OUTPUT" | grep "Usage:" > /dev/null || die "Expected usage message with -h"

# Test nonexistent model (expect failure)
"$TARGET_BINARY" "${TEST_TMPDIR}/nonexistent_model.xml" > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for nonexistent model"
fi

# Test invalid width (expect failure)
"$TARGET_BINARY" "$MODEL" --width=-10 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for negative width"
fi

# Test invalid geomgroup (expect failure)
"$TARGET_BINARY" "$MODEL" --geomgroup=123 > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for invalid geomgroup length"
fi

# Test invalid backend (expect failure)
"$TARGET_BINARY" "$MODEL" --backend=invalid_backend > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for invalid backend"
fi

# Test invalid label (expect failure)
"$TARGET_BINARY" "$MODEL" --label=invalid_label > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for invalid label"
fi

# Test invalid frame (expect failure)
"$TARGET_BINARY" "$MODEL" --frame=invalid_frame > /dev/null 2>&1
if [ $? -eq 0 ]; then
  die "Expected failure for invalid frame"
fi

# Test that the Filament backend actually renders. A headless environment
# without a render context (e.g. CI without a GPU) fails to initialize, which is
# tolerated here; only a missing-asset failure -- which passing arg validation
# alone would not catch -- is fatal.
OUT_PNG="${TEST_TMPDIR}/render_out.png"
RENDER_LOG=$("$TARGET_BINARY" "$MODEL" "$OUT_PNG" --width=64 --height=48 2>&1)
if echo "$RENDER_LOG" | grep -q "Failed to open filament asset"; then
  die "Filament asset resolution failed: $RENDER_LOG"
fi
if [ -f "$OUT_PNG" ]; then
  echo "Filament render produced an output image"
else
  echo "Skipping render output check (no render context available)"
fi
