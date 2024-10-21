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

MODEL_DIRS=(
  "${CMAKE_SOURCE_DIR}/model"
  "${CMAKE_SOURCE_DIR}/test"
)

die() { echo "$*" 1>&2 ; exit 1; }

test_model() {
  local EXPECTED_STR='Simulation time'
  local model="$1"
  echo "Testing $model" >&2

  local iterations=10
  # for particularly slow models, only run 2 steps under ASAN, or skip.
  if [[ ${TESTSPEED_ASAN:-0} != 0 ]]; then
    if [[ "$model" == */humanoid/100_humanoids.xml ||
          "$model" == */composite/particle.xml ||
          "$model" == */replicate/bunnies.xml ||
          "$model" == */replicate/leaves.xml ||
          "$model" == */replicate/particle.xml ||
          "$model" == */engine/testdata/collision_convex/perf/*
    ]]; then
      # these tests can take several minutes under ASAN
      return 0
    fi
    if [[ "$model" == */benchmark/testdata/humanoid200.xml ||
          "$model" == */engine/testdata/collision_convex/stacked_boxes.xml ||
          "$model" == */user/testdata/shark_22_ascii_fTetWild.xml ||
          "$model" == */user/testdata/shark_22_binary_fTetWild.xml
    ]]; then
      iterations=2
    fi
  fi

  # run testspeed, writing its output to stderr.
  # die if testspeed returns a failure code, or if it doesn't have the string
  # "Simulation time" in the output.
  ("$TARGET_BINARY" "$model" "$iterations" || die "testspeed failed") \
      | tee >(cat 1>&2) | grep -q "$EXPECTED_STR"

  if [ "$?" != 0 ]; then
    die "Expected string not found in output ($EXPECTED_STR)."
  fi
}

if [ -z "$TARGET_BINARY" ]; then
  die "Expecting environment variable TARGET_BINARY."
fi

if [ -z "$MUJOCO_DLL_DIR" ]; then
  # Extend PATH to include the directory containing the mujoco DLL.
  # This is needed on Windows.
  PATH=$PATH:$MUJOCO_DLL_DIR
fi

shopt -s globstar
for model_dir in ${MODEL_DIRS[@]}; do
  echo "Looking in $model_dir"
  for model in $model_dir/**/*.xml; do
    if [[ $(basename $model) == malformed* ]]; then
      echo "Skipping $model" >&2
      continue
    fi
    if [[ $(basename $model) == malformed* ]]; then
      echo "Skipping $model" >&2
      continue
    fi
    if grep -q "plugin" $model; then
      continue
    fi
    test_model "$model"
  done
done

cd $CURRENT_DIR
echo "PASS"
