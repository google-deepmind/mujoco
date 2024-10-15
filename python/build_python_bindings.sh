#!/bin/bash
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

PYTHONPATH="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
DISTPATH="${PYTHONPATH}"/dist
DISTPATTERN="mujoco-*.tar.gz"

FILES=$(find "$DISTPATH" -type f -name "${DISTPATTERN}")

if [ -n "$FILES" ]; then
    echo "Found the following distributions:"
    echo "$FILES"
    echo "$FILES" | xargs rm
    
    echo "Matching files removed."
else
    echo "No current MuJoCo python distributions found."
fi

pythonpath="${PYTHONPATH}"
bash "${pythonpath}/make_sdist.sh"

FILES=$(find "$DISTPATH" -type f -name "${DISTPATTERN}")

MUJOCO_PATH=$MUJOCO_PATH MUJOCO_PLUGIN_PATH=$MUJOCO_PLUGIN_PATH pip install ${FILES[0]}