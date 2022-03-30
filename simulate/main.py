# Copyright 2022 DeepMind Technologies Limited
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
# ==============================================================================
"""Python version of the simulate C++ sample"""

import mujoco
from mujoco.simulate import run_simulate_and_physics
import sys

if __name__ == '__main__':
  # print version, check compatibility
  print('MuJoCo version {}'.format(mujoco.mj_versionString()))
  if mujoco.mjVERSION_HEADER != mujoco.mj_version():
    raise mujoco.FatalError('Headers and library have different versions')

  if len(sys.argv) > 1:
    run_simulate_and_physics(sys.argv[1])
  else:
    run_simulate_and_physics()
