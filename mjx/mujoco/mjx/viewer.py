# Copyright 2023 DeepMind Technologies Limited
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
"""An example integration of MJX with the MuJoCo viewer."""

import logging
import time
from typing import Sequence

from absl import app
from absl import flags
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
import mujoco.viewer


_JIT = flags.DEFINE_bool('jit', True, 'To jit or not to jit.')
_MODEL_PATH = flags.DEFINE_string(
    'mjcf', None, 'Path to a MuJoCo MJCF file.', required=True
)


_VIEWER_GLOBAL_STATE = {
    'running': True,
}


def key_callback(key: int) -> None:
  if key == 32:  # Space bar
    _VIEWER_GLOBAL_STATE['running'] = not _VIEWER_GLOBAL_STATE['running']
    logging.info('RUNNING = %s', _VIEWER_GLOBAL_STATE['running'])


def _main(argv: Sequence[str]) -> None:
  """Launches MuJoCo passive viewer fed by MJX."""
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  jax.config.update('jax_debug_nans', True)

  print(f'Loading model from: {_MODEL_PATH.value}.')
  if _MODEL_PATH.value.endswith('.mjb'):
    m = mujoco.MjModel.from_binary_path(_MODEL_PATH.value)
  else:
    m = mujoco.MjModel.from_xml_path(_MODEL_PATH.value)
  d = mujoco.MjData(m)
  mx = mjx.put_model(m)
  dx = mjx.put_data(m, d)

  print(f'Default backend: {jax.default_backend()}')
  step_fn = mjx.step
  if _JIT.value:
    print('JIT-compiling the model physics step...')
    start = time.time()
    step_fn = jax.jit(step_fn).lower(mx, dx).compile()
    elapsed = time.time() - start
    print(f'Compilation took {elapsed}s.')

  viewer = mujoco.viewer.launch_passive(m, d, key_callback=key_callback)
  with viewer:
    while True:
      start = time.time()

      # TODO(robotics-simulation): recompile when changing disable flags, etc.
      dx = dx.replace(
          ctrl=jp.array(d.ctrl),
          act=jp.array(d.act),
          xfrc_applied=jp.array(d.xfrc_applied),
      )
      dx = dx.replace(
          qpos=jp.array(d.qpos), qvel=jp.array(d.qvel), time=jp.array(d.time)
      )  # handle resets
      mx = mx.tree_replace({
          'opt.gravity': m.opt.gravity,
          'opt.tolerance': m.opt.tolerance,
          'opt.ls_tolerance': m.opt.ls_tolerance,
          'opt.timestep': m.opt.timestep,
      })

      if _VIEWER_GLOBAL_STATE['running']:
        dx = step_fn(mx, dx)

      mjx.get_data_into(d, m, dx)
      viewer.sync()

      elapsed = time.time() - start
      if elapsed < m.opt.timestep:
        time.sleep(m.opt.timestep - elapsed)


def main():
  app.run(_main)


if __name__ == '__main__':
  main()
