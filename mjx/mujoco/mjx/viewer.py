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

import copy
import logging
import os

os.environ['XLA_FLAGS'] = '--xla_gpu_graph_min_graph_size=1'
import time  # pylint: disable=g-import-not-at-top
from typing import Sequence

from absl import app
from absl import flags
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
import mujoco.viewer
import warp as wp


_JIT = flags.DEFINE_bool('jit', True, 'To jit or not to jit.')
_MODEL_PATH = flags.DEFINE_string(
    'mjcf', None, 'Path to a MuJoCo MJCF file.', required=True
)
_IMPL = flags.DEFINE_string('impl', 'jax', 'MJX implementation.')
_WP_KERNEL_CACHE_DIR = flags.DEFINE_string(
    'wp_kernel_cache_dir',
    None,
    'Path to the Warp kernel cache directory.',
)
_NACONMAX = flags.DEFINE_integer(
    'naconmax',
    None,
    'Maximum number of contacts to simulate, warp only.',
)
_NJMAX = flags.DEFINE_integer(
    'njmax',
    None,
    'Maximum number of constraints to simulate, warp only.',
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

  # TODO(robotic-simulation): improved warp backend performance with MJX viewer
  if _IMPL.value == 'warp':
    logging.info(
        'The native MuJoCo Warp viewer is currently recommended for best'
        ' performance.',
    )

  if _WP_KERNEL_CACHE_DIR.value:
    wp.config.kernel_cache_dir = _WP_KERNEL_CACHE_DIR.value

  jax.config.update('jax_debug_nans', True)

  print(f'Loading model from: {_MODEL_PATH.value}.')
  if _MODEL_PATH.value.endswith('.mjb'):
    m = mujoco.MjModel.from_binary_path(_MODEL_PATH.value)
  else:
    m = mujoco.MjModel.from_xml_path(_MODEL_PATH.value)
  d = mujoco.MjData(m)
  mx = mjx.put_model(m, impl=_IMPL.value)
  if _IMPL.value == 'warp':
    # TODO(btaba): use put_data.
    dx = mjx.make_data(
        m, impl=_IMPL.value, naconmax=_NACONMAX.value, njmax=_NJMAX.value
    )
  else:
    dx = mjx.put_data(
        m, d, impl=_IMPL.value, naconmax=_NACONMAX.value, njmax=_NJMAX.value
    )

  print(f'Default backend: {jax.default_backend()}')
  step_fn = mjx.step

  def set_model_fn(mx, gravity, tolerance, ls_tolerance, timestep):
    return mx.tree_replace({
        'opt.gravity': jp.array(gravity),
        'opt.tolerance': jp.array(tolerance),
        'opt.ls_tolerance': jp.array(ls_tolerance),
        'opt.timestep': jp.array(timestep),
    })

  def set_data_fn(dx, ctrl, act, xfrc_applied, qpos, qvel, time_):
    return dx.tree_replace({
        'ctrl': jp.array(ctrl),
        'act': jp.array(act),
        'xfrc_applied': jp.array(xfrc_applied),
        'qpos': jp.array(qpos),
        'qvel': jp.array(qvel),
        'time': jp.array(time_),
    })

  if _JIT.value:
    print('JIT-compiling the model physics step...')
    start = time.time()
    step_fn = jax.jit(step_fn, donate_argnums=(1,), keep_unused=True).lower(mx, dx).compile()
    elapsed = time.time() - start
    print(f'Compilation took {elapsed}s.')
    set_model_fn = (
        jax.jit(set_model_fn, donate_argnums=(0,), keep_unused=True)
        .lower(mx, m.opt.gravity, m.opt.tolerance, m.opt.ls_tolerance, m.opt.timestep)
        .compile()
    )
    set_data_fn = (
        jax.jit(set_data_fn, donate_argnums=(0,), keep_unused=True)
        .lower(dx, d.ctrl, d.act, d.xfrc_applied, d.qpos, d.qvel, d.time)
        .compile()
    )

  viewer = mujoco.viewer.launch_passive(m, d, key_callback=key_callback)
  with viewer:
    opt = copy.copy(m.opt)
    while True:
      start = time.time()

      # TODO(robotics-simulation): recompile when changing disable flags, etc.
      dx = set_data_fn(dx, d.ctrl, d.act, d.xfrc_applied, d.qpos, d.qvel, d.time)

      if m.opt != opt:
        opt = copy.copy(m.opt)
        mx = set_model_fn(mx, m.opt.gravity, m.opt.tolerance, m.opt.ls_tolerance, m.opt.timestep)

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
