# Copyright 2025 DeepMind Technologies Limited
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
"""Tests for codegen'd smooth functions."""

import functools
import os
import tempfile

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import io
from mujoco.mjx._src import math
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import test_util as tu
from mujoco.mjx.warp import warp as wp  # pylint: disable=g-importing-member
import numpy as np

try:
  from mujoco.mjx.warp import smooth  # pylint: disable=g-import-not-at-top
except ImportError:
  smooth = None

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


class SmoothTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    if mjxw.WARP_INSTALLED:
      self.tempdir = tempfile.TemporaryDirectory()
      wp.config.kernel_cache_dir = self.tempdir.name
    np.random.seed(0)

  def tearDown(self):
    super().tearDown()
    if hasattr(self, 'tempdir'):
      self.tempdir.cleanup()

  def test_kinematics(self):
    """Tests kinematics with unbatched data."""
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = tu.load_test_file('pendula.xml')

    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='warp')

    rng = jax.random.PRNGKey(0)
    dx = mjx.make_data(m, impl='warp')
    rng, key = jax.random.split(rng)
    qpos = jax.random.uniform(key, (m.nq,))
    _, key1, key2 = jax.random.split(rng, 3)
    mocap_pos = jax.random.normal(key1, (m.nmocap, 3))
    mocap_quat = jax.random.normal(key2, (m.nmocap, 4))
    mocap_quat = math.normalize(mocap_quat, axis=0)
    dx = dx.replace(qpos=qpos, mocap_pos=mocap_pos, mocap_quat=mocap_quat)

    dx = jax.jit(smooth.kinematics)(mx, dx)

    d.qpos[:] = qpos
    d.mocap_pos[:] = mocap_pos
    d.mocap_quat[:] = mocap_quat
    mujoco.mj_kinematics(m, d)

    tu.assert_attr_eq(d, dx, 'xanchor')
    tu.assert_attr_eq(d, dx, 'xaxis')
    tu.assert_attr_eq(d, dx, 'xpos')
    tu.assert_attr_eq(d, dx, 'xquat')
    tu.assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat')
    tu.assert_attr_eq(d, dx, 'xipos')
    tu.assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
    tu.assert_attr_eq(d, dx, 'geom_xpos')
    tu.assert_eq(d.geom_xmat.reshape((-1, 3, 3)), dx.geom_xmat, 'geom_xmat')
    tu.assert_attr_eq(d, dx, 'site_xpos')
    tu.assert_eq(d.site_xmat.reshape((-1, 3, 3)), dx.site_xmat, 'site_xmat')

  def test_kinematics_vmap(self):
    """Tests kinematics with batched data."""
    if not mjxw.WARP_INSTALLED:
      self.skipTest('Warp not installed.')
    if not io.has_cuda_gpu_device():
      self.skipTest('No CUDA GPU device available.')

    m = tu.load_test_file('pendula.xml')

    batch_size = 7
    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='warp')

    worldids = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)
    # don't zero xmat, ximat, xquat, geom_xpos, or geom_xmat
    # these fields are precomputed in make_data
    for f in ('xanchor', 'xaxis', 'xpos', 'xipos', 'site_xpos', 'site_xmat'):
      dx_batch = dx_batch.replace(**{f: jp.zeros_like(getattr(dx_batch, f))})

    dx_batch = jax.jit(jax.vmap(smooth.kinematics, in_axes=(None, 0)))(
        mx, dx_batch
    )

    for i in range(batch_size):
      dx = dx_batch[i]

      d.qpos[:] = dx.qpos
      d.mocap_pos[:] = dx.mocap_pos
      d.mocap_quat[:] = dx.mocap_quat
      mujoco.mj_kinematics(m, d)

      tu.assert_attr_eq(d, dx, 'xanchor')
      tu.assert_attr_eq(d, dx, 'xaxis')
      tu.assert_attr_eq(d, dx, 'xpos')
      tu.assert_attr_eq(d, dx, 'xquat')
      tu.assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat')
      tu.assert_attr_eq(d, dx, 'xipos')
      tu.assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
      tu.assert_attr_eq(d, dx, 'geom_xpos')
      tu.assert_eq(d.geom_xmat.reshape((-1, 3, 3)), dx.geom_xmat, 'geom_xmat')
      tu.assert_attr_eq(d, dx, 'site_xpos')
      tu.assert_eq(d.site_xmat.reshape((-1, 3, 3)), dx.site_xmat, 'site_xmat')

  def test_kinematics_nested_vmap(self):
    """Tests kinematics with nested batch data."""
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = tu.load_test_file('pendula.xml')

    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='warp')

    worldids = jp.arange(16).reshape((4, 4))
    dx_batch = jax.vmap(jax.vmap(functools.partial(tu.make_data, m)))(worldids)
    # don't zero xmat, ximat, xquat, geom_xpos, or geom_xmat
    # these fields are precomputed in make_data
    for f in ('xanchor', 'xaxis', 'xpos', 'xipos', 'site_xpos', 'site_xmat'):
      dx_batch = dx_batch.replace(**{f: jp.zeros_like(getattr(dx_batch, f))})

    dx_batch = jax.jit(
        jax.vmap(
            jax.vmap(smooth.kinematics, in_axes=(None, 0)), in_axes=(None, 0)
        )
    )(mx, dx_batch)

    for i in range(4):
      for j in range(4):
        dx = dx_batch[i, j]

        d.qpos[:] = dx.qpos
        d.mocap_pos[:] = dx.mocap_pos
        d.mocap_quat[:] = dx.mocap_quat
        mujoco.mj_forward(m, d)

        tu.assert_attr_eq(d, dx, 'xanchor')
        tu.assert_attr_eq(d, dx, 'xaxis')
        tu.assert_attr_eq(d, dx, 'xpos')
        tu.assert_attr_eq(d, dx, 'xquat')
        tu.assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat')
        tu.assert_attr_eq(d, dx, 'xipos')
        tu.assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
        tu.assert_attr_eq(d, dx, 'geom_xpos')
        tu.assert_eq(d.geom_xmat.reshape((-1, 3, 3)), dx.geom_xmat, 'geom_xmat')
        tu.assert_attr_eq(d, dx, 'site_xpos')
        tu.assert_eq(d.site_xmat.reshape((-1, 3, 3)), dx.site_xmat, 'site_xmat')

  def test_kinematics_model_vmap(self):
    """Tests kinematics with vmap on model and data fields."""
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = tu.load_test_file('pendula.xml')

    batch_size = 7
    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='warp')
    # Add batch dimension to one model field.
    mx = mx.replace(
        geom_pos=jax.random.normal(
            jax.random.PRNGKey(0), (batch_size, m.ngeom, 3)
        )
    )

    worldids = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)
    # don't zero xmat, ximat, xquat, geom_xpos, or geom_xmat
    # these fields are precomputed in make_data
    for f in ('xanchor', 'xaxis', 'xpos', 'xipos', 'site_xpos', 'site_xmat'):
      dx_batch = dx_batch.replace(**{f: jp.zeros_like(getattr(dx_batch, f))})

    dx_batch = jax.jit(jax.vmap(smooth.kinematics, in_axes=(None, 0)))(
        mx, dx_batch
    )

    for i in range(batch_size):
      dx = dx_batch[i]

      d.qpos[:] = dx.qpos
      d.mocap_pos[:] = dx.mocap_pos
      d.mocap_quat[:] = dx.mocap_quat
      m.geom_pos[:] = mx.geom_pos[i]
      mujoco.mj_kinematics(m, d)

      tu.assert_attr_eq(d, dx, 'xanchor')
      tu.assert_attr_eq(d, dx, 'xaxis')
      tu.assert_attr_eq(d, dx, 'xpos')
      tu.assert_attr_eq(d, dx, 'xquat')
      tu.assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat')
      tu.assert_attr_eq(d, dx, 'xipos')
      tu.assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
      tu.assert_attr_eq(d, dx, 'geom_xpos')
      tu.assert_eq(d.geom_xmat.reshape((-1, 3, 3)), dx.geom_xmat, 'geom_xmat')
      tu.assert_attr_eq(d, dx, 'site_xpos')
      tu.assert_eq(d.site_xmat.reshape((-1, 3, 3)), dx.site_xmat, 'site_xmat')

if __name__ == '__main__':
  absltest.main()
