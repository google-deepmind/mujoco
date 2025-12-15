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
"""Tests for forward functions."""

import functools
import os
import tempfile

from absl.testing import absltest
from absl.testing import parameterized
import jax
import jax.numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import io
from mujoco.mjx._src import test_util
import mujoco.mjx.warp as mjxw
from mujoco.mjx.warp import test_util as tu
from mujoco.mjx.warp import warp as wp  # pylint: disable=g-importing-member
import numpy as np

try:
  from mujoco.mjx.warp import forward  # pylint: disable=g-import-not-at-top
except ImportError:
  forward = None

_FORCE_TEST = os.environ.get('MJX_WARP_FORCE_TEST', '0') == '1'


class ForwardTest(parameterized.TestCase):

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

  @parameterized.parameters(
      'pendula.xml',
      'humanoid/humanoid.xml',
  )
  def test_jit_caching(self, xml):
    """Tests jit caching on the full step function."""
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    batch_size = 7
    m = test_util.load_test_file(xml)
    mx = mjx.put_model(m, impl='warp')

    keys = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(keys)

    step_fn = jax.jit(jax.vmap(forward.step, in_axes=(None, 0)))
    dx_batch1 = step_fn(mx, dx_batch)
    jax.tree_util.tree_map(lambda x: x.block_until_ready(), dx_batch1)
    self.assertEqual(step_fn._cache_size(), 1)

    # Re-generate data and run step_fn again to test jit caching.
    keys = jp.arange(batch_size, batch_size * 2)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(keys)
    dx_batch2 = step_fn(mx, dx_batch)
    jax.tree_util.tree_map(lambda x: x.block_until_ready(), dx_batch2)
    self.assertEqual(step_fn._cache_size(), 1)

  @parameterized.product(
      xml=(
          'humanoid/humanoid.xml',
          'pendula.xml',
      ),
      batch_size=(1, 7),
  )
  def test_forward(self, xml: str, batch_size: int):
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = test_util.load_test_file(xml)
    m.opt.iterations = 10
    m.opt.ls_iterations = 10
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    mx = mjx.put_model(m, impl='warp')

    d = mujoco.MjData(m)
    worldids = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)

    dx_batch = jax.jit(jax.vmap(forward.forward, in_axes=(None, 0)))(
        mx, dx_batch
    )

    for i in range(batch_size):
      dx = dx_batch[i]

      d.qpos[:] = dx.qpos
      d.qvel[:] = dx.qvel
      d.ctrl[:] = dx.ctrl
      d.mocap_pos[:] = dx.mocap_pos
      d.mocap_quat[:] = dx.mocap_quat
      mujoco.mj_forward(m, d)

      # fwd_position
      tu.assert_attr_eq(dx, d, 'xpos')
      tu.assert_attr_eq(dx, d, 'xquat')
      tu.assert_attr_eq(dx, d, 'xipos')
      tu.assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat')
      tu.assert_attr_eq(dx, d, 'xanchor')
      tu.assert_attr_eq(dx, d, 'xaxis')
      tu.assert_attr_eq(dx, d, 'geom_xpos')
      tu.assert_eq(dx.geom_xmat, d.geom_xmat.reshape((-1, 3, 3)), 'geom_xmat')
      if m.nsite:
        tu.assert_attr_eq(dx, d, 'site_xpos')
        tu.assert_eq(dx.site_xmat, d.site_xmat.reshape((-1, 3, 3)), 'site_xmat')
      tu.assert_attr_eq(dx, d, 'cdof')
      tu.assert_attr_eq(dx._impl, d, 'cinert')
      tu.assert_attr_eq(dx, d, 'subtree_com')
      if m.nlight:
        tu.assert_attr_eq(dx._impl, d, 'light_xpos')
        tu.assert_attr_eq(dx._impl, d, 'light_xdir')
      if m.ncam:
        tu.assert_attr_eq(dx, d, 'cam_xpos')
        tu.assert_eq(dx.cam_xmat, d.cam_xmat.reshape((-1, 3, 3)), 'cam_xmat')
      tu.assert_attr_eq(dx, d, 'ten_length')
      tu.assert_attr_eq(dx._impl, d, 'ten_J')
      tu.assert_attr_eq(dx._impl, d, 'ten_wrapadr')
      tu.assert_attr_eq(dx._impl, d, 'ten_wrapnum')
      tu.assert_attr_eq(dx._impl, d, 'wrap_xpos')
      tu.assert_attr_eq(dx._impl, d, 'wrap_obj')
      tu.assert_attr_eq(dx._impl, d, 'crb')

      qm = np.zeros((m.nv, m.nv))
      mujoco.mj_fullM(m, qm, d.qM)
      # mjwarp adds padding to qM
      tu.assert_eq(qm, dx._impl.qM[: m.nv, : m.nv], 'qM')
      # qLD is fused in a cholesky factorize and solve, and not written to.

      tu.assert_contact_eq(d, dx, worldid=i)

      tu.assert_attr_eq(dx, d, 'actuator_length')
      actuator_moment = np.zeros((m.nu, m.nv))
      mujoco.mju_sparse2dense(
          actuator_moment,
          d.actuator_moment,
          d.moment_rownnz,
          d.moment_rowadr,
          d.moment_colind,
      )
      tu.assert_eq(dx._impl.actuator_moment, actuator_moment, 'actuator_moment')

      # fwd_velocity
      tu.assert_attr_eq(dx._impl, d, 'actuator_velocity')
      tu.assert_attr_eq(dx, d, 'cvel')
      tu.assert_attr_eq(dx, d, 'cdof_dot')
      tu.assert_attr_eq(dx._impl, d, 'qfrc_spring')
      tu.assert_attr_eq(dx._impl, d, 'qfrc_damper')
      tu.assert_attr_eq(dx, d, 'qfrc_gravcomp')
      tu.assert_attr_eq(dx, d, 'qfrc_fluid')
      tu.assert_attr_eq(dx, d, 'qfrc_passive')
      tu.assert_attr_eq(dx, d, 'qfrc_bias')
      tu.assert_efc_eq(d, dx, worldid=i)

      # fwd_actuation
      tu.assert_attr_eq(dx, d, 'act_dot')
      tu.assert_attr_eq(dx, d, 'actuator_force')
      tu.assert_attr_eq(dx, d, 'qfrc_actuator')

      # fwd_acceleration
      tu.assert_attr_eq(dx, d, 'qfrc_smooth')
      tu.assert_attr_eq(dx, d, 'qacc_smooth')

      np.testing.assert_allclose(
          dx.qacc, d.qacc, err_msg='qacc', rtol=1e-5, atol=1.0
      )


class StepTest(parameterized.TestCase):

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

  @parameterized.product(
      xml=(
          'humanoid/humanoid.xml',
          'pendula.xml',
      ),
      batch_size=(1, 7),
  )
  def test_step(self, xml: str, batch_size: int):
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    m = test_util.load_test_file(xml)
    m.opt.iterations = 10
    m.opt.ls_iterations = 10
    mx = mjx.put_model(m, impl='warp')

    d = mujoco.MjData(m)
    worldids = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)
    dx_batch_orig = dx_batch

    for _ in range(10):
      dx_batch = jax.jit(jax.vmap(forward.step, in_axes=(None, 0)))(
          mx, dx_batch
      )

    for i in range(batch_size):
      dx = dx_batch[i]
      dx_orig = dx_batch_orig[i]

      d.qpos[:] = dx_orig.qpos
      d.qvel[:] = dx_orig.qvel
      d.ctrl[:] = dx_orig.ctrl
      d.mocap_pos[:] = dx_orig.mocap_pos
      d.mocap_quat[:] = dx_orig.mocap_quat
      d.time = dx_orig.time
      mujoco.mj_step(m, d, 10)

      tu.assert_attr_eq(dx, d, 'qpos')
      tu.assert_attr_eq(dx, d, 'qvel')
      tu.assert_attr_eq(dx, d, 'time')
      tu.assert_attr_eq(dx, d, 'ctrl')
      tu.assert_attr_eq(dx, d, 'act')
      tu.assert_attr_eq(dx, d, 'mocap_pos')
      tu.assert_attr_eq(dx, d, 'mocap_quat')
      tu.assert_attr_eq(dx, d, 'sensordata')

  def test_step_leading_dim_mismatch(self):
    if not _FORCE_TEST:
      if not mjxw.WARP_INSTALLED:
        self.skipTest('Warp not installed.')
      if not io.has_cuda_gpu_device():
        self.skipTest('No CUDA GPU device available.')

    xml = 'humanoid/humanoid.xml'
    batch_size = 7

    m = test_util.load_test_file(xml)
    mx = mjx.put_model(m, impl='warp')

    worldids = jp.arange(batch_size)
    dx_batch = jax.vmap(functools.partial(tu.make_data, m))(worldids)
    dx_batch_orig = dx_batch

    with self.assertRaises(ValueError):
      dx_batch = dx_batch.replace(qpos=dx_batch.qpos[1:])
      _ = jax.jit(jax.vmap(forward.step, in_axes=(None, 0)))(mx, dx_batch)

    dx_batch = dx_batch_orig
    with self.assertRaises(ValueError):
      dx_batch = dx_batch.tree_replace(
          {'_impl.contact__pos': dx_batch._impl.contact__pos[1:]}
      )
      _ = jax.jit(jax.vmap(forward.step, in_axes=(None, 0)))(mx, dx_batch)


if __name__ == '__main__':
  absltest.main()
