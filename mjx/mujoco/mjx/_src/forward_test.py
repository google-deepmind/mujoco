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
"""Tests for forward functions."""

import itertools

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import forward
from mujoco.mjx._src import test_util
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import DisableBit
# pylint: enable=g-importing-member
import numpy as np


def _assert_attr_eq(a, b, attr, step, fname, atol=1e-3, rtol=1e-3):
  err_msg = f'mismatch: {attr} at step {step} in {fname}'
  a, b = getattr(a, attr), getattr(b, attr)
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


class ForwardTest(parameterized.TestCase):

  @parameterized.parameters(enumerate(test_util.TEST_FILES))
  def test_forward(self, seed, fname):
    """Test mujoco mj forward function matches mujoco_mjx forward function."""
    if fname in ('weld.xml',):
      return

    np.random.seed(seed)

    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)
    mx = mjx.device_put(m)
    dx = mjx.make_data(mx)
    forward_jit_fn = jax.jit(mjx.forward)

    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv) * 0.05
    for i in range(100):
      qpos, qvel = d.qpos.copy(), d.qvel.copy()
      mujoco.mj_step(m, d)
      dx = forward_jit_fn(mx, dx.replace(qpos=qpos, qvel=qvel))

      _assert_attr_eq(d, dx, 'qfrc_smooth', i, fname)
      _assert_attr_eq(d, dx, 'qacc_smooth', i, fname)

  @parameterized.parameters(itertools.product(test_util.TEST_FILES, (0, 1)))
  def test_step(self, fname, integrator_type):
    """Test mujoco mj step matches mujoco_mjx step."""
    if fname in (
        'mixed_joint_pendulum.xml',
        'ball_pendulum.xml',
        'convex.xml',
        'humanoid.xml',
        'triple_pendulum.xml',  # TODO(b/301485081)
        'weld.xml',
    ):
      # skip models with big constraint violations at step 0 or too slow to run
      return

    np.random.seed(integrator_type)
    m = test_util.load_test_file(fname)
    step_jit_fn = jax.jit(forward.step)

    m.opt.integrator = integrator_type
    int_typ = 'euler' if integrator_type == 0 else 'rk4'
    test_name = f'{fname} - {int_typ}'
    steps = 100 if int_typ == 'euler' else 30
    dt = m.opt.timestep
    m.opt.timestep = dt if int_typ == 'euler' else dt * 3

    mx = mjx.device_put(m)
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.normal(m.nv) * 0.05
    for i in range(steps):
      # in order to avoid re-jitting, reuse the same mj_data shape
      qpos, qvel = d.qpos, d.qvel
      d = mujoco.MjData(m)
      d.qpos, d.qvel = qpos, qvel
      dx = mjx.device_put(d)

      mujoco.mj_step(m, d)
      dx = step_jit_fn(mx, dx)

      _assert_attr_eq(d, dx, 'qpos', i, test_name, atol=1e-2)
      _assert_attr_eq(d, dx, 'qvel', i, test_name, atol=1e-2)
      _assert_attr_eq(d, dx, 'act', i, test_name)
      _assert_attr_eq(d, dx, 'time', i, test_name)

  def test_disable_eulerdamp(self):
    m = test_util.load_test_file('ant.xml')
    m.opt.disableflags = m.opt.disableflags | DisableBit.EULERDAMP

    d = mujoco.MjData(m)
    mx = mjx.device_put(m)
    self.assertTrue((mx.dof_damping > 0).any())
    dx = mjx.device_put(d)
    dx = jax.jit(forward.forward)(mx, dx)

    dx = dx.replace(qvel=jp.ones_like(dx.qvel), qacc=jp.ones_like(dx.qacc))
    dx = jax.jit(forward._euler)(mx, dx)
    np.testing.assert_allclose(dx.qvel, 1 + m.opt.timestep)


if __name__ == '__main__':
  absltest.main()
