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

  @parameterized.parameters(
      filter(lambda s: s not in ('equality.xml',), test_util.TEST_FILES)
  )
  def test_forward(self, fname):
    """Test mujoco mj forward function matches mujoco_mjx forward function."""
    np.random.seed(test_util.TEST_FILES.index(fname))

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

  @parameterized.parameters(
      filter(lambda s: s not in ('equality.xml',), test_util.TEST_FILES)
  )
  def test_step(self, fname):
    """Test mujoco mj step matches mujoco_mjx step."""
    np.random.seed(test_util.TEST_FILES.index(fname))
    m = test_util.load_test_file(fname)
    step_jit_fn = jax.jit(forward.step)

    mx = mjx.device_put(m)
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.normal(m.nv) * 0.05
    for i in range(100):
      # in order to avoid re-jitting, reuse the same mj_data shape
      qpos, qvel = d.qpos, d.qvel
      d = mujoco.MjData(m)
      d.qpos, d.qvel = qpos, qvel
      dx = mjx.device_put(d)

      mujoco.mj_step(m, d)
      dx = step_jit_fn(mx, dx)

      _assert_attr_eq(d, dx, 'qvel', i, fname, atol=1e-2)
      _assert_attr_eq(d, dx, 'qpos', i, fname, atol=1e-2)
      _assert_attr_eq(d, dx, 'act', i, fname)
      _assert_attr_eq(d, dx, 'time', i, fname)

  def test_rk4(self):
    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option integrator="RK4">
            <flag constraint="disable"/>
          </option>
          <worldbody>
            <light pos="0 0 1"/>
            <geom type="plane" size="1 1 .01" pos="0 0 -1"/>
            <body pos="0.15 0 0">
              <joint type="hinge" axis="0 1 0"/>
              <geom type="capsule" size="0.02" fromto="0 0 0 .1 0 0"/>
              <body pos="0.1 0 0">
                <joint type="slide" axis="1 0 0" stiffness="200"/>
                <geom type="capsule" size="0.015" fromto="-.1 0 0 .1 0 0"/>
              </body>
            </body>
          </worldbody>
        </mujoco>
        """)
    step_jit_fn = jax.jit(forward.step)

    mx = mjx.device_put(m)
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.normal(m.nv) * 0.05
    for i in range(100):
      # in order to avoid re-jitting, reuse the same mj_data shape
      qpos, qvel = d.qpos, d.qvel
      d = mujoco.MjData(m)
      d.qpos, d.qvel = qpos, qvel
      dx = mjx.device_put(d)

      mujoco.mj_step(m, d)
      dx = step_jit_fn(mx, dx)

      _assert_attr_eq(d, dx, 'qvel', i, 'test_rk4', atol=1e-2)
      _assert_attr_eq(d, dx, 'qpos', i, 'test_rk4', atol=1e-2)
      _assert_attr_eq(d, dx, 'act', i, 'test_rk4')
      _assert_attr_eq(d, dx, 'time', i, 'test_rk4')

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
