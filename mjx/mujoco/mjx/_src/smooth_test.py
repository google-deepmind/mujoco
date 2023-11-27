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
"""Tests for smooth dynamics functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import DisableBit
# pylint: enable=g-importing-member
import numpy as np


def _assert_eq(a, b, name, step, fname, atol=5e-4, rtol=5e-4):
  err_msg = f'mismatch: {name} at step {step} in {fname}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


def _assert_attr_eq(a, b, attr, step, fname, atol=5e-4, rtol=5e-4):
  err_msg = f'mismatch: {attr} at step {step} in {fname}'
  a, b = getattr(a, attr), getattr(b, attr)
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


class SmoothTest(parameterized.TestCase):

  @parameterized.parameters(enumerate(test_util.TEST_FILES))
  def test_smooth(self, seed, fname):
    """Tests mujoco mj smooth functions match mujoco_mjx smooth functions."""
    if fname in ('convex.xml', 'equality.xml'):
      return

    np.random.seed(seed)

    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)

    kinematics_jit_fn = jax.jit(mjx.kinematics)
    com_pos_jit_fn = jax.jit(mjx.com_pos)
    crb_jit_fn = jax.jit(mjx.crb)
    factor_m_fn = jax.jit(mjx.factor_m)
    com_vel_jit_fn = jax.jit(mjx.com_vel)
    rne_jit_fn = jax.jit(mjx.rne)
    mul_m_jit_fn = jax.jit(mjx.mul_m)
    transmission_jit_fn = jax.jit(mjx.transmission)

    mx = mjx.device_put(m)
    dx = mjx.make_data(mx)

    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    for i in range(100):
      qpos, qvel = d.qpos.copy(), d.qvel.copy()
      mujoco.mj_step(m, d)

      # kinematics
      dx = kinematics_jit_fn(mx, dx.replace(qpos=qpos, qvel=qvel))
      _assert_attr_eq(d, dx, 'xanchor', i, fname)
      _assert_attr_eq(d, dx, 'xaxis', i, fname)
      _assert_attr_eq(d, dx, 'xpos', i, fname)
      _assert_attr_eq(d, dx, 'xquat', i, fname)
      _assert_eq(d.xmat.reshape((-1, 3, 3)), dx.xmat, 'xmat', i, fname)
      _assert_attr_eq(d, dx, 'xipos', i, fname)
      _assert_eq(d.ximat.reshape((-1, 3, 3)), dx.ximat, 'ximat', i, fname)
      _assert_attr_eq(d, dx, 'geom_xpos', i, fname)
      _assert_eq(
          d.geom_xmat.reshape((-1, 3, 3)),
          dx.geom_xmat,
          'geom_xmat',
          i,
          fname,
      )

      # com_pos
      dx = com_pos_jit_fn(mx, dx)
      _assert_attr_eq(d, dx, 'subtree_com', i, fname)
      _assert_attr_eq(d, dx, 'cinert', i, fname)
      _assert_attr_eq(d, dx, 'cdof', i, fname)

      # crb
      dx = crb_jit_fn(mx, dx)
      _assert_attr_eq(d, dx, 'crb', i, fname)
      _assert_attr_eq(d, dx, 'qM', i, fname)

      # factor_m
      dx = factor_m_fn(mx, dx, dx.qM)
      _assert_attr_eq(d, dx, 'qLD', i, fname, atol=1e-3)
      _assert_attr_eq(d, dx, 'qLDiagInv', i, fname, atol=1e-3)

      # com_vel
      dx = com_vel_jit_fn(mx, dx)
      _assert_attr_eq(d, dx, 'cvel', i, fname)
      _assert_attr_eq(d, dx, 'cdof_dot', i, fname)

      # rne
      dx = rne_jit_fn(mx, dx)
      _assert_attr_eq(d, dx, 'qfrc_bias', i, fname)

      # mul_m (auxilliary function, not part of smooth step)
      vec = np.random.random(m.nv)
      mjx_vec = mul_m_jit_fn(mx, dx, jp.array(vec))
      mj_vec = np.zeros(m.nv)
      mujoco.mj_mulM(m, d, mj_vec, vec)
      _assert_eq(mj_vec, mjx_vec, 'mul_m', i, fname)

      # transmission
      dx = transmission_jit_fn(mx, dx)
      _assert_attr_eq(d, dx, 'actuator_length', i, fname)
      _assert_attr_eq(d, dx, 'actuator_moment', i, fname)


class DisableGravityTest(absltest.TestCase):

  def test_disabled(self):
    m = mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option timestep="0.01"/>
          <worldbody>
            <body>
              <joint type="free"/>
              <geom size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """)
    mx = mjx.device_put(m)
    d = mujoco.MjData(m)
    dx = mjx.device_put(d)

    # test with gravity
    step_jit_fn = jax.jit(mjx.step)
    dx = step_jit_fn(mx, dx)
    np.testing.assert_array_almost_equal(
        dx.qpos, np.array([0.0, 0.0, -9.81e-4, 1.0, 0.0, 0.0, 0.0]), decimal=7
    )

    # test with gravity disabled
    mx = mx.tree_replace(
        {'opt.disableflags': mx.opt.disableflags | DisableBit.GRAVITY}
    )
    dx = mjx.device_put(d)
    step_jit_fn = jax.jit(mjx.step)
    dx = step_jit_fn(mx, dx)
    np.testing.assert_equal(
        dx.qpos, np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    )


class SiteTest(absltest.TestCase):

  def test_site(self):
    """Tests that site positions and orientations match MuJoCo."""
    m = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <site name="origin"/>
          <body>
            <joint type="free"/>
            <geom pos="1 0 1" type="box" size="0.1 0.01 0.01"/>
            <site name="s1" pos="1.3 0 0"/>     <!-- pos only -->
            <site name="s2"/>                   <!-- no pos, no quat -->
            <site name="s3" quat="1 0 1 0"/>    <!- quat only -->
            <site name="s4" pos="0 1.5 0" quat="1 0 1 0"/>
            <site name="s5" pos="1 0 1"/>       <!-- same as ipos -->
            <site/>
          </body>
        </worldbody>
      </mujoco>
      """)
    d = mujoco.MjData(m)

    mx = mjx.device_put(m)
    dx = mjx.device_put(d)

    mujoco.mj_forward(m, d)
    dx = mjx.forward(mx, dx)

    np.testing.assert_array_almost_equal(dx.site_xpos, d.site_xpos)
    np.testing.assert_array_almost_equal(
        dx.site_xmat, d.site_xmat.reshape((-1, 3, 3))
    )


if __name__ == '__main__':
  absltest.main()
