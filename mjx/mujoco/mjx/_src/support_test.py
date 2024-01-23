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
"""Tests for support."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import support
from mujoco.mjx._src import test_util
import numpy as np


class SupportTest(parameterized.TestCase):

  def test_mul_m(self):
    m = test_util.load_test_file('pendula.xml')
    # first test sparse
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    vec = np.random.random(m.nv)
    mjx_vec = jax.jit(mjx.mul_m)(mx, dx, jp.array(vec))
    mj_vec = np.zeros(m.nv)
    mujoco.mj_mulM(m, d, mj_vec, vec)
    np.testing.assert_allclose(mjx_vec, mj_vec, atol=5e-5, rtol=5e-5)

    # also check dense
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    mjx_vec = jax.jit(mjx.mul_m)(mx, dx, jp.array(vec))
    np.testing.assert_allclose(mjx_vec, mj_vec, atol=5e-5, rtol=5e-5)

  def test_full_m(self):
    m = test_util.load_test_file('pendula.xml')
    # for the model to be sparse to exercise MJX full_M
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    mujoco.mj_step(m, d, 10)  # let dynamics get state significantly non-zero
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    mjx_full_m = jax.jit(support.full_m)(mx, dx)
    mj_full_m = np.zeros((m.nv, m.nv), dtype=np.float64)
    mujoco.mj_fullM(m, mj_full_m, d.qM)
    np.testing.assert_allclose(mjx_full_m, mj_full_m, atol=5e-5, rtol=5e-5)

  @parameterized.parameters('constraints.xml', 'pendula.xml')
  def test_jac(self, fname):
    np.random.seed(0)

    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    point = np.random.randn(3)
    body = np.random.choice(m.nbody)
    jacp, jacr = jax.jit(support.jac)(mx, dx, point, body)

    jacp_expected, jacr_expected = np.zeros((3, m.nv)), np.zeros((3, m.nv))
    mujoco.mj_jac(m, d, jacp_expected, jacr_expected, point, body)
    np.testing.assert_almost_equal(jacp, jacp_expected.T, 6)
    np.testing.assert_almost_equal(jacr, jacr_expected.T, 6)

  def test_xfrc_accumulate(self):
    """Tests that xfrc_accumulate ouput matches mj_xfrcAccumulate."""
    np.random.seed(0)

    m = test_util.load_test_file('pendula.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    self.assertFalse((dx.xipos == 0.0).all())

    xfrc = np.random.rand(*dx.xfrc_applied.shape)

    d.xfrc_applied[:] = xfrc
    dx = dx.replace(xfrc_applied=jp.array(xfrc))

    qfrc = jax.jit(support.xfrc_accumulate)(mx, dx)
    qfrc_expected = np.zeros(m.nv)
    for i in range(1, m.nbody):
      mujoco.mj_applyFT(
          m,
          d,
          d.xfrc_applied[i, :3],
          d.xfrc_applied[i, 3:],
          d.xipos[i],
          i,
          qfrc_expected,
      )

    np.testing.assert_almost_equal(qfrc, qfrc_expected, 6)


if __name__ == '__main__':
  absltest.main()
