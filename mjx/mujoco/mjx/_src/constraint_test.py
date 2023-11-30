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
"""Tests for constraint functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import constraint
from mujoco.mjx._src import test_util
# pylint: disable=g-importing-member
from mujoco.mjx._src.types import DisableBit
from mujoco.mjx._src.types import SolverType
# pylint: enable=g-importing-member
import numpy as np


def _assert_eq(a, b, name, step, fname, atol=5e-3, rtol=5e-3):
  err_msg = f'mismatch: {name} at step {step} in {fname}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=atol, rtol=rtol)


class ConstraintTest(parameterized.TestCase):

  @parameterized.parameters(enumerate(test_util.TEST_FILES))
  def test_constraints(self, seed, fname):
    """Test constraints."""
    np.random.seed(seed)

    # exclude convex.xml since convex contacts are not exactly equivalent
    if fname == 'convex.xml':
      return

    m = test_util.load_test_file(fname)
    d = mujoco.MjData(m)
    mx = mjx.device_put(m)
    dx = mjx.make_data(mx)

    forward_jit_fn = jax.jit(mjx.forward)

    # give the system a little kick to ensure we have non-identity rotations
    d.qvel = np.random.random(m.nv)
    for i in range(100):
      dx = dx.replace(qpos=jax.device_put(d.qpos), qvel=jax.device_put(d.qvel))
      mujoco.mj_step(m, d)
      dx = forward_jit_fn(mx, dx)

      nnz_filter = dx.efc_J.any(axis=1)

      mj_efc_j = d.efc_J.reshape((-1, m.nv))
      mjx_efc_j = dx.efc_J[nnz_filter]
      _assert_eq(mj_efc_j, mjx_efc_j, 'efc_J', i, fname)

      mjx_efc_d = dx.efc_D[nnz_filter]
      _assert_eq(d.efc_D, mjx_efc_d, 'efc_D', i, fname)

      mjx_efc_aref = dx.efc_aref[nnz_filter]
      _assert_eq(d.efc_aref, mjx_efc_aref, 'efc_aref', i, fname)

      mjx_efc_frictionloss = dx.efc_frictionloss[nnz_filter]
      _assert_eq(
          d.efc_frictionloss,
          mjx_efc_frictionloss,
          'efc_frictionloss',
          i,
          fname,
      )

  _JNT_RANGE = """
    <mujoco>
      <worldbody>
        <body pos="0 0 1">
          <joint type="slide" axis="1 0 0" range="-1.8 1.8" solreflimit=".08 1"
           damping="5e-4"/>
          <geom type="box" size="0.2 0.15 0.1" mass="1"/>
          <body>
            <joint axis="0 1 0" damping="2e-6"/>
            <geom type="capsule" fromto="0 0 0 0 0 1" size="0.045" mass=".1"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
  """

  def test_jnt_range(self):
    """Tests that mixed joint ranges are respected."""
    # TODO(robotics-simulation): also test ball
    m = mujoco.MjModel.from_xml_string(self._JNT_RANGE)
    m.opt.solver = SolverType.CG.value
    d = mujoco.MjData(m)
    d.qpos = np.array([2.0, 15.0])

    mx = mjx.device_put(m)
    dx = mjx.device_put(d)
    efc = jax.jit(constraint._instantiate_limit_slide_hinge)(mx, dx)

    # first joint is outside the joint range
    np.testing.assert_array_almost_equal(efc.J[0, 0], -1.0)

    # second joint has no range, so only one efc row
    self.assertEqual(efc.J.shape[0], 1)

  def test_disable_refsafe(self):
    m = test_util.load_test_file('ant.xml')

    timeconst = m.opt.timestep / 4.0  # timeconst < 2 * timestep
    solimp = jp.array([timeconst, 1.0])
    solref = jp.array([0.8, 0.99, 0.001, 0.2, 2])
    pos = jp.ones(3)

    m.opt.disableflags = m.opt.disableflags | DisableBit.REFSAFE
    mx = mjx.device_put(m)
    k, *_ = constraint._kbi(mx, solimp, solref, pos)
    self.assertEqual(k, 1 / (0.99**2 * timeconst**2))

    m.opt.disableflags = m.opt.disableflags & ~DisableBit.REFSAFE
    mx = mjx.device_put(m)
    k, *_ = constraint._kbi(mx, solimp, solref, pos)
    self.assertEqual(k, 1 / (0.99**2 * (2 * m.opt.timestep) ** 2))

  def test_disableconstraint(self):
    m = test_util.load_test_file('ant.xml')
    d = mujoco.MjData(m)

    m.opt.disableflags = m.opt.disableflags | DisableBit.CONSTRAINT
    mx, dx = mjx.device_put(m), mjx.device_put(d)
    dx = constraint.make_constraint(mx, dx)
    self.assertEqual(dx.efc_J.shape[0], 0)

  def test_disable_equality(self):
    m = test_util.load_test_file('equality.xml')
    d = mujoco.MjData(m)

    m.opt.disableflags = m.opt.disableflags | DisableBit.EQUALITY
    mx, dx = mjx.device_put(m), mjx.device_put(d)
    dx = constraint.make_constraint(mx, dx)
    self.assertEqual(dx.efc_J.shape[0], 0)

  def test_disable_contact(self):
    m = test_util.load_test_file('ant.xml')
    d = mujoco.MjData(m)
    d.qpos[2] = 0.0
    mujoco.mj_forward(m, d)

    m.opt.disableflags = m.opt.disableflags & ~DisableBit.CONTACT
    mx, dx = mjx.device_put(m), mjx.device_put(d)
    dx = dx.tree_replace(
        {'contact.frame': dx.contact.frame.reshape((-1, 3, 3))}
    )
    efc = constraint._instantiate_contact(mx, dx)
    self.assertIsNotNone(efc)

    m.opt.disableflags = m.opt.disableflags | DisableBit.CONTACT
    mx, dx = mjx.device_put(m), mjx.device_put(d)
    efc = constraint._instantiate_contact(mx, dx)
    self.assertIsNone(efc)


if __name__ == '__main__':
  absltest.main()
