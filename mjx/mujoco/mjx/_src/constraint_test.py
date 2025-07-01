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
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import constraint
from mujoco.mjx._src import test_util
import numpy as np


# tolerance for difference between MuJoCo and MJX constraint calculations,
# mostly due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class ConstraintTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    np.random.seed(42)

  @parameterized.parameters(
      {'cone': mujoco.mjtCone.mjCONE_PYRAMIDAL, 'rand_eq_active': False},
      {'cone': mujoco.mjtCone.mjCONE_ELLIPTIC, 'rand_eq_active': False},
      {'cone': mujoco.mjtCone.mjCONE_PYRAMIDAL, 'rand_eq_active': True},
      {'cone': mujoco.mjtCone.mjCONE_ELLIPTIC, 'rand_eq_active': True},
  )
  def test_constraints(self, cone, rand_eq_active):
    """Test constraints."""
    m = test_util.load_test_file('constraints.xml')
    m.opt.cone = cone
    d = mujoco.MjData(m)

    # sample a mix of active/inactive constraints at different timesteps
    for key in range(3):
      mujoco.mj_resetDataKeyframe(m, d, key)
      if rand_eq_active:
        d.eq_active[:] = np.random.randint(0, 2, size=m.neq)
      mujoco.mj_forward(m, d)
      mx = mjx.put_model(m)
      dx = mjx.put_data(m, d)
      dx = mjx.make_constraint(mx, dx)

      order = test_util.efc_order(m, d, dx)
      d_efc_j = d.efc_J.reshape((-1, m.nv))
      _assert_eq(d_efc_j, dx._impl.efc_J[order][: d.nefc], 'efc_J')
      _assert_eq(0, dx._impl.efc_J[order][d.nefc :], 'efc_J')
      _assert_eq(d.efc_aref, dx._impl.efc_aref[order][: d.nefc], 'efc_aref')
      _assert_eq(0, dx._impl.efc_aref[order][d.nefc :], 'efc_aref')
      _assert_eq(d.efc_D, dx._impl.efc_D[order][: d.nefc], 'efc_D')
      _assert_eq(d.efc_pos, dx._impl.efc_pos[order][: d.nefc], 'efc_pos')
      _assert_eq(dx._impl.efc_pos[order][d.nefc :], 0, 'efc_pos')
      _assert_eq(
          d.efc_frictionloss,
          dx._impl.efc_frictionloss[order][: d.nefc],
          'efc_frictionloss',
      )

  def test_disable_refsafe(self):
    m = test_util.load_test_file('constraints.xml')

    timeconst = m.opt.timestep / 4.0  # timeconst < 2 * timestep
    solimp = jp.array([timeconst, 1.0])
    solref = jp.array([0.8, 0.99, 0.001, 0.2, 2])
    pos = jp.ones(3)

    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.REFSAFE
    mx = mjx.put_model(m)
    k, *_ = constraint._kbi(mx, solimp, solref, pos)
    self.assertEqual(k, 1 / (0.99**2 * timeconst**2))

  def test_disable_constraint(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.CONSTRAINT
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 0)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 0)
    self.assertEqual(nc, 0)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx._impl.efc_J.shape[0], 0)

  def test_disable_equality(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.EQUALITY
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 0)
    self.assertEqual(nf, 2)
    self.assertEqual(nl, 5)
    self.assertEqual(nc, 180)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(
        dx._impl.efc_J.shape[0], 187
    )  # only joint/tendon limit, contact

  def test_disable_contact(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.CONTACT
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 20)
    self.assertEqual(nf, 2)
    self.assertEqual(nl, 5)
    self.assertEqual(nc, 0)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(
        dx._impl.efc_J.shape[0], 27
    )  # only equality, joint/tendon limit

  def test_disable_frictionloss(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.FRICTIONLOSS
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 20)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 5)
    self.assertEqual(nc, 180)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx._impl.efc_J.shape[0], 205)

  def test_margin(self):
    """Test margin."""
    m = mujoco.MjModel.from_xml_string("""
       <mujoco>
          <worldbody>
            <geom name="floor" size="0 0 .05" type="plane" condim="3"/>
            <body pos="0 0 0.1">
              <freejoint/>
              <geom size="0.1" margin="0.25"/>
            </body>
            <body pos="0 0 1">
              <joint type="hinge" limited="true" range="-1 1" margin="0.005"/>
              <geom size="1" margin="0.01"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    dx = mjx.make_constraint(mx, dx)

    order = test_util.efc_order(m, d, dx)
    _assert_eq(d.efc_pos, dx._impl.efc_pos[order][: d.nefc], 'efc_pos')
    _assert_eq(d.efc_margin, dx._impl.efc_margin[order][: d.nefc], 'efc_margin')


if __name__ == '__main__':
  absltest.main()
