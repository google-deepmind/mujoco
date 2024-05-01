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
  tol = _TOLERANCE * 10   # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class ConstraintTest(absltest.TestCase):

  def test_constraints(self):
    """Test constraints."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 100)  # at 100 steps mix of active/inactive constraints
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = mjx.put_data(m, d)
    dx = mjx.make_constraint(mx, dx)
    d_efc_j = d.efc_J.reshape((-1, m.nv))

    # ne, nf, nl order matches
    efl = d.ne + d.nf + d.nl
    _assert_eq(d_efc_j[:efl], dx.efc_J[:efl], 'efc_J')
    _assert_eq(d.efc_D[:efl], dx.efc_D[:efl], 'efc_D')
    _assert_eq(d.efc_aref[:efl], dx.efc_aref[:efl], 'efc_aref')
    _assert_eq(dx.efc_frictionloss, 0, 'efc_frictionloss')

    # contact order might not match, so check efcs contact by contact
    for i in range(d.ncon):
      geom_match = (dx.contact.geom == d.contact.geom[i]).all(axis=-1)
      geom_match &= (dx.contact.pos == d.contact.pos[i]).all(axis=-1)
      self.assertTrue(geom_match.any(), f'contact {i} not found in MJX contact')
      j = np.nonzero(geom_match)[0][0]
      self.assertEqual(d.contact.dim[i], dx.contact.dim[j])
      nc = max(1, (d.contact.dim[i] - 1) * 2)
      d_beg, dx_beg = d.contact.efc_address[i], dx.contact.efc_address[j]
      d_end, dx_end = d_beg + nc, dx_beg + nc
      _assert_eq(d_efc_j[d_beg:d_end], dx.efc_J[dx_beg:dx_end], 'efc_J')
      _assert_eq(d.efc_D[d_beg:d_end], dx.efc_D[dx_beg:dx_end], 'efc_D')
      d_efc_aref = d.efc_aref[d_beg:d_end]
      dx_efc_aref = dx.efc_aref[dx_beg:dx_end]
      _assert_eq(d_efc_aref, dx_efc_aref, 'efc_aref')

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
    self.assertEqual(dx.efc_J.shape[0], 0)

  def test_disable_equality(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.EQUALITY
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 0)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 2)
    self.assertEqual(nc, 148)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx.efc_J.shape[0], 150)  # only joint range, contact

  def test_disable_contact(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.CONTACT
    ne, nf, nl, nc = constraint.counts(constraint.make_efc_type(m))
    self.assertEqual(ne, 10)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 2)
    self.assertEqual(nc, 0)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx.efc_J.shape[0], 12)  # only joint range, limit


if __name__ == '__main__':
  absltest.main()
