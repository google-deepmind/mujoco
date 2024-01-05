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
    nnz = dx.efc_J.any(axis=1)
    _assert_eq(d.efc_J, dx.efc_J[nnz].reshape(-1), 'efc_J')
    _assert_eq(d.efc_D, dx.efc_D[nnz], 'efc_D')
    _assert_eq(d.efc_aref, dx.efc_aref[nnz], 'efc_aref')
    _assert_eq(d.efc_frictionloss, dx.efc_frictionloss[nnz], 'efc_frictionloss')

  def test_disable_refsafe(self):
    m = test_util.load_test_file('constraints.xml')

    timeconst = m.opt.timestep / 4.0  # timeconst < 2 * timestep
    solimp = jp.array([timeconst, 1.0])
    solref = jp.array([0.8, 0.99, 0.001, 0.2, 2])
    pos = jp.ones(3)

    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.REFSAFE
    mx = mjx.device_put(m)
    k, *_ = constraint._kbi(mx, solimp, solref, pos)
    self.assertEqual(k, 1 / (0.99**2 * timeconst**2))

  def test_disable_constraint(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.CONSTRAINT
    ne, nf, nl, nc = mjx.count_constraints(m)
    self.assertEqual(ne, 0)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 0)
    self.assertEqual(nc, 0)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx.efc_J.shape[0], 0)

  def test_disable_equality(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.EQUALITY
    ne, nf, nl, nc = mjx.count_constraints(m)
    self.assertEqual(ne, 0)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 2)
    self.assertEqual(nc, 64)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx.efc_J.shape[0], 66)  # only joint range, contact

  def test_disable_contact(self):
    m = test_util.load_test_file('constraints.xml')
    m.opt.disableflags = m.opt.disableflags | mjx.DisableBit.CONTACT
    ne, nf, nl, nc = mjx.count_constraints(m)
    self.assertEqual(ne, 10)
    self.assertEqual(nf, 0)
    self.assertEqual(nl, 2)
    self.assertEqual(nc, 0)
    dx = constraint.make_constraint(mjx.put_model(m), mjx.make_data(m))
    self.assertEqual(dx.efc_J.shape[0], 12)  # only joint range, limit


if __name__ == '__main__':
  absltest.main()
