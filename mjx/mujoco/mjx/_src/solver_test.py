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
import jax
import mujoco
from mujoco import mjx
from mujoco.mjx._src import test_util
import numpy as np


# tolerance for difference between MuJoCo and MJX constraint calculations,
# mostly due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name, tol=_TOLERANCE):
  tol = tol * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr)


class SolverTest(absltest.TestCase):

  def test_newton(self):
    """Test newton solver."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 100)  # at 100 steps mix of active/inactive constraints
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.solve)(mx, mjx.put_data(m, d))

    _assert_attr_eq(d, dx, 'qacc_warmstart')
    _assert_attr_eq(d, dx, 'qacc')
    _assert_attr_eq(d, dx, 'qfrc_constraint')
    nnz = dx.efc_J.any(axis=1)
    _assert_eq(d.efc_force, dx.efc_force[nnz], 'efc_force')

  def test_cg(self):
    """Test CG solver."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 100)  # at 100 steps mix of active/inactive constraints
    m.opt.solver = mujoco.mjtSolver.mjSOL_CG
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.solve)(mx, mjx.put_data(m, d))

    _assert_attr_eq(d, dx, 'qacc_warmstart')
    _assert_attr_eq(d, dx, 'qacc')
    _assert_attr_eq(d, dx, 'qfrc_constraint')
    nnz = dx.efc_J.any(axis=1)
    _assert_eq(d.efc_force, dx.efc_force[nnz], 'efc_force')

  def test_no_warmstart(self):
    """Test no warmstart."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 100)  # at 100 steps mix of active/inactive constraints
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_WARMSTART
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.solve)(mx, mjx.put_data(m, d))
    nnz = dx.efc_J.any(axis=1)
    # without warmstart, the solution is not as close
    _assert_eq(d.efc_force, dx.efc_force[nnz], 'efc_force', tol=2e-2)

  def test_dense(self):
    """Test solver works with dense mass matrices."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 100)  # at 100 steps mix of active/inactive constraints
    mujoco.mj_forward(m, d)
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.solve)(mx, mjx.put_data(m, d))

    _assert_attr_eq(d, dx, 'qacc_warmstart')
    _assert_attr_eq(d, dx, 'qacc')
    _assert_attr_eq(d, dx, 'qfrc_constraint')
    nnz = dx.efc_J.any(axis=1)
    _assert_eq(d.efc_force, dx.efc_force[nnz], 'efc_force')

if __name__ == '__main__':
  absltest.main()
