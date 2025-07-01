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
import mujoco
from mujoco import mjx
from mujoco.mjx._src import solver
from mujoco.mjx._src import test_util
from mujoco.mjx._src.types import ConeType
import numpy as np


# tolerance for difference between MuJoCo and MJX solver calculations,
# mostly due to float precision
_TOLERANCE = 5e-3


def _assert_eq(a, b, name, tol=_TOLERANCE):
  tol = tol * 10  # avoid test noise
  err_msg = f'mismatch: {name}'
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


def _assert_attr_eq(a, b, attr, tol=_TOLERANCE):
  _assert_eq(getattr(a, attr), getattr(b, attr), attr, tol=tol)


class SolverTest(parameterized.TestCase):

  @parameterized.parameters(
      # these scene challenges the solver, with CG you need to crank up
      # the iterations, otherwise it diverges
      (mujoco.mjtSolver.mjSOL_CG, mujoco.mjtCone.mjCONE_PYRAMIDAL, 100),
      (mujoco.mjtSolver.mjSOL_CG, mujoco.mjtCone.mjCONE_ELLIPTIC, 100),
      # Newton converges much more quickly, lower iterations to demonstrate
      # mgrad is being calculated optimally
      (mujoco.mjtSolver.mjSOL_NEWTON, mujoco.mjtCone.mjCONE_PYRAMIDAL, 2),
      (mujoco.mjtSolver.mjSOL_NEWTON, mujoco.mjtCone.mjCONE_ELLIPTIC, 2),
  )
  def test_solver(self, solver_, cone, iterations):
    """Test newton, CG solver with pyramidal, elliptic cones."""
    m = test_util.load_test_file('constraints.xml')
    m.opt.solver = solver_
    m.opt.cone = cone
    m.opt.iterations = iterations
    d = mujoco.MjData(m)

    def cost(qacc):
      jaref = np.zeros(d.nefc, dtype=float)
      cost = np.zeros(1)
      mujoco.mj_mulJacVec(m, d, jaref, qacc)
      mujoco.mj_constraintUpdate(m, d, jaref - d.efc_aref, cost, 0)
      return cost

    # sample a mix of active/inactive constraints at different timesteps
    for key in range(0, 3):
      mujoco.mj_resetDataKeyframe(m, d, key)
      mujoco.mj_step(m, d)  # step to generate warmstart

      # compare costs
      mj_cost = cost(d.qacc)
      ctx = solver.Context.create(mjx.put_model(m), mjx.put_data(m, d))
      mjx_cost = ctx.cost - ctx.gauss
      _assert_eq(mj_cost, mjx_cost, 'cost')

      # mj_forward overwrites qacc_warmstart, so let's restore it to what it was
      # before the step so that MJX does not have a trivial solution
      warmstart = d.qacc_warmstart.copy()
      mujoco.mj_forward(m, d)
      d.qacc_warmstart = warmstart
      dx = jax.jit(mjx.solve)(mjx.put_model(m), mjx.put_data(m, d))

      # MJX finds very similar solutions with the newton solver
      if solver_ == mujoco.mjtSolver.mjSOL_NEWTON:
        nnz = dx._impl.efc_J.any(axis=1)
        _assert_eq(d.efc_force, dx._impl.efc_force[nnz], 'efc_force')
        _assert_attr_eq(d, dx, 'qfrc_constraint')
        _assert_attr_eq(d, dx, 'qacc')

      # both CG and Newton find costs that are nearly the same as MuJoCo, often
      # lower (due to slight differences in the MJX linsearch algorithm)
      mj_cost = cost(d.qacc)
      mjx_cost = cost(dx.qacc)
      self.assertLess(mjx_cost, mj_cost * 1.015)

  def test_no_warmstart(self):
    """Test no warmstart."""
    m = test_util.load_test_file('constraints.xml')
    d = mujoco.MjData(m)
    # significant constraint forces keyframe 2
    mujoco.mj_resetDataKeyframe(m, d, 2)
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_WARMSTART
    mujoco.mj_forward(m, d)
    mx = mjx.put_model(m)
    dx = jax.jit(mjx.solve)(mx, mjx.put_data(m, d))
    nnz = dx._impl.efc_J.any(axis=1)
    # even without warmstart, newton converges quickly
    _assert_eq(d.efc_force, dx._impl.efc_force[nnz], 'efc_force', tol=2e-4)

  def test_sparse(self):
    """Test solver works with sparse mass matrices."""
    m = test_util.load_test_file('constraints.xml')
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    # significant constraint forces keyframe 2
    mujoco.mj_resetDataKeyframe(m, d, 2)

    # mj_forward overwrites qacc_warmstart, so let's restore it to what it was
    # at the beginning of the step so that MJX does not have a trivial solution
    warmstart = d.qacc_warmstart.copy()
    mujoco.mj_forward(m, d)
    d.qacc_warmstart = warmstart

    dx = jax.jit(mjx.solve)(mjx.put_model(m), mjx.put_data(m, d))

    _assert_attr_eq(d, dx, 'qacc')
    _assert_attr_eq(d, dx, 'qfrc_constraint')
    nnz = dx._impl.efc_J.any(axis=1)
    _assert_eq(d.efc_force, dx._impl.efc_force[nnz], 'efc_force')

  def test_quad_frictionloss(self):
    """Test a case with quadratic frictionloss constraints."""
    m = test_util.load_test_file('quadratic_frictionloss.xml')
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)

    dx = jax.jit(mjx.solve)(mjx.put_model(m), mjx.put_data(m, d))

    _assert_attr_eq(d, dx, 'qacc')
    _assert_attr_eq(d, dx, 'qfrc_constraint')
    nnz = dx._impl.efc_J.any(axis=1)
    _assert_eq(d.efc_force, dx._impl.efc_force[nnz], 'efc_force')

  # TODO(taylorhowell): condim=1 with ConeType.ELLIPTIC
  @parameterized.product(condim=(3, 4, 6), cone=tuple(ConeType))
  def test_condim(self, condim, cone):
    """Test contact dimension."""
    m = mujoco.MjModel.from_xml_string(f"""
       <mujoco>
          <worldbody>
            <geom size="0 0 1e-5" type="plane" condim="1"/>
            <body pos="0 0 0.09">
              <freejoint/>
              <geom size="0.1" condim="{condim}"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    m.opt.cone = cone
    solver.solve(mjx.put_model(m), mjx.put_data(m, mujoco.MjData(m)))


if __name__ == '__main__':
  absltest.main()
