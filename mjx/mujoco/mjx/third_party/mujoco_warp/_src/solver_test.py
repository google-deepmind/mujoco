# Copyright 2025 The Newton Developers
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

"""Tests for solver functions."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import solver
from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.math import safe_div
from mujoco.mjx.third_party.mujoco_warp._src.types import ConeType
from mujoco.mjx.third_party.mujoco_warp._src.types import SolverType

# tolerance for difference between MuJoCo and MJWarp solver calculations - mostly
# due to float precision
_TOLERANCE = 5e-3


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 20  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class SolverTest(parameterized.TestCase):
  @parameterized.product(cone=tuple(ConeType), solver_=tuple(SolverType))
  def test_constraint_update(self, cone, solver_):
    """Tests _update_constraint function is correct."""
    for keyframe in range(3):
      mjm, mjd, m, d = test_util.fixture(
        "constraints.xml",
        keyframe=keyframe,
        cone=cone,
        solver=solver_,
        iterations=0,
      )

      def cost(qacc):
        jaref = np.zeros(mjd.nefc, dtype=float)
        cost = np.zeros(1)
        mujoco.mj_mulJacVec(mjm, mjd, jaref, qacc)
        mujoco.mj_constraintUpdate(mjm, mjd, jaref - mjd.efc_aref, cost, 0)
        return cost

      mjd_cost = cost(mjd.qacc)

      # solve with 0 iterations just initializes constraints and costs and then exits
      d.efc.force.zero_()
      d.qfrc_constraint.zero_()
      mjwarp.solve(m, d)

      # Get the ordering indices based on efc_force, efc_state for MJWarp
      nefc = d.nefc.numpy()[0]
      efc_force = d.efc.force.numpy()[0, :nefc]
      efc_state = d.efc.state.numpy()[0, :nefc]
      # Get the ordering indices based on efc_force, efc_state for MuJoCo
      mjd_efc_force = mjd.efc_force[:nefc]
      mjd_efc_state = mjd.efc_state[:nefc]

      # Create sorting keys using lexsort (more efficient for multiple keys)
      d_sort_indices = np.lexsort((efc_force, efc_state))
      mjd_sort_indices = np.lexsort((mjd_efc_force, mjd_efc_state))

      efc_cost = d.efc.cost.numpy()[0] - d.efc.gauss.numpy()[0]
      qfrc_constraint = d.qfrc_constraint.numpy()[0]

      efc_sorted_force = efc_force[d_sort_indices]
      efc_sorted_state = efc_state[d_sort_indices]
      mjd_sorted_force = mjd_efc_force[mjd_sort_indices]
      mjd_sorted_state = mjd_efc_state[mjd_sort_indices]

      _assert_eq(efc_sorted_state, mjd_sorted_state, "efc_state")
      _assert_eq(efc_sorted_force, mjd_sorted_force, "efc_force")
      _assert_eq(efc_cost, mjd_cost, "cost")
      _assert_eq(qfrc_constraint, mjd.qfrc_constraint, "qfrc_constraint")

  def test_init_linesearch(self):
    """Test linesearch initialization."""
    for keyframe in range(3):
      # TODO(team): Add the case of elliptic cone friction
      mjm, mjd, m, d = test_util.fixture(
        "constraints.xml",
        keyframe=keyframe,
        cone=ConeType.PYRAMIDAL,
        iterations=0,
        ls_iterations=0,
      )

      # One step to obtain more non-zeros results
      mjwarp.step(m, d)

      # Calculate target values
      efc_search_np = d.efc.search.numpy()[0]
      efc_J_np = d.efc.J.numpy()[0]
      efc_gauss_np = d.efc.gauss.numpy()[0]
      efc_Ma_np = d.efc.Ma.numpy()[0]
      efc_Jaref_np = d.efc.Jaref.numpy()[0]
      efc_D_np = d.efc.D.numpy()[0]
      qfrc_smooth_np = d.qfrc_smooth.numpy()[0]
      nefc = d.nefc.numpy()[0]

      target_mv = np.zeros(mjm.nv)
      mujoco.mj_mulM(mjm, mjd, target_mv, efc_search_np)
      target_jv = efc_J_np @ efc_search_np
      target_quad_gauss = np.array(
        [
          efc_gauss_np,
          np.dot(efc_search_np, efc_Ma_np - qfrc_smooth_np),
          0.5 * np.dot(efc_search_np, target_mv),
        ]
      )
      target_quad = np.transpose(
        np.vstack(
          [
            0.5 * efc_Jaref_np * efc_Jaref_np * efc_D_np,
            target_jv * efc_Jaref_np * efc_D_np,
            0.5 * target_jv * target_jv * efc_D_np,
          ]
        )
      )

      # launch linesearch with 0 iteration just doing the initialization step
      d.efc.jv.zero_()
      d.efc.quad.zero_()
      solver._linesearch(m, d)

      efc_mv = d.efc.mv.numpy()[0]
      efc_jv = d.efc.jv.numpy()[0]
      efc_quad_gauss = d.efc.quad_gauss.numpy()[0]
      efc_quad = d.efc.quad.numpy()[0]
      _assert_eq(efc_mv, target_mv, "mv")
      _assert_eq(efc_jv[:nefc], target_jv[:nefc], "jv")
      _assert_eq(efc_quad_gauss, target_quad_gauss, "quad_gauss")
      _assert_eq(efc_quad[:nefc], target_quad[:nefc], "quad")

  @parameterized.parameters(
    (ConeType.PYRAMIDAL, False),
    (ConeType.ELLIPTIC, False),
    (ConeType.PYRAMIDAL, True),
    (ConeType.ELLIPTIC, True),
  )
  def test_update_gradient_CG(self, cone, sparse):
    """Test _update_gradient function is correct for the CG solver."""
    mjm, mjd, m, d = test_util.fixture(
      "humanoid/humanoid.xml",
      cone=cone,
      solver=SolverType.CG,
      sparse=sparse,
      iterations=0,
      keyframe=0,
    )

    # Solve with 0 iterations just initializes and exit
    mjwarp.solve(m, d)

    # Calculate Mgrad with Mujoco C
    mj_Mgrad = np.zeros(shape=(1, mjm.nv), dtype=float)
    mj_grad = np.tile(d.efc.grad.numpy(), (1, 1))
    mujoco.mj_solveM(mjm, mjd, mj_Mgrad, mj_grad)

    efc_Mgrad = d.efc.Mgrad.numpy()[0]
    _assert_eq(efc_Mgrad, mj_Mgrad[0], name="Mgrad")

  @parameterized.parameters(ConeType.PYRAMIDAL, ConeType.ELLIPTIC)
  def test_parallel_linesearch(self, cone):
    """Test that iterative and parallel linesearch leads to equivalent results."""

    _, _, m, d = test_util.fixture(
      "humanoid/humanoid.xml",
      cone=cone,
      ls_parallel=False,
      iterations=50,
      ls_iterations=50,
    )

    # One step to obtain more non-zeros results
    mjwarp.step(m, d)

    # Preparing for linesearch
    m.opt.iterations = 0
    mjwarp.fwd_velocity(m, d)
    mjwarp.fwd_acceleration(m, d, factorize=True)
    solver.solve(m, d)

    # Storing some initial values
    d_efc_Ma = d.efc.Ma.numpy().copy()
    d_efc_Jaref = d.efc.Jaref.numpy().copy()
    d_qacc = d.qacc.numpy().copy()

    # Launching iterative linesearch
    m.opt.ls_parallel = False
    solver._linesearch(m, d)
    alpha_iterative = d.efc.alpha.numpy().copy()

    # Launching parallel linesearch with 10 testing points
    m.nlsp = 10
    d.efc.Ma = wp.array2d(d_efc_Ma)
    d.efc.Jaref = wp.array(d_efc_Jaref)
    d.qacc = wp.array2d(d_qacc)
    m.opt.ls_parallel = True
    solver._linesearch(m, d)
    alpha_parallel_10 = d.efc.alpha.numpy().copy()

    # Launching parallel linesearch with 50 testing points
    m.nlsp = 50
    d.efc.Ma = wp.array2d(d_efc_Ma)
    d.efc.Jaref = wp.array(d_efc_Jaref)
    d.qacc = wp.array2d(d_qacc)
    solver._linesearch(m, d)
    alpha_parallel_50 = d.efc.alpha.numpy().copy()

    # Checking that iterative and parallel linesearch lead to similar results
    # and that increasing ls_iterations leads to better results
    _assert_eq(alpha_iterative, alpha_parallel_50, name="linesearch alpha")
    self.assertLessEqual(abs(alpha_iterative - alpha_parallel_50), abs(alpha_iterative - alpha_parallel_10))

  @parameterized.parameters(
    (ConeType.PYRAMIDAL, SolverType.CG, 10, 5, False, False),
    (ConeType.ELLIPTIC, SolverType.CG, 10, 5, False, False),
    (ConeType.PYRAMIDAL, SolverType.NEWTON, 5, 10, False, False),
    (ConeType.ELLIPTIC, SolverType.NEWTON, 5, 10, False, False),
    (ConeType.PYRAMIDAL, SolverType.NEWTON, 5, 64, True, True),
    (ConeType.ELLIPTIC, SolverType.NEWTON, 5, 64, True, True),
  )
  def test_solve(self, cone, solver_, iterations, ls_iterations, sparse, ls_parallel):
    """Tests solve."""
    for keyframe in range(3):
      mjm, mjd, m, d = test_util.fixture(
        "constraints.xml",
        keyframe=keyframe,
        sparse=sparse,
        cone=cone,
        solver=solver_,
        iterations=iterations,
        ls_iterations=ls_iterations,
        ls_parallel=ls_parallel,
      )

      mujoco.mj_forward(mjm, mjd)

      d.qacc.zero_()
      d.qfrc_constraint.zero_()
      d.efc.force.zero_()

      if solver_ == mujoco.mjtSolver.mjSOL_CG:
        mjwarp.factor_m(m, d)
      mjwarp.solve(m, d)

      def cost(qacc):
        jaref = np.zeros(mjd.nefc, dtype=float)
        cost = np.zeros(1)
        mujoco.mj_mulJacVec(mjm, mjd, jaref, qacc)
        mujoco.mj_constraintUpdate(mjm, mjd, jaref - mjd.efc_aref, cost, 0)
        return cost

      mj_cost = cost(mjd.qacc)
      mjwarp_cost = cost(d.qacc.numpy()[0])
      self.assertLessEqual(mjwarp_cost, mj_cost * 1.025)

      if m.opt.solver == mujoco.mjtSolver.mjSOL_NEWTON:
        _assert_eq(d.qacc.numpy()[0], mjd.qacc, "qacc")
        _assert_eq(d.qfrc_constraint.numpy()[0], mjd.qfrc_constraint, "qfrc_constraint")
        _assert_eq(d.efc.force.numpy()[0, : mjd.nefc], mjd.efc_force, "efc_force")

  @parameterized.parameters(
    (ConeType.PYRAMIDAL, SolverType.CG, 25, 5),
    (ConeType.PYRAMIDAL, SolverType.NEWTON, 2, 4),
  )
  def test_solve_batch(self, cone, solver_, iterations, ls_iterations):
    """Tests solve (batch)."""

    mjm0, mjd0, _, _ = test_util.fixture(
      "humanoid/humanoid.xml",
      keyframe=0,
      sparse=False,
      cone=cone,
      solver=solver_,
      iterations=iterations,
      ls_iterations=ls_iterations,
    )
    qacc_warmstart0 = mjd0.qacc_warmstart.copy()
    mujoco.mj_forward(mjm0, mjd0)
    mjd0.qacc_warmstart = qacc_warmstart0

    mjm1, mjd1, _, _ = test_util.fixture(
      "humanoid/humanoid.xml",
      keyframe=2,
      sparse=False,
      cone=cone,
      solver=solver_,
      iterations=iterations,
      ls_iterations=ls_iterations,
    )
    qacc_warmstart1 = mjd1.qacc_warmstart.copy()
    mujoco.mj_forward(mjm1, mjd1)
    mjd1.qacc_warmstart = qacc_warmstart1

    mjm2, mjd2, _, _ = test_util.fixture(
      "humanoid/humanoid.xml",
      keyframe=1,
      sparse=False,
      cone=cone,
      solver=solver_,
      iterations=iterations,
      ls_iterations=ls_iterations,
    )
    qacc_warmstart2 = mjd2.qacc_warmstart.copy()
    mujoco.mj_forward(mjm2, mjd2)
    mjd2.qacc_warmstart = qacc_warmstart2

    nefc_active = mjd0.nefc + mjd1.nefc + mjd2.nefc
    ne_active = mjd0.ne + mjd1.ne + mjd2.ne

    mjm, mjd, m, _ = test_util.fixture(
      "humanoid/humanoid.xml",
      sparse=False,
      cone=cone,
      solver=solver_,
      iterations=iterations,
      ls_iterations=ls_iterations,
    )
    d = mjwarp.put_data(mjm, mjd, nworld=3, njmax=2 * nefc_active)

    d.nefc = wp.array([nefc_active, nefc_active, nefc_active], dtype=wp.int32, ndim=1)
    d.ne = wp.array([ne_active, ne_active, ne_active], dtype=wp.int32, ndim=1)

    qacc_warmstart = np.vstack(
      [
        np.expand_dims(qacc_warmstart0, axis=0),
        np.expand_dims(qacc_warmstart1, axis=0),
        np.expand_dims(qacc_warmstart2, axis=0),
      ]
    )

    qM0 = np.zeros((mjm0.nv, mjm0.nv))
    mujoco.mj_fullM(mjm0, qM0, mjd0.qM)
    qM1 = np.zeros((mjm1.nv, mjm1.nv))
    mujoco.mj_fullM(mjm1, qM1, mjd1.qM)
    qM2 = np.zeros((mjm2.nv, mjm2.nv))
    mujoco.mj_fullM(mjm2, qM2, mjd2.qM)

    qM = np.vstack(
      [
        np.expand_dims(qM0, axis=0),
        np.expand_dims(qM1, axis=0),
        np.expand_dims(qM2, axis=0),
      ]
    )
    qacc_smooth = np.vstack(
      [
        np.expand_dims(mjd0.qacc_smooth, axis=0),
        np.expand_dims(mjd1.qacc_smooth, axis=0),
        np.expand_dims(mjd2.qacc_smooth, axis=0),
      ]
    )
    qfrc_smooth = np.vstack(
      [
        np.expand_dims(mjd0.qfrc_smooth, axis=0),
        np.expand_dims(mjd1.qfrc_smooth, axis=0),
        np.expand_dims(mjd2.qfrc_smooth, axis=0),
      ]
    )

    # Reshape the Jacobians
    efc_J0 = mjd0.efc_J.reshape((mjd0.nefc, mjm0.nv))
    efc_J1 = mjd1.efc_J.reshape((mjd1.nefc, mjm1.nv))
    efc_J2 = mjd2.efc_J.reshape((mjd2.nefc, mjm2.nv))

    efc_J_fill = np.zeros((3, d.njmax, m.nv))
    efc_J_fill[0, : mjd0.nefc, :] = efc_J0
    efc_J_fill[1, : mjd1.nefc, :] = efc_J1
    efc_J_fill[2, : mjd2.nefc, :] = efc_J2

    # Similarly for D and aref values
    efc_D0 = mjd0.efc_D[: mjd0.nefc]
    efc_D1 = mjd1.efc_D[: mjd1.nefc]
    efc_D2 = mjd2.efc_D[: mjd2.nefc]

    efc_D_fill = np.zeros((3, d.njmax))
    efc_D_fill[0, : mjd0.nefc] = efc_D0
    efc_D_fill[1, : mjd1.nefc] = efc_D1
    efc_D_fill[2, : mjd2.nefc] = efc_D2

    efc_aref0 = mjd0.efc_aref[: mjd0.nefc]
    efc_aref1 = mjd1.efc_aref[: mjd1.nefc]
    efc_aref2 = mjd2.efc_aref[: mjd2.nefc]

    efc_aref_fill = np.zeros((3, d.njmax))
    efc_aref_fill[0, : mjd0.nefc] = efc_aref0
    efc_aref_fill[1, : mjd1.nefc] = efc_aref1
    efc_aref_fill[2, : mjd2.nefc] = efc_aref2

    d.qacc_warmstart = wp.from_numpy(qacc_warmstart, dtype=wp.float32)
    d.qM = wp.from_numpy(qM, dtype=wp.float32)
    d.qacc_smooth = wp.from_numpy(qacc_smooth, dtype=wp.float32)
    d.qfrc_smooth = wp.from_numpy(qfrc_smooth, dtype=wp.float32)
    d.efc.J = wp.from_numpy(efc_J_fill, dtype=wp.float32)
    d.efc.D = wp.from_numpy(efc_D_fill, dtype=wp.float32)
    d.efc.aref = wp.from_numpy(efc_aref_fill, dtype=wp.float32)

    if solver_ == SolverType.CG:
      m0 = mjwarp.put_model(mjm0)
      d0 = mjwarp.put_data(mjm0, mjd0)
      mjwarp.factor_m(m0, d0)
      qLD0 = d0.qLD.numpy()

      m1 = mjwarp.put_model(mjm1)
      d1 = mjwarp.put_data(mjm1, mjd1)
      mjwarp.factor_m(m1, d1)
      qLD1 = d1.qLD.numpy()

      m2 = mjwarp.put_model(mjm2)
      d2 = mjwarp.put_data(mjm2, mjd2)
      mjwarp.factor_m(m2, d2)
      qLD2 = d2.qLD.numpy()

      qLD = np.vstack([qLD0, qLD1, qLD2])
      d.qLD = wp.from_numpy(qLD, dtype=wp.float32)

    d.qacc.zero_()
    d.qfrc_constraint.zero_()
    d.efc.force.zero_()
    solver.solve(m, d)

    def cost(m, d, qacc):
      jaref = np.zeros(d.nefc, dtype=float)
      cost = np.zeros(1)
      mujoco.mj_mulJacVec(m, d, jaref, qacc)
      mujoco.mj_constraintUpdate(m, d, jaref - d.efc_aref, cost, 0)
      return cost

    mj_cost0 = cost(mjm0, mjd0, mjd0.qacc)
    mjwarp_cost0 = cost(mjm0, mjd0, d.qacc.numpy()[0])
    self.assertLessEqual(mjwarp_cost0, mj_cost0 * 1.025)

    mj_cost1 = cost(mjm1, mjd1, mjd1.qacc)
    mjwarp_cost1 = cost(mjm1, mjd1, d.qacc.numpy()[1])
    self.assertLessEqual(mjwarp_cost1, mj_cost1 * 1.025)

    mj_cost2 = cost(mjm2, mjd2, mjd2.qacc)
    mjwarp_cost2 = cost(mjm2, mjd2, d.qacc.numpy()[2])
    self.assertLessEqual(mjwarp_cost2, mj_cost2 * 1.025)

    if m.opt.solver == SolverType.NEWTON:
      _assert_eq(d.qacc.numpy()[0], mjd0.qacc, "qacc0")
      _assert_eq(d.qacc.numpy()[1], mjd1.qacc, "qacc1")
      _assert_eq(d.qacc.numpy()[2], mjd2.qacc, "qacc2")

      _assert_eq(d.qfrc_constraint.numpy()[0], mjd0.qfrc_constraint, "qfrc_constraint0")
      _assert_eq(d.qfrc_constraint.numpy()[1], mjd1.qfrc_constraint, "qfrc_constraint1")
      _assert_eq(d.qfrc_constraint.numpy()[2], mjd2.qfrc_constraint, "qfrc_constraint2")

      # Get world 0 forces - equality constraints at start, inequality constraints later
      nieq0 = mjd0.nefc - mjd0.ne
      nieq1 = mjd1.nefc - mjd1.ne
      nieq2 = mjd2.nefc - mjd2.ne
      world0_eq_forces = d.efc.force.numpy()[0, : mjd0.ne]
      world0_ineq_forces = d.efc.force.numpy()[0, ne_active : ne_active + nieq0]
      world0_forces = np.concatenate([world0_eq_forces, world0_ineq_forces])
      _assert_eq(world0_forces, mjd0.efc_force, "efc_force0")

      # Get world 1 forces
      world1_eq_forces = d.efc.force.numpy()[1, : mjd1.ne]
      world1_ineq_forces = d.efc.force.numpy()[1, ne_active : ne_active + nieq1]
      world1_forces = np.concatenate([world1_eq_forces, world1_ineq_forces])
      _assert_eq(world1_forces, mjd1.efc_force, "efc_force1")

      # Get world 2 forces
      world2_eq_forces = d.efc.force.numpy()[2, : mjd2.ne]
      world2_ineq_forces = d.efc.force.numpy()[2, ne_active : ne_active + nieq2]
      world2_forces = np.concatenate([world2_eq_forces, world2_ineq_forces])
      _assert_eq(world2_forces, mjd2.efc_force, "efc_force2")

  def test_frictionloss(self):
    """Tests solver with frictionloss."""
    for keyframe in range(3):
      _, mjd, m, d = test_util.fixture("constraints.xml", keyframe=keyframe)
      mjwarp.solve(m, d)

      _assert_eq(d.nf.numpy()[0], mjd.nf, "nf")
      _assert_eq(d.qacc.numpy()[0], mjd.qacc, "qacc")
      _assert_eq(d.qfrc_constraint.numpy()[0], mjd.qfrc_constraint, "qfrc_constraint")
      _assert_eq(d.efc.force.numpy()[0, : mjd.nefc], mjd.efc_force, "efc_force")


if __name__ == "__main__":
  wp.init()
  absltest.main()
