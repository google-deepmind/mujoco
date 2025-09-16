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

"""Tests for forward dynamics functions."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src.types import BiasType
from mujoco.mjx.third_party.mujoco_warp._src.types import GainType
from mujoco.mjx.third_party.mujoco_warp._src.types import IntegratorType

# tolerance for difference between MuJoCo and mjwarp smooth calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class ForwardTest(parameterized.TestCase):
  # TODO(team): test sparse when actuator_moment and/or ten_J have sparse representation
  @parameterized.product(xml=["humanoid/humanoid.xml", "pendula.xml"])
  def test_fwd_velocity(self, xml):
    _, mjd, m, d = test_util.fixture(xml, kick=True)

    for arr in (d.actuator_velocity, d.qfrc_bias):
      arr.zero_()

    mjwarp.fwd_velocity(m, d)

    _assert_eq(d.actuator_velocity.numpy()[0], mjd.actuator_velocity, "actuator_velocity")
    _assert_eq(d.qfrc_bias.numpy()[0], mjd.qfrc_bias, "qfrc_bias")

  def test_fwd_velocity_tendon(self):
    _, mjd, m, d = test_util.fixture("tendon/fixed.xml", sparse=False)

    d.ten_velocity.zero_()
    mjwarp.fwd_velocity(m, d)

    _assert_eq(d.ten_velocity.numpy()[0], mjd.ten_velocity, "ten_velocity")

  @parameterized.parameters(
    ("actuation/actuation.xml", True),
    ("actuation/actuation.xml", False),
    ("actuation/actuators.xml", True),
    ("actuation/actuators.xml", False),
    ("actuation/muscle.xml", True),
    ("actuation/muscle.xml", False),
  )
  def test_actuation(self, xml, actuation):
    mjm, mjd, m, d = test_util.fixture(xml, actuation=actuation, keyframe=0)

    for arr in (d.qfrc_actuator, d.actuator_force, d.act_dot):
      arr.zero_()

    mjwarp.fwd_actuation(m, d)

    _assert_eq(d.qfrc_actuator.numpy()[0], mjd.qfrc_actuator, "qfrc_actuator")
    _assert_eq(d.actuator_force.numpy()[0], mjd.actuator_force, "actuator_force")

    if mjm.na:
      _assert_eq(d.act_dot.numpy()[0], mjd.act_dot, "act_dot")

      # next activations
      mujoco.mj_step(mjm, mjd)
      mjwarp.step(m, d)

      _assert_eq(d.act.numpy()[0], mjd.act, "act")

    # TODO(team): test actearly

  @parameterized.parameters(True, False)
  def test_clampctrl(self, clampctrl):
    _, mjd, _, d = test_util.fixture(
      xml="""
    <mujoco>
      <worldbody>
        <body>
          <joint name="joint" type="slide"/>
          <geom type="sphere" size=".1"/>
        </body>
      </worldbody>
      <actuator>
        <motor joint="joint" ctrlrange="-1 1"/>
      </actuator>
      <keyframe>
        <key ctrl="2"/>
      </keyframe>
    </mujoco>
    """,
      clampctrl=clampctrl,
      keyframe=0,
    )

    _assert_eq(d.ctrl.numpy()[0], mjd.ctrl, "ctrl")

  def test_fwd_acceleration(self):
    _, mjd, m, d = test_util.fixture("humanoid/humanoid.xml", kick=True)

    for arr in (d.qfrc_smooth, d.qacc_smooth):
      arr.zero_()

    mjwarp.fwd_acceleration(m, d)

    _assert_eq(d.qfrc_smooth.numpy()[0], mjd.qfrc_smooth, "qfrc_smooth")
    _assert_eq(d.qacc_smooth.numpy()[0], mjd.qacc_smooth, "qacc_smooth")

  @parameterized.parameters((True, True), (True, False), (False, True), (False, False))
  def test_euler(self, eulerdamp, sparse):
    mjm, mjd, _, _ = test_util.fixture("pendula.xml", kick=True, eulerdamp=eulerdamp, sparse=sparse)
    self.assertTrue((mjm.dof_damping > 0).any())

    m = mjwarp.put_model(mjm)
    d = mjwarp.put_data(mjm, mjd)
    mujoco.mj_forward(mjm, mjd)

    mujoco.mj_Euler(mjm, mjd)

    mjwarp.solve(m, d)  # compute efc.Ma
    mjwarp.euler(m, d)

    _assert_eq(d.qpos.numpy()[0], mjd.qpos, "qpos")
    _assert_eq(d.qvel.numpy()[0], mjd.qvel, "qvel")
    _assert_eq(d.act.numpy()[0], mjd.act, "act")

  def test_rungekutta4(self):
    mjm, mjd, m, d = test_util.fixture(
      xml="""
        <mujoco>
          <option integrator="RK4" iterations="1" ls_iterations="1">
            <flag constraint="disable"/>
          </option>
          <worldbody>
            <body>
              <joint type="hinge"/>
              <geom type="sphere" size=".1"/>
              <body pos="0.1 0 0">
                <joint type="hinge"/>
                <geom type="sphere" size=".1"/>
              </body>
            </body>
          </worldbody>
          <keyframe>
            <key qpos=".1 .2" qvel=".025 .05"/>
          </keyframe>
        </mujoco>
        """,
      keyframe=0,
    )

    mjwarp.rungekutta4(m, d)
    mujoco.mj_RungeKutta(mjm, mjd, 4)

    _assert_eq(d.qpos.numpy()[0], mjd.qpos, "qpos")
    _assert_eq(d.qvel.numpy()[0], mjd.qvel, "qvel")
    _assert_eq(d.time.numpy()[0], mjd.time, "time")
    _assert_eq(d.xpos.numpy()[0], mjd.xpos, "xpos")

    # test rungekutta determinism
    def rk_step() -> wp.array(dtype=wp.float32, ndim=2):
      d.qpos = wp.ones_like(d.qpos)
      d.qvel = wp.ones_like(d.qvel)
      d.act = wp.ones_like(d.act)
      mjwarp.rungekutta4(m, d)
      return d.qpos

    _assert_eq(rk_step().numpy()[0], rk_step().numpy()[0], "qpos")

  @parameterized.product(actuation=[True, False], passive=[True, False], sparse=[True, False])
  def test_implicit(self, actuation, passive, sparse):
    mjm, mjd, _, _ = test_util.fixture(
      "pendula.xml",
      integrator=IntegratorType.IMPLICITFAST,
      actuation=actuation,
      spring=passive,
      damper=passive,
      sparse=sparse,
    )

    mjm.actuator_gainprm[:, 2] = np.random.uniform(low=0.01, high=10.0, size=mjm.actuator_gainprm[:, 2].shape)

    # change actuators to velocity/damper to cover all codepaths
    mjm.actuator_gaintype[3] = GainType.AFFINE
    mjm.actuator_gaintype[6] = GainType.AFFINE
    mjm.actuator_biastype[0:3] = BiasType.AFFINE
    mjm.actuator_biastype[4:6] = BiasType.AFFINE
    mjm.actuator_biasprm[0:3, 2] = -1.0
    mjm.actuator_biasprm[4:6, 2] = -1.0
    mjm.actuator_ctrlrange[3:7] = 10.0
    mjm.actuator_gear[:] = 1.0

    mjd.qvel = np.random.uniform(low=-0.01, high=0.01, size=mjd.qvel.shape)
    mjd.ctrl = np.random.uniform(low=-0.1, high=0.1, size=mjd.ctrl.shape)
    mjd.act = np.random.uniform(low=-0.1, high=0.1, size=mjd.act.shape)
    mujoco.mj_forward(mjm, mjd)

    m = mjwarp.put_model(mjm)
    d = mjwarp.put_data(mjm, mjd)

    mujoco.mj_implicit(mjm, mjd)

    mjwarp.solve(m, d)  # compute efc.Ma
    mjwarp.implicit(m, d)

    _assert_eq(d.qpos.numpy()[0], mjd.qpos, "qpos")
    _assert_eq(d.act.numpy()[0], mjd.act, "act")

  def test_implicit_position(self):
    mjm, mjd, m, d = test_util.fixture("actuation/position.xml", keyframe=0, integrator=IntegratorType.IMPLICITFAST, kick=True)

    mujoco.mj_implicit(mjm, mjd)

    mjwarp.solve(m, d)  # compute efc.Ma
    mjwarp.implicit(m, d)

    _assert_eq(d.qpos.numpy()[0], mjd.qpos, "qpos")
    _assert_eq(d.qvel.numpy()[0], mjd.qvel, "qvel")

  def test_implicit_tendon_damping(self):
    mjm, mjd, m, d = test_util.fixture("tendon/damping.xml", keyframe=0, integrator=IntegratorType.IMPLICITFAST, kick=True)

    mujoco.mj_implicit(mjm, mjd)

    mjwarp.solve(m, d)  # compute efc.Ma
    mjwarp.implicit(m, d)

    _assert_eq(d.qpos.numpy()[0], mjd.qpos, "qpos")
    _assert_eq(d.qvel.numpy()[0], mjd.qvel, "qvel")

  @parameterized.product(
    xml=("humanoid/humanoid.xml", "pendula.xml", "constraints.xml", "collision.xml"), graph_conditional=(True, False)
  )
  def test_graph_capture(self, xml, graph_conditional):
    # TODO(team): test more environments
    if wp.get_device().is_cuda and wp.config.verify_cuda == False:
      _, _, m, d = test_util.fixture(xml)
      m.opt.graph_conditional = graph_conditional

      with wp.ScopedCapture() as capture:
        mjwarp.step(m, d)

      # step a few times to ensure no errors at the step boundary
      wp.capture_launch(capture.graph)
      wp.capture_launch(capture.graph)
      wp.capture_launch(capture.graph)

      self.assertTrue(d.time.numpy()[0] > 0.0)

  def test_forward_energy(self):
    _, mjd, _, d = test_util.fixture("humanoid/humanoid.xml", kick=True, energy=True)

    _assert_eq(d.energy.numpy()[0][0], mjd.energy[0], "potential energy")
    _assert_eq(d.energy.numpy()[0][1], mjd.energy[1], "kinetic energy")

  def test_tendon_actuator_force_limits(self):
    for keyframe in range(7):
      _, mjd, m, d = test_util.fixture("actuation/tendon_force_limit.xml", keyframe=keyframe)

      d.actuator_force.zero_()

      mjwarp.forward(m, d)

      _assert_eq(d.actuator_force.numpy()[0], mjd.actuator_force, "actuator_force")

  @parameterized.parameters(("humanoid/humanoid.xml", True), ("humanoid/humanoid.xml", False))
  def test_step1(self, xml, energy):
    # TODO(team): test more mjcfs
    mjm, mjd, m, d = test_util.fixture(xml, kick=True, energy=energy)

    # some of the fields updated by step1
    step1_field = [
      "xpos",
      "xquat",
      "xmat",
      "xipos",
      "ximat",
      "xanchor",
      "xaxis",
      "geom_xpos",
      "geom_xmat",
      "site_xmat",
      "subtree_com",
      "cinert",
      "cdof",
      "cam_xpos",
      "cam_xmat",
      "light_xpos",
      "light_xdir",
      "ten_length",
      "ten_J",
      "ten_wrapadr",
      "ten_wrapnum",
      "wrap_obj",
      "wrap_xpos",
      "qM",
      "qLD",
      "nefc",
      "efc_type",
      "efc_id",
      "efc_J",
      "efc_pos",
      "efc_margin",
      "efc_D",
      "efc_vel",
      "efc_aref",
      "efc_frictionloss",
      "actuator_length",
      "actuator_moment",
      "actuator_velocity",
      "ten_velocity",
      "cvel",
      "cdof_dot",
      "qfrc_spring",
      "qfrc_damper",
      "qfrc_gravcomp",
      "qfrc_fluid",
      "qfrc_passive",
      "qfrc_bias",
      "energy",
    ]
    if m.nflexvert:
      step1_field += ["flexvert_xpos"]
    if m.nflexedge:
      step1_field += ["flexedge_length", "flexedge_velocity"]

    def _getattr(arr):
      if (len(arr) >= 4) & (arr[:4] == "efc_"):
        return getattr(d.efc, arr[4:]), True
      return getattr(d, arr), False

    for arr in step1_field:
      attr, _ = _getattr(arr)
      if attr.dtype == float:
        attr.fill_(wp.nan)
      elif attr.dtype == int:
        attr.fill_(-1)
      else:
        attr.zero_()

    mujoco.mj_step1(mjm, mjd)
    mjwarp.step1(m, d)

    for arr in step1_field:
      d_arr, is_nefc = _getattr(arr)
      d_arr = d_arr.numpy()[0]
      mjd_arr = getattr(mjd, arr)
      if arr in ["xmat", "ximat", "geom_xmat", "site_xmat", "cam_xmat"]:
        mjd_arr = mjd_arr.reshape(-1)
        d_arr = d_arr.reshape(-1)
      elif arr == "qM":
        qM = np.zeros((mjm.nv, mjm.nv))
        mujoco.mj_fullM(mjm, qM, mjd.qM)
        mjd_arr = qM
      elif arr == "actuator_moment":
        actuator_moment = np.zeros((mjm.nu, mjm.nv))
        mujoco.mju_sparse2dense(actuator_moment, mjd.actuator_moment, mjd.moment_rownnz, mjd.moment_rowadr, mjd.moment_colind)
        mjd_arr = actuator_moment
      elif arr == "ten_J" and mjm.ntendon:
        ten_J = np.zeros((mjm.ntendon, mjm.nv))
        mujoco.mju_sparse2dense(ten_J, mjd.ten_J, mjd.ten_J_rownnz, mjd.ten_J_rowadr, mjd.ten_J_colind)
        mjd_arr = ten_J
      elif arr == "efc_J":
        if mjd.efc_J.shape[0] != mjd.nefc * mjm.nv:
          efc_J = np.zeros((mjd.nefc, mjm.nv))
          mujoco.mju_sparse2dense(efc_J, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
          mjd_arr = efc_J
        else:
          mjd_arr = mjd_arr.reshape((mjd.nefc, mjm.nv))
      elif arr == "qLD":
        vec = np.ones((1, mjm.nv))
        res = np.zeros((1, mjm.nv))
        mujoco.mj_solveM(mjm, mjd, res, vec)

        vec_wp = wp.array(vec, dtype=float)
        res_wp = wp.zeros((1, mjm.nv), dtype=float)
        mjwarp.solve_m(m, d, res_wp, vec_wp)

        d_arr = res_wp.numpy()[0]
        mjd_arr = res[0]
      if is_nefc:
        d_arr = d_arr[: d.nefc.numpy()[0]]

      _assert_eq(d_arr, mjd_arr, arr)

    # TODO(team): sensor_pos
    # TODO(team): sensor_vel

  @parameterized.parameters(
    ("humanoid/humanoid.xml", IntegratorType.EULER),
    ("humanoid/humanoid.xml", IntegratorType.IMPLICITFAST),
    ("humanoid/humanoid.xml", IntegratorType.RK4),
  )
  def test_step2(self, xml, integrator):
    mjm, mjd, m, _ = test_util.fixture(xml, kick=True, integrator=integrator)

    # some of the fields updated by step2
    step2_field = [
      "act_dot",
      "actuator_force",
      "qfrc_actuator",
      "qfrc_smooth",
      "qacc",
      "qacc_warmstart",
      "qvel",
      "qpos",
      "efc_force",
      "qfrc_constraint",
    ]

    def _getattr(arr):
      if (len(arr) >= 4) & (arr[:4] == "efc_"):
        return getattr(d.efc, arr[4:]), True
      return getattr(d, arr), False

    mujoco.mj_step1(mjm, mjd)

    # input
    ctrl = 0.1 * np.random.rand(mjm.nu)
    qfrc_applied = 0.1 * np.random.rand(mjm.nv)
    xfrc_applied = 0.1 * np.random.rand(mjm.nbody, 6)

    mjd.ctrl = ctrl
    mjd.qfrc_applied = qfrc_applied
    mjd.xfrc_applied = xfrc_applied

    d = mjwarp.put_data(mjm, mjd)

    for arr in step2_field:
      if arr in ["qpos", "qvel"]:
        continue
      attr, _ = _getattr(arr)
      if attr.dtype == float:
        attr.fill_(wp.nan)
      elif attr.dtype == int:
        attr.fill_(-1)
      else:
        attr.zero_()

    mujoco.mj_step2(mjm, mjd)
    mjwarp.step2(m, d)

    for arr in step2_field:
      d_arr, is_efc = _getattr(arr)
      d_arr = d_arr.numpy()[0]
      if is_efc:
        d_arr = d_arr[: d.nefc.numpy()[0]]
      _assert_eq(d_arr, getattr(mjd, arr), arr)


if __name__ == "__main__":
  wp.init()
  absltest.main()
