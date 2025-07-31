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

"""Tests for smooth dynamics functions."""

import mujoco
import numpy as np
import warp as wp
from absl.testing import absltest
from absl.testing import parameterized

import mujoco_warp as mjwarp

from mujoco.mjx.third_party.mujoco_warp._src import test_util
from mujoco.mjx.third_party.mujoco_warp._src import types

# tolerance for difference between MuJoCo and MJWarp smooth calculations - mostly
# due to float precision
_TOLERANCE = 5e-5


def _assert_eq(a, b, name):
  tol = _TOLERANCE * 10  # avoid test noise
  err_msg = f"mismatch: {name}"
  np.testing.assert_allclose(a, b, err_msg=err_msg, atol=tol, rtol=tol)


class SmoothTest(parameterized.TestCase):
  def test_kinematics(self):
    """Tests kinematics."""
    _, mjd, m, d = test_util.fixture("pendula.xml")

    for arr in (
      d.xanchor,
      d.xaxis,
      d.xpos,
      d.xquat,
      d.xmat,
      d.xipos,
      d.ximat,
      d.geom_xpos,
      d.geom_xmat,
      d.site_xpos,
      d.site_xmat,
    ):
      arr.zero_()

    mjwarp.kinematics(m, d)

    _assert_eq(d.xanchor.numpy()[0], mjd.xanchor, "xanchor")
    _assert_eq(d.xaxis.numpy()[0], mjd.xaxis, "xaxis")
    _assert_eq(d.xpos.numpy()[0], mjd.xpos, "xpos")
    _assert_eq(d.xquat.numpy()[0], mjd.xquat, "xquat")
    _assert_eq(d.xmat.numpy()[0], mjd.xmat.reshape((-1, 3, 3)), "xmat")
    _assert_eq(d.xipos.numpy()[0], mjd.xipos, "xipos")
    _assert_eq(d.ximat.numpy()[0], mjd.ximat.reshape((-1, 3, 3)), "ximat")
    _assert_eq(d.geom_xpos.numpy()[0], mjd.geom_xpos, "geom_xpos")
    _assert_eq(d.geom_xmat.numpy()[0], mjd.geom_xmat.reshape((-1, 3, 3)), "geom_xmat")
    _assert_eq(d.site_xpos.numpy()[0], mjd.site_xpos, "site_xpos")
    _assert_eq(d.site_xmat.numpy()[0], mjd.site_xmat.reshape((-1, 3, 3)), "site_xmat")

  def test_com_pos(self):
    """Tests com_pos."""
    _, mjd, m, d = test_util.fixture("pendula.xml")

    for arr in (d.subtree_com, d.cinert, d.cdof):
      arr.zero_()

    mjwarp.com_pos(m, d)
    _assert_eq(d.subtree_com.numpy()[0], mjd.subtree_com, "subtree_com")
    _assert_eq(d.cinert.numpy()[0], mjd.cinert, "cinert")
    _assert_eq(d.cdof.numpy()[0], mjd.cdof, "cdof")

  def test_camlight(self):
    """Tests camlight."""
    _, mjd, m, d = test_util.fixture("pendula.xml")

    d.cam_xpos.zero_()
    d.cam_xmat.zero_()
    d.light_xpos.zero_()
    d.light_xdir.zero_()

    mjwarp.camlight(m, d)
    _assert_eq(d.cam_xpos.numpy()[0], mjd.cam_xpos, "cam_xpos")
    _assert_eq(d.cam_xmat.numpy()[0], mjd.cam_xmat.reshape((-1, 3, 3)), "cam_xmat")
    _assert_eq(d.light_xpos.numpy()[0], mjd.light_xpos, "light_xpos")
    _assert_eq(d.light_xdir.numpy()[0], mjd.light_xdir, "light_xdir")

  @parameterized.parameters(True, False)
  def test_crb(self, sparse: bool):
    """Tests crb."""
    mjm, mjd, m, d = test_util.fixture("pendula.xml", sparse=sparse)

    d.crb.zero_()

    mjwarp.crb(m, d)
    _assert_eq(d.crb.numpy()[0], mjd.crb, "crb")

    if sparse:
      _assert_eq(d.qM.numpy()[0, 0], mjd.qM, "qM")
    else:
      qM = np.zeros((mjm.nv, mjm.nv))
      mujoco.mj_fullM(mjm, qM, mjd.qM)
      _assert_eq(d.qM.numpy()[0], qM, "qM")

  @parameterized.parameters(True, False)
  def test_factor_m(self, sparse: bool):
    """Tests factor_m."""
    _, mjd, m, d = test_util.fixture("pendula.xml", sparse=sparse)

    qLD = d.qLD.numpy()[0].copy()
    for arr in (d.qLD, d.qLDiagInv):
      arr.zero_()

    mjwarp.factor_m(m, d)

    if sparse:
      _assert_eq(d.qLD.numpy()[0, 0], mjd.qLD, "qLD (sparse)")
      _assert_eq(d.qLDiagInv.numpy()[0], mjd.qLDiagInv, "qLDiagInv")
    else:
      _assert_eq(d.qLD.numpy()[0], qLD, "qLD (dense)")

  @parameterized.parameters(True, False)
  def test_solve_m(self, sparse: bool):
    """Tests solve_m."""
    mjm, mjd, m, d = test_util.fixture("pendula.xml", sparse=sparse)

    qfrc_smooth = np.tile(mjd.qfrc_smooth, (1, 1))
    qacc_smooth = np.zeros(
      shape=(
        1,
        mjm.nv,
      ),
      dtype=float,
    )
    mujoco.mj_solveM(mjm, mjd, qacc_smooth, qfrc_smooth)

    d.qacc_smooth.zero_()

    mjwarp.solve_m(m, d, d.qacc_smooth, d.qfrc_smooth)
    _assert_eq(d.qacc_smooth.numpy()[0], qacc_smooth[0], "qacc_smooth")

  @parameterized.parameters(True, False)
  def test_rne(self, gravity):
    """Tests rne."""
    _, mjd, m, d = test_util.fixture("pendula.xml", gravity=gravity)

    d.qfrc_bias.zero_()

    mjwarp.rne(m, d)
    _assert_eq(d.qfrc_bias.numpy()[0], mjd.qfrc_bias, "qfrc_bias")

  @parameterized.parameters(True, False)
  def test_rne_postconstraint(self, gravity):
    """Tests rne_postconstraint."""
    mjm, mjd, m, d = test_util.fixture("pendula.xml", gravity=gravity)

    mjd.xfrc_applied = np.random.uniform(low=-0.01, high=0.01, size=mjd.xfrc_applied.shape)
    d.xfrc_applied = wp.array(np.expand_dims(mjd.xfrc_applied, axis=0), dtype=wp.spatial_vector)

    mujoco.mj_rnePostConstraint(mjm, mjd)

    for arr in (d.cacc, d.cfrc_int, d.cfrc_ext):
      arr.zero_()

    mjwarp.rne_postconstraint(m, d)

    _assert_eq(d.cacc.numpy()[0], mjd.cacc, "cacc")
    _assert_eq(d.cfrc_int.numpy()[0], mjd.cfrc_int, "cfrc_int")
    _assert_eq(d.cfrc_ext.numpy()[0], mjd.cfrc_ext, "cfrc_ext")

    _EQUALITY = """
      <mujoco>
        <option gravity="1 1 -1">
          <flag contact="disable"/>
        </option>
        <worldbody>
          <site name="siteworld"/>
          <body name="body0">
            <geom type="sphere" size=".1"/>
            <freejoint/>
          </body>
          <body name="body1">
            <geom type="sphere" size=".1"/>
            <site name="site1"/>
            <freejoint/>
          </body>
          <body name="body2">
            <geom type="sphere" size=".1"/>
            <freejoint/>
          </body>
          <body name="body3">
            <geom type="sphere" size=".1"/>
            <site name="site3" quat="0 1 0 0"/>
            <freejoint/>
          </body>
        </worldbody>
        <equality>
          <connect body1="body0" anchor="1 1 1"/>
          <connect site1="siteworld" site2="site1"/>
          <weld body1="body2" relpose="1 1 1 0 1 0 0"/>
          <weld site1="siteworld" site2="site3"/>
        </equality>
        <keyframe>
          <key qpos="0 0 0 1 0 0 0 1 1 1 1 0 0 0 0 0 0 1 0 0 0 1 1 1 1 0 0 0"/>
        </keyframe>
      </mujoco>
      """
    mjm, mjd, m, d = test_util.fixture(xml=_EQUALITY, kick=True, keyframe=0)

    mujoco.mj_rnePostConstraint(mjm, mjd)

    d.cfrc_ext.zero_()
    mjwarp.rne_postconstraint(m, d)

    _assert_eq(d.cfrc_ext.numpy()[0], mjd.cfrc_ext, "cfrc_ext (equality)")

    mjm, mjd, m, d = test_util.fixture("constraints.xml", keyframe=1, equality=False)

    mujoco.mj_rnePostConstraint(mjm, mjd)

    d.cfrc_ext.zero_()

    # clear equality constraint counts
    d.ne_connect.zero_()
    d.ne_weld.zero_()
    d.ne_jnt.zero_()

    mjwarp.rne_postconstraint(m, d)

    _assert_eq(d.cfrc_ext.numpy()[0], mjd.cfrc_ext, "cfrc_ext (contact)")

  def test_com_vel(self):
    """Tests com_vel."""
    _, mjd, m, d = test_util.fixture("pendula.xml")

    for arr in (d.cvel, d.cdof_dot):
      arr.zero_()

    mjwarp.com_vel(m, d)
    _assert_eq(d.cvel.numpy()[0], mjd.cvel, "cvel")
    _assert_eq(d.cdof_dot.numpy()[0], mjd.cdof_dot, "cdof_dot")

  @parameterized.parameters("pendula.xml", "actuation/site.xml", "actuation/slidercrank.xml")
  def test_transmission(self, xml):
    """Tests transmission."""
    mjm, mjd, m, d = test_util.fixture(xml)

    for arr in (d.actuator_length, d.actuator_moment):
      arr.zero_()

    actuator_moment = np.zeros((mjm.nu, mjm.nv))
    mujoco.mju_sparse2dense(
      actuator_moment,
      mjd.actuator_moment,
      mjd.moment_rownnz,
      mjd.moment_rowadr,
      mjd.moment_colind,
    )

    mjwarp._src.smooth.transmission(m, d)
    _assert_eq(d.actuator_length.numpy()[0], mjd.actuator_length, "actuator_length")
    _assert_eq(d.actuator_moment.numpy()[0], actuator_moment, "actuator_moment")

  @parameterized.product(keyframe=list(range(4)), cone=list(types.ConeType))
  def test_actuator_adhesion(self, keyframe, cone):
    """Tests adhesion actuator."""
    mjm, mjd, m, d = test_util.fixture("actuation/adhesion.xml", keyframe=keyframe, cone=cone)

    d.actuator_length.zero_()
    d.actuator_moment.zero_()
    mjwarp._src.collision_driver.collision(m, d)  # compute contact.includemargin
    mjwarp._src.constraint.make_constraint(m, d)  # compute contact.efc_address
    mjwarp._src.smooth.transmission(m, d)

    actuator_moment = np.zeros((mjm.nu, mjm.nv))
    mujoco.mju_sparse2dense(actuator_moment, mjd.actuator_moment, mjd.moment_rownnz, mjd.moment_rowadr, mjd.moment_colind)

    _assert_eq(d.actuator_length.numpy()[0], mjd.actuator_length, "actuator_length")
    _assert_eq(d.actuator_moment.numpy()[0], actuator_moment, "acutator_moment")

  def test_subtree_vel(self):
    """Tests subtree_vel."""
    mjm, mjd, m, d = test_util.fixture("pendula.xml")

    for arr in (d.subtree_linvel, d.subtree_angmom):
      arr.zero_()

    mujoco.mj_subtreeVel(mjm, mjd)
    mjwarp.subtree_vel(m, d)

    _assert_eq(d.subtree_linvel.numpy()[0], mjd.subtree_linvel, "subtree_linvel")
    _assert_eq(d.subtree_angmom.numpy()[0], mjd.subtree_angmom, "subtree_angmom")

  @parameterized.parameters(
    "tendon/fixed.xml",
    "tendon/site.xml",
    "tendon/pulley_site.xml",
    "tendon/fixed_site.xml",
    "tendon/pulley_fixed_site.xml",
    "tendon/site_fixed.xml",
    "tendon/pulley_site_fixed.xml",
    "tendon/wrap.xml",
    "tendon/pulley_wrap.xml",
  )
  def test_tendon(self, xml):
    """Tests tendon."""
    mjm, mjd, m, d = test_util.fixture(xml, keyframe=0)

    for arr in (d.ten_length, d.ten_J, d.actuator_length, d.actuator_moment):
      arr.zero_()

    mjwarp.tendon(m, d)
    mjwarp.transmission(m, d)

    _assert_eq(d.ten_length.numpy()[0], mjd.ten_length, "ten_length")
    _assert_eq(d.ten_J.numpy()[0], mjd.ten_J.reshape((mjm.ntendon, mjm.nv)), "ten_J")
    _assert_eq(d.wrap_xpos.numpy()[0], mjd.wrap_xpos, "wrap_xpos")
    _assert_eq(d.wrap_obj.numpy()[0], mjd.wrap_obj, "wrap_obj")
    _assert_eq(d.ten_wrapnum.numpy()[0], mjd.ten_wrapnum, "ten_wrapnum")
    _assert_eq(d.ten_wrapadr.numpy()[0], mjd.ten_wrapadr, "ten_wrapadr")
    _assert_eq(d.actuator_length.numpy()[0], mjd.actuator_length, "actuator_length")
    actuator_moment = np.zeros((mjm.nu, mjm.nv))
    mujoco.mju_sparse2dense(
      actuator_moment,
      mjd.actuator_moment,
      mjd.moment_rownnz,
      mjd.moment_rowadr,
      mjd.moment_colind,
    )
    _assert_eq(d.actuator_moment.numpy()[0], actuator_moment, "actuator_moment")

  @parameterized.parameters(True, False)
  def test_factor_solve_i(self, sparse):
    mjm, mjd, m, d = test_util.fixture(
      xml="""
    <mujoco>
      <worldbody>
        <body>
          <geom type="sphere" size=".1"/>
          <freejoint/>
        </body>
      </worldbody>
    </mujoco>
    """,
      sparse=sparse,
    )

    qM = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, qM, mjd.qM)

    d.qLD.zero_()
    if sparse:
      d.qLDiagInv.zero_()

    res = wp.zeros((1, mjm.nv), dtype=float)
    vec = wp.ones((1, mjm.nv), dtype=float)

    mjwarp._src.smooth.factor_solve_i(m, d, d.qM, d.qLD, d.qLDiagInv, res, vec)

    _assert_eq(res.numpy()[0], np.linalg.solve(qM, vec.numpy()[0]), "qM \\ 1")

  def test_tendon_armature(self):
    mjm, mjd, m, d = test_util.fixture("tendon/armature.xml", keyframe=0)

    # qM
    d.qM.zero_()

    mjwarp._src.smooth.crb(m, d)
    mjwarp._src.smooth.tendon_armature(m, d)

    qM = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, qM, mjd.qM)
    _assert_eq(d.qM.numpy()[0], qM, "qM")

    # qfrc_bias
    d.qfrc_bias.zero_()

    mjwarp._src.smooth.rne(m, d)
    mjwarp._src.smooth.tendon_bias(m, d, d.qfrc_bias)
    _assert_eq(d.qfrc_bias.numpy()[0], mjd.qfrc_bias, "qfrc_bias")


if __name__ == "__main__":
  wp.init()
  absltest.main()
