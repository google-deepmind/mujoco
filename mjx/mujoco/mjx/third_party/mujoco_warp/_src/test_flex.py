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

"""Tests for flex."""

import mujoco
import numpy as np
from absl.testing import absltest
from absl.testing import parameterized

import mujoco.mjx.third_party.mujoco_warp as mjw
from mujoco.mjx.third_party.mujoco_warp import test_data

_TRILINEAR_STRAIN_XML = """
<mujoco>
  <option gravity="0 0 -9.81">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <flexcomp type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
              pos="0 0 0.5" name="cube" dim="3" mass="1" radius="0.005"
              dof="trilinear">
      <edge equality="strain"/>
      <contact selfcollide="none"/>
    </flexcomp>
  </worldbody>
</mujoco>
"""

# tolerance for difference between MuJoCo and MJWarp, mostly due to float32
_TOLERANCE = 5e-4


class TrilinearFlexTest(parameterized.TestCase):
  def test_flexstrain_constraint_at_rest(self):
    """Test FLEXSTRAIN constraint count and residuals match MuJoCo at rest."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)

    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)

    mjw.fwd_position(m, d)
    mjw.make_constraint(m, d)

    # constraint counts should match
    ne_warp = d.ne.numpy()[0]
    self.assertEqual(ne_warp, mjd.ne, f"ne mismatch: warp={ne_warp}, mj={mjd.ne}")

    nefc_warp = d.nefc.numpy()[0]
    self.assertEqual(nefc_warp, mjd.nefc, f"nefc mismatch: warp={nefc_warp}, mj={mjd.nefc}")

    # residuals should match
    efc_pos = d.efc.pos.numpy()[0, :nefc_warp]
    efc_pos_mj = mjd.efc_pos[: mjd.nefc]
    np.testing.assert_allclose(efc_pos, efc_pos_mj, atol=1e-5, err_msg="FLEXSTRAIN residuals should match MuJoCo at rest")

  def test_flexstrain_constraint_perturbed(self):
    """Test FLEXSTRAIN residuals and Jacobians match MuJoCo under perturbation."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)
    mjd = mujoco.MjData(mjm)

    # perturb first node
    mjd.qpos[0] += 0.01
    mjd.qpos[3] += 0.005
    mujoco.mj_forward(mjm, mjd)

    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)

    mjw.fwd_position(m, d)
    mjw.make_constraint(m, d)

    nefc = d.nefc.numpy()[0]
    nv = mjm.nv
    self.assertEqual(nefc, mjd.nefc)

    # residuals
    efc_pos_warp = d.efc.pos.numpy()[0, :nefc]
    efc_pos_mj = mjd.efc_pos[: mjd.nefc]
    np.testing.assert_allclose(
      efc_pos_warp, efc_pos_mj, atol=_TOLERANCE, err_msg="FLEXSTRAIN residuals don't match MuJoCo under perturbation"
    )

    # Jacobians
    if mujoco.mj_isSparse(mjm):
      mj_efc_J = np.zeros((mjd.nefc, nv))
      mujoco.mju_sparse2dense(mj_efc_J, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
    else:
      mj_efc_J = mjd.efc_J.reshape((mjd.nefc, nv))

    if m.is_sparse:
      warp_efc_J = np.zeros((nefc, nv))
      mujoco.mju_sparse2dense(
        warp_efc_J,
        d.efc.J.numpy()[0, 0],
        d.efc.J_rownnz.numpy()[0, :nefc],
        d.efc.J_rowadr.numpy()[0, :nefc],
        d.efc.J_colind.numpy()[0, 0],
      )
    else:
      warp_efc_J = d.efc.J.numpy()[0, :nefc, :nv]

    np.testing.assert_allclose(warp_efc_J, mj_efc_J, atol=0.01, err_msg="FLEXSTRAIN Jacobians don't match MuJoCo")

  def test_flexstrain_constraint_rotated(self):
    """Test FLEXSTRAIN residuals and Jacobians match MuJoCo under large rotation perturbation."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)
    mjd = mujoco.MjData(mjm)

    # Apply a rotation perturbation: rotate all node positions around Y axis by 30 degrees
    # (0.5235 radians)
    theta = 0.5235
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    for i in range(0, mjm.nq, 3):
      x = mjd.qpos[i]
      z = mjd.qpos[i + 2]
      mjd.qpos[i] = x * cos_t - z * sin_t
      mjd.qpos[i + 2] = x * sin_t + z * cos_t

    mujoco.mj_forward(mjm, mjd)

    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)

    mjw.fwd_position(m, d)
    mjw.make_constraint(m, d)

    nefc = d.nefc.numpy()[0]
    nv = mjm.nv
    self.assertEqual(nefc, mjd.nefc)

    # residuals
    efc_pos_warp = d.efc.pos.numpy()[0, :nefc]
    efc_pos_mj = mjd.efc_pos[: mjd.nefc]
    np.testing.assert_allclose(
      efc_pos_warp, efc_pos_mj, atol=_TOLERANCE, err_msg="FLEXSTRAIN residuals don't match MuJoCo under rotation"
    )

    # Jacobians
    if mujoco.mj_isSparse(mjm):
      mj_efc_J = np.zeros((mjd.nefc, nv))
      mujoco.mju_sparse2dense(mj_efc_J, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
    else:
      mj_efc_J = mjd.efc_J.reshape((mjd.nefc, nv))

    if m.is_sparse:
      warp_efc_J = np.zeros((nefc, nv))
      mujoco.mju_sparse2dense(
        warp_efc_J,
        d.efc.J.numpy()[0, 0],
        d.efc.J_rownnz.numpy()[0, :nefc],
        d.efc.J_rowadr.numpy()[0, :nefc],
        d.efc.J_colind.numpy()[0, 0],
      )
    else:
      warp_efc_J = d.efc.J.numpy()[0, :nefc, :nv]

    np.testing.assert_allclose(
      warp_efc_J, mj_efc_J, atol=0.01, err_msg="FLEXSTRAIN Jacobians don't match MuJoCo under rotation"
    )

  def test_flexstrain_rotational_invariance(self):
    """Test that FLEXSTRAIN residuals are invariant under rigid translation."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)

    # Get reference residuals
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)
    mjw.fwd_position(m, d)
    mjw.make_constraint(m, d)
    nefc = d.nefc.numpy()[0]
    efc_pos_rest = d.efc.pos.numpy()[0, :nefc].copy()

    # Apply uniform translation to all nodes (rigid motion)
    mjd2 = mujoco.MjData(mjm)
    # All flex nodes have 3 DOFs (slide joints), shift all x by 0.1
    for i in range(0, mjm.nq, 3):
      mjd2.qpos[i] += 0.1  # shift x
    mujoco.mj_forward(mjm, mjd2)

    d2 = mjw.put_data(mjm, mjd2)
    mjw.fwd_position(m, d2)
    mjw.make_constraint(m, d2)
    nefc2 = d2.nefc.numpy()[0]
    efc_pos_shifted = d2.efc.pos.numpy()[0, :nefc2]

    # Residuals should remain near zero for rigid translation
    np.testing.assert_allclose(
      efc_pos_shifted, efc_pos_rest, atol=1e-4, err_msg="FLEXSTRAIN residuals should be invariant under rigid translation"
    )

  def test_trilinear_gravity_parity(self):
    """Test that trilinear simulation matches MuJoCo after multiple steps."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)

    # MuJoCo reference
    mjd = mujoco.MjData(mjm)
    for _ in range(10):
      mujoco.mj_step(mjm, mjd)

    # Warp
    mjd_warp = mujoco.MjData(mjm)
    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd_warp)
    z0 = d.qpos.numpy()[0, 2]
    for _ in range(10):
      mjw.step(m, d)

    qpos_warp = d.qpos.numpy()[0]
    qpos_mj = mjd.qpos

    # The cube should have fallen
    self.assertLess(qpos_warp[2], z0, "Cube should fall under gravity")

    # Allow larger tolerance for accumulated integration error
    np.testing.assert_allclose(qpos_warp, qpos_mj, atol=0.01, err_msg="Trilinear qpos diverges from MuJoCo after 10 steps")

  def test_trilinear_node_positions(self):
    """Test that flexnode_xpos are computed correctly from body kinematics."""
    mjm = mujoco.MjModel.from_xml_string(_TRILINEAR_STRAIN_XML)
    mjd = mujoco.MjData(mjm)
    mujoco.mj_forward(mjm, mjd)

    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)
    mjw.fwd_position(m, d)

    # Compute expected node positions: xpos_n = body_xpos + body_xmat @ flex_node
    nflexnode = mjm.nflexnode
    warp_xpos = d.flexnode_xpos.numpy()[0, :nflexnode]
    nodeadr = mjm.flex_nodeadr[0]
    nodenum = mjm.flex_nodenum[0]
    for n in range(nodenum):
      bodyid = mjm.flex_nodebodyid[nodeadr + n]
      body_xpos = mjd.xpos[bodyid]
      body_xmat = mjd.xmat[bodyid].reshape(3, 3)
      node_local = mjm.flex_node[nodeadr + n]
      expected = body_xpos + body_xmat @ node_local
      np.testing.assert_allclose(warp_xpos[n], expected, atol=1e-5, err_msg=f"flexnode_xpos mismatch for node {n}")

  def test_trilinear_passive_forces_parity(self):
    """Test passive forces (elasticity) match MuJoCo for trilinear flex."""
    xml = """
    <mujoco>
      <worldbody>
        <flexcomp type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
                  pos="0 0 0.5" name="cube" dim="3" mass="1" radius="0.005"
                  dof="trilinear">
          <elasticity young="1e4" poisson="0.1" damping="0.01"/>
          <contact selfcollide="none"/>
        </flexcomp>
      </worldbody>
    </mujoco>
    """
    mjm = mujoco.MjModel.from_xml_string(xml)
    mjd = mujoco.MjData(mjm)

    # perturb first node to generate non-zero elasticity forces
    mjd.qpos[0] += 0.01
    mjd.qpos[3] += 0.005
    mujoco.mj_forward(mjm, mjd)

    m = mjw.put_model(mjm)
    d = mjw.put_data(mjm, mjd)

    mjw.fwd_position(m, d)
    mjw.passive(m, d)

    qfrc_passive_warp = d.qfrc_passive.numpy()[0]
    qfrc_passive_mj = mjd.qfrc_passive

    # Verify they match
    np.testing.assert_allclose(
      qfrc_passive_warp, qfrc_passive_mj, atol=_TOLERANCE, err_msg="qfrc_passive mismatch for trilinear flex with elasticity"
    )

  @parameterized.parameters("strain", "true")
  def test_trilinear_equality_types(self, equality):
    """Test trilinear with different equality types."""
    xml = f"""
    <mujoco>
      <option gravity="0 0 -9.81">
        <flag contact="disable"/>
      </option>
      <worldbody>
        <flexcomp type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
                  pos="0 0 0.5" name="cube" dim="3" mass="1" radius="0.005"
                  dof="trilinear">
          <edge equality="{equality}"/>
          <contact selfcollide="none"/>
        </flexcomp>
      </worldbody>
    </mujoco>
    """
    mjm, mjd, m, d = test_data.fixture(xml=xml)

    # Should not crash
    mjw.forward(m, d)

    # Constraint count should match
    self.assertEqual(d.nefc.numpy()[0], mjd.nefc)

  def test_trilinear_contact_qfrc_constraint(self):
    """Test qfrc_constraint parity for trilinear flex with ground contacts."""
    xml = """
    <mujoco>
      <option gravity="0 0 -9.81"/>
      <worldbody>
        <geom type="plane" size="1 1 0.1"/>
        <flexcomp type="grid" count="3 3 3" spacing="0.1 0.1 0.1"
                  pos="0 0 0.05" name="cube" dim="3" mass="1" radius="0.02"
                  dof="trilinear">
          <edge equality="strain"/>
          <contact selfcollide="none"/>
        </flexcomp>
      </worldbody>
    </mujoco>
    """
    mjm, mjd, m, d = test_data.fixture(xml=xml)
    mjw.forward(m, d)

    # Verify contacts are generated
    nacon = d.nacon.numpy()[0]
    self.assertGreater(nacon, 0, "Expected contacts between flex and plane")
    self.assertEqual(nacon, mjd.ncon)

    # Verify qfrc_constraint parity
    qfrc_warp = d.qfrc_constraint.numpy()[0]
    qfrc_mj = mjd.qfrc_constraint
    np.testing.assert_allclose(qfrc_warp, qfrc_mj, atol=1e-4, err_msg="qfrc_constraint mismatch for trilinear flex contacts")


if __name__ == "__main__":
  absltest.main()
