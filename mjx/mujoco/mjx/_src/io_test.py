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
"""Tests for io functions."""

from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
import numpy as np


_MULTIPLE_CONVEX_OBJECTS = """
  <mujoco>
    <option timestep="0.001" jacobian="dense"/>
    <default>
      <geom solref=".006 1"/>
    </default>
    <asset>
      <mesh name="box" vertex="-1 -1 -1 1 -1 -1 1 1 -1 1 1 1 1 -1 1 -1 1 -1 -1 1 1 -1 -1 1" scale="1 1 .1"/>
      <mesh name="boxoid" vertex="-1 -1 -1 1 -1 -1 1 1 -1 1 1 1 1 -1 1 -1 1 -1 -1 1 .5 -1 -1 2" scale=".3 .2 .1"/>
      <mesh name="pentaprism" vertex="1 0 0 0.309 0.951 0 -0.809 0.588 0 -0.809 -0.588 0 0.309 -0.951 0 1 0 1 0.309 0.951 1 -0.809 0.588 1 -0.809 -0.588 1 0.309 -0.951 1" scale=".2 .2 .1"/>
    </asset>
    <worldbody>
      <geom type="plane" pos="0 0 -.5" size="3 3 .01"/>
      <geom type="mesh" mesh="box" pos="0 0 -.15" euler="3 7 30"/>
      <body pos="-.3 -.3 .3">
        <freejoint/>
        <geom type="mesh" mesh="boxoid" rgba=".8 0 0 1" euler="3 5 -130"/>
      </body>
      <body pos=".3 .3 0.3">
        <freejoint/>
        <geom type="box" euler="3 5 -80" size=".3 .2 .1" rgba="0 .8 0 1"/>
      </body>
      <body pos=".3 .3 .6">
        <freejoint/>
        <geom type="mesh" mesh="pentaprism" rgba="0 0 .8 1"/>
      </body>
      <body pos=".6 -.3 .3">
        <joint name="joint" axis="1 0 0" type="hinge" range="-45 45"/>
        <geom type="capsule" size=".2 .05" rgba=".6 0 .6 1"/>
      </body>
    </worldbody>
    <actuator>
      <motor joint="joint"/>
    </actuator>
  </mujoco>
"""

_MULTIPLE_CONSTRAINTS = """
  <mujoco>
    <worldbody>
      <geom type="plane" size="3 3 .01"/>
      <body name="cap1" pos="-.3 -.3 .2">
        <freejoint/>
        <geom type="capsule" size=".2 .05"/>
        <body name="cap2" pos=".6 -.3 .3">
          <joint axis="0 1 0" type="hinge" range="-45 45"/>
          <joint axis="1 0 0" type="hinge" range="-0.001 0.001"/>
          <geom type="capsule" size=".2 .05"/>
          <site pos="-0.214 -0.078 0" quat="0.664 0.664 -0.242 -0.242"/>
        </body>
      </body>
    </worldbody>
    <equality>
      <connect body1="cap2" anchor="0 0 1"/>
    </equality>
  </mujoco>
"""


class ModelIOTest(parameterized.TestCase):
  """IO tests for mjx.Model."""

  def test_put_model(self):
    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONVEX_OBJECTS)
    mx = mjx.put_model(m)
    self.assertEqual(mx.nq, m.nq)
    self.assertEqual(mx.nv, m.nv)
    self.assertEqual(mx.nu, m.nu)
    self.assertEqual(mx.na, m.na)
    self.assertEqual(mx.nbody, m.nbody)
    self.assertEqual(mx.njnt, m.njnt)
    self.assertEqual(mx.ngeom, m.ngeom)
    self.assertEqual(mx.nmesh, m.nmesh)
    self.assertEqual(mx.npair, m.npair)
    self.assertEqual(mx.nexclude, m.nexclude)
    self.assertEqual(mx.neq, m.neq)
    self.assertEqual(mx.nnumeric, m.nnumeric)
    self.assertEqual(mx.nM, m.nM)
    self.assertAlmostEqual(mx.opt.timestep, m.opt.timestep)

    np.testing.assert_allclose(mx.body_parentid, m.body_parentid)
    np.testing.assert_allclose(mx.geom_type, m.geom_type)
    np.testing.assert_allclose(mx.geom_bodyid, m.geom_bodyid)
    np.testing.assert_almost_equal(mx.geom_solref, m.geom_solref)
    np.testing.assert_almost_equal(mx.geom_pos, m.geom_pos)
    self.assertLen(mx.geom_convex_face, 6)
    self.assertLen(mx.geom_convex_vert, 6)
    self.assertLen(mx.geom_convex_edge, 6)
    self.assertLen(mx.geom_convex_facenormal, 6)

    np.testing.assert_allclose(mx.jnt_type, m.jnt_type)
    np.testing.assert_allclose(mx.jnt_dofadr, m.jnt_dofadr)
    np.testing.assert_allclose(mx.jnt_bodyid, m.jnt_bodyid)
    np.testing.assert_allclose(mx.jnt_limited, m.jnt_limited)
    np.testing.assert_almost_equal(mx.jnt_axis, m.jnt_axis)

    np.testing.assert_allclose(mx.actuator_trntype, m.actuator_trntype)
    np.testing.assert_allclose(mx.actuator_dyntype, m.actuator_dyntype)
    np.testing.assert_allclose(mx.actuator_gaintype, m.actuator_gaintype)
    np.testing.assert_allclose(mx.actuator_biastype, m.actuator_biastype)
    np.testing.assert_allclose(mx.actuator_trnid, m.actuator_trnid)

  def test_fluid_params(self):
    """Test that has_fluid_params is set when fluid params are present."""
    m = mjx.put_model(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option viscosity="3.0"/><worldbody/></mujoco>'
        )
    )
    self.assertTrue(m.opt.has_fluid_params)

  def test_implicit_not_implemented(self):
    """Test that MJX guards against models with unimplemented features."""

    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string(
              '<mujoco><option integrator="implicit"/><worldbody/></mujoco>'
          )
      )

  def test_cone_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string(
              '<mujoco><option cone="elliptic"/><worldbody/></mujoco>'
          )
      )

  def test_pgs_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string(
              '<mujoco><option solver="PGS"/><worldbody/></mujoco>'
          )
      )

  def test_tendon_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(mujoco.MjModel.from_xml_string("""
        <mujoco>
          <worldbody>
            <body>
              <joint name="left_hip" type="hinge"/>
              <geom size="0.05"/>
            </body>
          </worldbody>
          <tendon>
            <fixed>
              <joint coef="1" joint="left_hip"/>
            </fixed>
          </tendon>
        </mujoco>"""))

  def test_condim_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(mujoco.MjModel.from_xml_string("""
        <mujoco>
          <worldbody>
            <body>
              <freejoint/>
              <geom size="0.05" condim="1"/>
            </body>
            <body>
              <freejoint/>
              <geom size="0.05" condim="1"/>
            </body>
          </worldbody>
        </mujoco>"""))

  def test_cylinder_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(mujoco.MjModel.from_xml_string("""
        <mujoco>
          <worldbody>
            <body>
              <freejoint/>
              <geom type="cylinder" size="0.05 0.05"/>
            </body>
            <body>
              <freejoint/>
              <geom size="0.05"/>
            </body>
          </worldbody>
        </mujoco>"""))


class DataIOTest(parameterized.TestCase):
  """IO tests for mjx.Data."""

  def test_make_data(self):
    """Test that make_data returns the correct shapes."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONVEX_OBJECTS)
    d = mjx.make_data(m)

    nq = 22
    nbody = 5
    ncon = 46
    nv = 19
    nefc = 185

    self.assertEqual(d.qpos.shape, (nq,))
    self.assertEqual(d.qvel.shape, (nv,))
    self.assertEqual(d.act.shape, (0,))
    self.assertEqual(d.qacc_warmstart.shape, (nv,))
    self.assertEqual(d.ctrl.shape, (1,))
    self.assertEqual(d.qfrc_applied.shape, (nv,))
    self.assertEqual(d.xfrc_applied.shape, (nbody, 6))
    self.assertEqual(d.eq_active.shape, (0,))
    self.assertEqual(d.qacc.shape, (nv,))
    self.assertEqual(d.act_dot.shape, (0,))
    self.assertEqual(d.xpos.shape, (nbody, 3))
    self.assertEqual(d.xquat.shape, (nbody, 4))
    self.assertEqual(d.xmat.shape, (nbody, 3, 3))
    self.assertEqual(d.xipos.shape, (nbody, 3))
    self.assertEqual(d.ximat.shape, (nbody, 3, 3))
    self.assertEqual(d.xanchor.shape, (4, 3))
    self.assertEqual(d.xaxis.shape, (4, 3))
    self.assertEqual(d.geom_xpos.shape, (6, 3))
    self.assertEqual(d.geom_xmat.shape, (6, 3, 3))
    self.assertEqual(d.subtree_com.shape, (nbody, 3))
    self.assertEqual(d.cdof.shape, (nv, 6))
    self.assertEqual(d.cinert.shape, (nbody, 10))
    self.assertEqual(d.crb.shape, (nbody, 10))
    self.assertEqual(d.actuator_length.shape, (1,))
    self.assertEqual(d.actuator_moment.shape, (1, nv))
    self.assertEqual(d.qM.shape, (nv, nv))
    self.assertEqual(d.qLD.shape, (nv, nv))
    self.assertEqual(d.qLDiagInv.shape, (0,))
    self.assertEqual(d.contact.dist.shape, (ncon,))
    self.assertEqual(d.contact.pos.shape, (ncon, 3))
    self.assertEqual(d.contact.frame.shape, (ncon, 3, 3))
    self.assertEqual(d.contact.solref.shape, (ncon, 2))
    self.assertEqual(d.contact.solimp.shape, (ncon, 5))
    self.assertEqual(d.contact.geom1.shape, (ncon,))
    self.assertEqual(d.contact.geom2.shape, (ncon,))
    self.assertEqual(d.efc_J.shape, (nefc, nv))
    self.assertEqual(d.efc_frictionloss.shape, (nefc,))
    self.assertEqual(d.efc_D.shape, (nefc,))
    self.assertEqual(d.actuator_velocity.shape, (1,))
    self.assertEqual(d.cvel.shape, (nbody, 6))
    self.assertEqual(d.cdof_dot.shape, (nv, 6))
    self.assertEqual(d.qfrc_bias.shape, (nv,))
    self.assertEqual(d.qfrc_passive.shape, (nv,))
    self.assertEqual(d.efc_aref.shape, (nefc,))
    self.assertEqual(d.qfrc_actuator.shape, (nv,))
    self.assertEqual(d.qfrc_smooth.shape, (nv,))
    self.assertEqual(d.qacc_smooth.shape, (nv,))
    self.assertEqual(d.qfrc_constraint.shape, (nv,))
    self.assertEqual(d.qfrc_inverse.shape, (nv,))
    self.assertEqual(d.efc_force.shape, (nefc,))

  def test_put_data(self):
    """Test that put_data puts the correct data for dense and sparse."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d)

    # check a few fields
    np.testing.assert_allclose(dx.qpos, d.qpos)
    np.testing.assert_allclose(dx.xpos, d.xpos)
    np.testing.assert_allclose(dx.cvel, d.cvel)
    np.testing.assert_allclose(dx.cdof_dot, d.cdof_dot)

    # check that qM is transformed properly
    qm = np.zeros((m.nv, m.nv), dtype=np.float64)
    mujoco.mj_fullM(m, qm, d.qM)
    np.testing.assert_allclose(qm, mjx.full_m(mjx.put_model(m), dx))

    # 4 contacts, 2 for each capsule against the plane
    self.assertEqual(dx.contact.dist.shape, (4,))
    self.assertEqual(d.ncon, 1)  # however only 1 contact in this step
    np.testing.assert_allclose(dx.contact.dist[0], d.contact.dist[0])
    self.assertTrue(np.isinf(dx.contact.dist[1:]).all())
    self.assertEqual(dx.contact.frame.shape, (4, 3, 3))
    np.testing.assert_allclose(
        dx.contact.frame[0].reshape(9), d.contact.frame[0]
    )
    np.testing.assert_allclose(dx.contact.frame[1:], 0)

    # xmat, ximat, geom_xmat are all shape transformed
    self.assertEqual(dx.xmat.shape, (3, 3, 3))
    self.assertEqual(dx.ximat.shape, (3, 3, 3))
    self.assertEqual(dx.geom_xmat.shape, (3, 3, 3))
    self.assertEqual(dx.site_xmat.shape, (1, 3, 3))
    np.testing.assert_allclose(dx.xmat.reshape((3, 9)), d.xmat)
    np.testing.assert_allclose(dx.ximat.reshape((3, 9)), d.ximat)
    np.testing.assert_allclose(dx.geom_xmat.reshape((3, 9)), d.geom_xmat)
    np.testing.assert_allclose(dx.site_xmat.reshape((1, 9)), d.site_xmat)

    # efc_ are also shape transformed and padded
    self.assertEqual(dx.efc_J.shape, (21, 8))  # nefc, nv
    d_efc_j = d.efc_J.reshape((-1, 8))
    np.testing.assert_allclose(dx.efc_J[:3], d_efc_j[:3])  # connect eq
    np.testing.assert_allclose(dx.efc_J[3], d_efc_j[3])  # one active limit
    np.testing.assert_allclose(dx.efc_J[4], 0)  # one inactive limit
    np.testing.assert_allclose(dx.efc_J[5:9], d_efc_j[4:8])  # contact
    np.testing.assert_allclose(dx.efc_J[9:], 0)  # no contact

    # check another efc_ too
    self.assertEqual(dx.efc_aref.shape, (21,))  # nefc
    np.testing.assert_allclose(dx.efc_aref[:3], d.efc_aref[:3])
    np.testing.assert_allclose(dx.efc_aref[3], d.efc_aref[3])
    np.testing.assert_allclose(dx.efc_aref[4], 0)
    np.testing.assert_allclose(dx.efc_aref[5:9], d.efc_aref[4:8])
    np.testing.assert_allclose(dx.efc_aref[9:], 0)

    # check sparse transform is correct
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx_sparse = mjx.put_data(m, d)
    np.testing.assert_allclose(dx_sparse.efc_J, dx.efc_J, atol=1e-8)

    # check sparse mass matrices are correct
    np.testing.assert_allclose(dx_sparse.qM, d.qM, atol=1e-8)
    np.testing.assert_allclose(dx_sparse.qLD, d.qLD, atol=1e-8)
    np.testing.assert_allclose(dx_sparse.qLDiagInv, d.qLDiagInv, atol=1e-8)

    # check dense mass matrices are correct
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx_from_dense = mjx.put_data(m, d)
    qm = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, qm, d.qM)
    np.testing.assert_allclose(dx_from_dense.qM, qm, atol=1e-8)

  def test_get_data(self):
    """Test that get_data makes correct MjData."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d)
    d_2: mujoco.MjData = mjx.get_data(m, dx)

    # check a few fields
    np.testing.assert_allclose(d_2.qpos, d.qpos)
    np.testing.assert_allclose(d_2.xpos, d.xpos)
    np.testing.assert_allclose(d_2.cvel, d.cvel)
    np.testing.assert_allclose(d_2.cdof_dot, d.cdof_dot)
    np.testing.assert_allclose(d_2.qM, d.qM)

    # only 1 contact active
    self.assertEqual(d_2.contact.dist.shape, (1,))
    self.assertEqual(d_2.ncon, 1)
    np.testing.assert_allclose(d_2.contact.dist, d.contact.dist)
    self.assertEqual(d_2.contact.frame.shape, (1, 9))
    np.testing.assert_allclose(d_2.contact.frame, d.contact.frame)

    # xmat, ximat, geom_xmat, site_xmat are all shape transformed
    self.assertEqual(d_2.xmat.shape, (3, 9))
    self.assertEqual(d_2.ximat.shape, (3, 9))
    self.assertEqual(d_2.geom_xmat.shape, (3, 9))
    self.assertEqual(d_2.site_xmat.shape, (1, 9))
    np.testing.assert_allclose(d_2.xmat, d.xmat)
    np.testing.assert_allclose(d_2.ximat, d.ximat)
    np.testing.assert_allclose(d_2.geom_xmat, d.geom_xmat)
    np.testing.assert_allclose(d_2.site_xmat, d.site_xmat)

    # efc_* are also shape transformed and filtered
    self.assertEqual(d_2.efc_J.shape, (64,))  # nefc * nv
    np.testing.assert_allclose(d_2.efc_J, d.efc_J)
    self.assertEqual(d_2.efc_aref.shape, (8,))  # nefc
    np.testing.assert_allclose(d_2.efc_aref, d.efc_aref)

    # efc_address is created on demand
    np.testing.assert_allclose(d_2.contact.efc_address, d.contact.efc_address)

  def test_get_data_batched(self):
    """Test that get_data makes correct List[MjData] for batched Data."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d)
    # second data in batch has contact dist > 0, disables contact
    dx_b = jax.tree_map(lambda x: jp.stack((x, x + 0.05)), dx)
    ds = mjx.get_data(m, dx_b)
    self.assertLen(ds, 2)
    np.testing.assert_allclose(ds[0].qpos, d.qpos)
    np.testing.assert_allclose(ds[1].qpos, d.qpos + 0.05, atol=1e-8)
    self.assertEqual(ds[0].ncon, 1)
    self.assertEqual(ds[1].ncon, 0)

  def test_get_data_into(self):
    """Test that get_data_into correctly populates an MjData."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d)
    d_2 = mujoco.MjData(m)
    mjx.get_data_into(d_2, m, dx)

    # check a few fields
    np.testing.assert_allclose(d_2.qpos, d.qpos)
    np.testing.assert_allclose(d_2.xpos, d.xpos)
    np.testing.assert_allclose(d_2.qM, d.qM)

    # only 1 contact active
    self.assertEqual(d_2.contact.dist.shape, (1,))
    self.assertEqual(d_2.ncon, 1)
    np.testing.assert_allclose(d_2.contact.dist, d.contact.dist)
    self.assertEqual(d_2.contact.frame.shape, (1, 9))
    np.testing.assert_allclose(d_2.contact.frame, d.contact.frame)

if __name__ == '__main__':
  absltest.main()
