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

import os
from unittest import mock
from absl.testing import absltest
from absl.testing import parameterized
import jax
from jax import numpy as jp
import mujoco
from mujoco import mjx
from mujoco.mjx._src import io as mjx_io
from mujoco.mjx._src import test_util

# pylint: disable=g-importing-member
from mujoco.mjx._src.types import ConeType
from mujoco.mjx._src.types import Impl
from mujoco.mjx._src.types import JacobianType
# pylint: enable=g-importing-member
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
      <geom type="plane" size="3 3 .01" condim="6"/>
      <body name="cap1" pos="-.3 -.3 .2">
        <freejoint/>
        <geom type="capsule" size=".2 .05"/>
        <body name="cap2" pos=".6 -.3 .3">
          <joint name="joint1" axis="0 1 0" type="hinge" range="-45 45"/>
          <joint name="joint2" axis="1 0 0" type="hinge" range="-0.001 0.001"/>
          <geom type="capsule" size=".2 .05"/>
          <site pos="-0.214 -0.078 0" quat="0.664 0.664 -0.242 -0.242"/>
        </body>
      </body>
    </worldbody>
    <equality>
      <connect body1="cap2" anchor="0 0 1"/>
    </equality>
    <tendon>
      <fixed name="tendon_1" limited="false" stiffness=".1" damping=".2">
        <joint joint="joint1" coef=".1"/>
        <joint joint="joint2" coef="-.2"/>
      </fixed>
    </tendon>
  </mujoco>
"""

_SIMPLE_BODY = """
  <mujoco>
    <worldbody>
      <!-- nC < nM because this body has an inertia-aligned free joint -->
      <body name="simplebody">
        <freejoint/>
        <geom type="sphere" size="0.01"/>
      </body>
    </worldbody>
  </mujoco>
"""


class ModelIOTest(parameterized.TestCase):
  """IO tests for mjx.Model."""

  @parameterized.product(
      xml=(_MULTIPLE_CONVEX_OBJECTS, _MULTIPLE_CONSTRAINTS),
      impl=('jax', 'c'),
  )
  def test_put_model(self, xml, impl):
    m = mujoco.MjModel.from_xml_string(xml)
    mx = mjx.put_model(m, impl=impl)

    def assert_not_weak_type(x):
      if isinstance(x, jax.Array):
        assert not x.weak_type

    jax.tree_util.tree_map(assert_not_weak_type, mx)
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

    if impl == 'jax':
      # fields restricted to MuJoCo should not be populated
      self.assertFalse(hasattr(mx, 'bvh_aabb'))
    elif impl == 'c':
      # Options specific to C are populated.
      self.assertEqual(mx.opt.apirate, m.opt.apirate)
      # Fields private to C backend impl are populated.
      self.assertTrue(hasattr(mx._impl, 'bvh_aabb'))

    np.testing.assert_allclose(mx.body_parentid, m.body_parentid)
    np.testing.assert_allclose(mx.geom_type, m.geom_type)
    np.testing.assert_allclose(mx.geom_bodyid, m.geom_bodyid)
    np.testing.assert_almost_equal(mx.geom_solref, m.geom_solref)
    np.testing.assert_almost_equal(mx.geom_pos, m.geom_pos)

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

    np.testing.assert_equal(mx.wrap_type, m.wrap_type)
    np.testing.assert_equal(mx.wrap_objid, m.wrap_objid)
    np.testing.assert_equal(mx.wrap_prm, m.wrap_prm)

  def test_fluid_params(self):
    """Test that has_fluid_params is set when fluid params are present."""
    m = mjx.put_model(
        mujoco.MjModel.from_xml_string(
            '<mujoco><option viscosity="3.0"/><worldbody/></mujoco>'
        ),
        impl='jax',
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

  def test_pgs_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string(
              '<mujoco><option solver="PGS"/><worldbody/></mujoco>'
          )
      )

  def test_margin_gap_mesh_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string("""
        <mujoco>
          <asset>
            <mesh name="box" vertex="-1 -1 -1 1 -1 -1 1 1 -1 1 1 1 1 -1 1 -1 1 -1 -1 1 1 -1 -1 1" scale="1 1 1"/>
          </asset>
          <worldbody>
            <body>
              <freejoint/>
              <geom type="mesh" mesh="box" margin="0.3"/>
            </body>
            <body>
              <freejoint/>
              <geom size="0.05"/>
            </body>
          </worldbody>
        </mujoco>"""),
          impl='jax',
      )

  def test_implicitfast_fluid_not_implemented(self):
    with self.assertRaises(NotImplementedError):
      mjx.put_model(
          mujoco.MjModel.from_xml_string("""
        <mujoco>
          <option viscosity="3.0" integrator="implicitfast"/>
          <worldbody/>
        </mujoco>"""),
          impl='jax',
      )

  def test_wrap_inside(self):
    m = test_util.load_test_file('tendon/wrap_sidesite.xml')
    mx0 = mjx.put_model(m, impl='jax')
    np.testing.assert_equal(
        mx0._impl.is_wrap_inside,
        np.array([1, 0, 1, 0, 1, 1, 0]),
    )
    m.site_pos[2] = m.site_pos[1]
    mx1 = mjx.put_model(m, impl='jax')
    np.testing.assert_equal(
        mx1._impl.is_wrap_inside,
        np.array([0, 0, 1, 0, 1, 0, 0]),
    )


class DataIOTest(parameterized.TestCase):
  """IO tests for mjx.Data."""

  @parameterized.parameters('jax', 'c')
  def test_make_data(self, impl: str):
    """Test that make_data returns the correct shapes."""
    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONVEX_OBJECTS)
    d = mjx.make_data(m, impl=impl)

    nq = 22
    nbody = 5
    ncon = 46
    nm = 64
    nv = 19
    nefc = 185

    self.assertEqual(d._impl.nefc, nefc)
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
    self.assertEqual(d._impl.cdof.shape, (nv, 6))
    self.assertEqual(d._impl.cinert.shape, (nbody, 10))
    self.assertEqual(d._impl.crb.shape, (nbody, 10))
    self.assertEqual(d._impl.actuator_length.shape, (1,))
    self.assertEqual(d._impl.actuator_moment.shape, (1, nv))
    self.assertEqual(d._impl.contact.dist.shape, (ncon,))
    self.assertEqual(d._impl.contact.pos.shape, (ncon, 3))
    self.assertEqual(d._impl.contact.frame.shape, (ncon, 3, 3))
    self.assertEqual(d._impl.contact.solref.shape, (ncon, 2))
    self.assertEqual(d._impl.contact.solimp.shape, (ncon, 5))
    self.assertEqual(d._impl.contact.geom1.shape, (ncon,))
    self.assertEqual(d._impl.contact.geom2.shape, (ncon,))
    self.assertEqual(d._impl.efc_J.shape, (nefc, nv))
    self.assertEqual(d._impl.efc_frictionloss.shape, (nefc,))
    self.assertEqual(d._impl.efc_D.shape, (nefc,))
    self.assertEqual(d._impl.actuator_velocity.shape, (1,))
    self.assertEqual(d.cvel.shape, (nbody, 6))
    self.assertEqual(d._impl.cdof_dot.shape, (nv, 6))
    self.assertEqual(d.qfrc_bias.shape, (nv,))
    self.assertEqual(d.qfrc_passive.shape, (nv,))
    self.assertEqual(d._impl.efc_aref.shape, (nefc,))
    self.assertEqual(d.qfrc_actuator.shape, (nv,))
    self.assertEqual(d.qfrc_smooth.shape, (nv,))
    self.assertEqual(d.qacc_smooth.shape, (nv,))
    self.assertEqual(d.qfrc_constraint.shape, (nv,))
    self.assertEqual(d.qfrc_inverse.shape, (nv,))
    self.assertEqual(d._impl.efc_force.shape, (nefc,))

    if impl == 'jax':
      self.assertEqual(d._impl.qM.shape, (nv, nv))
      self.assertEqual(d._impl.qLD.shape, (nv, nv))
      self.assertEqual(d._impl.qLDiagInv.shape, (0,))
    elif impl == 'c':
      self.assertEqual(d._impl.qM.shape, (nm,))
      self.assertEqual(d._impl.qLD.shape, (nm,))
      self.assertEqual(d._impl.qLDiagInv.shape, (nv,))

    # test sparse
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mjx.make_data(m, impl=impl)
    self.assertEqual(d._impl.qM.shape, (nm,))
    self.assertEqual(d._impl.qLD.shape, (nm,))
    self.assertEqual(d._impl.qLDiagInv.shape, (nv,))

    if impl == 'c':
      # check C specific fields
      self.assertEqual(d._impl.light_xpos.shape, (m.nlight, 3))
      self.assertEqual(d._impl.bvh_active.shape, (m.nbvh,))

  @parameterized.parameters('jax', 'c')
  def test_put_data(self, impl: str):
    """Test that put_data puts the correct data for dense and sparse."""
    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)

    # check a few fields
    np.testing.assert_allclose(dx.qpos, d.qpos)
    np.testing.assert_allclose(dx.xpos, d.xpos)
    np.testing.assert_allclose(dx.cvel, d.cvel)
    np.testing.assert_allclose(dx._impl.cdof_dot, d.cdof_dot)

    # check that there are no weak types
    self.assertFalse(
        any(
            jax.tree_util.tree_flatten(
                jax.tree_util.tree_map(lambda x: x.weak_type, dx)
            )[0]
        )
    )

    if impl == 'jax':
      # check that qM is transformed properly
      qm = np.zeros((m.nv, m.nv), dtype=np.float64)
      mujoco.mj_fullM(m, qm, d.qM)
      np.testing.assert_allclose(qm, mjx.full_m(mjx.put_model(m), dx))
    elif impl == 'c':
      np.testing.assert_allclose(dx._impl.qM, d.qM)
      np.testing.assert_allclose(dx._impl.qLD, d.qLD)
      np.testing.assert_allclose(dx._impl.qLDiagInv, d.qLDiagInv)

    # 4 contacts, 2 for each capsule against the plane
    self.assertEqual(dx._impl.contact.dist.shape, (4,))
    self.assertEqual(d.ncon, 1)  # however only 1 contact in this step
    np.testing.assert_allclose(dx._impl.contact.dist[0], d.contact.dist[0])
    self.assertTrue((dx._impl.contact.dist[1:] > 0).all())
    self.assertEqual(dx._impl.contact.frame.shape, (4, 3, 3))
    np.testing.assert_allclose(
        dx._impl.contact.frame[0].reshape(9), d.contact.frame[0]
    )
    np.testing.assert_allclose(dx._impl.contact.frame[1:], 0)

    # xmat, ximat, geom_xmat are all shape transformed
    self.assertEqual(dx.xmat.shape, (3, 3, 3))
    self.assertEqual(dx.ximat.shape, (3, 3, 3))
    self.assertEqual(dx.geom_xmat.shape, (3, 3, 3))
    self.assertEqual(dx.site_xmat.shape, (1, 3, 3))
    np.testing.assert_allclose(dx.xmat.reshape((3, 9)), d.xmat)
    np.testing.assert_allclose(dx.ximat.reshape((3, 9)), d.ximat)
    np.testing.assert_allclose(dx.geom_xmat.reshape((3, 9)), d.geom_xmat)
    np.testing.assert_allclose(dx.site_xmat.reshape((1, 9)), d.site_xmat)

    # tendon data is correct
    np.testing.assert_allclose(dx._impl.ten_length, d.ten_length)
    np.testing.assert_equal(dx._impl.ten_wrapadr, np.zeros((1,)))
    np.testing.assert_equal(dx._impl.ten_wrapnum, np.zeros((1,)))
    np.testing.assert_equal(dx._impl.wrap_obj, np.zeros((2, 2)))
    np.testing.assert_equal(dx._impl.wrap_xpos, np.zeros((2, 6)))

    # efc_ are also shape transformed and padded
    self.assertEqual(dx._impl.efc_J.shape, (45, 8))  # nefc, nv
    d_efc_j = d.efc_J.reshape((-1, 8))
    np.testing.assert_allclose(dx._impl.efc_J[:3], d_efc_j[:3])  # connect eq
    np.testing.assert_allclose(
        dx._impl.efc_J[3], d_efc_j[3]
    )  # one active limit
    np.testing.assert_allclose(dx._impl.efc_J[4], 0)  # one inactive limit
    np.testing.assert_allclose(dx._impl.efc_J[5:15], d_efc_j[4:14])  # contact
    np.testing.assert_allclose(dx._impl.efc_J[15:], 0)  # no contact

    # check another efc_ too
    self.assertEqual(dx._impl.efc_aref.shape, (45,))  # nefc
    np.testing.assert_allclose(dx._impl.efc_aref[:3], d.efc_aref[:3])
    np.testing.assert_allclose(dx._impl.efc_aref[3], d.efc_aref[3])
    np.testing.assert_allclose(dx._impl.efc_aref[4], 0)
    np.testing.assert_allclose(dx._impl.efc_aref[5:15], d.efc_aref[4:14])
    np.testing.assert_allclose(dx._impl.efc_aref[15:], 0)

    # check sparse transform is correct
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx_sparse = mjx.put_data(m, d, impl=impl)
    np.testing.assert_allclose(dx_sparse._impl.efc_J, dx._impl.efc_J, atol=1e-8)

    # check sparse mass matrices are correct
    np.testing.assert_allclose(dx_sparse._impl.qM, d.qM, atol=1e-8)
    np.testing.assert_allclose(dx_sparse._impl.qLD, d.qLD, atol=1e-8)
    np.testing.assert_allclose(
        dx_sparse._impl.qLDiagInv, d.qLDiagInv, atol=1e-8
    )

    # check dense mass matrices are correct
    m.opt.jacobian = mujoco.mjtJacobian.mjJAC_DENSE
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx_from_dense = mjx.put_data(m, d, impl=impl)
    if impl == 'jax':
      qm = np.zeros((m.nv, m.nv))
      mujoco.mj_fullM(m, qm, d.qM)
      np.testing.assert_allclose(dx_from_dense._impl.qM, qm, atol=1e-8)
    elif impl == 'c':
      np.testing.assert_allclose(dx_from_dense._impl.qM, d.qM, atol=1e-8)

  @parameterized.parameters(
      ('jax', False), ('jax', True), ('c', False), ('c', True)
  )
  def test_get_data(self, impl: str, sparse: bool):
    """Test that get_data makes correct MjData."""
    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    if sparse:
      m.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)
    d_2: mujoco.MjData = mjx.get_data(m, dx)

    # check a few fields
    np.testing.assert_allclose(d_2.qpos, d.qpos)
    np.testing.assert_allclose(d_2.xpos, d.xpos)
    np.testing.assert_allclose(d_2.cvel, d.cvel)
    np.testing.assert_allclose(d_2.cdof_dot, d.cdof_dot)
    np.testing.assert_allclose(d_2.qM, d.qM)
    np.testing.assert_allclose(d_2.qLD, d.qLD, atol=1e-6)
    np.testing.assert_allclose(d_2.qLDiagInv, d.qLDiagInv, atol=1e-6)

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
    self.assertEqual(d_2.nefc, 14)
    if sparse:
      efc_j = np.zeros((d.nefc, m.nv))
      mujoco.mju_sparse2dense(
          efc_j,
          d.efc_J,
          d.efc_J_rownnz,
          d.efc_J_rowadr,
          d.efc_J_colind,
      )
      efc_j2 = np.zeros((d_2.nefc, m.nv))
      mujoco.mju_sparse2dense(
          efc_j2,
          d_2.efc_J,
          d_2.efc_J_rownnz,
          d_2.efc_J_rowadr,
          d_2.efc_J_colind,
      )
      np.testing.assert_allclose(efc_j, efc_j2)
    else:
      self.assertEqual(d_2.efc_J.shape, (112,))  # nefc * nv
      np.testing.assert_allclose(d_2.efc_J, d.efc_J)
    self.assertEqual(d_2.efc_aref.shape, (14,))  # nefc
    np.testing.assert_allclose(d_2.efc_aref, d.efc_aref)
    np.testing.assert_allclose(d_2.contact.efc_address, d.contact.efc_address)

    if impl == 'c':
      # check fields specific to the C implementation
      np.testing.assert_allclose(d_2.bvh_active, d.bvh_active)

  def test_get_data_simplebody(self):
    """Test that get_data works with simple bodies where nC < nM."""
    m = mujoco.MjModel.from_xml_string(_SIMPLE_BODY)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d)
    d_2: mujoco.MjData = mjx.get_data(m, dx)
    np.testing.assert_allclose(d_2.qLD, d.qLD, atol=1e-6)
    np.testing.assert_allclose(d_2.qLDiagInv, d.qLDiagInv, atol=1e-6)

  def test_get_data_runs(self):
    xml = """
      <mujoco>
        <compiler autolimits="true"/>
        <worldbody>
          <body name="box">
            <joint name="slide1" type="slide" axis="1 0 0" />
            <geom type="box" size=".05 .05 .05" mass="1"/>
          </body>
        </worldbody>
        <actuator>
          <motor joint="slide1"/>
        </actuator>
      </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    d = mujoco.MjData(m)
    dx = mjx.put_data(m, d)
    mjx.get_data(m, dx)

  @parameterized.parameters('jax', 'c')
  def test_get_data_batched(self, impl):
    """Test that get_data makes correct List[MjData] for batched Data."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)
    # second data in batch has contact dist > 0, disables contact
    dx_b = jax.tree_util.tree_map(lambda x: jp.stack((x, x + 0.05)), dx)
    ds = mjx.get_data(m, dx_b)
    self.assertLen(ds, 2)
    np.testing.assert_allclose(ds[0].qpos, d.qpos)
    np.testing.assert_allclose(ds[1].qpos, d.qpos + 0.05, atol=1e-8)
    self.assertEqual(ds[0].ncon, 1)
    self.assertEqual(ds[1].ncon, 0)

  @parameterized.parameters('jax', 'c')
  def test_get_data_into(self, impl):
    """Test that get_data_into correctly populates an MjData."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)
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

  @parameterized.parameters('jax', 'c')
  def test_get_data_into_wrong_shape(self, impl):
    """Tests that get_data_into throwsif input and output shapes don't match."""

    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)
    m_2 = mujoco.MjModel.from_xml_string(_MULTIPLE_CONVEX_OBJECTS)
    d_2 = mujoco.MjData(m_2)
    with self.assertRaisesRegex(ValueError, r'Input field.*has shape.*'):
      mjx.get_data_into(d_2, m, dx)

  @parameterized.parameters('jax', 'c')
  def test_make_matches_put(self, impl):
    """Test that make_data produces a pytree that matches put_data."""
    m = mujoco.MjModel.from_xml_string(_MULTIPLE_CONSTRAINTS)
    d = mujoco.MjData(m)
    mujoco.mj_step(m, d, 2)
    dx = mjx.put_data(m, d, impl=impl)

    step_fn = lambda d: d.replace(time=d.time + 1)
    step_fn_jit = jax.jit(step_fn).lower(dx).compile()

    # placing an MjData onto device should yield the same treedef mjx.Data as
    # calling make_data.  they should be interchangeable for jax functions:
    step_fn_jit(mjx.make_data(m, impl=impl))

  def test_contact_elliptic_condim1(self):
    """Test that condim=1 with ConeType.ELLIPTIC is not implemented."""
    m = mujoco.MjModel.from_xml_string("""
      <mujoco>
        <worldbody>
          <geom size="0 0 1e-5" type="plane" condim="1"/>
          <body>
            <freejoint/>
            <geom size="0.1" condim="1"/>
          </body>
        </worldbody>
      </mujoco>
    """)
    m.opt.cone = ConeType.ELLIPTIC
    with self.assertRaises(NotImplementedError):
      mjx.make_data(m)

  @parameterized.product(
      sensor=['accelerometer', 'force', 'torque'], equality=['connect', 'weld']
  )
  def test_sensor_constraint_compatibility(self, sensor, equality):
    """Test unsupported sensor and equality constraint combinations."""
    equality_constraint = f'{equality} body1="body1" body2="body2"'
    if equality == 'connect':
      equality_constraint += ' anchor="0 0 0"'
    m = mujoco.MjModel.from_xml_string(f"""
        <mujoco>
          <worldbody>
            <body name="body1">
              <freejoint/>
              <geom size="0.1"/>
              <site name="site1"/>
            </body>
            <body name="body2">
              <freejoint/>
              <geom size="0.1"/>
            </body>
          </worldbody>
          <equality>
            <{equality_constraint}/>
          </equality>
          <sensor>
            <{sensor} site="site1"/>
          </sensor>
        </mujoco>
      """)
    with self.assertRaises(NotImplementedError):
      mjx.put_model(m, impl='jax')

  @parameterized.parameters(JacobianType.DENSE, JacobianType.SPARSE)
  def test_qm_mapm2m(self, jacobian):
    """Test that qM is mapped to M."""
    m = test_util.load_test_file('humanoid/humanoid.xml')
    m.opt.jacobian = jacobian
    d = mujoco.MjData(m)
    mx = mjx.put_model(m, impl='jax')
    dx = mjx.make_data(m, impl='jax')
    dx = mjx.forward(mx, dx)

    mjx.get_data_into(d, m, dx)

    res_mj = np.zeros((1, m.nv))
    mujoco.mj_solveM(m, d, res_mj, np.ones((1, m.nv)))

    res = mjx._src.smooth.solve_m(mx, dx, jp.ones(m.nv))

    np.testing.assert_allclose(res_mj[0], res, rtol=1e-3, atol=1e-3)


class FullCompatTest(parameterized.TestCase):
  """Tests for the _full_compat flag."""

  def test_full_compat_deprecated(self):
    """Tests that _full_compat is deprecated."""
    xml = """
      <mujoco>
        <worldbody>
          <body name="box">
            <joint name="slide1" type="slide" axis="1 0 0" />
            <geom type="box" size=".05 .05 .05" mass="1"/>
          </body>
        </worldbody>
        <actuator>
          <motor joint="slide1"/>
        </actuator>
      </mujoco>
    """
    m = mujoco.MjModel.from_xml_string(xml)
    with self.assertWarns(DeprecationWarning):
      out = mjx_io.put_model(m, _full_compat=True)
      self.assertEqual(out.impl, Impl.C)
    with self.assertWarns(DeprecationWarning):
      out = mjx_io.make_data(m, _full_compat=True)
      self.assertEqual(out.impl, Impl.C)


# Test cases for `_resolve_impl_and_device` where the device is
# specified by the user and the device is available.
_DEVICE_TEST_CASES = [
    # Arguments use the following format:
    # (device_type_str, impl_str,
    #  (expected_device, expected_impl)))
    # No backend specified.
    ('cpu', None, ('cpu', Impl.C)),
    ('gpu-notnvidia', None, ('gpu', Impl.JAX)),
    ('gpu-nvidia', None, ('gpu', Impl.WARP)),
    ('tpu', None, ('tpu', Impl.JAX)),
    # JAX backend specified.
    ('cpu', 'jax', ('cpu', Impl.JAX)),
    ('gpu-notnvidia', 'jax', ('gpu', Impl.JAX)),
    ('gpu-nvidia', 'jax', ('gpu', Impl.JAX)),
    ('tpu', 'jax', ('tpu', Impl.JAX)),
    # WARP backend specified.
    ('cpu', 'warp', ('cpu', 'error')),
    ('gpu-notnvidia', 'warp', ('cpu', 'error')),
    ('gpu-nvidia', 'warp', ('gpu', Impl.WARP)),
    ('tpu', 'warp', ('tpu', 'error')),
    # C backend specified.
    ('cpu', 'c', ('cpu', Impl.C)),
    ('gpu-notnvidia', 'c', ('cpu', 'error')),
    ('gpu-nvidia', 'c', ('cpu', 'error')),
    ('tpu', 'c', ('tpu', 'error')),
]

# Test cases for `_resolve_impl_and_device` where the user does NOT
# specify a device. We mock the JAX default device.
_DEFAULT_DEVICE_TEST_CASES = [
    # Arguments use the following format:
    # (jax.default_device, impl_str,
    #  (expected_device, expected_impl))
    # No backend impl specified.
    ('cpu', None, ('cpu', Impl.C)),
    ('gpu-notnvidia', None, ('gpu', Impl.JAX)),
    ('gpu-nvidia', None, ('gpu', Impl.WARP)),
    ('tpu', None, ('tpu', Impl.JAX)),
    # JAX backend impl specified.
    ('cpu', 'jax', ('cpu', Impl.JAX)),
    ('gpu-notnvidia', 'jax', ('gpu', Impl.JAX)),
    ('gpu-nvidia', 'jax', ('gpu', Impl.JAX)),
    ('tpu', 'jax', ('tpu', Impl.JAX)),
    # WARP backend impl specified.
    ('cpu', 'warp', ('cpu', 'error')),
    ('gpu-notnvidia', 'warp', ('cpu', 'error')),
    ('gpu-nvidia', 'warp', ('gpu', Impl.WARP)),
    ('tpu', 'warp', ('tpu', 'error')),
    # C backend impl specified, CPU should always be available.
    ('cpu', 'c', ('cpu', Impl.C)),
    ('gpu-notnvidia', 'c', ('cpu', Impl.C)),
    ('gpu-nvidia', 'c', ('cpu', Impl.C)),
    ('tpu', 'c', ('cpu', Impl.C)),
]


class ResolveImplAndDeviceTest(parameterized.TestCase):
  """Tests for the _resolve_impl_and_device function."""

  def setUp(self):
    super().setUp()

    # Create mock devices
    self.mock_cpu = mock.Mock(spec=jax.Device)
    self.mock_cpu.platform = 'cpu'
    self.mock_cpu.device_kind = 'Mock CPU'
    self.mock_cpu.id = 0

    self.mock_nvidia_gpu = mock.Mock(spec=jax.Device)
    self.mock_nvidia_gpu.platform = 'gpu'
    self.mock_nvidia_gpu.device_kind = 'NVIDIA Mocked GPU'
    self.mock_nvidia_gpu.id = 0

    self.mock_other_gpu = mock.Mock(spec=jax.Device)
    self.mock_other_gpu.platform = 'gpu'
    self.mock_other_gpu.device_kind = 'Other Mocked GPU'
    self.mock_other_gpu.id = 1

    self.mock_tpu = mock.Mock(spec=jax.Device)
    self.mock_tpu.platform = 'tpu'
    self.mock_tpu.device_kind = 'Mock TPU'
    self.mock_tpu.id = 0

    # Patch jax.devices for the entire test class using enter_context
    self.mock_jax_devices = self.enter_context(mock.patch('jax.devices'))
    self.mock_default_backend = self.enter_context(
        mock.patch('jax.default_backend')
    )

  @parameterized.named_parameters(
      (f'{str(args[0])}_{str(args[1])}', *args) for args in _DEVICE_TEST_CASES
  )
  @mock.patch.dict(
      os.environ, {'MJX_WARP_ENABLED': 'true', 'MJX_C_DEFAULT_ENABLED': 'true'}
  )
  def test_resolve_with_device(
      self,
      device_type_str,
      impl_str,
      expected,
  ):
    """Tests various combinations of device and backend impls."""
    input_device = {
        'cpu': self.mock_cpu,
        'gpu-nvidia': self.mock_nvidia_gpu,
        'gpu-notnvidia': self.mock_other_gpu,
        'tpu': self.mock_tpu,
    }[device_type_str]

    def devices_side_effect(backend=None):
      # assume the user-specified device is always available
      if backend == 'cpu':
        return [self.mock_cpu]
      elif backend == 'gpu':
        if 'nvidia' in device_type_str:
          return [self.mock_nvidia_gpu]
        return [self.mock_other_gpu]
      elif backend == 'tpu':
        return [self.mock_tpu]
      elif backend == 'cuda':
        return [self.mock_nvidia_gpu]

      raise AssertionError('Should not be called.')

    self.mock_jax_devices.side_effect = devices_side_effect

    expected_device, expected_impl = expected
    if expected_impl == 'error':
      with self.assertRaises(AssertionError):
        mjx_io._resolve_impl_and_device(
            impl=impl_str, device=input_device
        )
      return

    actual_impl, actual_device = (
        mjx_io._resolve_impl_and_device(
            impl=impl_str, device=input_device
        )
    )

    self.assertEqual(actual_impl, expected_impl)
    self.assertIsNotNone(actual_device)
    self.assertEqual(actual_device.platform, expected_device)

  @parameterized.named_parameters(
      (f'{str(args[0])}_{str(args[1])}', *args)
      for args in _DEFAULT_DEVICE_TEST_CASES
  )
  @mock.patch.dict(
      os.environ, {'MJX_WARP_ENABLED': 'true', 'MJX_C_DEFAULT_ENABLED': 'true'}
  )
  def test_resolve_without_device(
      self,
      default_device_str,
      impl_str,
      expected,
  ):
    """Tests various combinations of jax.default_device and backend impls."""
    default_devices = {
        'cpu': [self.mock_cpu],
        'gpu-nvidia': [self.mock_nvidia_gpu, self.mock_cpu],
        'gpu-notnvidia': [self.mock_other_gpu, self.mock_cpu],
        'tpu': [self.mock_tpu, self.mock_cpu],
    }[default_device_str]

    def devices_side_effect(backend=None):
      if backend == 'cpu':
        return [self.mock_cpu]  # CPU is always available
      if backend == 'gpu' and default_device_str == 'gpu-notnvidia':
        return [self.mock_other_gpu]
      if backend == 'gpu' and default_device_str == 'gpu-nvidia':
        return [self.mock_nvidia_gpu]
      if backend == 'cuda' and default_device_str == 'gpu-nvidia':
        return [self.mock_nvidia_gpu]
      if backend == 'tpu' and default_device_str == 'tpu':
        return [self.mock_tpu]
      if backend is None:
        return default_devices
      if backend == 'cuda':
        raise RuntimeError('cuda backend not supported')
      raise AssertionError('jax.devices error')

    self.mock_jax_devices.side_effect = devices_side_effect
    default_device_side_effect_str = {
        'cpu': 'cpu',
        'gpu-nvidia': 'gpu',
        'gpu-notnvidia': 'gpu',
        'tpu': 'tpu',
    }[default_device_str]
    self.mock_default_backend.side_effect = (
        lambda: default_device_side_effect_str
    )

    expected_device, expected_impl = expected
    if (
        expected_impl == 'error'
        and default_device_str != 'gpu-nvidia'
        and impl_str == 'warp'
    ):
      with self.assertRaisesRegex(RuntimeError, 'cuda backend not supported'):
        mjx_io._resolve_impl_and_device(
            impl=impl_str, device=None
        )
      return

    if expected_impl == 'error':
      with self.assertRaises(AssertionError):
        mjx_io._resolve_impl_and_device(
            impl=impl_str, device=None
        )
      return

    actual_impl, actual_device = (
        mjx_io._resolve_impl_and_device(
            impl=impl_str, device=None
        )
    )

    self.assertEqual(actual_impl, expected_impl)
    self.assertIsNotNone(actual_device)
    self.assertEqual(actual_device.platform, expected_device)

  @mock.patch.dict(os.environ, {'MJX_WARP_ENABLED': 'false'})
  def test_resolve_warp_disabled(self):
    """Tests behavior when MJX_WARP_ENABLED is false."""
    self.mock_jax_devices.side_effect = lambda backend=None: (
        [self.mock_nvidia_gpu, self.mock_cpu]
        if backend is None
        else ([self.mock_nvidia_gpu] if backend == 'gpu' else [self.mock_cpu])
    )
    self.mock_default_backend.side_effect = lambda: 'gpu'

    # Default to JAX instead of WARP on NVIDIA GPU.
    impl, device = mjx_io._resolve_impl_and_device(
        impl=None, device=None
    )
    self.assertEqual(impl, Impl.JAX)
    self.assertEqual(device.platform, 'gpu')

    # Specifying an NVIDIA GPU should still choose JAX.
    impl, device = mjx_io._resolve_impl_and_device(
        impl=None, device=self.mock_nvidia_gpu
    )
    self.assertEqual(impl, Impl.JAX)
    self.assertEqual(device.platform, 'gpu')

    # Requesting warp explicitly should fail since it is disabled.
    with self.assertRaises(AssertionError):
      mjx_io._resolve_impl_and_device(
          impl='warp', device=self.mock_nvidia_gpu
      )
    with self.assertRaises(AssertionError):
      mjx_io._resolve_impl_and_device(impl='warp', device=None)

  @mock.patch.dict(os.environ, {'MJX_C_DEFAULT_ENABLED': 'false'})
  def test_resolve_c_disabled(self):
    """Tests behavior when MJX_C_DEFAULT_ENABLED is false."""
    # Users expect that CPU defaults to the JAX impl. But in the future, it will
    # default to the C backend implementation. This test checks that
    # MJX_C_DEFAULT_ENABLED=false defaults to the old behavior, until the
    # migration to MJEP-15 is complete.
    self.mock_jax_devices.side_effect = lambda backend=None: ([self.mock_cpu])
    self.mock_default_backend.side_effect = lambda: 'cpu'

    # Default to JAX instead of C on CPU.
    impl, device = mjx_io._resolve_impl_and_device(
        impl=None, device=None
    )
    self.assertEqual(impl, Impl.JAX)
    self.assertEqual(device.platform, 'cpu')

    # Specifing CPU should still choose JAX.
    impl, device = mjx_io._resolve_impl_and_device(
        impl=None, device=self.mock_cpu
    )
    self.assertEqual(impl, Impl.JAX)
    self.assertEqual(device.platform, 'cpu')

    # Specifying C should choose C!
    impl, device = mjx_io._resolve_impl_and_device(
        impl='c', device=None
    )
    self.assertEqual(impl, Impl.C)
    self.assertEqual(device.platform, 'cpu')

    impl, device = mjx_io._resolve_impl_and_device(
        impl='c', device=self.mock_cpu
    )
    self.assertEqual(impl, Impl.C)
    self.assertEqual(device.platform, 'cpu')


if __name__ == '__main__':
  absltest.main()
