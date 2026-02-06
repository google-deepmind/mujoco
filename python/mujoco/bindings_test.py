# Copyright 2022 DeepMind Technologies Limited
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
"""Tests for MuJoCo Python bindings."""

import contextlib
import copy
from etils import epath
import pickle
import sys

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import numpy as np

TEST_XML = r"""
<mujoco model="test">
  <compiler coordinate="local" angle="radian" eulerseq="xyz"/>
  <size nkey="2"/>
  <option timestep="0.002" gravity="0 0 -9.81"/>
  <visual>
    <global fovy="50" />
    <quality shadowsize="51" />
  </visual>
  <worldbody>
    <geom name="myplane" type="plane" size="10 10 1"/>
    <body name="mybox" pos="0 0 0.1">
      <geom name="mybox" type="box" size="0.1 0.1 0.1" mass="0.25"/>
      <freejoint name="myfree"/>
    </body>
    <body>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <site pos="0 0 -1" name="mysite" type="sphere"/>
      <joint name="myhinge" type="hinge" axis="0 1 0" damping="1"/>
    </body>
    <body>
      <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
      <joint name="myball" type="ball"/>
    </body>
    <body mocap="true" pos="42 0 42">
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="myactuator" joint="myhinge"/>
  </actuator>
  <sensor>
    <jointvel name="myjointvel" joint="myhinge"/>
    <accelerometer name="myaccelerometer" site="mysite"/>
  </sensor>
</mujoco>
"""

TEST_XML_SENSOR = r"""
<mujoco model="test">
  <worldbody>
    <geom name="myplane" type="plane" size="10 10 1"/>
  </worldbody>
  <sensor>
    <user objtype="geom" objname="myplane"
          datatype="real" needstage="vel" dim="1"/>
  </sensor>
</mujoco>
"""

TEST_XML_PLUGIN = r"""
<mujoco>
  <option gravity="0 0 0"/>
  <extension>
    <plugin plugin="mujoco.elasticity.cable"/>
  </extension>
  <worldbody>
    <composite type="cable" curve="s" count="41 1 1" size="1" offset="0 0 1" initial="none">
      <plugin plugin="mujoco.elasticity.cable">
        <config key="twist" value="1e6"/>
        <config key="bend" value="1e9"/>
      </plugin>
      <joint kind="main" damping="2"/>
      <geom type="capsule" size=".005" density="1"/>
    </composite>
  </worldbody>
</mujoco>
"""

TEST_XML_TEXTURE = r"""
<mujoco>
  <asset>
    <texture name="tex" type="2d" builtin="checker" rgb1=".2 .3 .4" rgb2=".1 0.15 0.2"
      width="512" height="512" mark="cross" markrgb=".8 .8 .8"/>
    <material name="mat" reflectance="0.3" texture="tex" texrepeat="1 1" texuniform="true"/>
  </asset>
  <worldbody>
    <geom type="plane" size="1 1 1" material="mat"/>
  </worldbody>
</mujoco>
"""


@contextlib.contextmanager
def temporary_callback(setter, callback):
  setter(callback)
  yield
  setter(None)


class MuJoCoBindingsTest(parameterized.TestCase):

  def setUp(self):
    super().setUp()
    self.model: mujoco.MjModel = mujoco.MjModel.from_xml_string(TEST_XML)
    self.data = mujoco.MjData(self.model)

  def test_load_xml_can_handle_name_clash(self):
    xml_1 = r"""
<mujoco>
  <worldbody>
    <geom name="plane" type="plane" size="1 1 1"/>
    <include file="model_.xml"/>
    <include file="model__.xml"/>
  </worldbody>
</mujoco>"""
    xml_2 = rb"""<mujoco><geom name="box" type="box" size="1 1 1"/></mujoco>"""
    xml_3 = rb"""<mujoco><geom name="ball" type="sphere" size="1"/></mujoco>"""
    model = mujoco.MjModel.from_xml_string(
        xml_1, {'model_.xml': xml_2, 'model__.xml': xml_3}
    )
    self.assertEqual(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'plane'), 0
    )
    self.assertEqual(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'box'), 1
    )
    self.assertEqual(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'ball'), 2
    )

  def test_load_xml_repeated_asset_name(self):
    # Assets aren't allowed to have the same filename (even if they have
    # different paths).
    with self.assertRaisesRegex(ValueError, r'Repeated.*'):
      mujoco.MjModel.from_xml_string(
          '<mujoco/>', {'asset.xml': b'asset1', 'path/asset.xml': b'asset2'}
      )

  def test_can_read_array(self):
    np.testing.assert_array_equal(
        self.model.body_pos,
        [[0, 0, 0], [0, 0, 0.1], [0, 0, 0], [0, 0, 0], [42.0, 0, 42.0]],
    )

  def test_can_set_array(self):
    self.data.qpos = 0.12345
    np.testing.assert_array_equal(
        self.data.qpos, [0.12345] * len(self.data.qpos)
    )

  def test_array_is_a_view(self):
    qpos_ref = self.data.qpos
    self.data.qpos = 0.789
    np.testing.assert_array_equal(qpos_ref, [0.789] * len(self.data.qpos))

  # This test is disabled on PyPy as it uses sys.getrefcount
  # However PyPy is not officially supported by MuJoCo
  @absltest.skipIf(
      sys.implementation.name == 'pypy', reason='requires sys.getrefcount'
  )
  def test_array_keeps_struct_alive(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML)
    qpos0 = model.qpos0
    qpos_spring = model.qpos_spring

    # This only fails reliably under ASAN, which detects heap-use-after-free.
    # However, often the assertEqual is enough since the memory block is
    # already reused between mjModel deallocation and the subsequent read.
    qpos0[:] = 1
    del model
    self.assertEqual(qpos0[0], 1)

    # When running under test coverage tools, the refcount of objects can be
    # higher than normal. To take this into account, we first measure the
    # refcount of a dummy object with no other referrer.
    dummy = []
    base_refcount = sys.getrefcount(dummy) - 1

    # Here `base` is actually a PyCapsule that holds the raw mjModel* rather
    # than the actual MjModel wrapper object itself.
    capsule = qpos0.base
    self.assertEqual(sys.getrefcount(capsule) - base_refcount, 3)
    del qpos0
    self.assertEqual(sys.getrefcount(capsule) - base_refcount, 2)
    del qpos_spring
    self.assertEqual(sys.getrefcount(capsule) - base_refcount, 1)

  def test_named_indexing_actuator_ctrl(self):
    actuator_id = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'myactuator'
    )
    self.assertIs(
        self.data.actuator('myactuator'), self.data.actuator(actuator_id)
    )
    self.assertIs(
        self.data.actuator('myactuator').ctrl,
        self.data.actuator(actuator_id).ctrl,
    )
    self.assertEqual(self.data.actuator('myactuator').ctrl.shape, (1,))

    # Test that the indexer is returning a view into the underlying struct.
    ctrl_from_indexer = self.data.actuator('myactuator').ctrl
    self.data.ctrl[actuator_id] = 5
    np.testing.assert_array_equal(ctrl_from_indexer, [5])
    self.data.actuator('myactuator').ctrl = 7
    np.testing.assert_array_equal(self.data.ctrl[actuator_id], [7])

  def test_named_indexing_invalid_names_in_model(self):
    with self.assertRaisesRegex(
        KeyError,
        r"Invalid name 'badgeom'\. Valid names: \['mybox', 'myplane'\]",
    ):
      self.model.geom('badgeom')

  def test_named_indexing_no_name_argument_in_model(self):
    with self.assertRaisesRegex(
        KeyError,
        r"Invalid name ''\. Valid names: \['myball', 'myfree', 'myhinge'\]",
    ):
      self.model.joint()

  def test_named_indexing_invalid_names_in_data(self):
    with self.assertRaisesRegex(
        KeyError,
        r"Invalid name 'badgeom'\. Valid names: \['mybox', 'myplane'\]",
    ):
      self.data.geom('badgeom')

  def test_named_indexing_no_name_argument_in_data(self):
    with self.assertRaisesRegex(
        KeyError,
        r"Invalid name ''\. Valid names: \['myball', 'myfree', 'myhinge'\]",
    ):
      self.data.jnt()

  def test_named_indexing_invalid_index_in_model(self):
    with self.assertRaisesRegex(
        IndexError, r'Invalid index 3\. Valid indices from 0 to 2'
    ):
      self.model.geom(3)
    with self.assertRaisesRegex(
        IndexError, r'Invalid index -1\. Valid indices from 0 to 2'
    ):
      self.model.geom(-1)

  def test_named_indexing_invalid_index_in_data(self):
    with self.assertRaisesRegex(
        IndexError, r'Invalid index 3\. Valid indices from 0 to 2'
    ):
      self.data.geom(3)
    with self.assertRaisesRegex(
        IndexError, r'Invalid index -1\. Valid indices from 0 to 2'
    ):
      self.data.geom(-1)

  def test_named_indexing_geom_size(self):
    box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'mybox')
    self.assertIs(self.model.geom('mybox'), self.model.geom(box_id))
    self.assertIs(self.model.geom('mybox').size, self.model.geom(box_id).size)
    self.assertEqual(self.model.geom('mybox').size.shape, (3,))

    # Test that the indexer is returning a view into the underlying struct.
    size_from_indexer = self.model.geom('mybox').size
    self.model.geom_size[box_id] = [7, 11, 13]
    np.testing.assert_array_equal(size_from_indexer, [7, 11, 13])
    self.model.geom('mybox').size = [5, 3, 2]
    np.testing.assert_array_equal(self.model.geom_size[box_id], [5, 3, 2])

  def test_named_indexing_geom_quat(self):
    box_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, 'mybox')
    self.assertIs(self.model.geom('mybox'), self.model.geom(box_id))
    self.assertIs(self.model.geom('mybox').quat, self.model.geom(box_id).quat)
    self.assertEqual(self.model.geom('mybox').quat.shape, (4,))

    # Test that the indexer is returning a view into the underlying struct.
    quat_from_indexer = self.model.geom('mybox').quat
    self.model.geom_quat[box_id] = [5, 10, 15, 20]
    np.testing.assert_array_equal(quat_from_indexer, [5, 10, 15, 20])
    self.model.geom('mybox').quat = [12, 9, 6, 3]
    np.testing.assert_array_equal(self.model.geom_quat[box_id], [12, 9, 6, 3])

  def test_named_indexing_ragged_qpos(self):
    balljoint_id = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_JOINT, 'myball'
    )
    self.assertIs(self.data.joint('myball'), self.data.joint(balljoint_id))
    self.assertIs(
        self.data.joint('myball').qpos, self.data.joint(balljoint_id).qpos
    )
    self.assertEqual(self.data.joint('myball').qpos.shape, (4,))

    # Test that the indexer is returning a view into the underlying struct.
    qpos_from_indexer = self.data.joint('myball').qpos
    qpos_idx = self.model.jnt_qposadr[balljoint_id]
    self.data.qpos[qpos_idx : qpos_idx + 4] = [4, 5, 6, 7]
    np.testing.assert_array_equal(qpos_from_indexer, [4, 5, 6, 7])
    self.data.joint('myball').qpos = [9, 8, 7, 6]
    np.testing.assert_array_equal(
        self.data.qpos[qpos_idx : qpos_idx + 4], [9, 8, 7, 6]
    )

  def test_named_indexing_ragged2d_cdof(self):
    freejoint_id = mujoco.mj_name2id(
        self.model, mujoco.mjtObj.mjOBJ_JOINT, 'myfree'
    )
    self.assertIs(self.data.joint('myfree'), self.data.joint(freejoint_id))
    self.assertIs(
        self.data.joint('myfree').cdof, self.data.joint(freejoint_id).cdof
    )
    self.assertEqual(self.data.joint('myfree').cdof.shape, (6, 6))

    # Test that the indexer is returning a view into the underlying struct.
    cdof_from_indexer = self.data.joint('myfree').cdof
    dof_idx = self.model.jnt_dofadr[freejoint_id]
    self.data.cdof[dof_idx : dof_idx + 6, :] = np.reshape(range(36), (6, 6))
    np.testing.assert_array_equal(
        cdof_from_indexer, np.reshape(range(36), (6, 6))
    )
    self.data.joint('myfree').cdof = 42
    np.testing.assert_array_equal(
        self.data.cdof[dof_idx : dof_idx + 6], [[42] * 6] * 6
    )

  def test_named_indexing_repr_in_data(self):
    expected_repr = """<_MjDataGeomViews
  id: 1
  name: 'mybox'
  xmat: array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
  xpos: array([0., 0., 0.])
>"""
    self.assertEqual(expected_repr, repr(self.data.geom('mybox')))

  def test_named_indexing_body_repr_in_data(self):
    view_repr = repr(self.data.body('mybox'))
    self.assertStartsWith(view_repr, '<_MjDataBodyViews')
    self.assertIn('xpos: array([0., 0., 0.])', view_repr)
    self.assertEndsWith(view_repr, '>')

  def test_named_indexing_repr_in_model(self):
    view_repr = repr(self.model.geom('mybox'))
    self.assertStartsWith(view_repr, '<_MjModelGeomViews')
    self.assertIn('size: array([0.1, 0.1, 0.1])', view_repr)
    self.assertEndsWith(view_repr, '>')

  def test_addresses_differ_between_structs(self):
    model2 = mujoco.MjModel.from_xml_string(TEST_XML)
    data2 = mujoco.MjData(model2)

    self.assertGreater(self.model._address, 0)
    self.assertGreater(self.data._address, 0)
    self.assertGreater(model2._address, 0)
    self.assertGreater(data2._address, 0)
    self.assertLen(
        {
            self.model._address,
            self.data._address,
            model2._address,
            data2._address,
        },
        4,
    )

  def test_mjvisual_repr(self):
    # Regression test for issue #2488.
    vis_repr = repr(self.model.vis)
    self.assertNotEmpty(vis_repr)
    self.assertIn('MjVisual', vis_repr)

  def test_mjmodel_can_read_and_write_opt(self):
    self.assertEqual(self.model.opt.timestep, 0.002)
    np.testing.assert_array_equal(self.model.opt.gravity, [0, 0, -9.81])

    opt = self.model.opt
    self.model.opt.timestep = 0.001
    self.assertEqual(opt.timestep, 0.001)

    gravity = opt.gravity
    self.model.opt.gravity[1] = 0.1
    np.testing.assert_array_equal(gravity, [0, 0.1, -9.81])
    self.model.opt.gravity = 0.2
    np.testing.assert_array_equal(gravity, [0.2, 0.2, 0.2])

  def test_mjmodel_can_read_and_write_stat(self):
    self.assertNotEqual(self.model.stat.meanmass, 0)

    stat = self.model.stat
    self.model.stat.meanmass = 1.2
    self.assertEqual(stat.meanmass, 1.2)

  def test_mjmodel_can_read_and_write_vis(self):
    self.assertEqual(self.model.vis.quality.shadowsize, 51)

    self.model.vis.quality.shadowsize = 100
    self.assertEqual(self.model.vis.quality.shadowsize, 100)

  def test_mjmodel_can_access_names_directly(self):
    # mjModel offers direct access to names array, to allow usecases other than
    # id2name
    model_name = str(
        self.model.names[0 : self.model.names.find(b'\0')], 'utf-8'
    )
    self.assertEqual(model_name, 'test')

    start_index = self.model.name_geomadr[0]
    end_index = self.model.names.find(b'\0', start_index)
    geom_name = str(self.model.names[start_index:end_index], 'utf-8')
    self.assertEqual(geom_name, 'myplane')

  def test_mjmodel_names_doesnt_copy(self):
    names = self.model.names
    self.assertIs(names, self.model.names)

  def test_vis_global_exposed_as_global_(self):
    self.assertEqual(self.model.vis.global_.fovy, 50)
    self.model.vis.global_.fovy = 100
    self.assertEqual(self.model.vis.global_.fovy, 100)

  def test_mjoption_can_make_default(self):
    opt = mujoco.MjOption()
    self.assertEqual(opt.timestep, 0.002)
    np.testing.assert_array_equal(opt.gravity, [0, 0, -9.81])

  def test_mjoption_can_copy(self):
    opt1 = mujoco.MjOption()
    opt1.timestep = 0.001
    opt1.gravity = 2

    opt2 = copy.copy(opt1)
    self.assertEqual(opt2.timestep, 0.001)
    np.testing.assert_array_equal(opt2.gravity, [2, 2, 2])

    # Make sure opt2 is actually a copy.
    opt1.timestep = 0.005
    opt1.gravity = 5
    self.assertEqual(opt2.timestep, 0.001)
    np.testing.assert_array_equal(opt2.gravity, [2, 2, 2])

  def test_mjmodel_can_copy(self):
    model_copy = copy.copy(self.model)

    self.assertEqual(
        mujoco.mj_id2name(model_copy, mujoco.mjtObj.mjOBJ_JOINT, 0), 'myfree'
    )

    self.assertEqual(
        mujoco.mj_id2name(model_copy, mujoco.mjtObj.mjOBJ_GEOM, 0), 'myplane'
    )
    self.assertEqual(
        mujoco.mj_id2name(model_copy, mujoco.mjtObj.mjOBJ_GEOM, 1), 'mybox'
    )

    # Make sure it's a copy.
    self.model.geom_size[1] = 0.5
    np.testing.assert_array_equal(self.model.geom_size[1], [0.5, 0.5, 0.5])
    np.testing.assert_array_equal(model_copy.geom_size[1], [0.1, 0.1, 0.1])

  def test_mjdata_can_copy(self):
    self.data.qpos = [0, 0, 0.1*np.sqrt(2) - 0.001,
                      np.cos(np.pi/8), np.sin(np.pi/8), 0, 0, 0,
                      1, 0, 0, 0]  # fmt: skip
    mujoco.mj_forward(self.model, self.data)

    data_copy = copy.copy(self.data)
    self.assertEqual(data_copy.ncon, 2)

    # Make sure contact details are copied.
    self.assertEqual(data_copy.contact[0].dist, self.data.contact[0].dist)

    # Make sure it's a copy.
    mujoco.mj_resetData(self.model, self.data)
    mujoco.mj_forward(self.model, self.data)
    mujoco.mj_forward(self.model, data_copy)
    self.assertEqual(self.data.ncon, 4)
    self.assertEqual(data_copy.ncon, 2)

    mujoco.mj_resetData(self.model, data_copy)
    mujoco.mj_forward(self.model, data_copy)
    self.assertEqual(data_copy.ncon, 4)

    # Test copying into existing data.
    data_copy.qpos[1] = 1.234
    data_copy.geom_xpos[2] = 5.678
    self.assertFalse(np.array_equal(self.data.qpos, data_copy.qpos))
    self.assertFalse(np.array_equal(self.data.geom_xpos, data_copy.geom_xpos))
    mujoco.mj_copyData(self.data, self.model, data_copy)
    np.testing.assert_array_equal(self.data.qpos, data_copy.qpos)
    np.testing.assert_array_equal(self.data.geom_xpos, data_copy.geom_xpos)

  def test_mjdata_can_read_warning_array(self):
    warnings = self.data.warning
    self.assertLen(warnings, mujoco.mjtWarning.mjNWARNING)
    self.data.qpos[0] = float('NaN')
    mujoco.mj_checkPos(self.model, self.data)
    self.assertEqual(warnings[mujoco.mjtWarning.mjWARN_BADQPOS].number, 1)

  def test_mjcontact_can_copy(self):
    mujoco.mj_forward(self.model, self.data)

    contact_copy = []
    for i in range(4):
      contact_copy.append(copy.copy(self.data.contact[i]))
    # Sort contacts in anticlockwise order
    contact_copy = sorted(
        contact_copy, key=lambda x: np.arctan2(x.pos[1], x.pos[0])
    )
    np.testing.assert_allclose(contact_copy[0].pos[:2], [-0.1, -0.1])
    np.testing.assert_allclose(contact_copy[1].pos[:2], [0.1, -0.1])
    np.testing.assert_allclose(contact_copy[2].pos[:2], [0.1, 0.1])
    np.testing.assert_allclose(contact_copy[3].pos[:2], [-0.1, 0.1])

    # Make sure they're actually copies.
    for i in range(4):
      self.data.contact[i].pos[:2] = 55
    np.testing.assert_allclose(self.data.contact[0].pos[:2], [55, 55])
    np.testing.assert_allclose(self.data.contact[1].pos[:2], [55, 55])
    np.testing.assert_allclose(self.data.contact[2].pos[:2], [55, 55])
    np.testing.assert_allclose(self.data.contact[3].pos[:2], [55, 55])
    np.testing.assert_allclose(contact_copy[0].pos[:2], [-0.1, -0.1])
    np.testing.assert_allclose(contact_copy[1].pos[:2], [0.1, -0.1])
    np.testing.assert_allclose(contact_copy[2].pos[:2], [0.1, 0.1])
    np.testing.assert_allclose(contact_copy[3].pos[:2], [-0.1, 0.1])

  def test_mj_step(self):
    displacement = 0.25
    self.data.qpos[2] += displacement
    mujoco.mj_forward(self.model, self.data)

    gravity = -self.model.opt.gravity[2]
    expected_contact_time = np.sqrt(2 * displacement / gravity)

    # Grab a reference to the contacts upfront so that we know that they're
    # a view into mjData rather than a copy.
    contact = self.data.contact

    self.model.opt.timestep = 2**-9  # 0.001953125; allows exact comparisons
    self.assertEqual(self.data.time, 0)
    while self.data.time < expected_contact_time:
      self.assertEqual(self.data.ncon, 0)
      self.assertEmpty(self.data.efc_type)
      self.assertTrue(self.data.efc_type.flags['OWNDATA'])
      prev_time = self.data.time
      mujoco.mj_step(self.model, self.data)
      self.assertEqual(self.data.time, prev_time + self.model.opt.timestep)

    mujoco.mj_forward(self.model, self.data)
    self.assertEqual(self.data.ncon, 4)
    self.assertLen(self.data.efc_type, 16)
    self.assertFalse(self.data.efc_type.flags['OWNDATA'])

    # Sort contacts in anticlockwise order
    sorted_contact = sorted(
        contact, key=lambda x: np.arctan2(x.pos[1], x.pos[0])
    )
    np.testing.assert_allclose(sorted_contact[0].pos[:2], [-0.1, -0.1])
    np.testing.assert_allclose(sorted_contact[1].pos[:2], [0.1, -0.1])
    np.testing.assert_allclose(sorted_contact[2].pos[:2], [0.1, 0.1])
    np.testing.assert_allclose(sorted_contact[3].pos[:2], [-0.1, 0.1])

    mujoco.mj_resetData(self.model, self.data)
    self.assertEqual(self.data.ncon, 0)
    self.assertEmpty(self.data.efc_type)
    self.assertTrue(self.data.efc_type.flags['OWNDATA'])

  def test_mj_step_multiple(self):
    self.model.opt.timestep = 2**-9  # 0.001953125; allows exact comparisons
    self.assertEqual(self.data.time, 0)
    for _ in range(10):
      prev_time = self.data.time
      mujoco.mj_step(self.model, self.data, nstep=7)
      self.assertEqual(self.data.time, prev_time + 7 * self.model.opt.timestep)
    self.assertIn('Optionally, repeat nstep times.', mujoco.mj_step.__doc__)

  def test_mj_contact_list(self):
    self.assertEmpty(self.data.contact)

    expected_ncon = 4
    mujoco.mj_forward(self.model, self.data)
    self.assertLen(self.data.contact, expected_ncon)
    np.testing.assert_array_equal(self.data.contact.geom, [[0, 1]] * 4)

    expected_pos = []
    for contact in self.data.contact:
      expected_pos.append(np.random.uniform(size=3))
      contact.pos = expected_pos[-1]
    self.assertLen(expected_pos, expected_ncon)
    np.testing.assert_array_equal(self.data.contact.pos, expected_pos)

    expected_friction = []
    for contact in self.data.contact:
      expected_friction.append(np.random.uniform(size=5))
      contact.friction = expected_friction[-1]
    self.assertLen(expected_friction, expected_ncon)
    np.testing.assert_array_equal(self.data.contact.friction, expected_friction)

    expected_H = []  # pylint: disable=invalid-name
    for contact in self.data.contact:
      expected_H.append(np.random.uniform(size=36))
      contact.H = expected_H[-1]
    self.assertLen(expected_H, expected_ncon)
    np.testing.assert_array_equal(self.data.contact.H, expected_H)

    expected_geom = []
    for i, contact in enumerate(self.data.contact):
      expected_geom.append([i, i + 1])
      contact.geom = expected_geom[-1]
    self.assertLen(expected_geom, expected_ncon)
    np.testing.assert_array_equal(self.data.contact.geom, expected_geom)

  def test_realloc_con_efc(self):
    self.assertEmpty(self.data.contact)

    ncon = 9
    nefc = 11
    mujoco._functions._realloc_con_efc(self.data, ncon, nefc)

    ncon = 13
    nefc = 17
    nj = 21
    mujoco._functions._realloc_con_efc(self.data, ncon=ncon, nefc=nefc, nJ=nj)

    self.assertLen(self.data.contact, ncon)
    self.assertEqual(self.data.efc_id.shape, (nefc,))
    self.assertEqual(self.data.efc_J.shape, (nj,))
    self.assertEqual(self.data.efc_KBIP.shape, (nefc, 4))

    expected_error = (
        r'Insufficient arena memory, currently allocated memory=' +
        r'"[0-9]+[A-Z]?". Increase using <size memory="X"/>.'
    )
    with self.assertRaisesRegex(mujoco.FatalError, expected_error):
      mujoco._functions._realloc_con_efc(self.data, 100000000, 100000000)
    self.assertEmpty(self.data.contact)
    self.assertEmpty(self.data.efc_id)

  def test_mj_struct_list_equality(self):
    model2 = mujoco.MjModel.from_xml_string(TEST_XML)
    data2 = mujoco.MjData(model2)

    mujoco.mj_forward(self.model, self.data)
    self.assertEqual(self.data.ncon, 4)
    mujoco.mj_forward(model2, data2)
    self.assertEqual(data2.ncon, 4)
    self.assertEqual(data2.contact, self.data.contact)

    self.data.qpos[3:7] = [np.cos(np.pi / 8), np.sin(np.pi / 8), 0, 0]
    self.data.qpos[2] *= (np.sqrt(2) - 1) * 0.1 - 1e-6
    mujoco.mj_forward(self.model, self.data)
    self.assertEqual(self.data.ncon, 2)
    self.assertNotEqual(data2.contact, self.data.contact)

    # Check that we can compare slices of different lengths
    self.assertNotEqual(data2.contact, self.data.contact)

    # Check that comparing things of different types do not raise an error
    self.assertNotEqual(self.data.contact, self.data.warning)
    self.assertNotEqual(self.data.contact, 5)

  @parameterized.named_parameters([
      ('MjOption', mujoco.MjOption, 'tolerance'),
      ('MjWarningStat', mujoco.MjWarningStat, 'number'),
      ('MjTimerStat', mujoco.MjTimerStat, 'number'),
      ('MjSolverStat', mujoco.MjSolverStat, 'neval'),
      ('MjContact', mujoco.MjContact, 'dist'),
      ('MjStatistic', mujoco.MjStatistic, 'extent'),
      ('MjLROpt', mujoco.MjLROpt, 'maxforce'),
      ('MjvPerturb', mujoco.MjvPerturb, 'select'),
      ('MjvCamera', mujoco.MjvCamera, 'fixedcamid'),
  ])
  def test_mj_struct_equality(self, cls, attr):
    struct = cls()
    struct2 = cls()
    setattr(struct, attr, 1)
    self.assertNotEqual(struct, struct2)
    setattr(struct2, attr, 1)
    self.assertEqual(struct, struct2)

    self.assertNotEqual(struct, 3)
    self.assertIsNotNone(struct)

    # mutable structs shouldn't declare __hash__
    with self.assertRaises(TypeError):
      hash(struct)

  def test_mj_struct_equality_array(self):
    contact1 = mujoco.MjContact()
    contact2 = mujoco.MjContact()
    contact1.H[3] = 1
    self.assertNotEqual(contact1, contact2)
    contact2.H[3] = 1
    self.assertEqual(contact1, contact2)

  @parameterized.named_parameters([
      ('MjOption', mujoco.MjOption, 'tolerance'),
      ('MjWarningStat', mujoco.MjWarningStat, 'number'),
      ('MjTimerStat', mujoco.MjTimerStat, 'number'),
      ('MjSolverStat', mujoco.MjSolverStat, 'neval'),
      ('MjContact', mujoco.MjContact, 'dist'),
      ('MjStatistic', mujoco.MjStatistic, 'extent'),
      ('MjLROpt', mujoco.MjLROpt, 'maxforce'),
      ('MjvPerturb', mujoco.MjvPerturb, 'select'),
      ('MjvCamera', mujoco.MjvCamera, 'fixedcamid'),
  ])
  def test_mj_struct_repr(self, cls, attr):
    struct = cls()
    setattr(struct, attr, 1)
    representation = repr(struct)
    self.assertStartsWith(representation, f'<{cls.__name__}')
    self.assertIn(f'{attr}: 1', representation)
    self.assertEqual(str(struct), repr(struct))

  def test_mj_struct_repr_for_subclass(self):
    class MjWarningStatSubclass(mujoco.MjWarningStat):
      # ptr attribute could cause an infinite recursion, if the repr
      # implementation simply looked at all attributes.

      @property
      def ptr(self):
        return self

    # repr should include name of subclass.
    expected_repr = """<MjWarningStatSubclass
  lastinfo: 0
  number: 0
>"""

    self.assertEqual(repr(MjWarningStatSubclass()), expected_repr)

  def test_mju_rotVecQuat(self):  # pylint: disable=invalid-name
    vec = [1, 0, 0]
    quat = [np.cos(np.pi / 8), 0, 0, np.sin(np.pi / 8)]
    expected = np.array([1, 1, 0]) / np.sqrt(2)

    # Check that the output argument works, and that the binding returns None.
    res = np.empty(3, np.float64)
    self.assertIsNone(mujoco.mju_rotVecQuat(res, vec, quat))
    np.testing.assert_allclose(res, expected)

    # Check that the function can be called via keyword arguments.
    mujoco.mju_rotVecQuat(vec=vec, quat=quat, res=res)
    np.testing.assert_allclose(res, expected)

    # Check that the res argument must have the right size.
    with self.assertRaises(TypeError):
      mujoco.mju_rotVecQuat(np.zeros(4, np.float64), vec, quat)

    # Check that the vec argument must have the right size.
    with self.assertRaises(TypeError):
      mujoco.mju_rotVecQuat(res, [1, 2, 3, 4], quat)

    # Check that the quat argument must have the right size.
    with self.assertRaises(TypeError):
      mujoco.mju_rotVecQuat(res, vec, [1, 2, 3])

    # The following check needs to be done with a fully initialized array,
    # since pybind11 prints out the array's contents when generating TypeErrors.
    # Using `np.empty` here results in msan errors.

    # Check that the output argument must have the correct dtype.
    with self.assertRaises(TypeError):
      res = np.zeros(3, np.int32)
      mujoco.mju_rotVecQuat(res, vec, quat)

  def test_getsetstate(self):  # pylint: disable=invalid-name
    mujoco.mj_step(self.model, self.data)

    # Test for invalid state signature
    invalid_sig = 2**mujoco.mjtState.mjNSTATE.value
    expected_message = (
        f'mj_stateSize: invalid state signature {invalid_sig} >= 2^mjNSTATE'
    )
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, expected_message):
      mujoco.mj_stateSize(self.model, invalid_sig)

    sig = mujoco.mjtState.mjSTATE_INTEGRATION
    size = mujoco.mj_stateSize(self.model, sig)

    state_bad_size = np.empty(size + 1, np.float64)
    expected_message = 'state size should equal mj_stateSize(m, sig)'
    with self.assertRaisesWithLiteralMatch(TypeError, expected_message):
      mujoco.mj_getState(self.model, self.data, state_bad_size, sig)

    # Get initial state.
    state0 = np.empty(size, np.float64)
    mujoco.mj_getState(self.model, self.data, state0, sig)

    # Step, get next state.
    mujoco.mj_step(self.model, self.data)
    state1a = np.empty(size, np.float64)
    mujoco.mj_getState(self.model, self.data, state1a, sig)

    # Reset to initial state, step again, get state again.
    mujoco.mj_setState(self.model, self.data, state0, sig)
    mujoco.mj_step(self.model, self.data)
    state1b = np.empty(size, np.float64)
    mujoco.mj_getState(self.model, self.data, state1b, sig)

    # Expect next states to be equal.
    np.testing.assert_array_equal(state1a, state1b)

    # Test mj_copyState
    data2 = mujoco.MjData(self.model)
    mujoco.mj_copyState(self.model, self.data, data2, sig)
    state1c = np.empty(size, np.float64)
    mujoco.mj_getState(self.model, data2, state1c, sig)
    np.testing.assert_array_equal(state1a, state1c)

  def test_mj_setKeyframe(self):  # pylint: disable=invalid-name
    mujoco.mj_step(self.model, self.data)

    # Test for invalid keyframe
    invalid_key = 2
    expected_message = (
        f'mj_setKeyframe: index must be smaller than {invalid_key} (keyframes'
        ' allocated in model)'
    )
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, expected_message):
      mujoco.mj_setKeyframe(self.model, self.data, invalid_key)

    valid_key = 1
    time = self.data.time
    qpos = self.data.qpos.copy()
    qvel = self.data.qvel.copy()
    act = self.data.act.copy()
    mujoco.mj_setKeyframe(self.model, self.data, valid_key)

    # Step, assert that time has changed.
    mujoco.mj_step(self.model, self.data)
    self.assertNotEqual(time, self.data.time)

    # Reset to keyframe, assert that time, qpos, qvel, act are the same.
    mujoco.mj_resetDataKeyframe(self.model, self.data, valid_key)
    self.assertEqual(time, self.data.time)
    np.testing.assert_array_equal(qpos, self.data.qpos)
    np.testing.assert_array_equal(qvel, self.data.qvel)
    np.testing.assert_array_equal(act, self.data.act)

  def test_mj_angmomMat(self):  # pylint: disable=invalid-name
    self.data.qvel = np.ones(self.model.nv, np.float64)
    mujoco.mj_forward(self.model, self.data)
    mujoco.mj_subtreeVel(self.model, self.data)

    mat = np.empty((3, 10), np.float64)
    mujoco.mj_angmomMat(self.model, self.data, mat, 0)
    np.testing.assert_almost_equal(
        mat @ self.data.qvel, self.data.subtree_angmom[0, :]
    )

  def test_mj_jacSite(self):  # pylint: disable=invalid-name
    mujoco.mj_forward(self.model, self.data)
    site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'mysite')

    # Call mj_jacSite with only jacp.
    jacp = np.empty((3, 10), np.float64)
    mujoco.mj_jacSite(self.model, self.data, jacp, None, site_id)

    expected_jacp = np.array([
        [0, 0, 0, 0, 0, 0, -1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    np.testing.assert_array_equal(jacp, expected_jacp)

    # Call mj_jacSite with only jacr.
    jacr = np.empty((3, 10), np.float64)
    mujoco.mj_jacSite(self.model, self.data, None, jacr, site_id)

    expected_jacr = np.array([
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ])
    np.testing.assert_array_equal(jacr, expected_jacr)

    # Call mj_jacSite with both jacp and jacr.
    jacp[:] = 0
    jacr[:] = 0
    mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)
    np.testing.assert_array_equal(jacp, expected_jacp)
    np.testing.assert_array_equal(jacr, expected_jacr)

    # Check that the jacp argument must have the right size.
    with self.assertRaises(TypeError):
      mujoco.mj_jacSite(
          self.model, self.data, np.empty((3, 6), jacp.dtype), None, site_id
      )

    # Check that the jacr argument must have the right size.
    with self.assertRaises(TypeError):
      mujoco.mj_jacSite(
          self.model, self.data, None, np.empty((4, 7), jacr.dtype), site_id
      )

    # The following two checks need to be done with fully initialized arrays,
    # since pybind11 prints out the array's contents when generating TypeErrors.
    # Using `np.empty` here results in msan errors.

    # Check that the jacp argument must have the right dtype.
    with self.assertRaises(TypeError):
      mujoco.mj_jacSite(
          self.model, self.data, np.zeros(jacp.shape, int), None, site_id
      )

    # Check that the jacr argument must have the right dtype.
    with self.assertRaises(TypeError):
      mujoco.mj_jacSite(
          self.model, self.data, None, np.zeros(jacr.shape, int), site_id
      )

  def test_docstrings(self):  # pylint: disable=invalid-name
    self.assertEqual(
        mujoco.mj_versionString.__doc__,
        """mj_versionString() -> str

Return the current version of MuJoCo as a null-terminated string.
""",
    )
    self.assertEqual(
        mujoco.mj_Euler.__doc__,
        """mj_Euler(m: mujoco._structs.MjModel, d: mujoco._structs.MjData) -> None

Euler integrator, semi-implicit in velocity.
""",
    )

  def test_float_constant(self):
    self.assertEqual(mujoco.mjMAXVAL, 1e10)
    self.assertEqual(mujoco.mjMINVAL, 1e-15)

  def test_string_constants(self):
    self.assertLen(mujoco.mjDISABLESTRING, mujoco.mjtDisableBit.mjNDISABLE)
    self.assertLen(mujoco.mjENABLESTRING, mujoco.mjtEnableBit.mjNENABLE)
    self.assertLen(mujoco.mjTIMERSTRING, mujoco.mjtTimer.mjNTIMER)
    self.assertLen(mujoco.mjLABELSTRING, mujoco.mjtLabel.mjNLABEL)
    self.assertLen(mujoco.mjFRAMESTRING, mujoco.mjtFrame.mjNFRAME)
    self.assertLen(mujoco.mjVISSTRING, mujoco.mjtVisFlag.mjNVISFLAG)
    self.assertLen(mujoco.mjRNDSTRING, mujoco.mjtRndFlag.mjNRNDFLAG)
    self.assertEqual(mujoco.mjDISABLESTRING[11], 'Actuation')
    self.assertEqual(
        mujoco.mjVISSTRING[mujoco.mjtVisFlag.mjVIS_INERTIA],
        ('Inertia', '0', 'I'),
    )

  def test_enum_values(self):
    self.assertEqual(mujoco.mjtJoint.mjJNT_FREE, 0)
    self.assertEqual(mujoco.mjtJoint.mjJNT_BALL, 1)
    self.assertEqual(mujoco.mjtJoint.mjJNT_SLIDE, 2)
    self.assertEqual(mujoco.mjtJoint.mjJNT_HINGE, 3)
    self.assertEqual(mujoco.mjtEnableBit.mjENBL_OVERRIDE, 1 << 0)
    self.assertEqual(mujoco.mjtEnableBit.mjENBL_ENERGY, 1 << 1)
    self.assertEqual(mujoco.mjtEnableBit.mjENBL_FWDINV, 1 << 2)
    self.assertEqual(mujoco.mjtEnableBit.mjNENABLE, 6)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_PLANE, 0)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_HFIELD, 1)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_SPHERE, 2)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_ARROW, 100)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_ARROW1, 101)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_ARROW2, 102)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_TRIANGLE, 108)
    self.assertEqual(mujoco.mjtGeom.mjGEOM_NONE, 1001)

  def test_enum_from_int(self):
    self.assertEqual(mujoco.mjtJoint.mjJNT_FREE, mujoco.mjtJoint(0))
    self.assertEqual(mujoco.mjtGeom.mjGEOM_ARROW, mujoco.mjtGeom(value=100))
    with self.assertRaises(ValueError):
      mujoco.mjtJoint(1000)
    with self.assertRaises(ValueError):
      mujoco.mjtJoint(-1)

  def test_enum_as_index(self):
    x = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k']
    self.assertEqual(x[mujoco.mjtFrame.mjFRAME_WORLD], 'h')
    self.assertEqual(
        x[mujoco.mjtFrame.mjFRAME_GEOM : mujoco.mjtFrame.mjFRAME_CAMERA],
        ['c', 'd'],
    )

  def test_enum_ops(self):
    # Note: when modifying this test, make sure the enum value is an odd number
    #       so that the division tests are correctly exercised.
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD, 7)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD, 7.0)
    self.assertEqual(7, mujoco.mjtFrame.mjFRAME_WORLD)
    self.assertEqual(7.0, mujoco.mjtFrame.mjFRAME_WORLD)
    self.assertEqual(
        mujoco.mjtFrame.mjFRAME_WORLD, mujoco.mjtFrame.mjFRAME_WORLD
    )
    self.assertNotEqual(
        mujoco.mjtFrame.mjFRAME_WORLD, mujoco.mjtFrame.mjFRAME_NONE
    )

    self.assertEqual(-mujoco.mjtFrame.mjFRAME_WORLD, -7)
    self.assertIsInstance(-mujoco.mjtFrame.mjFRAME_WORLD, int)

    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD + 1, 8)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD + 1, int)
    self.assertEqual(2 + mujoco.mjtFrame.mjFRAME_WORLD, 9)
    self.assertIsInstance(2 + mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD + 1.75, 8.75)
    self.assertEqual(2.75 + mujoco.mjtFrame.mjFRAME_WORLD, 9.75)

    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD - 2, 5)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD - 2, int)
    self.assertEqual(8 - mujoco.mjtFrame.mjFRAME_WORLD, 1)
    self.assertIsInstance(8 - mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD - 2.25, 4.75)
    self.assertEqual(8.25 - mujoco.mjtFrame.mjFRAME_WORLD, 1.25)

    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD * 3, 21)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD * 3, int)
    self.assertEqual(3 * mujoco.mjtFrame.mjFRAME_WORLD, 21)
    self.assertIsInstance(3 * mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD * 3.5, 24.5)
    self.assertEqual(3.5 * mujoco.mjtFrame.mjFRAME_WORLD, 24.5)

    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD / 2, 3.5)
    self.assertEqual(17.5 / mujoco.mjtFrame.mjFRAME_WORLD, 2.5)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD // 2, 3)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD // 2, int)
    self.assertEqual(-mujoco.mjtFrame.mjFRAME_WORLD // 2, -4)
    self.assertIsInstance(-mujoco.mjtFrame.mjFRAME_WORLD // 2, int)
    self.assertEqual(20 // mujoco.mjtFrame.mjFRAME_WORLD, 2)
    self.assertIsInstance(20 // mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(-20 // mujoco.mjtFrame.mjFRAME_WORLD, -3)
    self.assertIsInstance(-20 // mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD // 2.0, 3)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD // 2.0, float)
    self.assertEqual(-mujoco.mjtFrame.mjFRAME_WORLD // 2.0, -4)
    self.assertIsInstance(-mujoco.mjtFrame.mjFRAME_WORLD // 2.0, float)
    self.assertEqual(20.0 // mujoco.mjtFrame.mjFRAME_WORLD, 2)
    self.assertIsInstance(20.0 // mujoco.mjtFrame.mjFRAME_WORLD, float)
    self.assertEqual(-20 // mujoco.mjtFrame.mjFRAME_WORLD, -3)
    self.assertIsInstance(-20.0 // mujoco.mjtFrame.mjFRAME_WORLD, float)

    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD % 4, 3)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD % 4, int)
    self.assertEqual(-mujoco.mjtFrame.mjFRAME_WORLD % -4, -3)
    self.assertIsInstance(-mujoco.mjtFrame.mjFRAME_WORLD % -4, int)
    self.assertEqual(-mujoco.mjtFrame.mjFRAME_WORLD % 4, 1)
    self.assertIsInstance(-mujoco.mjtFrame.mjFRAME_WORLD % 4, int)
    self.assertEqual(mujoco.mjtFrame.mjFRAME_WORLD % -4, -1)
    self.assertIsInstance(mujoco.mjtFrame.mjFRAME_WORLD % -4, int)
    self.assertEqual(9 % mujoco.mjtFrame.mjFRAME_WORLD, 2)
    self.assertIsInstance(9 % mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(-9 % -mujoco.mjtFrame.mjFRAME_WORLD, -2)
    self.assertIsInstance(-9 % -mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(-9 % mujoco.mjtFrame.mjFRAME_WORLD, 5)
    self.assertIsInstance(-9 % mujoco.mjtFrame.mjFRAME_WORLD, int)
    self.assertEqual(9 % -mujoco.mjtFrame.mjFRAME_WORLD, -5)
    self.assertIsInstance(9 % -mujoco.mjtFrame.mjFRAME_WORLD, int)

    with self.assertRaises(ZeroDivisionError):
      _ = mujoco.mjtFrame.mjFRAME_WORLD / 0
    with self.assertRaises(ZeroDivisionError):
      _ = 1 / mujoco.mjtFrame.mjFRAME_NONE
    with self.assertRaises(ZeroDivisionError):
      _ = mujoco.mjtFrame.mjFRAME_WORLD // 0
    with self.assertRaises(ZeroDivisionError):
      _ = 1 // mujoco.mjtFrame.mjFRAME_NONE
    with self.assertRaises(ZeroDivisionError):
      _ = mujoco.mjtFrame.mjFRAME_WORLD % 0
    with self.assertRaises(ZeroDivisionError):
      _ = 1 % mujoco.mjtFrame.mjFRAME_NONE

    self.assertEqual(
        mujoco.mjtDisableBit.mjDSBL_GRAVITY | mujoco.mjtDisableBit.mjDSBL_LIMIT,
        136,
    )
    self.assertEqual(mujoco.mjtDisableBit.mjDSBL_SPRING | 33, 33)
    self.assertEqual(mujoco.mjtDisableBit.mjDSBL_SPRING & 33, 32)
    self.assertEqual(mujoco.mjtDisableBit.mjDSBL_SPRING ^ 33, 1)
    self.assertEqual(33 | mujoco.mjtDisableBit.mjDSBL_SPRING, 33)
    self.assertEqual(33 & mujoco.mjtDisableBit.mjDSBL_SPRING, 32)
    self.assertEqual(33 ^ mujoco.mjtDisableBit.mjDSBL_SPRING, 1)
    self.assertEqual(
        mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL << 1,
        mujoco.mjtDisableBit.mjDSBL_WARMSTART,
    )
    self.assertEqual(
        mujoco.mjtDisableBit.mjDSBL_CLAMPCTRL >> 4,
        mujoco.mjtDisableBit.mjDSBL_CONTACT,
    )

  def test_can_raise_error(self):
    self.data.pstack = self.data.narena
    with self.assertRaisesRegex(
        mujoco.FatalError, r'\Amj_stackAlloc: out of memory, stack overflow'
    ):
      mujoco.mj_forward(self.model, self.data)

  def test_timer_installed_by_default(self):
    timer_step = mujoco.mjtTimer.mjTIMER_STEP
    self.assertEqual(self.data.timer[timer_step].number, 0)
    self.assertEqual(self.data.timer[timer_step].duration, 0.0)

    mujoco.mj_step(self.model, self.data)
    self.assertEqual(self.data.timer[timer_step].number, 1)
    duration_1 = self.data.timer[timer_step].duration
    self.assertGreater(duration_1, 0.0)

    mujoco.mj_step(self.model, self.data, 5)
    self.assertEqual(self.data.timer[timer_step].number, 6)
    self.assertGreater(self.data.timer[timer_step].duration, duration_1)

  def test_mjcb_time(self):

    class CallCounter:

      def __init__(self):
        self.count = 0

      def __call__(self):
        self.count += 1
        return self.count - 1

    call_counter = CallCounter()
    with temporary_callback(mujoco.set_mjcb_time, call_counter):
      self.assertIs(mujoco.get_mjcb_time(), call_counter)

      # Check that the callback setter and getter aren't callin g the function.
      self.assertEqual(call_counter.count, 0)

      mujoco.mj_forward(self.model, self.data)
      self.assertGreater(call_counter.count, 0)

    self.assertIsNone(mujoco.get_mjcb_time())

  def test_mjcb_time_exception(self):

    class TestError(RuntimeError):
      pass

    def raises_exception():
      raise TestError('string', (1, 2, 3), {'a': 1, 'b': 2})

    with temporary_callback(mujoco.set_mjcb_time, raises_exception):
      with self.assertRaises(TestError) as e:
        mujoco.mj_forward(self.model, self.data)
      self.assertEqual(
          e.exception.args, ('string', (1, 2, 3), {'a': 1, 'b': 2})
      )

    # Should not raise now that we've cleared the callback.
    mujoco.mj_forward(self.model, self.data)

  def test_mjcb_time_wrong_return_type(self):
    with temporary_callback(mujoco.set_mjcb_time, lambda: 'string'):
      with self.assertRaisesWithLiteralMatch(
          TypeError, 'mjcb_time callback did not return a number'
      ):
        mujoco.mj_forward(self.model, self.data)

  def test_mjcb_time_not_callable(self):
    with self.assertRaisesWithLiteralMatch(
        TypeError, 'callback is not an Optional[Callable]'
    ):
      mujoco.set_mjcb_time(1)

  def test_mjcb_sensor(self):

    class SensorCallback:

      def __init__(self, test, expected_model, expected_data):
        self.test = test
        self.expected_model = expected_model
        self.expected_data = expected_data
        self.count = 0

      def __call__(self, m, d, stage):
        self.test.assertIs(m, self.expected_model)
        self.test.assertIs(d, self.expected_data)
        self.test.assertEqual(stage, mujoco.mjtStage.mjSTAGE_VEL)
        d.sensordata[0] = 17
        self.count += 1

    model_with_sensor = mujoco.MjModel.from_xml_string(TEST_XML_SENSOR)
    data_with_sensor = mujoco.MjData(model_with_sensor)
    sensor_callback = SensorCallback(self, model_with_sensor, data_with_sensor)
    self.assertEqual(sensor_callback.count, 0)

    with temporary_callback(mujoco.set_mjcb_sensor, sensor_callback):
      mujoco.mj_forward(model_with_sensor, data_with_sensor)

    self.assertEqual(sensor_callback.count, 1)
    self.assertEqual(data_with_sensor.sensordata[0], 17)

  # This test is disabled on PyPy as it uses sys.getrefcount
  # However PyPy is not officially supported by MuJoCo
  @absltest.skipIf(
      sys.implementation.name == 'pypy', reason='requires sys.getrefcount'
  )
  def test_mjcb_control_not_leak_memory(self):
    model_instances = []
    data_instances = []
    for _ in range(10):
      mujoco.set_mjcb_control(None)
      model_instances.append(mujoco.MjModel.from_xml_string('<mujoco/>'))
      data_instances.append(mujoco.MjData(model_instances[-1]))
      mujoco.set_mjcb_control(lambda m, d: None)
      mujoco.mj_step(model_instances[-1], data_instances[-1])
    mujoco.set_mjcb_control(None)
    while data_instances:
      d = data_instances.pop()
      self.assertEqual(sys.getrefcount(d), 2)
      del d
    while model_instances:
      m = model_instances.pop()
      self.assertEqual(sys.getrefcount(m), 2)

  # This test is disabled on PyPy as it uses sys.getrefcount
  # However PyPy is not officially supported by MuJoCo
  @absltest.skipIf(
      sys.implementation.name == 'pypy', reason='requires sys.getrefcount'
  )
  def test_mjdata_holds_ref_to_model(self):
    data = mujoco.MjData(mujoco.MjModel.from_xml_string('<mujoco/>'))
    model = data.model
    # references: one in `data.model, one in `model`, one in the temporary ref
    # passed to getrefcount.
    self.assertEqual(sys.getrefcount(data.model), 3)
    del data
    self.assertEqual(sys.getrefcount(model), 2)

  def test_can_initialize_mjv_structs(self):
    self.assertIsInstance(mujoco.MjvScene(), mujoco.MjvScene)
    self.assertIsInstance(mujoco.MjvCamera(), mujoco.MjvCamera)
    self.assertIsInstance(mujoco.MjvGLCamera(), mujoco.MjvGLCamera)
    self.assertIsInstance(mujoco.MjvGeom(), mujoco.MjvGeom)
    self.assertIsInstance(mujoco.MjvLight(), mujoco.MjvLight)
    self.assertIsInstance(mujoco.MjvOption(), mujoco.MjvOption)
    self.assertIsInstance(mujoco.MjvScene(), mujoco.MjvScene)
    self.assertIsInstance(mujoco.MjvScene(self.model, 100), mujoco.MjvScene)
    self.assertIsInstance(mujoco.MjvFigure(), mujoco.MjvFigure)

  def test_mjv_camera(self):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    # IDs should be integers
    camera.fixedcamid = 2**31 - 1
    self.assertEqual(camera.fixedcamid, 2**31 - 1)
    with self.assertRaises(TypeError):
      camera.fixedcamid = 0.5

  def test_mjv_scene(self):
    scene = mujoco.MjvScene(model=self.model, maxgeom=100)
    # scene.geoms is a fixed-length tuple of length maxgeom.
    self.assertEqual(scene.ngeom, 0)
    self.assertEqual(scene.maxgeom, 100)
    self.assertLen(scene.geoms, scene.maxgeom)
    # When the scene is updated, geoms are added to the scene
    # (ngeom is incremented)
    mujoco.mj_forward(self.model, self.data)
    mujoco.mjv_updateScene(
        self.model,
        self.data,
        mujoco.MjvOption(),
        None,
        mujoco.MjvCamera(),
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )
    self.assertGreater(scene.ngeom, 0)

  def test_mjv_scene_without_model(self):
    scene = mujoco.MjvScene()
    self.assertEqual(scene.scale, 1.0)
    self.assertEqual(scene.maxgeom, 0)

  def test_mj_ray(self):
    # mj_ray has tricky argument types
    geomid = np.zeros(1, np.int32)
    mujoco.mj_forward(self.model, self.data)
    mujoco.mj_ray(
        self.model, self.data, [0, 0, 0], [0, 0, 1], None, 0, 0, geomid, None
    )
    # Check the normal argument is optional
    mujoco.mj_ray(
        self.model,
        self.data,
        [0, 0, 0],
        [0, 0, 1],
        [0, 0, 0, 0, 0, 0],
        0,
        0,
        geomid,
    )
    # Check that named arguments work
    normal = np.zeros(3, np.float64)
    mujoco.mj_ray(
        m=self.model,
        d=self.data,
        pnt=[0, 0, 0],
        vec=[0, 0, 1],
        geomgroup=None,
        flg_static=0,
        bodyexclude=0,
        geomid=geomid,
        normal=normal,
    )

  def test_mju_ray_geom(self):
    # Test mju_rayGeom with a plane at origin
    pos = np.zeros(3)
    mat = np.eye(3).flatten()
    size = np.array([10.0, 10.0, 1.0])
    pnt = np.array([5.0, 5.0, 5.0])
    # Normalize direction for Euclidean distance
    vec = np.array([-1.0, -1.0, -1.0])
    vec = vec / np.linalg.norm(vec)
    normal = np.zeros(3)

    # Call with normal argument
    dist = mujoco.mju_rayGeom(
        pos, mat, size, pnt, vec, mujoco.mjtGeom.mjGEOM_PLANE, normal
    )
    expected_dist = np.sqrt(3 * 5 * 5)
    np.testing.assert_allclose(dist, expected_dist)
    np.testing.assert_allclose(normal, [0, 0, 1])

    # Call without normal argument (should still work)
    dist2 = mujoco.mju_rayGeom(
        pos, mat, size, pnt, vec, mujoco.mjtGeom.mjGEOM_PLANE
    )
    np.testing.assert_allclose(dist2, expected_dist)

  def test_mj_multi_ray(self):
    nray = 3
    geom1 = np.zeros(1, np.int32)
    pnt = np.array([-0.3, 0, 0.1])
    vec = np.array([[1, 0, 0], [0, 0, 1], [0, 0, -1]], np.float64)
    dist_ex = np.array([0.2, -1, 0.1])
    geom_ex = np.array([1, -1, 0])
    geomid = np.zeros(nray, np.int32)
    dist = np.zeros(nray, np.float64)

    mujoco.mj_forward(self.model, self.data)
    mujoco.mj_multiRay(
        m=self.model,
        d=self.data,
        pnt=pnt,
        vec=vec.flatten(),
        geomgroup=None,
        flg_static=1,
        bodyexclude=-1,
        geomid=geomid,
        dist=dist,
        normal=None,
        nray=nray,
        cutoff=mujoco.mjMAXVAL,
    )

    for i in range(0, 3):
      self.assertEqual(
          dist[i],
          mujoco.mj_ray(
              self.model, self.data, pnt, vec[i], None, 1, -1, geom1, None
          ),
      )
      self.assertEqual(geomid[i], geom1)
      self.assertEqual(geomid[i], geom_ex[i])
      self.assertAlmostEqual(dist[i], dist_ex[i])

  def test_inverse_fd_none(self):
    eps = 1e-6
    flg_centered = 0
    mujoco.mjd_inverseFD(
        self.model,
        self.data,
        eps,
        flg_centered,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    )

  def test_geom_distance(self):
    mujoco.mj_forward(self.model, self.data)
    fromto = np.empty(6, np.float64)
    dist = mujoco.mj_geomDistance(self.model, self.data, 0, 2, 200, fromto)
    self.assertEqual(dist, 41.9)
    np.testing.assert_array_equal(
        fromto, np.array((42.0, 0.0, 0.0, 42.0, 0.0, 41.9))
    )

  def test_inverse_fd(self):
    eps = 1e-6
    flg_centered = 0
    df_dq = np.zeros((self.model.nv, self.model.nv))
    df_dv = np.zeros((self.model.nv, self.model.nv))
    df_da = np.zeros((self.model.nv, self.model.nv))
    ds_dq = np.zeros((self.model.nv, self.model.nsensordata))
    ds_dv = np.zeros((self.model.nv, self.model.nsensordata))
    ds_da = np.zeros((self.model.nv, self.model.nsensordata))
    dm_dq = np.zeros((self.model.nv, self.model.nM))
    mujoco.mjd_inverseFD(
        self.model,
        self.data,
        eps,
        flg_centered,
        df_dq,
        df_dv,
        df_da,
        ds_dq,
        ds_dv,
        ds_da,
        dm_dq,
    )
    self.assertGreater(np.linalg.norm(df_dq), eps)
    self.assertGreater(np.linalg.norm(df_dv), eps)
    self.assertGreater(np.linalg.norm(df_da), eps)
    self.assertGreater(np.linalg.norm(ds_dq), eps)
    self.assertGreater(np.linalg.norm(ds_dv), eps)
    self.assertGreater(np.linalg.norm(ds_da), eps)

  def test_mjd_sub_quat(self):
    quat1 = np.array((0.2, 0.3, 0.3, 0.4))
    quat2 = np.array((0.1, 0.2, 0.4, 0.5))
    d1 = np.empty(9, np.float64)
    d2 = np.empty(9, np.float64)
    mujoco.mjd_subQuat(quat1, quat2, d1, d2)
    d3 = np.empty((3, 3), np.float64)
    d4 = np.empty((3, 3), np.float64)
    mujoco.mjd_subQuat(quat1, quat2, None, d3)
    mujoco.mjd_subQuat(quat1, quat2, d4, None)
    np.testing.assert_array_equal(d2, d3.flatten())
    np.testing.assert_array_equal(d1, d4.flatten())

  def test_mjd_quat_integrate(self):
    scale = 0.1
    vel = np.array((0.2, 0.3, 0.3))
    d_quat = np.empty(9, np.float64)
    d_vel = np.empty(9, np.float64)
    d_h = np.empty(3, np.float64)
    mujoco.mjd_quatIntegrate(vel, scale, d_quat, d_vel, d_h)

  def test_banded(self):
    n_total = 4
    n_band = 1
    n_dense = 1
    dense = np.array([
        [1.0, 0, 0, 0.1],
        [0, 2.0, 0, 0.2],
        [0, 0, 3.0, 0.3],
        [0.1, 0.2, 0.3, 4.0],
    ])
    band = np.zeros(n_band * (n_total - n_dense) + n_dense * n_total)
    mujoco.mju_dense2Band(band, dense, n_total, n_band, n_dense)
    for i in range(4):
      index = mujoco.mju_bandDiag(i, n_total, n_band, n_dense)
      self.assertEqual(band[index], i + 1)
    dense2 = np.zeros((n_total, n_total))
    flg_sym = 1
    mujoco.mju_band2Dense(dense2, band, n_total, n_band, n_dense, flg_sym)
    np.testing.assert_array_equal(dense, dense2)
    vec = np.array([[2.0], [2.0], [3.0], [4.0]])
    res = np.zeros_like(vec)
    n_vec = 1
    mujoco.mju_bandMulMatVec(
        res, band, vec, n_total, n_band, n_dense, n_vec, flg_sym
    )
    np.testing.assert_array_equal(res, dense @ vec)
    diag_add = 0
    diag_mul = 0
    mujoco.mju_cholFactorBand(
        band, n_total, n_band, n_dense, diag_add, diag_mul
    )
    mujoco.mju_cholSolveBand(res, band, vec, n_total, n_band, n_dense)
    np.testing.assert_almost_equal(res, np.linalg.solve(dense, vec))

  def test_mju_box_qp(self):
    n = 5
    res = np.zeros(n)
    r = np.zeros((n, n + 7))
    index = np.zeros(n, np.int32)
    h = np.eye(n)
    g = np.ones((n,))
    lower = -np.ones((n,))
    upper = np.ones((n,))
    rank = mujoco.mju_boxQP(res, r, index, h, g, lower, upper)
    self.assertGreater(rank, -1)

  def test_mju_fill(self):
    res = np.empty(3, np.float64)
    mujoco.mju_fill(res, 1.5)
    np.testing.assert_array_equal(res, np.full(3, 1.5))

  def test_mju_eye(self):
    eye4 = np.empty((4, 4), np.float64)
    mujoco.mju_eye(eye4)
    np.testing.assert_array_equal(eye4, np.eye(4))

  def test_mju_symmetrize(self):
    mat = np.linspace(0, 1, 16).reshape(4, 4)
    res = np.empty((4, 4), np.float64)
    mujoco.mju_symmetrize(res, mat)
    np.testing.assert_array_equal(res, 0.5 * (mat + mat.T))

  def test_mju_clip(self):
    self.assertEqual(mujoco.mju_clip(1.5, 1.0, 2.0), 1.5)
    self.assertEqual(mujoco.mju_clip(1.5, 2.0, 3.0), 2.0)
    self.assertEqual(mujoco.mju_clip(1.5, 0.0, 1.0), 1.0)

  def test_mju_mul_vec_mat_vec(self):
    vec1 = np.array([1.0, 2.0, 3.0])
    vec2 = np.array([3.0, 2.0, 1.0])
    mat = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]])
    self.assertEqual(mujoco.mju_mulVecMatVec(vec1, mat, vec2), 204.0)

  def test_mju_dense_to_sparse(self):
    mat = np.array([[0.0, 1.0, 0.0], [2.0, 0.0, 3.0]])
    expected_vals = np.array([1.0, 2.0, 3.0])
    expected_rownnz = np.array([1, 2])
    expected_rowadr = np.array([0, 1])
    expected_colind = np.array([1, 0, 2])
    vals = np.zeros(3)
    row_nnz = np.zeros(2, np.int32)
    row_adr = np.zeros(2, np.int32)
    col_ind = np.zeros(3, np.int32)
    status = mujoco.mju_dense2sparse(vals, mat, row_nnz, row_adr, col_ind)
    np.testing.assert_equal(status, 0)
    np.testing.assert_array_equal(vals, expected_vals)
    np.testing.assert_array_equal(row_nnz, expected_rownnz)
    np.testing.assert_array_equal(row_adr, expected_rowadr)
    np.testing.assert_array_equal(col_ind, expected_colind)

  def test_mju_sparse_to_dense(self):
    expected = np.array([[0.0, 1.0, 0.0], [2.0, 0.0, 3.0]])
    mat = np.array((1.0, 2.0, 3.0))
    rownnz = np.array([1, 2])
    rowadr = np.array([0, 1])
    colind = np.array([1, 0, 2])
    res = np.zeros((2, 3))
    mujoco.mju_sparse2dense(res, mat, rownnz, rowadr, colind)
    np.testing.assert_array_equal(res, expected)

  def test_mju_euler_to_quat(self):
    quat = np.zeros(4)
    euler = np.array([0, np.pi / 2, 0])
    seq = 'xyz'
    mujoco.mju_euler2Quat(quat, euler, seq)
    expected_quat = np.array([np.sqrt(0.5), 0, np.sqrt(0.5), 0.0])
    np.testing.assert_almost_equal(quat, expected_quat)

    error = 'mju_euler2Quat: seq must contain exactly 3 characters'
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, error):
      mujoco.mju_euler2Quat(quat, euler, 'xy')
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, error):
      mujoco.mju_euler2Quat(quat, euler, 'xyzy')
    error = "mju_euler2Quat: seq[2] is 'p', should be one of x, y, z, X, Y, Z"
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, error):
      mujoco.mju_euler2Quat(quat, euler, 'xYp')

  @parameterized.product(flg_html=(False, True), flg_pad=(False, True))
  def test_mj_printSchema(self, flg_html, flg_pad):  # pylint: disable=invalid-name
    # Make sure that mj_printSchema doesn't raise an exception
    # (e.g. because the internal output buffer is too small)
    self.assertIn('mujoco', mujoco.mj_printSchema(flg_html, flg_pad))

  def test_pickle_mjdata_clean(self):
    data2 = pickle.loads(pickle.dumps(self.data))
    attr_to_compare = ('time', 'qpos', 'qvel', 'mocap_pos')
    self._assert_attributes_equal(data2, self.data, attr_to_compare)

  def test_pickle_mjdata(self):
    mujoco.mj_step(self.model, self.data)
    data2 = pickle.loads(pickle.dumps(self.data))
    attr_to_compare = (
        'time',
        'qpos',
        'qvel',
        'qacc',
        'xpos',
        'mocap_pos',
        'warning',
        'energy',
        'contact',
        'efc_J',
    )
    self._assert_attributes_equal(data2, self.data, attr_to_compare)
    for _ in range(10):
      mujoco.mj_step(self.model, self.data)
      mujoco.mj_step(self.model, data2)
    self._assert_attributes_equal(data2, self.data, attr_to_compare)

  def test_pickle_mjdata_sparse(self):
    self.model.opt.jacobian = mujoco.mjtJacobian.mjJAC_SPARSE
    mujoco.mj_step(self.model, self.data)
    data2 = pickle.loads(pickle.dumps(self.data))
    attr_to_compare = (
        'time',
        'qpos',
        'qvel',
        'qacc',
        'xpos',
        'mocap_pos',
        'warning',
        'energy',
        'contact',
        'efc_J',
    )
    self._assert_attributes_equal(data2, self.data, attr_to_compare)
    for _ in range(10):
      mujoco.mj_step(self.model, self.data)
      mujoco.mj_step(self.model, data2)
    self._assert_attributes_equal(data2, self.data, attr_to_compare)

  def test_pickle_mjmodel(self):
    model2 = pickle.loads(pickle.dumps(self.model))
    attr_to_compare = (
        'nq',
        'nmat',
        'body_pos',
        'names',
    )
    self._assert_attributes_equal(model2, self.model, attr_to_compare)

  def test_indexer_name_id(self):
    xml = r"""
<mujoco>
  <worldbody>
    <geom name="mygeom" size="1" pos="0 0 1"/>
    <geom size="2" pos="0 0 2"/>
    <geom size="3" pos="0 0 3"/>
    <geom name="myothergeom" size="4" pos="0 0 4"/>
    <geom size="5" pos="0 0 5"/>
  </worldbody>
</mujoco>
"""

    model = mujoco.MjModel.from_xml_string(xml)
    self.assertEqual(model.geom('mygeom').id, 0)
    self.assertEqual(model.geom('myothergeom').id, 3)
    self.assertEqual(model.geom(0).name, 'mygeom')
    self.assertEqual(model.geom(1).name, '')
    self.assertEqual(model.geom(2).name, '')
    self.assertEqual(model.geom(3).name, 'myothergeom')
    self.assertEqual(model.geom(4).name, '')
    self.assertEqual(model.geom(0).size[0], 1)
    self.assertEqual(model.geom(1).size[0], 2)
    self.assertEqual(model.geom(2).size[0], 3)
    self.assertEqual(model.geom(3).size[0], 4)
    self.assertEqual(model.geom(4).size[0], 5)

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    self.assertEqual(data.geom('mygeom').id, 0)
    self.assertEqual(data.geom('myothergeom').id, 3)
    self.assertEqual(data.geom(0).name, 'mygeom')
    self.assertEqual(data.geom(1).name, '')
    self.assertEqual(data.geom(2).name, '')
    self.assertEqual(data.geom(3).name, 'myothergeom')
    self.assertEqual(data.geom(4).name, '')
    self.assertEqual(data.geom(0).xpos[2], 1)
    self.assertEqual(data.geom(1).xpos[2], 2)
    self.assertEqual(data.geom(2).xpos[2], 3)
    self.assertEqual(data.geom(3).xpos[2], 4)
    self.assertEqual(data.geom(4).xpos[2], 5)

  def test_load_plugin(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_PLUGIN)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

  def test_copy_mjdata_with_plugin(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_PLUGIN)
    data1 = mujoco.MjData(model)
    self.assertIs(data1.model, model)
    mujoco.mj_step(model, data1)
    data2 = copy.copy(data1)
    mujoco.mj_step(model, data1)
    mujoco.mj_step(model, data2)
    np.testing.assert_array_equal(data1.qpos, data2.qpos)
    self.assertIs(data1.model, data2.model)

  def test_deepcopy_mjdata_with_plugin(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_PLUGIN)
    data1 = mujoco.MjData(model)
    self.assertIs(data1.model, model)
    mujoco.mj_step(model, data1)
    data2 = copy.deepcopy(data1)
    mujoco.mj_step(model, data1)
    mujoco.mj_step(model, data2)
    np.testing.assert_array_equal(data1.qpos, data2.qpos)
    self.assertIsNot(data1.model, data2.model)
    self.assertNotEqual(data1.model._address, data2.model._address)

  def test_texture_size(self):
    model = mujoco.MjModel.from_xml_string(TEST_XML_TEXTURE)
    self.assertEqual(model.tex('tex').data.shape, (512, 512, 3))

  def test_xml_dependencies(self):
    model_path = str(epath.resource_path("mujoco") / "testdata" / "msh.xml")
    msh_path =str(epath.resource_path("mujoco") / "testdata" / "abdomen_1_body.msh")

    model_path = model_path.replace('\\', '/')
    msh_path = msh_path.replace('\\', '/')
    dependencies = mujoco.mju_getXMLDependencies(model_path)
    self.assertIn(model_path, dependencies)
    self.assertIn(msh_path, dependencies)

  def test_mj_read_ctrl_and_init_ctrl_delay(self):
    xml = r"""
<mujoco>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1"/>
      <joint name="hinge" type="hinge"/>
    </body>
  </worldbody>
  <actuator>
    <position name="actuator" joint="hinge" delay="0.01" nsample="4"/>
  </actuator>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Initialize the delay buffer with known values
    # actuator_history[i, 0] = nsample, actuator_history[i, 1] = interp
    nhistory = model.actuator_history[0, 0]
    self.assertEqual(nhistory, 4)
    times = np.array([0.0, 0.01, 0.02, 0.03])
    values = np.array([1.0, 2.0, 3.0, 4.0])
    mujoco.mj_initCtrlHistory(model, data, 0, times, values)

    # Read back a value using zero-order hold
    # mj_readCtrl auto-subtracts delay: lookup_time = read_time - delay
    # delay = 0.01, so:
    #   read_time=0.02 -> lookup at 0.01 -> value 2.0
    #   read_time=0.03 -> lookup at 0.02 -> value 3.0
    result = mujoco.mj_readCtrl(model, data, 0, 0.02, interp=0)
    self.assertEqual(result, 2.0)  # ZOH returns value at t=0.01

    # Test with times=None (uses existing timestamps)
    new_values = np.array([5.0, 6.0, 7.0, 8.0])
    mujoco.mj_initCtrlHistory(model, data, 0, None, new_values)
    # read_time=0.02 -> lookup at 0.01 -> value 6.0
    result = mujoco.mj_readCtrl(model, data, 0, 0.02, interp=0)
    self.assertEqual(result, 6.0)

    # Test dimension validation errors
    with self.assertRaises(TypeError):
      # wrong times
      mujoco.mj_initCtrlHistory(model, data, 0, np.zeros(3), values)
    with self.assertRaises(TypeError):
      # wrong values
      mujoco.mj_initCtrlHistory(model, data, 0, times, np.zeros(5))

  def test_mj_read_sensor_and_init_sensor_delay(self):
    xml = r"""
<mujoco>
  <worldbody>
    <body>
      <geom type="sphere" size="0.1"/>
      <joint name="hinge" type="hinge"/>
      <site name="site"/>
    </body>
  </worldbody>
  <sensor>
    <accelerometer name="accel" site="site" delay="0.01" nsample="3"/>
  </sensor>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # Initialize the delay buffer with known values
    # sensor_history[i, 0] = nsample, sensor_history[i, 1] = interp
    nhistory = model.sensor_history[0, 0]
    dim = model.sensor_dim[0]
    self.assertEqual(nhistory, 3)
    self.assertEqual(dim, 3)  # accelerometer has dim=3
    times = np.array([0.0, 0.01, 0.02])
    values = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]], dtype=np.float64)
    mujoco.mj_initSensorHistory(model, data, 0, times, values, phase=0.0)

    # Read back a value using zero-order hold
    # mj_readSensor auto-subtracts delay: lookup_time = read_time - delay
    # delay = 0.01, so:
    #   read_time=0.02 -> lookup at 0.01 -> value [4, 5, 6]
    result = np.zeros(dim)
    mujoco.mj_readSensor(model, data, 0, 0.02, result, interp=0)
    # ZOH returns value at t=0.01
    np.testing.assert_array_equal(result, [4, 5, 6])

    # Test with times=None (uses existing timestamps)
    new_values = np.array([
        [10, 11, 12], [13, 14, 15], [16, 17, 18]], dtype=np.float64)
    mujoco.mj_initSensorHistory(model, data, 0, None, new_values, phase=0.0)
    # read_time=0.02 -> lookup at 0.01 -> value [13, 14, 15]
    mujoco.mj_readSensor(model, data, 0, 0.02, result, interp=0)
    np.testing.assert_array_equal(result, [13, 14, 15])

    # Test dimension validation errors
    with self.assertRaises(TypeError):
      # wrong result size
      mujoco.mj_readSensor(model, data, 0, 0.02, np.zeros(2), interp=0)
    with self.assertRaises(TypeError):
      # wrong times size
      mujoco.mj_initSensorHistory(model, data, 0, np.zeros(2), values, 0.0)
    with self.assertRaises(TypeError):
      # wrong values rows
      mujoco.mj_initSensorHistory(model, data, 0, times, np.zeros((4, 3)), 0.0)
    with self.assertRaises(TypeError):
      # wrong values cols
      mujoco.mj_initSensorHistory(model, data, 0, times, np.zeros((3, 2)), 0.0)

  def test_init_sensor_history_pedagogical(self):
    # A framequat sensor reports body orientation as a unit quaternion.
    # Quaternions are never zero: the identity quaternion is [1, 0, 0, 0].
    # This test demonstrates why mj_initSensorHistory is needed: after
    # mj_makeData, the history buffer is filled with zeros, which is invalid.
    xml = r"""
<mujoco>
  <worldbody>
    <body name="body">
      <freejoint/>
      <geom type="sphere" size="0.1"/>
    </body>
  </worldbody>
  <sensor>
    <framequat name="quat" objtype="body" objname="body" delay="0.01" nsample="5"/>
  </sensor>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    dim = model.sensor_dim[0]
    nsample = model.sensor_history[0, 0]
    delay = model.sensor_delay[0]
    self.assertEqual(dim, 4)
    self.assertEqual(nsample, 5)
    self.assertEqual(delay, 0.01)

    # After mj_makeData, reading from the delay buffer gives all zeros.
    # For a quaternion sensor, this is invalid data.
    result = np.zeros(dim)
    mujoco.mj_readSensor(model, data, 0, delay, result, interp=0)
    np.testing.assert_array_equal(result, [0, 0, 0, 0])

    # To get valid sensor values, we temporarily set delay to 0 so that
    # mj_forward populates sensordata directly (without using the delay
    # buffer), then restore the original delay value.
    saved_delay = model.sensor_delay.copy()
    model.sensor_delay[:] = 0
    mujoco.mj_forward(model, data)
    model.sensor_delay[:] = saved_delay

    # Now sensordata contains the valid identity quaternion.
    np.testing.assert_array_equal(data.sensordata, [1, 0, 0, 0])

    # Use mj_initSensorHistory to fill the buffer with valid quaternion values.
    # Passing None for times keeps the existing timestamps in the buffer.
    values = np.tile(data.sensordata, (nsample, 1))
    mujoco.mj_initSensorHistory(model, data, 0, None, values, phase=0.0)

    # Now reading from the delay buffer gives the valid identity quaternion.
    mujoco.mj_readSensor(model, data, 0, delay, result, interp=0)
    np.testing.assert_array_equal(result, [1, 0, 0, 0])

  def _assert_attributes_equal(self, actual_obj, expected_obj, attr_to_compare):
    for name in attr_to_compare:
      actual_value = getattr(actual_obj, name)
      expected_value = getattr(expected_obj, name)
      try:
        if isinstance(expected_value, np.ndarray):
          np.testing.assert_array_equal(actual_value, expected_value)
        else:
          self.assertEqual(actual_value, expected_value)
      except AssertionError as e:
        self.fail(
            "Attribute '{}' differs from expected value: {}".format(
                name, str(e)
            )
        )


if __name__ == '__main__':
  absltest.main()
