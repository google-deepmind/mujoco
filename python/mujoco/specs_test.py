# Copyright 2024 DeepMind Technologies Limited
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
"""Tests for mjSpec bindings."""

import inspect
import textwrap

from absl.testing import absltest
import mujoco
import numpy as np


def get_linenumber():
  cf = inspect.currentframe()
  return cf.f_back.f_lineno


class SpecsTest(absltest.TestCase):

  def test_basic(self):
    # Create a spec.
    spec = mujoco.MjSpec()

    # Check that euler sequence order is set correctly.
    self.assertEqual(spec.eulerseq[0], ord('x'))
    spec.eulerseq = ['z', 'y', 'x']
    self.assertEqual(spec.eulerseq[0], ord('z'))

    # Add a body, check that it has default orientation.
    body = spec.worldbody.add_body()
    self.assertEqual(body.name, '')
    np.testing.assert_array_equal(body.quat, [1, 0, 0, 0])

    # Change the name of the body and read it back twice.
    body.name = 'foobar'
    self.assertEqual(body.name, 'foobar')
    body.name = 'baz'
    self.assertEqual(body.name, 'baz')

    # Change the position of the body and read it back.
    body.pos = [1, 2, 3]
    np.testing.assert_array_equal(body.pos, [1, 2, 3])
    self.assertEqual(body.pos.shape, (3,))

    # Change the orientation of the body and read it back.
    body.quat = [0, 1, 0, 0]
    np.testing.assert_array_equal(body.quat, [0, 1, 0, 0])
    self.assertEqual(body.quat.shape, (4,))

    # Add a site to the body with user data and read it back.
    site = body.add_site()
    site.name = 'sitename'
    site.type = mujoco.mjtGeom.mjGEOM_BOX
    site.userdata = [1, 2, 3, 4, 5, 6]
    self.assertEqual(site.name, 'sitename')
    self.assertEqual(site.type, mujoco.mjtGeom.mjGEOM_BOX)
    np.testing.assert_array_equal(site.userdata, [1, 2, 3, 4, 5, 6])

    # Check that the site and body have no id before compilation.
    self.assertEqual(body.id, -1)
    self.assertEqual(site.id, -1)

    # Compile the spec and check for expected values in the model.
    model = spec.compile()
    self.assertEqual(spec.worldbody.id, 0)
    self.assertEqual(body.id, 1)
    self.assertEqual(site.id, 0)
    self.assertEqual(model.nbody, 2)  # 2 bodies, including the world body
    np.testing.assert_array_equal(model.body_pos[1], [1, 2, 3])
    np.testing.assert_array_equal(model.body_quat[1], [0, 1, 0, 0])
    self.assertEqual(model.nsite, 1)
    self.assertEqual(model.nuser_site, 6)
    np.testing.assert_array_equal(model.site_user[0], [1, 2, 3, 4, 5, 6])

    self.assertEqual(spec.to_xml(), textwrap.dedent("""\
        <mujoco model="MuJoCo Model">
          <compiler angle="radian"/>

          <size nuser_site="6"/>

          <worldbody>
            <body name="baz" pos="1 2 3" quat="0 1 0 0">
              <site name="sitename" pos="0 0 0" type="box" user="1 2 3 4 5 6"/>
            </body>
          </worldbody>
        </mujoco>
    """),)

  def test_kwarg(self):
    # Create a spec.
    spec = mujoco.MjSpec()

    # Add a body.
    body = spec.worldbody.add_body(
        name='body', pos=[1, 2, 3], quat=[0, 0, 0, 1]
    )
    self.assertEqual(body.name, 'body')
    np.testing.assert_array_equal(body.pos, [1, 2, 3])
    np.testing.assert_array_equal(body.quat, [0, 0, 0, 1])

    # Add a geom.
    geom = body.add_geom(
        name='geom',
        pos=[3, 2, 1],
        fromto=[1, 2, 3, 4, 5, 6],
        contype=3,
    )

    self.assertEqual(geom.name, 'geom')
    np.testing.assert_array_equal(geom.pos, [3, 2, 1])
    np.testing.assert_array_equal(geom.fromto, [1, 2, 3, 4, 5, 6])
    self.assertEqual(geom.contype, 3)

    # Add a site to the body with user data and read it back.
    site = body.add_site(
        name='sitename',
        pos=[0, 1, 2],
        quat=[1, 0, 0, 0],
        fromto=[0, 1, 2, 3, 4, 5],
        size=[3, 2, 1],
        type=mujoco.mjtGeom.mjGEOM_BOX,
        material='material',
        group=7,
        rgba=[1, 1, 1, 0.5],
        userdata=[1, 2, 3, 4, 5, 6],
        info='info',
    )
    self.assertEqual(site.name, 'sitename')
    np.testing.assert_array_equal(site.pos, [0, 1, 2])
    np.testing.assert_array_equal(site.quat, [1, 0, 0, 0])
    np.testing.assert_array_equal(site.fromto, [0, 1, 2, 3, 4, 5])
    np.testing.assert_array_equal(site.size, [3, 2, 1])
    self.assertEqual(site.type, mujoco.mjtGeom.mjGEOM_BOX)
    self.assertEqual(site.material, 'material')
    self.assertEqual(site.group, 7)
    np.testing.assert_array_equal(site.rgba, [1, 1, 1, 0.5])
    np.testing.assert_array_equal(site.userdata, [1, 2, 3, 4, 5, 6])
    self.assertEqual(site.info, 'info')

    # Add camera.
    cam = body.add_camera(orthographic=1, resolution=[10, 20])
    self.assertEqual(cam.orthographic, 1)
    np.testing.assert_array_equal(cam.resolution, [10, 20])

    # Add joint.
    jnt = body.add_joint(type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    self.assertEqual(jnt.type, mujoco.mjtJoint.mjJNT_HINGE)
    np.testing.assert_array_equal(jnt.axis, [0, 1, 0])

    # Add light.
    light = body.add_light(attenuation=[1, 2, 3])
    np.testing.assert_array_equal(light.attenuation, [1, 2, 3])

    # Invalid input for valid keyword argument.
    with self.assertRaises(ValueError):
      body.add_geom(pos='pos')  # wrong type for array

    with self.assertRaises(ValueError):
      body.add_geom(pos=[0, 1])  # wrong size

    with self.assertRaises(ValueError):
      body.add_geom(type='type')  # wrong type for value

    with self.assertRaises(ValueError):
      body.add_geom(userdata='')  # wrong type of vector

    # Invalid keyword argument.
    with self.assertRaises(TypeError):
      body.add_geom(vel='vel')

  def test_load_xml(self):
    filename = '../../test/testdata/model.xml'
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION

    # Load from file.
    spec1 = mujoco.MjSpec()
    spec1.from_file(filename)
    model1 = spec1.compile()
    data1 = mujoco.MjData(model1)
    mujoco.mj_step(model1, data1)
    size1 = mujoco.mj_stateSize(model1, state_type)
    state1 = np.empty(size1, np.float64)
    mujoco.mj_getState(model1, data1, state1, state_type)

    # Load from string.
    spec2 = mujoco.MjSpec()
    with open(filename, 'r') as file:
      spec2.from_string(file.read().rstrip())
    model2 = spec2.compile()
    data2 = mujoco.MjData(model2)
    mujoco.mj_step(model2, data2)
    size2 = mujoco.mj_stateSize(model2, state_type)
    state2 = np.empty(size2, np.float64)
    mujoco.mj_getState(model2, data2, state2, state_type)

    # Check that the state is the same.
    np.testing.assert_array_equal(state1, state2)

  def test_compile_errors_with_line_info(self):
    spec = mujoco.MjSpec()

    added_on_line = get_linenumber() + 1
    geom = spec.worldbody.add_geom()
    geom.name = 'MyGeom'
    geom.info = f'geom added on line {added_on_line}'

    # Try to compile, get error.
    expected_error = (
        'Error: size 0 must be positive in geom\n'
        + f'Element name \'MyGeom\', id 0, geom added on line {added_on_line}'
    )
    with self.assertRaisesRegex(ValueError, expected_error):
      spec.compile()

  def test_recompile(self):
    # Create a spec.
    spec = mujoco.MjSpec()

    # Add movable body1.
    body1 = spec.worldbody.add_body()
    geom = body1.add_geom()
    geom.size[0] = 1
    geom.pos = [1, 1, 0]
    joint = body1.add_joint()
    joint.type = mujoco.mjtJoint.mjJNT_BALL

    # Compile model, make data.
    model = spec.compile()
    data = mujoco.MjData(model)

    # Simulate for 1 second.
    while data.time < 1:
      mujoco.mj_step(model, data)

    # Add movable body2.
    body2 = spec.worldbody.add_body()
    body2.pos[1] = 3
    geom = body2.add_geom()
    geom.size[0] = 1
    geom.pos = [0, 1, 0]
    joint = body2.add_joint()
    joint.type = mujoco.mjtJoint.mjJNT_BALL

    # Recompile model and data while maintaining the state.
    model_new, data_new = spec.recompile(model, data)

    # Check that the state is preserved.
    np.testing.assert_array_equal(model_new.body_pos[1], model.body_pos[1])
    np.testing.assert_array_equal(data_new.qpos[:4], data.qpos)
    np.testing.assert_array_equal(data_new.qvel[:3], data.qvel)

  def test_uncompiled_spec_cannot_be_written(self):
    spec = mujoco.MjSpec()

    # Cannot write XML of an uncompiled spec.
    expected_error = 'XML Write error: Only compiled model can be written'
    with self.assertRaisesWithLiteralMatch(mujoco.FatalError, expected_error):
      spec.to_xml()

  def test_modelname_default_class(self):
    spec = mujoco.MjSpec()
    spec.modelname = 'test'

    main = spec.default()
    main.geom.size[0] = 2

    def1 = spec.add_default('def1', main)
    def1.geom.size[0] = 3

    spec.worldbody.add_geom(def1)
    spec.worldbody.add_geom(main)

    spec.compile()
    self.assertEqual(spec.to_xml(), textwrap.dedent("""\
        <mujoco model="test">
          <compiler angle="radian"/>

          <default>
            <geom size="2 0 0"/>
            <default class="def1">
              <geom size="3 0 0"/>
            </default>
          </default>

          <worldbody>
            <geom class="def1"/>
            <geom/>
          </worldbody>
        </mujoco>
    """))
    spec = mujoco.MjSpec()
    spec.modelname = 'test'

    main = spec.default()
    main.geom.size[0] = 2

    def1 = spec.add_default('def1', main)
    def1.geom.size[0] = 3

    spec.worldbody.add_geom(def1)
    spec.worldbody.add_geom(main)

    spec.compile()
    self.assertEqual(spec.to_xml(), textwrap.dedent("""\
        <mujoco model="test">
          <compiler angle="radian"/>

          <default>
            <geom size="2 0 0"/>
            <default class="def1">
              <geom size="3 0 0"/>
            </default>
          </default>

          <worldbody>
            <geom class="def1"/>
            <geom/>
          </worldbody>
        </mujoco>
    """))

  def test_element_list(self):
    spec = mujoco.MjSpec()
    sensor1 = spec.add_sensor()
    sensor2 = spec.add_sensor()
    sensor3 = spec.add_sensor()
    sensor1.name = 'sensor1'
    sensor2.name = 'sensor2'
    sensor3.name = 'sensor3'
    self.assertLen(spec.sensors, 3)
    self.assertEqual(spec.sensors[0].name, 'sensor1')
    self.assertEqual(spec.sensors[1].name, 'sensor2')
    self.assertEqual(spec.sensors[2].name, 'sensor3')

  def test_iterators(self):
    spec = mujoco.MjSpec()
    geom1 = spec.worldbody.add_geom()
    geom2 = spec.worldbody.add_geom()
    geom3 = spec.worldbody.add_geom()
    geom1.name = 'geom1'
    geom2.name = 'geom2'
    geom3.name = 'geom3'
    geom = spec.worldbody.first_geom()
    i = 1
    while geom:
      self.assertEqual(geom.name, 'geom' + str(i))
      geom = spec.worldbody.next_geom(geom)
      i += 1

  def test_assets(self):
    cube = """
      v -1 -1  1
      v  1 -1  1
      v -1  1  1
      v  1  1  1
      v -1  1 -1
      v  1  1 -1
      v -1 -1 -1
      v  1 -1 -1"""
    spec = mujoco.MjSpec()
    mesh = spec.add_mesh()
    mesh.name = 'cube'
    mesh.file = 'cube.obj'
    geom = spec.worldbody.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_MESH
    geom.meshname = 'cube'
    model = spec.compile({'cube.obj': cube})
    self.assertEqual(model.nmeshvert, 8)

  def test_delete(self):
    filename = '../../test/testdata/model.xml'

    spec = mujoco.MjSpec()
    spec.from_file(filename)

    model = spec.compile()
    self.assertIsNotNone(model)
    self.assertEqual(model.nsite, 11)
    self.assertEqual(model.nsensor, 11)

    head = spec.find_body('head')
    self.assertIsNotNone(head)
    site = head.first_site()
    self.assertIsNotNone(site)

    site.delete()
    spec.sensors[-1].delete()
    spec.sensors[-1].delete()

    model = spec.compile()
    self.assertIsNotNone(model)
    self.assertEqual(model.nsite, 10)
    self.assertEqual(model.nsensor, 9)

  def test_plugin(self):
    xml = """
    <mujoco>
      <extension>
        <plugin plugin="mujoco.elasticity.cable"/>
      </extension>
    </mujoco>
    """

    spec = mujoco.MjSpec()
    spec.from_string(xml)
    self.assertIsNotNone(spec.worldbody)

    body = spec.worldbody.add_body()
    body.plugin.name = 'mujoco.elasticity.cable'
    body.plugin.id = spec.add_plugin()
    body.plugin.active = True
    self.assertEqual(body.plugin.id, 0)

    geom = body.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_BOX
    geom.size[0] = 1
    geom.size[1] = 1
    geom.size[2] = 1

    model = spec.compile()
    self.assertIsNotNone(model)
    self.assertEqual(model.nplugin, 1)
    self.assertEqual(model.body_plugin[1], 0)

  def test_recompile_error(self):
    main_xml = """
    <mujoco>
      <worldbody>
        <body>
          <geom size="0.1"/>
        </body>
      </worldbody>
    </mujoco>
    """

    spec = mujoco.MjSpec()
    spec.from_string(main_xml)
    model = spec.compile()
    data = mujoco.MjData(model)

    spec.add_material().name = 'yellow'
    spec.add_material().name = 'yellow'

    with self.assertRaisesRegex(
        ValueError, "Error: repeated name 'yellow' in material"
    ):
      spec.recompile(model, data)

if __name__ == '__main__':
  absltest.main()
