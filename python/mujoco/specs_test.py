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

import gc
import inspect
import os
import textwrap
import typing
import zipfile

from absl.testing import absltest
from etils import epath
import mujoco
import numpy as np


def get_linenumber():
  cf = inspect.currentframe()
  return cf.f_back.f_lineno


class SpecsTest(absltest.TestCase):

  def test_typing(self):
    spec = mujoco.MjSpec()
    self.assertIsInstance(spec, mujoco.MjSpec)
    self.assertIsInstance(spec.worldbody, mujoco.MjsBody)
    self.assertIsInstance(spec.worldbody, typing.get_args(mujoco.MjStruct))

  def test_basic(self):
    # Create a spec.
    spec = mujoco.MjSpec()

    # Check that euler sequence order is set correctly.
    self.assertEqual(spec.compiler.eulerseq[0], 'x')
    spec.compiler.eulerseq = ['z', 'y', 'x']
    self.assertEqual(spec.compiler.eulerseq[0], 'z')

    # Change single elements of euler sequence.
    spec.compiler.eulerseq[0] = 'y'
    spec.compiler.eulerseq[1] = 'z'
    self.assertEqual(spec.compiler.eulerseq[0], 'y')
    self.assertEqual(spec.compiler.eulerseq[1], 'z')

    # eulerseq is iterable
    self.assertEqual('yzx', ''.join(spec.compiler.eulerseq))

    # supports `len`
    self.assertLen(spec.compiler.eulerseq, 3)

    # field checks for out-of-bound access on read and on write
    with self.assertRaises(IndexError):
      spec.compiler.eulerseq[3] = 'x'

    with self.assertRaises(IndexError):
      spec.compiler.eulerseq[-1] = 'x'

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
    body.pos = [4, 2, 3]
    np.testing.assert_array_equal(body.pos, [4, 2, 3])
    self.assertEqual(body.pos.shape, (3,))

    # Change single element of position.
    body.pos[0] = 1
    np.testing.assert_array_equal(body.pos, [1, 2, 3])

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

    # Compile the spec and check for expected values in the model.
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    self.assertEqual(model.nbody, 2)  # 2 bodies, including the world body
    np.testing.assert_array_equal(model.bind(body).pos, [1, 2, 3])
    np.testing.assert_array_equal(model.bind(body).quat, [0, 1, 0, 0])
    np.testing.assert_array_equal(data.bind(body).xpos, [1, 2, 3])
    self.assertEqual(model.nsite, 1)
    self.assertEqual(model.nuser_site, 6)
    np.testing.assert_array_equal(model.site_user[0], [1, 2, 3, 4, 5, 6])

    xml = textwrap.dedent("""\
        <mujoco model="MuJoCo Model">
          <compiler angle="radian"/>

          <size nuser_site="6"/>

          <worldbody>
            <body name="baz" pos="1 2 3" quat="0 1 0 0">
              <site name="sitename" pos="0 0 0" type="box" user="1 2 3 4 5 6"/>
            </body>
          </worldbody>
        </mujoco>
    """)
    self.assertEqual(spec.to_xml(), xml)

  def test_kwarg(self):
    # Create a spec.
    spec = mujoco.MjSpec()

    # Add material.
    material = spec.add_material(texrepeat=[1, 2], emission=-1)
    np.testing.assert_array_equal(material.texrepeat, [1, 2])
    self.assertEqual(material.emission, -1)

    # Add mesh.
    mesh = spec.add_mesh(refpos=[1, 2, 3])
    np.testing.assert_array_equal(mesh.refpos, [1, 2, 3])

    # Add pair.
    pair = spec.add_pair(gap=0.1)
    self.assertEqual(pair.gap, 0.1)

    # Add equality.
    equality = spec.add_equality(objtype=mujoco.mjtObj.mjOBJ_SITE)
    self.assertEqual(equality.objtype, mujoco.mjtObj.mjOBJ_SITE)

    # Add tendon.
    tendon = spec.add_tendon(stiffness=2, springlength=[0.1, 0.2])
    self.assertEqual(tendon.stiffness, 2)
    np.testing.assert_array_equal(tendon.springlength, [0.1, 0.2])

    # Add actuator.
    actuator = spec.add_actuator(actdim=10, ctrlrange=[-1, 10])
    self.assertEqual(actuator.actdim, 10)
    np.testing.assert_array_equal(actuator.ctrlrange, [-1, 10])

    # Add skin.
    skin = spec.add_skin(inflate=2.0, vertid=[[1, 1], [2, 2]])
    self.assertEqual(skin.inflate, 2.0)
    np.testing.assert_array_equal(skin.vertid, [[1, 1], [2, 2]])

    # Add texture.
    texture = spec.add_texture(builtin=0, nchannel=3)
    self.assertEqual(texture.builtin, 0)
    self.assertEqual(texture.nchannel, 3)

    # Add text.
    text = spec.add_text(data='data', info='info')
    self.assertEqual(text.data, 'data')
    self.assertEqual(text.info, 'info')

    # Add tuple.
    tuple_ = spec.add_tuple(objprm=[2.0, 3.0, 5.0], objname=['obj'])
    np.testing.assert_array_equal(tuple_.objprm, [2.0, 3.0, 5.0])
    self.assertEqual(tuple_.objname[0], 'obj')

    # Add flex.
    flex = spec.add_flex(friction=[1, 2, 3], texcoord=[1.0, 2.0, 3.0])
    np.testing.assert_array_equal(flex.friction, [1, 2, 3])
    np.testing.assert_array_equal(flex.texcoord, [1.0, 2.0, 3.0])

    # Add hfield.
    hfield = spec.add_hfield(nrow=2, content_type='type')
    self.assertEqual(hfield.nrow, 2)
    self.assertEqual(hfield.content_type, 'type')

    # Add key.
    key = spec.add_key(time=1.2, qpos=[1.0, 2.0])
    self.assertEqual(key.time, 1.2)
    np.testing.assert_array_equal(key.qpos, [1.0, 2.0])

    # Add numeric.
    numeric = spec.add_numeric(data=[1.0, 1.1, 1.2], size=2)
    np.testing.assert_array_equal(numeric.data, [1.0, 1.1, 1.2])
    self.assertEqual(numeric.size, 2)

    # Add exclude.
    exclude = spec.add_exclude(bodyname2='body2')
    self.assertEqual(exclude.bodyname2, 'body2')

    # Add sensor.
    sensor = spec.add_sensor(
        needstage=mujoco.mjtStage.mjSTAGE_ACC, objtype=mujoco.mjtObj.mjOBJ_SITE
    )
    self.assertEqual(sensor.needstage, mujoco.mjtStage.mjSTAGE_ACC)
    self.assertEqual(sensor.objtype, mujoco.mjtObj.mjOBJ_SITE)

    # Add plugin.
    plugin = spec.add_plugin(
        name='instance_name',
        plugin_name='mujoco.plugin',
        active=True,
        info='info',
    )
    self.assertEqual(plugin.name, 'instance_name')
    self.assertEqual(plugin.plugin_name, 'mujoco.plugin')
    self.assertEqual(plugin.active, True)
    self.assertEqual(plugin.info, 'info')

    # Add a body.
    body = spec.worldbody.add_body(
        name='body', pos=[1, 2, 3], quat=[0, 0, 0, 1]
    )
    self.assertEqual(body.name, 'body')
    np.testing.assert_array_equal(body.pos, [1, 2, 3])
    np.testing.assert_array_equal(body.quat, [0, 0, 0, 1])

    # Add a body with a plugin.
    body_with_plugin = spec.worldbody.add_body(plugin=plugin)
    self.assertEqual(body_with_plugin.plugin.name, 'instance_name')
    self.assertEqual(body_with_plugin.plugin.plugin_name, 'mujoco.plugin')
    self.assertEqual(body_with_plugin.plugin.active, True)
    self.assertEqual(body_with_plugin.plugin.info, 'info')

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

    # Add frame.
    framea0 = body.add_frame(name='framea', pos=[1, 2, 3], quat=[0, 0, 0, 1])
    np.testing.assert_array_equal(framea0.pos, [1, 2, 3])
    np.testing.assert_array_equal(framea0.quat, [0, 0, 0, 1])

    frameb0 = body.add_frame(
        framea0, name='frameb', pos=[4, 5, 6], quat=[0, 1, 0, 0]
    )

    framea1 = body.first_frame()
    frameb1 = body.next_frame(framea1)

    self.assertEqual(framea1.name, framea0.name)
    self.assertEqual(frameb1.name, frameb0.name)
    np.testing.assert_array_equal(framea1.pos, framea0.pos)
    np.testing.assert_array_equal(framea1.quat, framea0.quat)
    np.testing.assert_array_equal(frameb1.pos, frameb0.pos)
    np.testing.assert_array_equal(frameb1.quat, frameb0.quat)

    # Add joint.
    joint = body.add_joint(type=mujoco.mjtJoint.mjJNT_HINGE, axis=[0, 1, 0])
    self.assertEqual(joint.type, mujoco.mjtJoint.mjJNT_HINGE)
    np.testing.assert_array_equal(joint.axis, [0, 1, 0])

    # Add freejoint.
    freejoint = body.add_freejoint()
    self.assertEqual(freejoint.type, mujoco.mjtJoint.mjJNT_FREE)
    freejoint_align = body.add_freejoint(align=True)
    self.assertEqual(freejoint_align.align, True)

    with self.assertRaises(TypeError) as cm:
      body.add_freejoint(axis=[1, 2, 3])
    self.assertEqual(
        str(cm.exception),
        'Invalid axis keyword argument. Valid options are: align, group, name.',
    )

    # Add light.
    light = body.add_light(attenuation=[1, 2, 3])
    np.testing.assert_array_equal(light.attenuation, [1, 2, 3])

    # Invalid input for valid keyword argument.
    with self.assertRaises(ValueError) as cm:
      body.add_geom(pos='pos')
    self.assertEqual(
        str(cm.exception),
        'pos should be a list/array.',
    )

    with self.assertRaises(ValueError) as cm:
      body.add_geom(pos=[0, 1])
    self.assertEqual(
        str(cm.exception),
        'pos should be a list/array of size 3.',
    )

    with self.assertRaises(ValueError) as cm:
      body.add_geom(type='type')
    self.assertEqual(
        str(cm.exception),
        'type is the wrong type.',
    )

    with self.assertRaises(ValueError) as cm:
      body.add_geom(userdata='')
    self.assertEqual(
        str(cm.exception),
        'userdata has the wrong type.',
    )

    # Invalid keyword argument.
    with self.assertRaises(TypeError) as cm:
      body.add_geom(vel='vel')
    self.assertEqual(
        str(cm.exception),
        'Invalid vel keyword argument. Valid options are: name, type, pos,'
        ' quat, axisangle, xyaxes, zaxis, euler, fromto, size, contype,'
        ' conaffinity, condim, priority, friction, solmix, solref, solimp,'
        ' margin, gap, mass, density, typeinertia, fluid_ellipsoid,'
        ' fluid_coefs, material, rgba, group, hfieldname, meshname, fitscale,'
        ' userdata, plugin, info.',
    )

    # Orientation keyword arguments.
    geom_axisangle = body.add_geom(axisangle=[1, 2, 3, 4])
    geom_xyaxes = body.add_geom(xyaxes=[1, 2, 3, 4, 5, 6])
    geom_zaxis = body.add_geom(zaxis=[1, 2, 3])
    geom_euler = body.add_geom(euler=[1, 2, 3])

    self.assertEqual(
        geom_axisangle.alt.type, mujoco.mjtOrientation.mjORIENTATION_AXISANGLE
    )
    self.assertEqual(
        geom_xyaxes.alt.type, mujoco.mjtOrientation.mjORIENTATION_XYAXES
    )
    self.assertEqual(
        geom_zaxis.alt.type, mujoco.mjtOrientation.mjORIENTATION_ZAXIS
    )
    self.assertEqual(
        geom_euler.alt.type, mujoco.mjtOrientation.mjORIENTATION_EULER
    )
    np.testing.assert_array_equal(geom_axisangle.alt.axisangle, [1, 2, 3, 4])
    np.testing.assert_array_equal(geom_xyaxes.alt.xyaxes, [1, 2, 3, 4, 5, 6])
    np.testing.assert_array_equal(geom_zaxis.alt.zaxis, [1, 2, 3])
    np.testing.assert_array_equal(geom_euler.alt.euler, [1, 2, 3])

    body_iaxisangle = spec.worldbody.add_body(iaxisangle=[1, 2, 3, 4])
    body_ixyaxes = spec.worldbody.add_body(ixyaxes=[1, 2, 3, 4, 5, 6])
    body_izaxis = spec.worldbody.add_body(izaxis=[1, 2, 3])
    body_ieuler = spec.worldbody.add_body(ieuler=[1, 2, 3])
    body_euler_ieuler = spec.worldbody.add_body(
        euler=[1, 2, 3], ieuler=[4, 5, 6]
    )
    np.testing.assert_array_equal(body_iaxisangle.ialt.axisangle, [1, 2, 3, 4])
    np.testing.assert_array_equal(body_ixyaxes.ialt.xyaxes, [1, 2, 3, 4, 5, 6])
    np.testing.assert_array_equal(body_izaxis.ialt.zaxis, [1, 2, 3])
    np.testing.assert_array_equal(body_ieuler.ialt.euler, [1, 2, 3])
    np.testing.assert_array_equal(body_euler_ieuler.alt.euler, [1, 2, 3])
    np.testing.assert_array_equal(body_euler_ieuler.ialt.euler, [4, 5, 6])

    # Test invalid orientation keyword arguments.
    with self.assertRaises(ValueError) as cm:
      body.add_geom(axisangle=[1, 2, 3])
    self.assertEqual(
        str(cm.exception),
        'axisangle should be a list/array of size 4.',
    )
    with self.assertRaises(ValueError) as cm:
      body.add_geom(xyaxes=[1, 2, 3, 4, 5])
    self.assertEqual(
        str(cm.exception),
        'xyaxes should be a list/array of size 6.',
    )
    with self.assertRaises(ValueError) as cm:
      body.add_geom(zaxis=[1, 2, 3, 4])
    self.assertEqual(
        str(cm.exception),
        'zaxis should be a list/array of size 3.',
    )
    with self.assertRaises(ValueError) as cm:
      body.add_geom(euler=[1])
    self.assertEqual(
        str(cm.exception),
        'euler should be a list/array of size 3.',
    )
    with self.assertRaises(ValueError) as cm:
      body.add_geom(axisangle=[1, 2, 3, 4], euler=[1, 2, 3])
    self.assertEqual(
        str(cm.exception),
        'Only one of: axisangle, xyaxes, zaxis, oreuler can be set.',
    )
    with self.assertRaises(ValueError) as cm:
      spec.worldbody.add_body(iaxisangle=[1, 2, 3, 4], ieuler=[1, 2, 3])
    self.assertEqual(
        str(cm.exception),
        'Only one of: iaxisangle, ixyaxes, izaxis, orieuler can be set.',
    )

  def test_load_xml(self):
    file_path = epath.resource_path("mujoco") / "testdata" / "model.xml"
    filename = file_path.as_posix()
    state_type = mujoco.mjtState.mjSTATE_INTEGRATION

    # Load from file.
    spec1 = mujoco.MjSpec.from_file(filename)
    model1 = spec1.compile()
    data1 = mujoco.MjData(model1)
    mujoco.mj_step(model1, data1)
    size1 = mujoco.mj_stateSize(model1, state_type)
    state1 = np.empty(size1, np.float64)
    mujoco.mj_getState(model1, data1, state1, state_type)

    # Load from string.
    with open(filename, 'r') as file:
      spec2 = mujoco.MjSpec.from_string(file.read().rstrip())
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
        + f"Element name 'MyGeom', id 0, geom added on line {added_on_line}"
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

  def test_uncompiled_spec_can_be_written(self):
    spec = mujoco.MjSpec()
    spec.to_xml()

  def test_modelname_default_class(self):
    XML = textwrap.dedent("""\
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
    """)

    spec = mujoco.MjSpec()
    spec.modelname = 'test'

    main = spec.default
    main.geom.size[0] = 2

    def1 = spec.add_default('def1', main)
    def1.geom.size[0] = 3

    spec.worldbody.add_geom(def1)
    spec.worldbody.add_geom(main)

    spec.compile()
    self.assertEqual(spec.to_xml(), XML)
    spec = mujoco.MjSpec()
    spec.modelname = 'test'

    main = spec.default
    main.geom.size[0] = 2
    def1 = spec.add_default('def1', main)
    def1.geom.size[0] = 3

    geom1 = spec.worldbody.add_geom(def1)
    geom2 = spec.worldbody.add_geom()
    self.assertEqual(geom1.classname.name, 'def1')
    self.assertEqual(geom2.classname.name, 'main')

    spec.compile()
    self.assertEqual(spec.to_xml(), XML)

    spec = mujoco.MjSpec()
    spec.modelname = 'test'

    main = spec.default
    main.geom.size[0] = 2
    def1 = spec.add_default('def1', main)
    def1.geom.size[0] = 3

    geom1 = spec.worldbody.add_geom(size=[3, 0, 0])
    geom2 = spec.worldbody.add_geom(size=[2, 0, 0])
    geom1.classname = def1
    geom2.classname = main  # actually redundant, since main is always applied

    spec.compile()
    self.assertEqual(spec.to_xml(), XML)

    # test delete default
    def1 = spec.find_default('def1')
    spec.detach_default(def1)
    def1 = spec.find_default('def1')
    self.assertIsNone(def1)

  def test_element_list(self):
    spec = mujoco.MjSpec()
    sensor1 = spec.add_sensor()
    sensor2 = spec.add_sensor()
    sensor3 = spec.add_sensor()
    actuator1 = spec.add_actuator()
    actuator2 = spec.add_actuator()
    actuator3 = spec.add_actuator()
    sensor1.name = 'sensor1'
    sensor2.name = 'sensor2'
    sensor3.name = 'sensor3'
    actuator1.name = 'actuator1'
    actuator2.name = 'actuator2'
    actuator3.name = 'actuator3'
    self.assertLen(spec.sensors, 3)
    self.assertLen(spec.actuators, 3)
    self.assertEqual(spec.sensors[0].name, 'sensor1')
    self.assertEqual(spec.sensors[1].name, 'sensor2')
    self.assertEqual(spec.sensors[2].name, 'sensor3')
    self.assertEqual(spec.actuators[0].name, 'actuator1')
    self.assertEqual(spec.actuators[1].name, 'actuator2')
    self.assertEqual(spec.actuators[2].name, 'actuator3')
    self.assertEqual(spec.sensor('sensor1'), sensor1)
    self.assertEqual(spec.sensor('sensor2'), sensor2)
    self.assertEqual(spec.sensor('sensor3'), sensor3)
    self.assertEqual(spec.actuator('actuator1'), actuator1)
    self.assertEqual(spec.actuator('actuator2'), actuator2)
    self.assertEqual(spec.actuator('actuator3'), actuator3)

  def test_body_list(self):
    main_xml = """
    <mujoco>
      <worldbody>
        <body name="body1">
          <body name="body3">
            <site name="site1"/>
            <site name="site2"/>
            <site name="site3"/>
            <site name="site4"/>
            <body name="body4">
              <site name="site5"/>
              <joint name="joint1"/>
              <geom name="geom1" size="1"/>
            </body>
          </body>
        </body>
        <body name="body2"/>
      </worldbody>
    </mujoco>
    """
    spec = mujoco.MjSpec.from_string(main_xml)
    bodytype = mujoco.mjtObj.mjOBJ_BODY
    self.assertLen(spec.bodies, 5)
    self.assertLen(spec.sites, 5)
    self.assertLen(spec.worldbody.find_all('body'), 4)
    self.assertLen(spec.worldbody.find_all('site'), 5)
    self.assertLen(spec.worldbody.find_all('joint'), 1)
    self.assertLen(spec.worldbody.find_all('geom'), 1)
    self.assertEqual(spec.bodies[1].name, 'body1')
    self.assertEqual(spec.bodies[2].name, 'body2')
    self.assertEqual(spec.bodies[3].name, 'body3')
    self.assertEqual(spec.bodies[4].name, 'body4')
    self.assertEqual(spec.bodies[1].parent, spec.worldbody)
    self.assertEqual(spec.bodies[2].parent, spec.worldbody)
    self.assertEqual(spec.bodies[3].parent, spec.bodies[1])
    self.assertEqual(spec.bodies[4].parent, spec.bodies[3])
    self.assertLen(spec.worldbody.find_all(bodytype), 4)
    self.assertLen(spec.bodies[1].find_all(bodytype), 2)
    self.assertLen(spec.bodies[3].find_all(bodytype), 1)
    self.assertEqual(spec.worldbody.find_all('body')[0].name, 'body1')
    self.assertEqual(spec.worldbody.find_all('body')[1].name, 'body2')
    self.assertEqual(spec.worldbody.find_all('body')[2].name, 'body3')
    self.assertEqual(spec.worldbody.find_all('body')[3].name, 'body4')
    self.assertEqual(spec.bodies[1].find_all('body')[0].name, 'body3')
    self.assertEqual(spec.bodies[1].find_all('body')[1].name, 'body4')
    self.assertEqual(spec.bodies[3].find_all('body')[0].name, 'body4')
    self.assertEmpty(spec.bodies[2].find_all('body'))
    self.assertEmpty(spec.bodies[4].find_all('body'))
    self.assertEqual(spec.worldbody.find_all('site')[0].name, 'site1')
    self.assertEqual(spec.worldbody.find_all('site')[1].name, 'site2')
    self.assertEqual(spec.worldbody.find_all('site')[2].name, 'site3')
    self.assertEqual(spec.worldbody.find_all('site')[3].name, 'site4')
    self.assertEqual(spec.worldbody.find_all('site')[4].name, 'site5')
    self.assertEmpty(spec.bodies[2].sites)
    self.assertLen(spec.bodies[3].sites, 4)
    self.assertLen(spec.bodies[4].sites, 1)
    self.assertEqual(spec.bodies[3].sites[0].name, 'site1')
    self.assertEqual(spec.bodies[3].sites[1].name, 'site2')
    self.assertEqual(spec.bodies[3].sites[2].name, 'site3')
    self.assertEqual(spec.bodies[3].sites[3].name, 'site4')
    self.assertEqual(spec.bodies[4].sites[0].name, 'site5')
    self.assertEqual(spec.bodies[3].sites[0].parent, spec.bodies[3])
    self.assertEqual(spec.bodies[3].sites[1].parent, spec.bodies[3])
    self.assertEqual(spec.bodies[3].sites[2].parent, spec.bodies[3])
    self.assertEqual(spec.bodies[3].sites[3].parent, spec.bodies[3])
    self.assertEqual(spec.bodies[4].sites[0].parent, spec.bodies[4])
    with self.assertRaises(ValueError) as cm:
      spec.worldbody.find_all('actuator')
    self.assertEqual(
        str(cm.exception),
        'body.find_all supports the types: body, frame, geom, site,'
        ' joint, light, camera.',
    )
    body4 = spec.worldbody.find_all('body')[3]
    body4.name = 'body4_new'
    self.assertEqual(spec.bodies[4].name, 'body4_new')

  def test_geom_list(self):
    main_xml = """
    <mujoco>
      <worldbody>
        <body name="body1"/>
      </worldbody>
    </mujoco>
    """
    spec = mujoco.MjSpec.from_string(main_xml)
    geom1 = spec.worldbody.add_geom(name='geom1')
    geom2 = spec.worldbody.add_geom(name='geom2')
    geom3 = spec.body('body1').add_geom(name='geom3')

    self.assertEqual(spec.geoms, [geom1, geom2, geom3])
    self.assertEqual(spec.geom('geom1'), geom1)
    self.assertEqual(spec.geom('geom2'), geom2)
    self.assertEqual(spec.geom('geom3'), geom3)

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
    spec.modelname = 'test'
    mesh = spec.add_mesh()
    mesh.name = 'cube'
    mesh.file = 'cube.obj'
    geom = spec.worldbody.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_MESH
    geom.meshname = 'cube'
    spec.assets = {'cube.obj': cube}
    model = spec.compile()
    self.assertEqual(model.nmeshvert, 8)
    self.assertEqual(spec.assets['cube.obj'], cube)
    self.assertIs(
        spec.assets['cube.obj'], cube,
        'Asset dict should contain a reference, not a copy'
    )

    xml = """
    <mujoco model="test">
      <compiler angle="radian"/>

      <asset>
        <mesh name="cube" file="cube.obj"/>
      </asset>

      <worldbody>
        <geom type="mesh" mesh="cube"/>
      </worldbody>
    </mujoco>
    """
    assets = {'cube.obj': cube}
    spec = mujoco.MjSpec.from_string(xml, assets=assets)
    model = spec.compile()
    self.assertEqual(model.nmeshvert, 8)
    self.assertEqual(spec.assets['cube.obj'], cube)
    self.assertIs(
        spec.assets['cube.obj'], cube,
        'Asset dict should contain a reference, not a copy'
    )
    del assets
    gc.collect()
    self.assertEqual(spec.assets['cube.obj'], cube)

  def test_include(self):
    included_xml = """
      <mujoco>
        <worldbody>
          <body>
            <geom type="box" size="1 1 1"/>
          </body>
        </worldbody>
      </mujoco>
    """
    spec = mujoco.MjSpec.from_string(
        textwrap.dedent("""
      <mujoco model="MuJoCo Model">
        <include file="included.xml"/>
      </mujoco>
    """),
        include={'included.xml': included_xml.encode('utf-8')},
    )
    self.assertEqual(
        spec.worldbody.first_body().first_geom().type, mujoco.mjtGeom.mjGEOM_BOX
    )

  def test_delete(self):
    file_path = epath.resource_path("mujoco") / "testdata" / "model.xml"
    spec = mujoco.MjSpec.from_file(file_path.as_posix())

    model = spec.compile()
    self.assertIsNotNone(model)
    self.assertEqual(model.nsite, 11)
    self.assertEqual(model.nsensor, 11)

    head = spec.body('head')
    self.assertIsNotNone(head)
    site = head.first_site()
    self.assertIsNotNone(site)
    self.assertEqual(site, spec.site('head'))

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

    spec = mujoco.MjSpec.from_string(xml)
    self.assertIsNotNone(spec.worldbody)

    body = spec.worldbody.add_body()
    body.plugin.plugin_name = 'mujoco.elasticity.cable'
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

    spec = mujoco.MjSpec.from_string(main_xml)
    model = spec.compile()
    data = mujoco.MjData(model)

    spec.add_material().name = 'yellow'
    spec.add_material().name = 'yellow'

    with self.assertRaisesRegex(
        ValueError, "Error: repeated name 'yellow' in material"
    ):
      spec.recompile(model, data)

  def test_delete_unused_plugin(self):
    spec = mujoco.MjSpec.from_string("""
      <mujoco model="MuJoCo Model">
        <extension>
          <plugin plugin="mujoco.pid">
            <instance name="pid1">
              <config key="kp" value="4.0"/>
            </instance>
          </plugin>
        </extension>

        <worldbody>
          <body>
            <geom size="1"/>
          </body>
        </worldbody>
      </mujoco>
    """)
    plugin = spec.plugins[0]
    self.assertIsNotNone(plugin)
    plugin.delete()

    model = spec.compile()
    self.assertIsNotNone(model)
    self.assertEqual(model.nplugin, 0)

  def test_access_option_stat_visual(self):
    spec = mujoco.MjSpec.from_string("""
      <mujoco model="MuJoCo Model">
        <option timestep="0.001"/>
        <statistic meansize="0.05"/>
        <visual>
          <quality shadowsize="4096"/>
          <headlight active="0"/>
          <rgba camera="0 0 0 0"/>
        </visual>
      </mujoco>
    """)
    self.assertEqual(spec.option.timestep, 0.001)
    self.assertEqual(spec.stat.meansize, 0.05)
    self.assertEqual(spec.visual.quality.shadowsize, 4096)
    self.assertEqual(spec.visual.headlight.active, 0)
    self.assertEqual(spec.visual.global_, getattr(spec.visual, 'global'))
    np.testing.assert_array_equal(spec.visual.rgba.camera, [0, 0, 0, 0])

    spec.option.timestep = 0.002
    spec.stat.meansize = 0.06
    spec.visual.quality.shadowsize = 8192
    spec.visual.headlight.active = 1
    spec.visual.rgba.camera = [1, 1, 1, 1]

    model = spec.compile()

    self.assertEqual(model.opt.timestep, 0.002)
    self.assertEqual(model.stat.meansize, 0.06)
    self.assertEqual(model.vis.quality.shadowsize, 8192)
    self.assertEqual(model.vis.headlight.active, 1)
    np.testing.assert_array_equal(model.vis.rgba.camera, [1, 1, 1, 1])

  def test_assign_list_element(self):
    spec = mujoco.MjSpec()
    material = spec.add_material()
    texture_index = mujoco.mjtTextureRole.mjTEXROLE_RGB

    # Assign a string to a list element.
    material.textures[texture_index] = 'texture_name'
    self.assertEqual(material.textures[texture_index], 'texture_name')

    # Assign a complete list
    material.textures = ['', 'new_name', '', '', '']
    self.assertEqual(material.textures[texture_index], 'new_name')

    # textures is iterable
    self.assertEqual('new_name', ''.join(material.textures))

    # supports `len`
    self.assertLen(material.textures, 5)

    # field checks for out-of-bound access on read and on write
    with self.assertRaises(IndexError):
      material.textures[5] = 'x'

    with self.assertRaises(IndexError):
      material.textures[-1] = 'x'

  def test_assign_texture(self):
    spec = mujoco.MjSpec()
    texture = spec.add_texture(name='texture', height=2, width=2)
    texture.data = np.zeros((2, 2, 3), dtype=np.uint8).tobytes()
    spec.compile()

  def test_attach_units(self):
    child = mujoco.MjSpec()
    parent = mujoco.MjSpec()
    parent.compiler.degree = not child.compiler.degree
    body = child.worldbody.add_body(euler=[90, 0, 0])
    frame = parent.worldbody.add_frame(euler=[-mujoco.mjPI / 2, 0, 0])
    frame.attach_body(body, prefix='child-')
    model = parent.compile()
    np.testing.assert_almost_equal(model.body_quat[1], [1, 0, 0, 0])

  def test_attach_to_site(self):
    parent = mujoco.MjSpec()
    parent.assets = {'cube.obj': 'cube_content'}
    site = parent.worldbody.add_site(pos=[1, 2, 3], quat=[0, 0, 0, 1])
    site.name = 'site'

    # Attach body to site and compile.
    child1 = mujoco.MjSpec()
    child1.assets = {'cube1.obj': 'cube1_content'}
    body1 = child1.worldbody.add_body()
    self.assertIs(body1, site.attach_body(body1, prefix='_'))
    self.assertIsNotNone(child1.worldbody)
    body1.pos = [1, 1, 1]
    model1 = parent.compile()
    self.assertIsNotNone(model1)
    self.assertEqual(model1.nbody, 2)
    np.testing.assert_array_equal(model1.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model1.body_quat[1], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')

    # Attach entire spec to site and compile again.
    child2 = mujoco.MjSpec()
    child2.assets = {'cube2.obj': 'cube2_content'}
    body2 = child2.worldbody.add_body(name='body')
    self.assertIsNotNone(parent.attach(child2, site=site, prefix='child2-'))
    self.assertIsNotNone(child2.worldbody)
    self.assertEqual(child2.parent, parent)
    body2.pos = [-1, -1, -1]
    model2 = parent.compile()
    self.assertIsNotNone(model2)
    self.assertEqual(model2.nbody, 3)
    np.testing.assert_array_equal(model2.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model2.body_pos[2], [2, 3, 2])
    np.testing.assert_array_equal(model2.body_quat[1], [0, 0, 0, 1])
    np.testing.assert_array_equal(model2.body_quat[2], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')
    self.assertEqual(parent.assets['cube2.obj'], 'cube2_content')

    # Attach another spec to site (referenced by name) and compile again.
    child3 = mujoco.MjSpec()
    child3.assets = {'cube3.obj': 'cube3_content'}
    body3 = child3.worldbody.add_body(name='body')
    self.assertIsNotNone(parent.attach(child3, site='site', prefix='child3-'))
    self.assertIsNotNone(child3.worldbody)
    self.assertEqual(child3.parent, parent)
    body3.pos = [-2, -2, -2]
    model3 = parent.compile()
    self.assertIsNotNone(model3)
    self.assertEqual(model3.nbody, 4)
    np.testing.assert_array_equal(model3.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model3.body_pos[2], [2, 3, 2])
    np.testing.assert_array_equal(model3.body_pos[3], [3, 4, 1])
    np.testing.assert_array_equal(model3.body_quat[1], [0, 0, 0, 1])
    np.testing.assert_array_equal(model3.body_quat[2], [0, 0, 0, 1])
    np.testing.assert_array_equal(model3.body_quat[3], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')
    self.assertEqual(parent.assets['cube2.obj'], 'cube2_content')
    self.assertEqual(parent.assets['cube3.obj'], 'cube3_content')

    # Fail to attach to a site that does not exist.
    child4 = mujoco.MjSpec()
    with self.assertRaisesRegex(ValueError, 'Site not found.'):
      parent.attach(child4, site='invalid_site', prefix='child3-')

  def test_body_to_frame(self):
    spec = mujoco.MjSpec()
    body = spec.worldbody.add_body(pos=[1, 2, 3])
    spec.compile()
    frame = body.to_frame()
    np.testing.assert_array_equal(frame.pos, [1, 2, 3])

  def test_attach_to_frame(self):
    parent = mujoco.MjSpec()
    parent.assets = {'cube.obj': 'cube_content'}
    frame = parent.worldbody.add_frame(pos=[1, 2, 3], quat=[0, 0, 0, 1])
    frame.name = 'frame'

    # Attach body to frame and compile.
    child1 = mujoco.MjSpec()
    child1.assets = {'cube1.obj': 'cube1_content'}
    body1 = child1.worldbody.add_body()
    self.assertIs(body1, frame.attach_body(body1, prefix='_'))
    self.assertIsNotNone(child1.worldbody)
    body1.pos = [1, 1, 1]
    model1 = parent.compile()
    self.assertIsNotNone(model1)
    self.assertEqual(model1.nbody, 2)
    np.testing.assert_array_equal(model1.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model1.body_quat[1], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')

    # Attach entire spec to frame and compile again.
    child2 = mujoco.MjSpec()
    child2.assets = {'cube2.obj': 'cube2_content'}
    body2 = child2.worldbody.add_body(name='body')
    body2.set_frame(child2.worldbody.add_frame(pos=[-1, -1, 1]))
    self.assertIsNotNone(parent.attach(child2, frame=frame, prefix='child-'))
    self.assertIsNotNone(child2.worldbody)
    self.assertEqual(child2.parent, parent)
    body2.pos = [-1, -1, -1]
    model2 = parent.compile()
    self.assertIsNotNone(model2)
    self.assertEqual(model2.nbody, 3)
    np.testing.assert_array_equal(model2.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model2.body_pos[2], [3, 4, 3])
    np.testing.assert_array_equal(model2.body_quat[1], [0, 0, 0, 1])
    np.testing.assert_array_equal(model2.body_quat[2], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')
    self.assertEqual(parent.assets['cube2.obj'], 'cube2_content')

    # Attach another spec to frame (referenced by name) and compile again.
    child3 = mujoco.MjSpec()
    child3.assets = {'cube2.obj': 'new_cube2_content'}
    body3 = child3.worldbody.add_body(name='body')
    body3.set_frame(child3.worldbody.add_frame(pos=[-1, -1, 1]))
    self.assertIsNotNone(parent.attach(child3, frame='frame', prefix='child3-'))
    self.assertIsNotNone(child3.worldbody)
    self.assertEqual(child3.parent, parent)
    body3.pos = [-2, -2, -2]
    model3 = parent.compile()
    self.assertIsNotNone(model3)
    self.assertEqual(model3.nbody, 4)
    np.testing.assert_array_equal(model3.body_pos[1], [0, 1, 4])
    np.testing.assert_array_equal(model3.body_pos[2], [3, 4, 3])
    np.testing.assert_array_equal(model3.body_pos[3], [4, 5, 2])
    np.testing.assert_array_equal(model3.body_quat[1], [0, 0, 0, 1])
    np.testing.assert_array_equal(model3.body_quat[2], [0, 0, 0, 1])
    np.testing.assert_array_equal(model3.body_quat[3], [0, 0, 0, 1])
    self.assertEqual(parent.assets['cube.obj'], 'cube_content')
    self.assertEqual(parent.assets['cube2.obj'], 'new_cube2_content')

    # Fail to attach to a frame that does not exist.
    child4 = mujoco.MjSpec()
    with self.assertRaisesRegex(ValueError, 'Frame not found.'):
      parent.attach(child4, frame='invalid_frame', prefix='child3-')

  def test_bind(self):
    spec = mujoco.MjSpec.from_string("""
    <mujoco>
      <worldbody>
        <body name="main">
          <geom name="main" size="0.15 0.15 0.15" mass="1" type="box"/>
          <freejoint/>
          <body name="box">
            <joint name="box" type="hinge" range="-1 +1"/>
            <geom name="box" size="0.15 0.15 0.15" mass="1" type="box"/>
          </body>
          <body name="sphere">
            <joint name="sphere" type="hinge" range="-1 +1"/>
            <geom name="sphere" size="0.15 0.15 0.15" mass="1" type="box"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """)
    joint_box = spec.joint('box')
    joint_sphere = spec.joint('sphere')
    joints = [joint_box, joint_sphere]
    mj_model = spec.compile()
    mj_data = mujoco.MjData(mj_model)
    np.testing.assert_array_equal(mj_data.bind(joint_box).qpos, 0)
    np.testing.assert_array_equal(mj_model.bind(joint_box).qposadr, 7)
    np.testing.assert_array_equal(mj_data.bind(joints).qpos, [0, 0])
    np.testing.assert_array_equal(mj_model.bind(joints).qposadr, [7, 8])
    np.testing.assert_array_equal(mj_data.bind([]).qpos, [])
    np.testing.assert_array_equal(mj_model.bind([]).qposadr, [])
    with self.assertRaisesRegex(
        AttributeError, "object has no attribute 'invalid'"
    ):
      print(mj_model.bind(joints).invalid)
    invalid_spec = mujoco.MjSpec()
    invalid_spec.worldbody.add_body(name='main')
    with self.assertRaisesRegex(
        ValueError,
        'The mjSpec does not match mjModel. Please recompile the mjSpec.',
    ):
      print(mj_model.bind(invalid_spec.body('main')))

  def test_incorrect_hfield_size(self):
    nrow = 300
    ncol = 400
    hdata = np.random.uniform(size=(1, 1))
    model_spec = mujoco.MjSpec()
    model_spec.add_hfield(
        name='hfield',
        size=[1, 1, 1, 1e-3],
        ncol=ncol,
        nrow=nrow,
        userdata=hdata.flatten(),
    )
    model_spec.worldbody.add_geom(
        name='hfield',
        type=mujoco.mjtGeom.mjGEOM_HFIELD,
        pos=np.array([0, 0, 1]),
        hfieldname='hfield',
    )
    with self.assertRaisesRegex(
        ValueError, r"Error: elevation data length must match nrow\*ncol\n"
        "Element name 'hfield', id 0",
    ):
      model_spec.compile()

  def test_address(self):
    spec1 = mujoco.MjSpec()
    spec2 = mujoco.MjSpec()
    spec3 = spec1.copy()

    self.assertGreater(spec1._address, 0)
    self.assertGreater(spec2._address, 0)
    self.assertGreater(spec3._address, 0)
    self.assertLen({spec1._address, spec2._address, spec3._address}, 3)

if __name__ == '__main__':
  absltest.main()
