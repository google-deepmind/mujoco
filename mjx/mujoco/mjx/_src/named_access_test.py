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
"""Test for `mjx.Model` named element access."""

import textwrap

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
from mujoco import mjx


class MjxModelNamedAccessTest(parameterized.TestCase):
  """Tests for named element access in mjx.Model."""

  def test_named_element_access(self) -> None:
    """Test that named access returns the same IDs as MjModel."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco model="test">
          <worldbody>
            <geom name="plane" type="plane" size="1 1 1"/>
            <body name="body" pos="0 0 0">
              <joint name="joint" type="slide" axis="1 0 0" range="-5 5"/>
              <geom name="box" type="box" size=".2 .1 .1" rgba=".9 .3 .3 1"/>
            </body>
          </worldbody>
          <actuator>
            <motor joint="joint" name="motor"/>
          </actuator>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    self.assertEqual(model.body("body").id, modelx.body("body").id)
    self.assertEqual(model.joint("joint").id, modelx.joint("joint").id)
    self.assertEqual(model.actuator("motor").id, modelx.actuator("motor").id)
    self.assertEqual(model.geom("plane").id, modelx.geom("plane").id)
    self.assertEqual(model.geom("box").id, modelx.geom("box").id)

  def test_body_access_by_name(self) -> None:
    """Test body access by name."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="test_body" pos="0 0 1"/>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    body_view = modelx.body("test_body")
    self.assertEqual(body_view.name, "test_body")
    self.assertEqual(body_view.id, model.body("test_body").id)

  def test_body_access_by_id(self) -> None:
    """Test body access by id."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="test_body" pos="0 0 1"/>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    body_view = modelx.body(1)  # 0 is worldbody
    self.assertEqual(body_view.id, 1)
    self.assertEqual(body_view.name, "test_body")

  def test_joint_access(self) -> None:
    """Test joint access."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="body">
              <joint name="hinge_joint" type="hinge"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    joint_view = modelx.joint("hinge_joint")
    self.assertEqual(joint_view.name, "hinge_joint")
    self.assertEqual(joint_view.id, model.joint("hinge_joint").id)

    # Test jnt alias
    jnt_view = modelx.jnt("hinge_joint")
    self.assertEqual(jnt_view.id, joint_view.id)

  def test_geom_access(self) -> None:
    """Test geom access."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="5 5 0.1"/>
            <body>
              <geom name="ball" type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    geom_view = modelx.geom("ball")
    self.assertEqual(geom_view.name, "ball")
    self.assertEqual(geom_view.id, model.geom("ball").id)

  def test_actuator_access(self) -> None:
    """Test actuator access."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="body">
              <joint name="joint" type="hinge"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
          <actuator>
            <motor name="motor1" joint="joint" gear="100"/>
          </actuator>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    actuator_view = modelx.actuator("motor1")
    self.assertEqual(actuator_view.name, "motor1")
    self.assertEqual(actuator_view.id, model.actuator("motor1").id)

  def test_site_access(self) -> None:
    """Test site access."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="body">
              <site name="end_effector" pos="0 0 0.5"/>
              <geom type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    site_view = modelx.site("end_effector")
    self.assertEqual(site_view.name, "end_effector")
    self.assertEqual(site_view.id, model.site("end_effector").id)

  def test_camera_access(self) -> None:
    """Test camera access."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <camera name="main_cam" pos="0 -3 2" xyaxes="1 0 0 0 1 1"/>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    camera_view = modelx.camera("main_cam")
    self.assertEqual(camera_view.name, "main_cam")
    self.assertEqual(camera_view.id, model.camera("main_cam").id)

    # Test cam alias
    cam_view = modelx.cam("main_cam")
    self.assertEqual(cam_view.id, camera_view.id)

  def test_key_not_found(self) -> None:
    """Test that KeyError is raised for non-existent elements."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="body"/>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    with self.assertRaises(KeyError):
      modelx.body("nonexistent")

  def test_index_out_of_range(self) -> None:
    """Test that IndexError is raised for out-of-range indices."""
    model = mujoco.MjModel.from_xml_string(textwrap.dedent("""
        <mujoco>
          <worldbody>
            <body name="body"/>
          </worldbody>
        </mujoco>
    """))
    modelx = mjx.put_model(model)

    with self.assertRaises(IndexError):
      modelx.body(100)

    with self.assertRaises(IndexError):
      modelx.body(-1)


if __name__ == "__main__":
  absltest.main()
