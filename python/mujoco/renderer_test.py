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
"""Tests for the MuJoCo renderer."""

from absl.testing import absltest
from absl.testing import parameterized
import mujoco
import numpy as np


@absltest.skipUnless(
    hasattr(mujoco, 'GLContext'), 'MuJoCo rendering is disabled'
)
class MuJoCoRendererTest(parameterized.TestCase):

  def test_renderer_unknown_camera_name(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="a"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.Renderer(model, 50, 50) as renderer:
      mujoco.mj_forward(model, data)
      with self.assertRaisesRegex(ValueError, r'camera "b" does not exist'):
        renderer.update_scene(data, 'b')

  def test_renderer_camera_under_range(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="a"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.Renderer(model, 50, 50) as renderer:
      mujoco.mj_forward(model, data)
      with self.assertRaisesRegex(ValueError, '-2 is out of range'):
        renderer.update_scene(data, -2)

  def test_renderer_camera_over_range(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="a"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.Renderer(model, 50, 50) as renderer:
      mujoco.mj_forward(model, data)
      with self.assertRaisesRegex(ValueError, '1 is out of range'):
        renderer.update_scene(data, 1)

  def test_renderer_renders_scene(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    <geom name="white_box" type="box" size="1 1 1" rgba="1 1 1 1"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    with mujoco.Renderer(model, 50, 50) as renderer:
      mujoco.mj_forward(model, data)
      renderer.update_scene(data, 'closeup')

      pixels = renderer.render().flatten()
      not_all_black = False

      # Pixels should all be a neutral color.
      for pixel in pixels:
        if pixel > 0:
          not_all_black = True
          break
      self.assertTrue(not_all_black)

  def test_renderer_output_without_out(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    <geom name="white_box" type="box" size="1 1 1" rgba="1 1 1 1"/>
  </worldbody>
</mujoco>
"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, 50, 50) as renderer:
      renderer.update_scene(data, 'closeup')
      pixels = [renderer.render()]

      colors = (
          (1.0, 0.0, 0.0, 1.0),
          (0.0, 1.0, 0.0, 1.0),
          (0.0, 0.0, 1.0, 1.0),
      )

      for i, color in enumerate(colors):
        model.geom_rgba[0, :] = color
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, 'closeup')
        pixels.append(renderer.render())
        self.assertIsNot(pixels[-2], pixels[-1])

        # Pixels should change over steps.
        self.assertFalse((pixels[i + 1] == pixels[i]).all())

  def test_renderer_output_with_out(self):
    xml = """
<mujoco>
  <worldbody>
    <camera name="closeup" pos="0 -6 0" xyaxes="1 0 0 0 1 100"/>
    <geom name="white_box" type="box" size="1 1 1" rgba="1 1 1 1"/>
  </worldbody>
</mujoco>
"""
    render_size = (50, 50)
    render_out = np.zeros((*render_size, 3), np.uint8)
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with mujoco.Renderer(model, *render_size) as renderer:
      renderer.update_scene(data, 'closeup')

      self.assertTrue(np.all(render_out == 0))

      pixels = renderer.render(out=render_out)

      # Pixels should always refer to the same `render_out` array.
      self.assertIs(pixels, render_out)
      self.assertFalse(np.all(render_out == 0))

      failing_render_size = (10, 10)
      self.assertNotEqual(failing_render_size, render_size)
      with self.assertRaises(ValueError):
        renderer.render(out=np.zeros((*failing_render_size, 3), np.uint8))


if __name__ == '__main__':
  absltest.main()
