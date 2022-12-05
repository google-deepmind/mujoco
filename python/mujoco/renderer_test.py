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


@absltest.skipUnless(hasattr(mujoco, 'GLContext'),
                     'MuJoCo rendering is disabled')
class MuJoCoRendererTest(parameterized.TestCase):
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
    renderer = mujoco.Renderer(model, 50, 50)
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
if __name__ == '__main__':
  absltest.main()
