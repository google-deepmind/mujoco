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
"""Tests for MuJoCo Python rendering."""

from absl.testing import absltest
import mujoco
import numpy as np


@absltest.skipUnless(
    hasattr(mujoco, 'GLContext'), 'MuJoCo rendering is disabled'
)
class MuJoCoRenderTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.gl = mujoco.GLContext(640, 480)
    self.gl.make_current()

  def tearDown(self):
    super().tearDown()
    del self.gl

  def test_can_render(self):
    """Test that the bindings can successfully render a simple image.

    This test sets up a basic MuJoCo rendering context similar to the example in
    https://mujoco.readthedocs.io/en/latest/programming#visualization
    It calls `mjr_rectangle` rather than `mjr_render` so that we can assert an
    exact rendered image without needing golden data. The purpose of this test
    is to ensure that the bindings can correctly return pixels in Python, rather
    than to test MuJoCo's rendering pipeline itself.
    """

    self.model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    self.data = mujoco.MjData(self.model)

    scene = mujoco.MjvScene(self.model, maxgeom=0)
    mujoco.mjv_updateScene(
        self.model,
        self.data,
        mujoco.MjvOption(),
        mujoco.MjvPerturb(),
        mujoco.MjvCamera(),
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )

    context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

    # MuJoCo's default render buffer size is 640x480.
    full_rect = mujoco.MjrRect(0, 0, 640, 480)
    mujoco.mjr_rectangle(full_rect, 0, 0, 0, 1)

    blue_rect = mujoco.MjrRect(56, 67, 234, 123)
    mujoco.mjr_rectangle(blue_rect, 0, 0, 1, 1)

    expected_upside_down_image = np.zeros((480, 640, 3), dtype=np.uint8)
    expected_upside_down_image[67 : 67 + 123, 56 : 56 + 234, 2] = 255

    upside_down_image = np.empty((480, 640, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(upside_down_image, None, full_rect, context)
    np.testing.assert_array_equal(upside_down_image, expected_upside_down_image)

    # Check that mjr_readPixels can accept a flattened array.
    upside_down_image[:] = 0
    mujoco.mjr_readPixels(
        np.reshape(upside_down_image, -1), None, full_rect, context
    )
    np.testing.assert_array_equal(upside_down_image, expected_upside_down_image)
    context.free()

  def test_figure_renders_after_hidden_skin(self):
    xml = """
    <mujoco>
      <worldbody>
        <body name="box" pos="0 0 1">
          <freejoint/>
          <geom type="sphere" size="0.1"/>
        </body>
      </worldbody>
      <deformable>
        <skin name="triangle" group="3"
              vertex="0 0 0  0.2 0 0  0 0.2 0" face="0 1 2">
          <bone body="box" bindpos="0 0 0" bindquat="1 0 0 0"
                vertid="0 1 2" vertweight="1 1 1"/>
        </skin>
      </deformable>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    scene = mujoco.MjvScene(model, maxgeom=10)
    mujoco.mjv_updateScene(
        model,
        data,
        mujoco.MjvOption(),
        None,
        mujoco.MjvCamera(),
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )

    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
    full_rect = mujoco.MjrRect(0, 0, 640, 480)
    mujoco.mjr_render(full_rect, scene, context)

    figure = mujoco.MjvFigure()
    mujoco.mjv_defaultFigure(figure)
    figure.flg_legend = 0
    figure.flg_extend = 0
    figure.range[0][:] = (-1.0, 1.0)
    figure.range[1][:] = (-1.0, 1.0)
    figure.linergb[0] = (1.0, 0.0, 1.0)
    figure.linedata[0][0:4] = (-1.0, -1.0, 1.0, 1.0)
    figure.linepnt[0] = 2
    mujoco.mjr_figure(full_rect, figure, context)

    image = np.empty((480, 640, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(image, None, full_rect, context)
    magenta = (image[:, :, 0] > 200) & (image[:, :, 1] < 100) & (
        image[:, :, 2] > 200
    )
    self.assertGreater(np.count_nonzero(magenta), 0)
    context.free()

  def test_safe_to_free_context_twice(self):
    self.model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    self.data = mujoco.MjData(self.model)

    scene = mujoco.MjvScene(self.model, maxgeom=0)
    mujoco.mjv_updateScene(
        self.model,
        self.data,
        mujoco.MjvOption(),
        None,
        mujoco.MjvCamera(),
        mujoco.mjtCatBit.mjCAT_ALL,
        scene,
    )

    context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150)
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)

    context.free()
    context.free()

  def test_mjrrect_repr(self):
    rect = mujoco.MjrRect(1, 2, 3, 4)
    rect_repr = repr(rect)
    self.assertIn('MjrRect', rect_repr)
    self.assertIn('left: 1', rect_repr)


if __name__ == '__main__':
  absltest.main()
