"""Exercises native Studio rendering and viewer lifetime."""

import os

from absl.testing import absltest
import mujoco
from mujoco.experimental.studio import launch_thread
from mujoco.experimental.studio import native_viewer
from mujoco.experimental.studio import viewer_protocol


class StudioSmokeTest(absltest.TestCase):

  def test_native_viewer(self):
    model = mujoco.MjModel.from_xml_string('''
      <mujoco>
        <worldbody>
          <light pos="0 0 3"/>
          <geom type="plane" size="2 2 .1"/>
          <body pos="0 0 1">
            <freejoint/>
            <geom type="sphere" size=".2" rgba="1 .2 .1 1"/>
          </body>
        </worldbody>
      </mujoco>
    ''')
    endpoint, simulation = launch_thread.make_thread_endpoints()
    viewer = native_viewer.NativeViewer(
        viewer_protocol.ViewerConfig(
            title='Bazel Studio smoke',
            width=96,
            height=96,
            gfx=os.environ.get('STUDIO_GFX', 'opengl'),
        ),
        endpoint,
        model=model,
    )
    try:
      mujoco.mjv_defaultFreeCamera(viewer.model, viewer.camera)
      for _ in range(3):
        self.assertTrue(viewer.prepare_next_frame())
        mujoco.mj_step(viewer.model, viewer.data)
        viewer.sync()
      texture = viewer.render_to_texture(viewer.model, viewer.data, 0, 64, 64)
      self.assertGreater(texture, 0)
    finally:
      viewer.close()
      simulation.close()
    self.assertFalse(viewer.is_running())


if __name__ == '__main__':
  absltest.main()
