"""Checks the native Studio modules included in the optional package."""

import importlib
import sys

from absl.testing import absltest
from mujoco import _render_filament
from mujoco.experimental.dear_imgui import dear_imgui
from mujoco.experimental.implot import implot
from mujoco.experimental.studio import native_viewer_cc
from mujoco.experimental.studio import renderer
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import window


class StudioImportTest(absltest.TestCase):

  def test_native_modules(self):
    for module in (
        _render_filament, dear_imgui, implot, native_viewer_cc,
        renderer, sim, ux, window,
    ):
      self.assertIsNotNone(module.__file__)
    if sys.platform != 'win32':
      for name in ('headless_ui', 'state_payload'):
        module = importlib.import_module(
            'mujoco.experimental.studio.web.' + name
        )
        self.assertIsNotNone(module.__file__)


if __name__ == '__main__':
  absltest.main()
