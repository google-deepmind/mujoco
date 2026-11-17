# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Tests for the requests that the Filament Renderer builds from its views."""

import types
from unittest import mock

from absl.testing import absltest
import numpy as np

try:
  from mujoco.rendering.filament import renderer  # pylint: disable=g-import-not-at-top
except ImportError:
  renderer = None


class _FakeContext:
  """Stands in for an mjrf.Context, recording the frames it is asked for."""

  def __init__(self):
    self.requests = []
    self.reads = []

  def set_clear_color(self, color):
    pass

  def create_scene(self, params):
    return mock.Mock()

  def create_render_target(self, config):
    return mock.Mock()

  def render(self, requests, reads):
    self.requests = list(requests)
    self.reads = list(reads)

  def wait_for_frame(self, frame):
    pass


def _fake_render_request():
  return types.SimpleNamespace(target=None, viewport=types.SimpleNamespace())


def _viewport(request):
  viewport = request.viewport
  return (viewport.left, viewport.bottom, viewport.width, viewport.height)


@absltest.skipIf(renderer is None, 'Filament rendering is not installed')
class RendererTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    mjrf = renderer.mjrf
    self.enter_context(
        mock.patch.object(mjrf, 'RenderRequest', _fake_render_request)
    )
    self.enter_context(mock.patch.object(mjrf, 'ReadRequest', mock.Mock))
    self.enter_context(mock.patch.object(mjrf, 'SceneParams', mock.Mock))
    self.enter_context(mock.patch.object(mjrf, 'RenderTargetConfig', mock.Mock))
    self.ctx = _FakeContext()
    self.renderer = renderer.Renderer(self.ctx)
    self.renderer.scene('scn')

  def test_window_view_renders_into_the_window(self):
    self.renderer.resize_window(64, 48)

    self.renderer.render([
        renderer.View(
            scene='scn', target=renderer.WINDOW, viewport=(0.5, 0.0, 0.5, 1.0)
        )
    ])

    (request,) = self.ctx.requests
    self.assertIsNone(request.target)
    self.assertEqual(_viewport(request), (32, 0, 32, 48))
    self.assertEmpty(self.ctx.reads)

  def test_target_view_renders_into_its_target(self):
    target = self.renderer.target('out', np.zeros((48, 64, 3), dtype=np.uint8))

    self.renderer.render([renderer.View(scene='scn', target='out')])

    (request,) = self.ctx.requests
    self.assertIs(request.target, target)
    self.assertEqual(_viewport(request), (0, 0, 64, 48))
    self.assertLen(self.ctx.reads, 1)

  def test_unknown_target_raises(self):
    with self.assertRaisesRegex(ValueError, 'Target not found: missing'):
      self.renderer.render([renderer.View(scene='scn', target='missing')])


if __name__ == '__main__':
  absltest.main()
