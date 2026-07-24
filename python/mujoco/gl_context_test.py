# Copyright 2025 DeepMind Technologies Limited
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
"""Tests for lazy OpenGL backend resolution in gl_context."""

from unittest import mock

from absl.testing import absltest
from mujoco.rendering.classic import gl_context


@absltest.skipUnless(
    hasattr(gl_context, 'GLContext'), 'rendering is disabled (MUJOCO_GL=disable)'
)
class GLContextTest(absltest.TestCase):

  def test_construction_delegates_to_resolved_backend(self):
    created = {}

    class _FakeBackend:

      def __init__(self, width, height):
        created['size'] = (width, height)

    with mock.patch.object(
        gl_context, '_resolve_gl_context', return_value=_FakeBackend
    ):
      ctx = gl_context.GLContext(640, 480)

    self.assertIsInstance(ctx, _FakeBackend)
    self.assertEqual(created['size'], (640, 480))

  def test_missing_native_libraries_raise_importerror_on_construction(self):
    native_error = AttributeError(
        "'NoneType' object has no attribute 'glGetError'"
    )
    with mock.patch.object(
        gl_context, '_resolve_gl_context', side_effect=native_error
    ):
      with self.assertRaises(ImportError) as cm:
        gl_context.GLContext(640, 480)
    self.assertIs(cm.exception.__cause__, native_error)

  def test_missing_python_packages_raise_importerror_on_construction(self):
    import_error = ImportError("No module named 'OpenGL'")
    with mock.patch.object(
        gl_context, '_resolve_gl_context', side_effect=import_error
    ):
      with self.assertRaises(ImportError) as cm:
        gl_context.GLContext(640, 480)
    self.assertIs(cm.exception.__cause__, import_error)


if __name__ == '__main__':
  absltest.main()
