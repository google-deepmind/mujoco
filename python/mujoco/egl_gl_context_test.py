# Copyright 2026 DeepMind Technologies Limited
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
"""Tests for the EGL GLContext's __del__ safety on partial construction."""

import gc
import sys
from unittest import mock

from absl.testing import absltest

try:
  from mujoco import egl as egl_module
except ImportError:
  # The EGL backend can only be imported on systems where libEGL/libOpenGL
  # are present (see mujoco/egl/egl_ext.py). That is an environment
  # precondition orthogonal to the bug under test here, so we skip rather
  # than fail when it is unavailable (e.g. this is the case for the CI
  # matrix, which runs with MUJOCO_GL=disable).
  egl_module = None


@absltest.skipIf(
    egl_module is None,
    'The EGL backend could not be imported in this environment.',
)
class EglGlContextTest(absltest.TestCase):
  """Regression test for GLContext.__del__ on a partially-constructed instance.

  If `GLContext.__init__` raises before `self._context` is assigned -- e.g.
  because the EGL driver does not support the PLATFORM_DEVICE extension and
  `create_initialized_egl_device_display()` returns `EGL_NO_DISPLAY`, so
  `__init__` raises `ImportError` -- garbage collection later runs
  `__del__` -> `free()`, which used to unconditionally read `self._context`
  and raise `AttributeError: 'GLContext' object has no attribute
  '_context'`. That AttributeError masked the real, actionable ImportError
  raised from `__init__`. See issue #991, and the same bug class fixed for
  `Renderer` in #3225 (issue #3213).

  We can't rely on the test host's EGL driver actually lacking
  PLATFORM_DEVICE support, so we force that branch deterministically by
  patching `create_initialized_egl_device_display` to return
  `EGL.EGL_NO_DISPLAY`, exactly as it would on such a host. This drives a
  real `GLContext.__init__` call through its real failure path, rather than
  bypassing `__init__` altogether.
  """

  def test_del_safe_when_init_fails_without_platform_device_support(self):
    unraisable = []
    old_hook = sys.unraisablehook
    sys.unraisablehook = lambda args: unraisable.append(args)
    try:
      with mock.patch.object(egl_module, 'EGL_DISPLAY', None), \
          mock.patch.object(
              egl_module,
              'create_initialized_egl_device_display',
              return_value=egl_module.EGL.EGL_NO_DISPLAY,
          ):
        # __init__ must still raise ImportError -- that's the real,
        # actionable error this bug used to mask.
        with self.assertRaises(ImportError):
          egl_module.GLContext(640, 480)
        gc.collect()
    finally:
      sys.unraisablehook = old_hook

    self.assertEqual(
        [u.exc_type.__name__ for u in unraisable],
        [],
        msg=(
            'GLContext.__del__ raised on a partially-constructed instance; '
            'see #991.'
        ),
    )


if __name__ == '__main__':
  absltest.main()
