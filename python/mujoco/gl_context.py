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
"""Exports GLContext for MuJoCo Python bindings."""

# pylint: disable=g-import-not-at-top

import os
import platform


def _select_gl_context():
  """Selects the OpenGL context based on the current platform a list of user
  preferences specified via the MUJOCO_GL environment variable."""

  enabled = ('enable', 'enabled', 'on', 'true', '1', '')
  disabled = ('disable', 'disabled', 'off', 'false', '0')
  backends = ('glfw', 'glx', 'egl', 'osmesa', 'wgl', 'cgl')
  system = platform.system()
  if system == 'Linux':
    available = ('glx', 'egl', 'osmesa')
  elif system == 'Windows':
    available = ('wgl',)
  elif system == 'Darwin':
    available = ('cgl',)
  else:
    available = ()

  preferences = os.environ.get('MUJOCO_GL', '').lower().strip().split(',')

  valid = backends + enabled + disabled
  invalid = [x for x in preferences if x not in valid]
  if invalid:
    raise RuntimeError(
        'Invalid value for environment variable MUJOCO_GL.\n'
        f'  Specified: {",".join(preferences)}\n'
        f'  Invalid:   {",".join(invalid)}\n'
        f'  Valid:     {",".join(valid)}')

  avail = available + enabled + disabled
  if not any(x in avail for x in preferences):
    raise RuntimeError(
        'None of the backends in environment variable MUJOCO_GL are '
        'available on the platform.\n'
        f'  Specified: {",".join(preferences)}\n'
        f'  Available: {",".join(avail)}')

  for preference in preferences:

    if preference in disabled:
      return None

    if preference in enabled:
      from mujoco.glfw import GLContext
      return GLContext

    if preference in available:
      if preference == 'osmesa':
        from mujoco.osmesa import GLContext
        return GLContext
      elif preference == 'egl':
        from mujoco.egl import GLContext
        return GLContext
      elif preference == 'cgl':
        from mujoco.cgl import GLContext
        return GLContext
      elif preference in ('glfw', 'glx', 'wgl'):
        from mujoco.glfw import GLContext
        return GLContext


_GL_CONTEXT = _select_gl_context()
if _GL_CONTEXT:
  GLContext = _GL_CONTEXT
