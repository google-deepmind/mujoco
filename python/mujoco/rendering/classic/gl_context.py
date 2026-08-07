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

import ctypes
import ctypes.util
import os
import platform

# pylint: disable=g-import-not-at-top
_SYSTEM = platform.system()
_MUJOCO_GL = os.environ.get('MUJOCO_GL', '').lower().strip()
if _MUJOCO_GL not in ('disable', 'disabled', 'off', 'false', '0'):
  _VALID_MUJOCO_GL = ('enable', 'enabled', 'on', 'true', '1' , 'glfw', '')
  if _SYSTEM == 'Linux':
    _VALID_MUJOCO_GL += ('glx', 'egl', 'osmesa')
  elif _SYSTEM == 'Windows':
    _VALID_MUJOCO_GL += ('wgl',)
  elif _SYSTEM == 'Darwin':
    _VALID_MUJOCO_GL += ('cgl',)
  if _MUJOCO_GL not in _VALID_MUJOCO_GL:
    raise RuntimeError(
        f'invalid value for environment variable MUJOCO_GL: {_MUJOCO_GL}')

  def _resolve_gl_context():
    """Imports and returns the configured backend's GLContext class."""
    if _SYSTEM == 'Linux' and _MUJOCO_GL == 'osmesa':
      from mujoco.osmesa import GLContext as _GLContext
    elif _SYSTEM == 'Linux' and _MUJOCO_GL == 'egl':
      from mujoco.egl import GLContext as _GLContext
    elif _SYSTEM == 'Darwin':
      from mujoco.cgl import GLContext as _GLContext
    else:
      from mujoco.glfw import GLContext as _GLContext
    return _GLContext

  class GLContext:
    """Configured OpenGL backend, resolved lazily on construction."""

    def __new__(cls, *args, **kwargs):
      try:
        backend = _resolve_gl_context()
      except (ImportError, AttributeError) as exc:
        raise ImportError(
            f'Could not initialize the OpenGL backend for '
            f'MUJOCO_GL={_MUJOCO_GL!r}. Rendering is unavailable; the required '
            'OpenGL libraries may not be installed. Set MUJOCO_GL=disable to '
            'skip GL initialization entirely.'
        ) from exc
      return backend(*args, **kwargs)
