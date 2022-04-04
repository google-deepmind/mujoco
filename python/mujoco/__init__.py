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
"""Python bindings for MuJoCo."""

import ctypes
import ctypes.util
import os
import platform
import subprocess

_SYSTEM = platform.system()
if _SYSTEM == 'Windows':
  ctypes.WinDLL(os.path.join(os.path.dirname(__file__), 'mujoco.dll'))

from mujoco._callbacks import *
from mujoco._constants import *
from mujoco._enums import *
from mujoco._errors import *
from mujoco._functions import *
from mujoco._render import *
from mujoco._structs import *

# pylint: disable=g-import-not-at-top
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

  if _SYSTEM == 'Linux' and _MUJOCO_GL == 'osmesa':
    from mujoco.osmesa import GLContext
  elif _SYSTEM == 'Linux' and _MUJOCO_GL == 'egl':
    from mujoco.egl import GLContext
  else:
    from mujoco.glfw import GLContext

HEADERS_DIR = os.path.join(os.path.dirname(__file__), 'include')

__version__ = mj_versionString()  # pylint: disable=undefined-variable
