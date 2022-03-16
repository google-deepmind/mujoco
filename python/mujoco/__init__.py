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
import os
import platform
import subprocess

HEADERS_DIR = os.path.join(os.path.dirname(__file__), 'include')

_MUJOCO_GL_ENABLE = ('enable', 'enabled', 'on', 'true', '1' , '')
_MUJOCO_GL_DISABLE = ('disable', 'disabled', 'off', 'false', '0')
_MUJOCO_GL = os.environ.get('MUJOCO_GL', '').lower().strip()
_MUJOCO_GL_IS_VALID = True

_SYSTEM = platform.system()
if _SYSTEM == 'Linux':
  libglew_name = None
  if _MUJOCO_GL in _MUJOCO_GL_ENABLE + ('glfw', 'glx'):
    libglew_name = 'libglew.so'
  elif _MUJOCO_GL == 'egl':
    libglew_name = 'libglewegl.so'
  elif _MUJOCO_GL == 'osmesa':
    libglew_name = 'libglewosmesa.so'
  elif _MUJOCO_GL not in _MUJOCO_GL_DISABLE:
    _MUJOCO_GL_IS_VALID = False
  if libglew_name is not None:
    ctypes.CDLL(os.path.join(os.path.dirname(__file__), libglew_name),
                ctypes.RTLD_GLOBAL)
    ctypes.CDLL(
        os.path.join(os.path.dirname(__file__), 'libmujoco.so.2.1.2'),
        ctypes.RTLD_GLOBAL)
  else:
    ctypes.CDLL(
        os.path.join(os.path.dirname(__file__), 'libmujoco_nogl.so.2.1.2'),
        ctypes.RTLD_GLOBAL)
elif _SYSTEM == 'Windows':
  if _MUJOCO_GL in _MUJOCO_GL_ENABLE + ('glfw', 'wgl'):
    ctypes.WinDLL(os.path.join(os.path.dirname(__file__), 'mujoco.dll'))
  elif _MUJOCO_GL in _MUJOCO_GL_DISABLE:
    ctypes.WinDLL(os.path.join(os.path.dirname(__file__), 'mujoco_nogl.dll'))
  else:
    _MUJOCO_GL_IS_VALID = False

if not _MUJOCO_GL_IS_VALID:
  raise RuntimeError(
      f'invalid value for environment variable MUJOCO_GL: {_MUJOCO_GL}')

from mujoco._callbacks import *
from mujoco._constants import *
from mujoco._enums import *
from mujoco._errors import *
from mujoco._functions import *
from mujoco._structs import *

# pylint: disable=g-import-not-at-top
if _MUJOCO_GL not in _MUJOCO_GL_DISABLE:
  from mujoco._render import *
  if _SYSTEM != 'Linux':
    from mujoco.glfw import GLContext
  else:
    _dl_handle = ctypes.CDLL(None)
    if hasattr(_dl_handle, 'OSMesaCreateContextExt'):
      from mujoco.osmesa import GLContext
    elif hasattr(_dl_handle, 'eglCreateContext'):
      from mujoco.egl import GLContext
    else:
      from mujoco.glfw import GLContext
