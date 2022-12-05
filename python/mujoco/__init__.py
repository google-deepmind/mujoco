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
elif _SYSTEM == 'Darwin':
  proc_translated = subprocess.run(
      ['sysctl', '-n', 'sysctl.proc_translated'], capture_output=True).stdout
  try:
    is_rosetta = bool(int(proc_translated))
  except ValueError:
    is_rosetta = False
  if is_rosetta and platform.machine() == 'x86_64':
    raise ImportError(
        'You are running an x86_64 build of Python on an Apple Silicon '
        'machine. This is not supported by MuJoCo. Please install and run a '
        'native, arm64 build of Python.')

from mujoco._callbacks import *
from mujoco._constants import *
from mujoco._enums import *
from mujoco._errors import *
from mujoco._functions import *
from mujoco._render import *
from mujoco._structs import *
from mujoco.gl_context import *
from mujoco.renderer import Renderer

HEADERS_DIR = os.path.join(os.path.dirname(__file__), 'include/mujoco')
PLUGINS_DIR = os.path.join(os.path.dirname(__file__), 'plugin')

PLUGIN_HANDLES = []

def _load_all_bundled_plugins():
  for directory, _, filenames in os.walk(PLUGINS_DIR):
    for filename in filenames:
      PLUGIN_HANDLES.append(ctypes.CDLL(os.path.join(directory, filename)))

_load_all_bundled_plugins()

__version__ = mj_versionString()  # pylint: disable=undefined-variable
