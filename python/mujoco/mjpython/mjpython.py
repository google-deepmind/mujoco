#!/usr/bin/env python
# Copyright 2023 DeepMind Technologies Limited
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
"""Python interpreter trampoline for macOS to support non-block Cocoa GUI.

This script executes a native binary that runs the CPython interpreter entry
point in a separate thread, thus leaving the macOS main thread free for Cocoa
GUI calls without blocking the user's Python script. In other words, Python's
idea of the "main thread" is different from the thread that holds the
com.apple.main-thread DispatchQueue.
"""
import ctypes
import importlib.util
import os
import platform
import sys

if platform.system() != 'Darwin':
  raise RuntimeError('This script only works on macOS')

_NSGetExecutablePath = getattr(ctypes.CDLL(None), '_NSGetExecutablePath')


def get_executable_path():
  c_path_size = ctypes.c_int32(0)
  _NSGetExecutablePath(None, ctypes.byref(c_path_size))
  c_path = (ctypes.c_char * c_path_size.value)()
  _NSGetExecutablePath(ctypes.byref(c_path), ctypes.byref(c_path_size))
  return c_path.value.decode()


def main(argv):
  module_dir = os.path.dirname(importlib.util.find_spec('mujoco').origin)
  os.environ['MJPYTHON_BIN'] = os.path.join(
      module_dir, 'MuJoCo (mjpython).app/Contents/MacOS/mjpython')

  # Conda doesn't create a separate shared library for Python.
  # We instead use the Python binary itself, which can be dlopened just as well.
  os.environ['MJPYTHON_LIBPYTHON'] = get_executable_path()

  # argv[0] is currently the path to this script.
  # Replace it with sys.executable to preserve e.g. virtualenv path.
  argv[0] = sys.executable

  os.execve(os.environ['MJPYTHON_BIN'], argv, os.environ)


if __name__ == '__main__':
  main(sys.argv)
