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

import importlib.util
import os
import sys
import sysconfig


def main(argv):
  os.environ['MJPYTHON_LIBPYTHON'] = os.path.join(
      sysconfig.get_config_var('PYTHONFRAMEWORKPREFIX'),
      sysconfig.get_config_var('INSTSONAME'),
  )
  argv[0] = sys.executable
  mujoco_dir = os.path.dirname(importlib.util.find_spec('mujoco').origin)
  os.execve(
      os.path.join(mujoco_dir, 'MuJoCo (mjpython).app/Contents/MacOS/mjpython'),
      argv, os.environ)


if __name__ == '__main__':
  main(sys.argv)
