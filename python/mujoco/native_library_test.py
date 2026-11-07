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
"""Tests that the installed package can be used to build native modules."""

import ctypes
import importlib.util
import os
import platform
import shutil
import subprocess
import tempfile
from unittest import mock

from absl.testing import absltest
import mujoco


_PACKAGE_DIR = os.path.dirname(os.path.abspath(mujoco.__file__))


def _linker_library_name():
  system = platform.system()
  if system == 'Windows':
    return 'mujoco.lib'
  if system == 'Darwin':
    return 'libmujoco.dylib'
  return 'libmujoco.so'


def _load_setup_module():
  """Loads setup.py without running its setuptools.setup() call."""
  setup_path = os.path.join(os.path.dirname(_PACKAGE_DIR), 'setup.py')
  if not os.path.isfile(setup_path):
    return None
  try:
    import setuptools  # pylint: disable=g-import-not-at-top
  except ImportError:
    return None
  spec = importlib.util.spec_from_file_location('_mujoco_setup', setup_path)
  if spec is None or spec.loader is None:
    return None
  module = importlib.util.module_from_spec(spec)
  with mock.patch.object(setuptools, 'setup'):
    spec.loader.exec_module(module)
  return module


class NativeLibraryTest(absltest.TestCase):

  def test_linker_library_is_packaged(self):
    self.assertTrue(
        os.path.exists(os.path.join(_PACKAGE_DIR, _linker_library_name())),
        f'{_linker_library_name()} is missing from {_PACKAGE_DIR}; native '
        'modules cannot link against the packaged MuJoCo library',
    )

  def test_headers_are_packaged(self):
    self.assertTrue(
        os.path.exists(
            os.path.join(_PACKAGE_DIR, 'include', 'mujoco', 'mujoco.h')
        )
    )

  def test_framework_builds_skip_linker_library(self):
    setup = _load_setup_module()
    if setup is None:
      self.skipTest('setup.py is only available in a source checkout')

    builder = object.__new__(setup.BuildCMakeExtension)
    builder._mujoco_framework_path = '/fake/mujoco.framework'
    with tempfile.TemporaryDirectory() as tmp:
      with mock.patch.dict(os.environ, {'MUJOCO_PATH': tmp}):
        # a framework build has no standalone versioned dylib to copy, and must
        # not fail the build
        builder._copy_linker_library(tmp)
      self.assertEmpty(os.listdir(tmp))

  @absltest.skipIf(
      platform.system() == 'Windows', 'MSVC is not available on the test PATH'
  )
  def test_build_extension_against_package(self):
    """Compiles and loads a small shared library using `-lmujoco`."""
    cc = shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
    if cc is None:
      self.skipTest('no C compiler available')

    with tempfile.TemporaryDirectory() as tmp:
      source = os.path.join(tmp, 'probe.c')
      with open(source, 'w', encoding='utf-8') as f:
        f.write(
            '#include <mujoco/mujoco.h>\n'
            'int probe(void) { return mj_version(); }\n'
        )

      is_apple = platform.system() == 'Darwin'
      flags = ['-dynamiclib'] if is_apple else ['-shared', '-fPIC']
      output = os.path.join(tmp, 'libprobe' + ('.dylib' if is_apple else '.so'))
      subprocess.run(
          [
              cc,
              *flags,
              f'-I{os.path.join(_PACKAGE_DIR, "include")}',
              f'-L{_PACKAGE_DIR}',
              '-lmujoco',
              f'-Wl,-rpath,{_PACKAGE_DIR}',
              source,
              '-o',
              output,
          ],
          check=True,
          capture_output=True,
      )

      probe = ctypes.CDLL(output)
      probe.probe.restype = ctypes.c_int
      self.assertEqual(probe.probe(), mujoco.mj_version())


if __name__ == '__main__':
  absltest.main()
