"""Regression tests for wheel header staging (run from the python directory)."""

import pathlib
import runpy
import tempfile
import unittest
from unittest import mock

import setuptools


class HeaderStagingTest(unittest.TestCase):

  def test_headers_have_one_mujoco_directory(self):
    with mock.patch.object(setuptools, 'setup'):
      setup = runpy.run_path('setup.py')
    with tempfile.TemporaryDirectory() as tmp:
      root = pathlib.Path(tmp)
      include = root / 'include'
      header = include / 'mujoco' / 'mujoco.h'
      header.parent.mkdir(parents=True)
      header.write_text('/* test header */', encoding='utf-8')
      ext = setup['CMakeExtension']('mujoco._structs')
      dist = setuptools.Distribution({'ext_modules': [ext]})
      command = setup['BuildCMakeExtension'](dist)
      command.ensure_finalized()
      command.build_lib = str(root / 'build')
      command._mujoco_include_path = str(include)
      command._copy_mujoco_headers()
      staged = root / 'build' / 'mujoco' / 'include' / 'mujoco'
      self.assertTrue((staged / 'mujoco.h').is_file())
      self.assertEqual((staged / 'mujoco.h').read_text(), header.read_text())
      self.assertFalse((staged / 'mujoco').exists())


if __name__ == '__main__':
  unittest.main()
