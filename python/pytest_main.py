"""Runs optional suites with their required dependencies present."""

import argparse
import importlib
import os
import sys
import tempfile

import pytest
from python.runfiles import runfiles


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--require-module', action='append', default=[])
  arguments, pytest_arguments = parser.parse_known_args()
  resolver = runfiles.Create()
  pytest_arguments = [
      resolver.Rlocation(argument) if not argument.startswith('-') else argument
      for argument in pytest_arguments
  ]
  tempfile.tempdir = os.environ['TEST_TMPDIR']
  for module in arguments.require_module:
    importlib.import_module(module)
  if output := os.environ.get('XML_OUTPUT_FILE'):
    pytest_arguments.append('--junitxml=' + output)
  return pytest.main(pytest_arguments)


if __name__ == '__main__':
  sys.exit(main())
