"""Tests to ensure that all Mujoco functions and structs are correctly handled.


This file contains tests that verify:
- All functions defined in Mujoco's introspect module are either bound in the
 generated bindings.cc file or explicitly excluded in constants.py.
- All structs defined in Mujoco's introspect module are either bound in the
 generated bindings.cc file or explicitly skipped in SKIPPED_STRUCTS in
 constants.py.


These tests help maintain the integrity of the generated WASM bindings by
ensuring that no functions or structs are accidentally missed or incorrectly
handled during the code generation process.
"""

import re

import unittest
from introspect import functions as introspect_functions
from introspect import structs as introspect_structs

from helpers import common
from helpers import constants
from helpers import function_utils


def _get_resource_content(file_path: str) -> str:
  """Reads resource file content using resources.GetResource."""
  try:
    with open(file_path, 'r') as f:
      return f.read()
  except FileNotFoundError:
    print(f'Warning: Resource {file_path} not found.')
    return ''
  except IOError as e:
    print(f'Error reading resource {file_path}: {e}')
    return ''


def _get_bound_functions_from_cc() -> set[str]:
  """Reads bindings.cc and extracts the names of bound functions."""
  content = _get_resource_content('./codegen/generated/bindings.cc')
  if not content:
    return set()

  bound_functions = set()
  # Find all strings within function("...") calls.
  matches = re.findall(r'function\("([^"]+)"', content)
  bound_functions.update(matches)
  return bound_functions


def _get_bound_structs_from_cc() -> set[str]:
  """Reads bindings.cc and extracts the names of bound structs."""
  content = _get_resource_content('./codegen/generated/bindings.cc')
  if not content:
    return set()

  bound_structs = set()
  # Find all strings within class_<...>("...") calls.
  matches = re.findall(r'class_<[^>]+>\("([^"]+)"\)', content)
  bound_structs.update(matches)
  return bound_structs


class BindingCoverageTest(unittest.TestCase):

  def test_function_coverage(self):
    """Asserts that each function is either excluded or bound."""
    all_functions = set(introspect_functions.FUNCTIONS.keys())
    excluded_functions = {
        name
        for name in all_functions
        if function_utils.is_excluded_function_name(name)
    }
    bound_functions = _get_bound_functions_from_cc()

    missing_functions = []
    for func_name in all_functions:
      if (func_name not in excluded_functions
          and func_name not in bound_functions):
        missing_functions.append(func_name)

    if missing_functions:
      error_message = (
          f"""The following functions from functions.py are neither excluded in
         constants.py nor bound in bindings.cc:


         {", ".join(sorted(missing_functions))}


         Please either add them to a exclusion list in
         constants.py or create a binding in bindings.cc.""")
      self.fail(error_message)

  def test_struct_coverage(self):
    """Asserts that each struct is either not bound or bound in structs.cc."""
    bound_structs = _get_bound_structs_from_cc()
    all_structs = {
        common.uppercase_first_letter(struct_name)
        for struct_name in introspect_structs.STRUCTS.keys()
    }
    skipped_structs = {
        common.uppercase_first_letter(struct_name)
        for struct_name in constants.SKIPPED_STRUCTS
    }
    missing_structs = []
    for struct_name in all_structs:
      if (struct_name not in skipped_structs
          and struct_name not in bound_structs):
        missing_structs.append(struct_name)
    error_messages = []

    if missing_structs:
      error_messages.append(
          f"""The following structs are defined in structs.py but are neither
           bound in bindings.cc nor listed in SKIPPED_STRUCTS:


           {", ".join(sorted(missing_structs))}


           Please either add them to SKIPPED_STRUCTS or create its binding
           in bindings.cc.""")

    if error_messages:
      self.fail('\n\n'.join(error_messages))


if __name__ == '__main__':
  unittest.main()
