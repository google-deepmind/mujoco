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
"""Tests that the API reference documentation is complete and up to date."""

import re

import os
import sys
import unittest as googletest
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
sys.path.insert(0, os.path.join(_REPO_ROOT, 'doc', 'generate'))
import generate_api_header
import generate_functions
import generate_schema

# Functions in headers that are intentionally not in functions.rst.
_FUNCTIONS_TO_SKIP = set()

# Types (STRUCT/ENUM) in headers that are intentionally not in APItypes.rst.
_TYPES_TO_SKIP = set()

# Type names documented in APItypes.rst that aren't STRUCT/ENUM in headers.
# These are typedefs, callbacks, and scalar types.
_EXTRA_DOCUMENTED_TYPES = {
    # scalar typedefs
    'mjtByte',
    'mjtBool',
    'mjtNum',
    'mjtSize',
    # C++ type aliases
    'mjByteVec',
    'mjDoubleVec',
    'mjFloatVec',
    'mjFloatVecVec',
    'mjIntVec',
    'mjIntVecVec',
    'mjString',
    'mjStringVec',
    # function pointer typedefs (callbacks)
    'mjfAct',
    'mjfCanDecode',
    'mjfCloseResource',
    'mjfCollision',
    'mjfConFilt',
    'mjfDecode',
    'mjfEncode',
    'mjfGeneric',
    'mjfGetResourceDir',
    'mjfItemEnable',
    'mjfLogHandler',
    'mjfOpenResource',
    'mjfReadResource',
    'mjfResourceModified',
    'mjfSensor',
    'mjfTime',
}


class DocTest(googletest.TestCase):

  def test_api_header(self):
    """Checks that references.h matches the generated output."""
    header_file = os.path.join(_REPO_ROOT, 'doc', 'includes', 'references.h')
    source = generate_api_header.generate_reference_header(
        generate_api_header.read_headers()
    )
    with open(header_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'references.h' needs to be updated.")

  def test_schema(self):
    """Checks that XMLschema.rst matches the generated output."""
    schema_file = os.path.join(_REPO_ROOT, 'doc', 'XMLschema.rst')
    source = generate_schema.generate()
    with open(schema_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'XMLschema.rst' needs to be updated.")

  def test_functions(self):
    """Checks that functions.rst matches the generated output."""
    functions_file = os.path.join(_REPO_ROOT, 'doc', 'APIreference', 'functions.rst')
    source = generate_functions.generate()
    with open(functions_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'functions.rst' needs to be updated.")

  def test_all_functions_included(self):
    """Checks that every public C function has an entry in functions.rst."""

    functions_file = os.path.join(_REPO_ROOT, 'doc', 'APIreference', 'functions.rst')
    with open(functions_file, 'r', encoding='utf-8') as file:
      content = file.read()

    documented = set(
        re.findall(r'^\.\. _(mj[a-zA-Z0-9_]+):', content, flags=re.MULTILINE)
    )

    api = generate_api_header.read_headers()
    header_funcs = {token for token, d in api.items() if d.c_type == 'FUNCTION'}

    errors = []
    for token in sorted(header_funcs - documented - _FUNCTIONS_TO_SKIP):
      d = api[token]
      errors.append(f'  undocumented: {token} (section: {d.section!r})')
    for token in sorted(documented - header_funcs):
      errors.append(f'  stale: {token} (in functions.rst but not in headers)')

    if errors:
      msg = 'functions.rst mismatches:\n' + '\n'.join(errors)
      self.fail(msg)

  def test_all_types_included(self):
    """Checks that every public struct and enum has an entry in APItypes.rst."""

    types_file = os.path.join(_REPO_ROOT, 'doc', 'APIreference', 'APItypes.rst')
    with open(types_file, 'r', encoding='utf-8') as file:
      content = file.read()

    documented = set(
        re.findall(r'^\.\. _(mj[a-zA-Z0-9_]+):', content, flags=re.MULTILINE)
    )

    api = generate_api_header.read_headers()
    header_types = {
        token for token, d in api.items() if d.c_type in ('STRUCT', 'ENUM')
    }

    errors = []
    for token in sorted(header_types - documented - _TYPES_TO_SKIP):
      d = api[token]
      errors.append(
          f'  undocumented: {token} ({d.c_type}, section: {d.section!r})'
      )
    for token in sorted(documented - header_types - _EXTRA_DOCUMENTED_TYPES):
      errors.append(f'  stale: {token} (in APItypes.rst but not in headers)')

    if errors:
      msg = 'APItypes.rst mismatches:\n' + '\n'.join(errors)
      self.fail(msg)


if __name__ == '__main__':
  googletest.main()
