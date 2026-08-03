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
import generate_default_table
import generate_functions
import generate_mjcf_map
import generate_mjcf_table
import generate_read_table
import generate_schema
import mjcf_schema

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

  def test_mjcf_table(self):
    """Checks that mjcf_table.inc matches the schema-generated output."""
    table_file = os.path.join(_REPO_ROOT, 'src', 'xml', 'generated',
                              'mjcf_table.inc')
    source = generate_mjcf_table.generate()
    with open(table_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'mjcf_table.inc' needs to be updated.")

  def test_default_table(self):
    """Checks that mjcf_default_table.inc matches the schema-generated output."""
    table_file = os.path.join(_REPO_ROOT, 'src', 'xml', 'generated',
                              'mjcf_default_table.inc')
    source = generate_default_table.generate()
    with open(table_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'mjcf_default_table.inc' needs to be updated.")

  def test_mjcf_map(self):
    """Checks that mjcf_map.h matches the schema-generated output."""
    map_file = os.path.join(_REPO_ROOT, 'src', 'xml', 'generated',
                            'mjcf_map.h')
    source = generate_mjcf_map.generate()
    with open(map_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'mjcf_map.h' needs to be updated.")

  def test_read_table(self):
    """Checks that mjcf_read_table.inc matches the schema-generated output."""
    table_file = os.path.join(_REPO_ROOT, 'src', 'xml', 'generated',
                              'mjcf_read_table.inc')
    source = generate_read_table.generate()
    with open(table_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'mjcf_read_table.inc' needs to be updated.")

  def test_schema_enum_coverage(self):
    """Checks schema enums against the C enums they bind.

    Every schema constant must be a member of the bound C enum, and every C
    member must be a schema keyword, a count sentinel (mjN*), or a documented
    exemption -- so adding a C enum member without updating the schema fails
    here.
    """
    # C members deliberately not exposed as XML keywords
    exempt = {
        'bodysleep': {  # resolved states of 'auto', not settable
            'mjSLEEP_AUTO_ALLOWED', 'mjSLEEP_AUTO_NEVER'},
        'geomtype': {   # rendering-only types and the missing-geom sentinel
            'mjGEOM_ARROW', 'mjGEOM_ARROW1', 'mjGEOM_ARROW2', 'mjGEOM_LINE',
            'mjGEOM_LINEBOX', 'mjGEOM_FLEX', 'mjGEOM_SKIN', 'mjGEOM_LABEL',
            'mjGEOM_TRIANGLE', 'mjGEOM_NONE'},
        'texrole': {    # not settable from XML
            'mjTEXROLE_USER'},
    }
    # deliberately partial: keywords are a documented subset of the C enum
    partial = {'frameobj'}

    enums_c = {}
    for name in ('mjtype.h', 'mjspec.h'):
      path = os.path.join(_REPO_ROOT, 'include', 'mujoco', name)
      with open(path, 'r', encoding='utf-8') as file:
        content = file.read()
      for m in re.finditer(r'typedef enum (mjt\w+)\s*\{(.*?)\}\s*\1;',
                           content, re.S):
        enums_c[m.group(1)] = re.findall(r'^\s*(mj[A-Z]\w+)', m.group(2),
                                         re.M)

    schema_path = os.path.join(_REPO_ROOT, 'src', 'xml', 'mjcf.schema')
    schema = mjcf_schema.parse_file(schema_path)
    errors = []
    for name, enum in schema.enums.items():
      if not enum.ctype:
        continue
      if enum.ctype not in enums_c:
        errors.append(f'  {name}: C enum {enum.ctype} not found in headers')
        continue
      members = set(enums_c[enum.ctype])
      constants = {value for _, value in enum.items}
      for bad in sorted(constants - members):
        errors.append(f'  {name}: {bad} is not a member of {enum.ctype}')
      if name in partial:
        continue
      uncovered = {m for m in members - constants
                   if not re.match(r'mjN[A-Z]', m)} - exempt.get(name, set())
      for miss in sorted(uncovered):
        errors.append(f'  {name}: {enum.ctype} member {miss} has no keyword '
                      '(add it to the schema or to the exemptions here)')
    if errors:
      self.fail('schema enum coverage:\n' + '\n'.join(errors))

  def test_schema(self):
    """Checks that XMLschema.rst matches the generated output."""
    schema_file = os.path.join(_REPO_ROOT, 'doc', 'XMLschema.rst')
    source = generate_schema.generate()
    with open(schema_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'XMLschema.rst' needs to be updated.")

  def test_functions(self):
    """Checks that functions.rst matches the generated output."""
    functions_file = os.path.join(
        _REPO_ROOT, 'doc', 'APIreference', 'functions.rst'
    )
    source = generate_functions.generate()
    with open(functions_file, 'r', encoding='utf-8') as file:
      if source != file.read():
        self.fail("The file 'functions.rst' needs to be updated.")

  def test_all_functions_included(self):
    """Checks that every public C function has an entry in functions.rst."""

    functions_file = os.path.join(
        _REPO_ROOT, 'doc', 'APIreference', 'functions.rst'
    )
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

    types_file = os.path.join(
        _REPO_ROOT, 'doc', 'APIreference', 'APItypes.rst'
    )
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

  def test_element_constraints_diamond_inheritance(self):
    con = mjcf_schema.Constraint(
        kind='exclusive', bundles=(('a',), ('b',)), doc=None, line=1)
    common_group = mjcf_schema.Group(
        name='common', variant=False, members=[con], doc=None, line=1)
    group1 = mjcf_schema.Group(
        name='group1', variant=False,
        members=[mjcf_schema.Use(group='common', line=1)], doc=None, line=1)
    group2 = mjcf_schema.Group(
        name='group2', variant=False,
        members=[mjcf_schema.Use(group='common', line=1)], doc=None, line=1)
    elem = mjcf_schema.Element(
        name='elem', spec=None, facets={},
        members=[mjcf_schema.Use(group='group1', line=1),
                 mjcf_schema.Use(group='group2', line=1)],
        doc=None, line=1)
    schema = mjcf_schema.Schema(
        enums={},
        groups={'common': common_group, 'group1': group1, 'group2': group2},
        elements={'elem': elem},
        path='<test>')
    cons = generate_mjcf_table._element_constraints(schema, elem)
    self.assertEqual(len(cons), 1)  # pylint: disable=g-generic-assert


if __name__ == '__main__':
  googletest.main()
