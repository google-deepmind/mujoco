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
"""Generates the default-value check table from src/xml/mjcf.schema.

The schema declares attribute defaults, but the defaults that act live in
the C default-constructors (mjs_default*, mj_defaultOption, ...). This
emits src/xml/mjcf_default_table.inc: one row per defaulted attribute,
binding the declared values to the field they describe, consumed by
SchemaDefaultsTest, which compares every row against a freshly-constructed
spec -- so a schema default that disagrees with the C defaults is a test
failure, not documentation drift. Checked in and gated by
test/doc/doc_test.py.
"""

import os
import sys

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import generate_read_table
import mjcf_schema
_REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
SCHEMA_PATH = os.path.join(_REPO_ROOT, 'src', 'xml', 'mjcf.schema')

# kind codes shared with the test
KIND_BY_CTYPE = {'double': 0, 'float': 1, 'int': 2,
                 'mjtByte': 3, 'mjtBool': 3, 'mjtNum': 4}

_HEADER = '''\
// Copyright 2026 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// GENERATED FILE, DO NOT EDIT. Generated from src/xml/mjcf.schema by
// doc/generate/generate_default_table.py; test/doc/doc_test.py checks
// freshness.
//
// One row per schema attribute with a declared default, binding the declared
// values to the bound field. SchemaDefaultsTest compares every row against a
// freshly-constructed spec: the schema's defaults must agree with the C
// default-constructors. Rows are {attr, offset, kind, len, ndecl, values};
// values beyond ndecl are expected to be zero; kind: 0=double 1=float 2=int
// 3=byte 4=mjtNum.

// clang-format off
struct mjXDefaultEntry {
  const char* attr;
  int offset;
  int kind;
  int len;
  int ndecl;
  double value[8];
};

struct mjXDefaultTable {
  const char* structname;
  const mjXDefaultEntry* entries;
  int n;
};

'''


def _values(schema, attr, ctype):
  """(ndecl, [C value expressions]) for an attribute's declared default."""
  default = attr.default
  if attr.type == 'enum':
    constants = dict(schema.enums[attr.target].items)
    return 1, [f'(double){constants[default]}']
  if attr.type == 'bool':
    return 1, ['1' if default == 'true' else '0']
  values = default if isinstance(default, tuple) else (default,)
  return len(values), [repr(v) for v in values]


def collect(schema, structs):
  """struct key -> list of row tuples, deduplicated across elements."""
  tables = {}
  seen = {}
  for element in schema.elements.values():
    if not element.spec:
      continue
    sub = element.facets.get('field')
    key = f'{element.spec}.{sub}' if sub else element.spec
    fields = structs.get(key)
    if fields is None:
      continue
    prefix = f'{sub}.' if sub else ''
    for attr in schema.expanded_attrs(element):
      if attr.default is None or attr.type in ('string', 'file', 'chars',
                                               'ref', 'id', 'flags'):
        continue
      field = attr.facets.get('field', attr.name)
      entry = fields.get(field)
      if entry is None:
        if 'reading' in attr.facets:
          continue  # custom lowering with no direct binding
        raise ValueError(f'{element.name}.{attr.name}: no field '
                         f'{element.spec}.{field}')
      ctype, dim = entry
      if ctype not in KIND_BY_CTYPE and not ctype.startswith('mjt'):
        raise ValueError(f'{element.name}.{attr.name}: default bound to '
                         f'field {field} ({ctype})')
      kind = KIND_BY_CTYPE.get(ctype, 2)  # other mjt enums are int-sized
      length = dim if dim is not None else '1'
      ndecl, values = _values(schema, attr, ctype)
      if ndecl > 8:
        raise ValueError(f'{element.name}.{attr.name}: {ndecl} default '
                         'values exceed the row capacity')
      row = (attr.name, f'(int)offsetof({element.spec}, {prefix}{field})',
             kind, str(length), ndecl, values)
      prior = seen.get((key, field))
      if prior is not None:
        if prior != (ndecl, values):
          raise ValueError(f'{element.name}.{attr.name}: conflicting '
                           f'defaults for {key}.{field}')
        continue
      seen[(key, field)] = (ndecl, values)
      tables.setdefault(key, []).append(row)
  return tables


def generate() -> str:
  schema = mjcf_schema.parse_file(SCHEMA_PATH)
  structs = generate_read_table.parse_spec_structs(
      generate_read_table.SPEC_H_PATH, generate_read_table.MODEL_H_PATH)
  tables = collect(schema, structs)
  out = [_HEADER]
  for key in sorted(tables):
    array = 'kDefaults_' + key.replace('.', '_')
    out.append(f'static const mjXDefaultEntry {array}[] = {{')
    for attr, offset, kind, length, ndecl, values in tables[key]:
      vals = ', '.join(values)
      out.append(f'  {{"{attr}", {offset}, {kind}, {length}, {ndecl}, '
                 f'{{{vals}}}}},')
    out.append('};')
    out.append('')
  out.append('static const mjXDefaultTable kDefaultTables[] = {')
  for key in sorted(tables):
    array = 'kDefaults_' + key.replace('.', '_')
    root = key.split('.')[0]
    out.append(f'  {{"{root}", {array}, '
               f'(int)(sizeof({array}) / sizeof({array}[0]))}},')
  out.append('};')
  out.append('static const int kDefaultTablesN = '
             '(int)(sizeof(kDefaultTables) / sizeof(kDefaultTables[0]));')
  out.append('// clang-format on')
  return '\n'.join(out) + '\n'


def main() -> int:
  if len(sys.argv) > 2:
    sys.exit('usage: generate_default_table.py [output.inc]')
  text = generate()
  if len(sys.argv) == 2:
    with open(sys.argv[1], 'w', encoding='utf-8') as file:
      file.write(text)
  else:
    sys.stdout.write(text)
  return 0


if __name__ == '__main__':
  sys.exit(main())
