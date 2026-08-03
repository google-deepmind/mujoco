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
"""Generates the MJCF[] grammar table from src/xml/mjcf.schema.

The table (src/xml/generated/mjcf_table.inc) is the element tree consumed by the
mjXSchema validator: rows of {name, cardinality, attributes...} with
{"<"}/{">"} nesting markers. It is checked in and gated by
test/doc/doc_test.py, which regenerates it from the schema and diffs.
"""

import sys

import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import mjcf_schema
_REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
SCHEMA_PATH = os.path.join(_REPO_ROOT, 'src', 'xml', 'mjcf.schema')

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
// doc/generate/generate_mjcf_table.py; test/doc/doc_test.py checks freshness.
//
// The MJCF grammar table consumed by mjXSchema: rows of {name, cardinality,
// attributes...} with {"<"}/{">"} nesting markers. worldbody, frame and
// replicate have no rows: mjXSchema::NameMatch validates them against the
// body row (see the alias= facets in mjcf.schema).

// clang-format off
'''

_WIDTH = 100


def _wrap_row(parts: list[str], indent: int) -> list[str]:
  """One {...}, initializer, wrapped at _WIDTH with hanging indent."""
  lines = []
  line = ' ' * indent + '{' + parts[0]
  for part in parts[1:]:
    candidate = f'{line}, {part}'
    if len(candidate) + 2 > _WIDTH:  # room for "},"
      lines.append(line + ',')
      line = ' ' * (indent + 4) + part
    else:
      line = candidate
  lines.append(line + '},')
  return lines


KIND_CHAR = {'exclusive': 'e', 'together': 't', 'requires': 'r',
             'oneof': 'o'}


def _element_constraints(schema, element):
  """Element's own constraints plus those of transitively used groups."""
  cons = list(element.constraints())
  visited = set()
  stack = [m.group for m in element.members
           if isinstance(m, mjcf_schema.Use)]
  while stack:
    name = stack.pop()
    if name in visited:
      continue
    visited.add(name)
    group = schema.groups[name]
    for member in group.members:
      if isinstance(member, mjcf_schema.Constraint):
        cons.append(member)
      elif isinstance(member, mjcf_schema.Use):
        stack.append(member.group)
  return cons


def generate() -> str:
  """Generates the C++ grammar table header file contents from mjcf.schema."""
  schema = mjcf_schema.parse_file(SCHEMA_PATH)
  out = []
  constraints = []
  count = 0

  def emit_entry(lines: list[str]):
    nonlocal count
    out.extend(lines)
    count += 1

  def visit(element: mjcf_schema.Element, card: str, indent: int,
            project: bool):
    # a row in default context is the element's defaultable projection
    attrs = [a for a in schema.expanded_attrs(element)]
    if project:
      attrs = [a for a in attrs
               if a.name not in ('name', 'class')
               and not a.facets.get('nodefault')]
    parts = [f'"{element.xml_name()}"', f'"{card}"']
    parts += [f'"{a.name}"' for a in attrs]
    row_index = count
    emit_entry(_wrap_row(parts, indent))

    # presence constraints whose attributes all survive in this row
    row_attrs = {a.name for a in attrs}
    for con in _element_constraints(schema, element):
      if all(all(n in row_attrs for n in b) for b in con.bundles):
        spec = '|'.join(' '.join(b) for b in con.bundles)
        constraints.append(f"  {{{row_index}, '{KIND_CHAR[con.kind]}', "
                           f'"{spec}"}},')

    children = [c for c in element.children() if c.name != element.name]
    if project:
      # plugin configuration is not settable per-class
      children = [c for c in children if c.name != 'plugin']
    if not children:
      return
    emit_entry([' ' * indent + '{"<"},'])
    for child in children:
      decl = schema.elements[child.name]
      child_project = project or (
          element.name == 'default' and not child.name.startswith('default_'))
      visit(decl, child.card, indent + 4, child_project)
      if indent == 0:
        out.append('')
    emit_entry([' ' * indent + '{">"},'])

  visit(schema.elements['mujoco'], '!', 0, False)

  body = '\n'.join(out)
  con_body = '\n'.join(constraints)
  return (_HEADER +
          'std::vector<const char*> MJCF[] = {\n' + body + '\n};\n'
          '// clang-format on\n\n'
          'const int nMJCF = sizeof(MJCF) / sizeof(MJCF[0]);\n\n'
          '// presence constraints, indexed into MJCF[]; enforced by\n'
          '// mjXSchema::Check. spec: attribute bundles, space-joined,\n'
          "// '|'-separated; kind: e=exclusive t=together r=requires o=oneof\n"
          '// clang-format off\n'
          'const mjXConstraintDef MJCF_constraints[] = {\n' + con_body +
          '\n};\n'
          '// clang-format on\n\n'
          'const int nMJCF_constraints = '
          'sizeof(MJCF_constraints) / sizeof(MJCF_constraints[0]);\n')


def main() -> int:
  if len(sys.argv) > 2:
    sys.exit('usage: generate_mjcf_table.py [output.inc]')
  text = generate()
  if len(sys.argv) == 2:
    with open(sys.argv[1], 'w', encoding='utf-8') as file:
      file.write(text)
  else:
    sys.stdout.write(text)
  return 0


if __name__ == '__main__':
  sys.exit(main())
