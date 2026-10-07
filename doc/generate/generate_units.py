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
"""Generates the table of attribute dimensions from src/xml/mjcf.schema.

Emits doc/XMLunits.rst: one row per real-valued MJCF attribute with its
physical dimension (the dim facet), linked to its entry in XMLreference.rst.
The U and Y symbols are resolved with the element's control and output facets.
Included by XMLreference.rst and gated by test/doc/doc_test.py.
"""

import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
  import resource_loader  # pyrefly: ignore[missing-import]
  import mjcf_schema  # pyrefly: ignore[missing-import]
except ImportError:
  raise

SCHEMA_PATH = str(resource_loader.resolve_path('src/xml/mjcf.schema'))
REFERENCE_PATH = str(resource_loader.resolve_path('doc/XMLreference.rst'))

# shown instead of a formula for the keyword dimensions
KEYWORD_TEXT = {
    'solref': ':math:`T,\\ 1` or :math:`T^{-2},\\ T^{-1}`',
    'custom': '*varies*',
    'opaque': '*other*',
}


def _anchors() -> set[str]:
  with open(REFERENCE_PATH, 'r', encoding='utf-8') as file:
    return {m.group(1) for m in re.finditer(r'^\.\. _([^:]+):$', file.read(), re.M)}


def _resolve(dim: mjcf_schema.Dim,
             element: mjcf_schema.Element) -> mjcf_schema.Dim:
  """Substitutes U and Y with the element's control and output dimensions."""
  if dim.kind != 'product' or not dim.symbols() & set(element.dims):
    return dim
  components = []
  for comp in dim.components:
    exps = {}
    for sym, exp in comp:
      inner = element.dims.get(sym)
      if inner is None:
        exps[sym] = exps.get(sym, 0) + exp
      elif inner.kind != 'product':
        return inner  # custom or opaque control/output
      else:
        for isym, iexp in inner.components[0]:
          exps[isym] = exps.get(isym, 0) + exp * iexp
    components.append(tuple((s, e) for s, e in exps.items() if e))
  return mjcf_schema.Dim(kind='product', components=tuple(components))


def _math(dim: mjcf_schema.Dim) -> str:
  """RST for a dimension."""
  if dim.kind != 'product':
    return KEYWORD_TEXT[dim.kind]
  parts = []
  for comp in dim.components:
    if not comp:
      parts.append('1')
    else:
      parts.append('\\,'.join(sym if exp == 1 else f'{sym}^{{{exp}}}'
                              for sym, exp in comp))
  return ':math:`' + ',\\ '.join(parts) + '`'


def _walk(schema: mjcf_schema.Schema) -> list[tuple[str, mjcf_schema.Element]]:
  """Elements in document order with their link prefix, each listed once.

  The prefix is the element's XML name under mujoco, and parent-name below
  that, as in the anchors of XMLreference.rst. body and the elements aliased
  to it (worldbody, frame, replicate) are prefixed by their own name, and
  their children by body. An element reached from several parents is listed
  under the first one whose anchors exist. default classes repeat elements
  and are skipped.
  """
  anchors = _anchors()
  out = []
  listed = set()
  expanded = set()

  def is_body(element):
    return element.name == 'body' or element.facets.get('alias') == 'body'

  def visit(element, parent_name, depth):
    if is_body(element) or depth <= 1:
      prefix = element.xml_name()
    else:
      prefix = f'{parent_name}-{element.xml_name()}'
    if element.name not in listed:
      reals = [a for a in schema.expanded_attrs(element)
               if a.type in mjcf_schema.REAL_TYPES]
      if all(f'{prefix}-{a.name}' in anchors for a in reals):
        listed.add(element.name)
        if reals:
          out.append((prefix, element))
    if element.name in expanded:
      return
    expanded.add(element.name)
    name = 'body' if is_body(element) else element.xml_name()
    for child in element.children():
      if child.name != 'default':
        visit(schema.elements[child.name], name, depth + 1)

  visit(schema.elements['mujoco'], '', 0)
  return out


def generate() -> str:
  """Returns the content of doc/XMLunits.rst."""
  schema = mjcf_schema.parse_file(SCHEMA_PATH)
  anchors = _anchors()
  rows = []
  for prefix, element in _walk(schema):
    for attr in schema.expanded_attrs(element):
      if attr.type not in mjcf_schema.REAL_TYPES:
        continue
      anchor = f'{prefix}-{attr.name}'
      if anchor not in anchors:
        raise ValueError(f'anchor {anchor} not found in XMLreference.rst')
      label = anchor.replace('-', '/')
      rows.append((f':ref:`{label}<{anchor}>`', _math(_resolve(attr.dim, element))))

  missing = sorted({a.name for _, a in mjcf_schema.missing_dims(schema)})
  if missing:
    raise ValueError(f'attributes without a dimension: {missing}')

  lines = [
      '..',
      '  DO NOT EDIT. THIS FILE IS AUTOMATICALLY GENERATED by',
      '  doc/generate/generate_units.py from src/xml/mjcf.schema.',
      '',
      '.. list-table::',
      '   :header-rows: 1',
      '   :widths: 40 60',
      '',
      '   * - Attribute',
      '     - Dimension',
  ]
  for ref, dim in rows:
    lines += [f'   * - {ref}', f'     - {dim}']
  return '\n'.join(lines) + '\n'


def main() -> int:
  text = generate()
  if len(sys.argv) > 1:
    with open(sys.argv[1], 'w', encoding='utf-8') as file:
      file.write(text)
  else:
    sys.stdout.write(text)
  return 0


if __name__ == '__main__':
  sys.exit(main())
