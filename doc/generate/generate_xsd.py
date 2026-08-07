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
"""Generates the published XML Schema (mjcf.xsd) from mjcf.schema.

The XSD is deliberately permissive: it must never reject a legal model.
XSD 1.0 cannot express MJCF's order-insensitive children with per-child
cardinality (xs:all forbids maxOccurs > 1), so every content model is an
unbounded xs:choice, and presence constraints (exclusive/together/requires/
oneof) are not expressible at all. Both are carried as xs:documentation
annotations.

Element tags repeat across contexts with different content (e.g. joint under
body, default and equality), so every element is declared locally inside its
parent's complexType; the named complexTypes carry the schema element names.
Inside <default>, children use projected types (attributes minus name/class
minus nodefault), mirroring the grammar-table projection.

The generated file is checked in as src/xml/generated/mjcf.xsd and gated by
test/doc/doc_test.py, which regenerates it from the schema and diffs.
"""

import os
import re
import sys
from xml.sax.saxutils import escape

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _SCRIPT_DIR)
import mjcf_schema
_REPO_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
SCHEMA_PATH = os.path.join(_REPO_ROOT, 'src', 'xml', 'mjcf.schema')
MJMODEL_H_PATH = os.path.join(_REPO_ROOT, 'include', 'mujoco', 'mjmodel.h')

# schema scalar type -> XSD base type
SCALAR_XSD = {'int': 'xs:int', 'double': 'xs:double', 'float': 'xs:float',
              'string': 'xs:string', 'file': 'xs:string'}

_CONSTRAINT_TEXT = {
    'exclusive': 'at most one of',
    'together': 'together or absent',
    'requires': 'first requires second',
    'oneof': 'at least one of',
}


def parse_dims():
  """Return {name: value} for the mjN* dimension macros in mjmodel.h."""
  with open(MJMODEL_H_PATH, encoding='utf-8') as f:
    text = f.read()
  return {m.group(1): int(m.group(2))
          for m in re.finditer(r'^#define\s+(mjN\w+)\s+(\d+)', text, re.M)}


def _num(value):
  """Format a numeric default the shortest exact way: 2.0 -> '2'."""
  if isinstance(value, float) and value == int(value) and abs(value) < 1e15:
    return str(int(value))
  return repr(value)


def _default_str(attr):
  """Return the attribute default as an XSD default string, or None."""
  d = attr.default
  if d is None:
    return None
  if isinstance(d, tuple):
    return ' '.join(_num(v) for v in d)
  if isinstance(d, (int, float)):
    return _num(d)
  return str(d)


class _Emitter:
  """Accumulates the XSD document with shared simple types deduped."""

  def __init__(self, schema):
    self.schema = schema
    self.dims = parse_dims()
    self.lines = []
    self.vector_types = {}   # type name -> definition lines
    self.emitted = set()     # (element name, projected) complexTypes emitted
    self.pending = []

  def out(self, indent, text):
    """Appends a line of text at the specified indentation level."""
    self.lines.append(' ' * indent + text)

  def doc(self, indent, texts):
    """Emit an annotation block for the given documentation lines."""
    texts = [t for t in texts if t]
    if not texts:
      return
    self.out(indent, '<xs:annotation>')
    for t in texts:
      self.out(indent + 2, f'<xs:documentation>{escape(t)}</xs:documentation>')
    self.out(indent, '</xs:annotation>')

  def resolve(self, bound):
    """Resolve an arity bound: int, mjN* macro name, or None."""
    if isinstance(bound, str):
      return self.dims[bound]
    return bound

  def vector_type(self, base, lo, hi):
    """Return (and register) the named list type for a vector attribute."""
    if hi is None:
      name = f'{base}list'
    elif lo == hi:
      name = f'{base}{hi}'
    else:
      name = f'{base}{lo}to{hi}'
    if name not in self.vector_types:
      lines = [f'<xs:simpleType name="{name}">']
      if hi is None and lo <= 1:
        lines += [f'  <xs:list itemType="{SCALAR_XSD[base]}"/>']
      else:
        lines += ['  <xs:restriction>',
                  '    <xs:simpleType>',
                  f'      <xs:list itemType="{SCALAR_XSD[base]}"/>',
                  '    </xs:simpleType>']
        if lo == hi:
          lines += [f'    <xs:length value="{hi}"/>']
        else:
          if lo > 0:
            lines += [f'    <xs:minLength value="{lo}"/>']
          if hi is not None:
            lines += [f'    <xs:maxLength value="{hi}"/>']
        lines += ['  </xs:restriction>']
      lines += ['</xs:simpleType>']
      self.vector_types[name] = lines
    return name

  def attr_type(self, attr, ctx):
    """Return (type name or None, inline restriction lines) for an attribute.

    A None type name with inline lines means the attribute carries an
    anonymous simpleType restriction.
    """
    numeric_facets = [f for f in ('min', 'max', 'positive')
                      if f in attr.facets or attr.facets.get(f)]
    if attr.type == 'bool':
      return 'kw_bool', []
    if attr.type == 'enum':
      return f'kw_{attr.target}', []
    if attr.type == 'flags':
      return f'kwlist_{attr.target}', []
    if attr.type in ('string', 'file', 'ref', 'id'):
      return 'xs:string', []
    if attr.type == 'chars':
      lo, hi = attr.arity.lo, attr.arity.hi
      lines = ['<xs:simpleType>', '  <xs:restriction base="xs:string">']
      if 'pattern' in attr.facets:
        lines += [f'    <xs:pattern value="{escape(attr.facets["pattern"])}"/>']
      elif lo == hi:
        lines += [f'    <xs:length value="{hi}"/>']
      else:
        lines += [f'    <xs:minLength value="{lo}"/>',
                  f'    <xs:maxLength value="{hi}"/>']
      lines += ['  </xs:restriction>', '</xs:simpleType>']
      return None, lines
    # numeric scalars and vectors
    base = SCALAR_XSD[attr.type]
    lo, hi = attr.arity.lo, self.resolve(attr.arity.hi)
    if (lo, hi) == (1, 1):
      if not numeric_facets:
        return base, []
      lines = ['<xs:simpleType>', f'  <xs:restriction base="{base}">']
      if 'min' in attr.facets:
        lines += [f'    <xs:minInclusive value="{_num(attr.facets["min"])}"/>']
      if 'max' in attr.facets:
        lines += [f'    <xs:maxInclusive value="{_num(attr.facets["max"])}"/>']
      if attr.facets.get('positive'):
        lines += ['    <xs:minExclusive value="0"/>']
      lines += ['  </xs:restriction>', '</xs:simpleType>']
      return None, lines
    if numeric_facets:
      raise ValueError(f'{ctx}.{attr.name}: numeric facets on a vector '
                       'attribute are not supported by the XSD emitter')
    return self.vector_type(attr.type, lo, hi), []

  def emit_attr(self, indent, attr, ctx):
    """Emit one xs:attribute."""
    tname, inline = self.attr_type(attr, ctx)
    parts = [f'name="{attr.name}"']
    if tname:
      parts.append(f'type="{tname}"')
    if attr.facets.get('required'):
      parts.append('use="required"')
    default = _default_str(attr)
    if default is not None:
      parts.append(f'default="{escape(default)}"')
    head = f'<xs:attribute {" ".join(parts)}'
    if not inline and not attr.doc:
      self.out(indent, head + '/>')
      return
    self.out(indent, head + '>')
    self.doc(indent + 2, [attr.doc])
    for line in inline:
      self.out(indent + 2, line)
    self.out(indent, '</xs:attribute>')

  def type_name(self, element_name, projected):
    return f'default_{element_name}' if projected else element_name

  def constraint_docs(self, element):
    """Presence constraints as documentation lines (not XSD-expressible)."""
    import generate_mjcf_table
    docs = []
    for con in generate_mjcf_table._element_constraints(self.schema, element):
      bundles = ['+'.join(b) for b in con.bundles]
      docs.append(f'constraint: {_CONSTRAINT_TEXT[con.kind]}: '
                  f'{", ".join(bundles)}')
    return docs

  def emit_complex_type(self, name, projected):
    """Emit the complexType for one schema element (possibly projected)."""
    element = self.schema.elements[name]
    tname = self.type_name(name, projected)
    self.out(2, f'<xs:complexType name="{tname}">')
    docs = [element.doc] if not projected else []
    docs += self.constraint_docs(element)
    cards = [f'{c.name} ({c.card})' for c in element.children()]
    if cards:
      docs.append('children, with cardinality the XSD cannot enforce '
                  '(? at most one, ! exactly one, * any number, R recursive): '
                  + ', '.join(cards))
    self.doc(4, docs)

    children = list(element.children())
    if projected:
      children = [c for c in children if c.name != 'plugin']
    if children:
      self.out(4, '<xs:choice minOccurs="0" maxOccurs="unbounded">')
      for child in children:
        target = self.schema.elements[child.name]
        tag = target.xml_name()
        if name == 'mujoco' and child.name == 'body':
          # the top-level body is spelled worldbody (mjXSchema::NameMatch)
          target = self.schema.elements['worldbody']
          tag = 'worldbody'
        child_projected = (projected or
                           (name == 'default' and
                            not child.name.startswith('default_') and
                            child.name != 'default'))
        ctype = self.type_name(target.name, child_projected)
        self.out(6, f'<xs:element name="{tag}" type="{ctype}"/>')
        self.queue(target.name, child_projected)
      # the include directive is spliced before parsing and may appear
      # anywhere; admit it in every content model
      self.out(6, '<xs:element name="include" type="include"/>')
      self.out(4, '</xs:choice>')

    attrs = self.schema.expanded_attrs(element)
    if projected:
      attrs = [a for a in attrs
               if a.name not in ('name', 'class')
               and not a.facets.get('nodefault')]
    for attr in attrs:
      self.emit_attr(4, attr, tname)
    self.out(2, '</xs:complexType>')
    self.out(0, '')

  def queue(self, name, projected):
    """Enqueues an element name and projection flag for processing."""
    self.pending.append((name, projected))

  def generate(self):
    """Return the complete XSD document as a string."""
    schema = self.schema
    self.out(0, '<?xml version="1.0" encoding="UTF-8"?>')
    self.out(0, '<!-- Generated by generate_xsd.py from mjcf.schema.')
    self.out(0, '     Do not edit by hand; see test/doc/doc_test.py.')
    self.out(0, '')
    self.out(0, '     This schema is deliberately permissive: it never')
    self.out(0, '     rejects a legal model, but accepts some models the')
    self.out(0, '     parser rejects. Child cardinality and attribute')
    self.out(0, '     presence constraints are carried as documentation')
    self.out(0, '     annotations. -->')
    self.out(0, '<xs:schema xmlns:xs="http://www.w3.org/2001/XMLSchema">')
    self.out(0, '')

    # keyword simple types: bool, then one per schema enum
    self.out(2, '<xs:simpleType name="kw_bool">')
    self.out(4, '<xs:restriction base="xs:string">')
    for kw in ('false', 'true'):
      self.out(6, f'<xs:enumeration value="{kw}"/>')
    self.out(4, '</xs:restriction>')
    self.out(2, '</xs:simpleType>')
    self.out(0, '')
    flags_targets = set()
    for element in schema.elements.values():
      for attr in schema.expanded_attrs(element):
        if attr.type == 'flags':
          flags_targets.add(attr.target)
    for enum in schema.enums.values():
      self.out(2, f'<xs:simpleType name="kw_{enum.name}">')
      self.doc(4, [enum.doc])
      self.out(4, '<xs:restriction base="xs:string">')
      for kw, _ in enum.items:
        self.out(6, f'<xs:enumeration value="{escape(kw)}"/>')
      self.out(4, '</xs:restriction>')
      self.out(2, '</xs:simpleType>')
      if enum.name in flags_targets:
        self.out(2, f'<xs:simpleType name="kwlist_{enum.name}">')
        self.out(4, f'<xs:list itemType="kw_{enum.name}"/>')
        self.out(2, '</xs:simpleType>')
      self.out(0, '')

    # complexTypes, walked from the root so projections are discovered;
    # vector simple types are collected on the way and emitted after
    body_mark = len(self.lines)
    self.pending = [('mujoco', False)]
    while self.pending:
      name, projected = self.pending.pop(0)
      if (name, projected) in self.emitted:
        continue
      self.emitted.add((name, projected))
      self.emit_complex_type(name, projected)

    unreached = set(schema.elements) - {n for n, _ in self.emitted}
    if unreached:
      raise ValueError(f'elements unreachable from mujoco: {unreached}')

    # the include directive: spliced before parsing, not part of mjcf.schema
    self.out(2, '<xs:complexType name="include">')
    self.doc(4, ['includes another MJCF file; resolved before parsing'])
    self.out(4, '<xs:attribute name="file" type="xs:string" use="required"/>')
    self.out(2, '</xs:complexType>')
    self.out(0, '')

    vec_lines = []
    for tname in sorted(self.vector_types):
      for line in self.vector_types[tname]:
        vec_lines.append('  ' + line)
      vec_lines.append('')
    self.lines[body_mark:body_mark] = vec_lines

    self.out(2, '<xs:element name="mujoco" type="mujoco"/>')
    self.out(0, '</xs:schema>')
    return '\n'.join(self.lines) + '\n'


def generate():
  """Generate the mjcf.xsd content as a string."""
  schema = mjcf_schema.parse_file(SCHEMA_PATH)
  return _Emitter(schema).generate()


def main():
  """CLI entry point: generate mjcf.xsd to stdout or a file."""
  if len(sys.argv) > 2:
    sys.exit('usage: generate_xsd.py [output.xsd]')
  text = generate()
  if len(sys.argv) == 2:
    with open(sys.argv[1], 'w', encoding='utf-8') as f:
      f.write(text)
  else:
    sys.stdout.write(text)
  return 0


if __name__ == '__main__':
  sys.exit(main())
