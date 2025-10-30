# Copyright 2023 DeepMind Technologies Limited
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
"""Generates structs.py.

The JSON input can be generated via:
  clang -Xclang -ast-dump=json -fsyntax-only -fparse-all-comments -x c mujoco.h
"""

import itertools
import json
import os
import re
from typing import Any, Mapping, Sequence, Union

from absl import app
from absl import flags

from introspect import ast_nodes
from introspect import type_parsing
from . import formatter

_JSON_PATH = flags.DEFINE_string(
    'json_path', None,
    'Path to the JSON file representing the Clang AST for mujoco.h')

ClangJsonNode = Mapping[str, Any]

_ANONYMOUS_KEY_PATTERN = re.compile(r'\d+:\d+(?=\))')
_EXCLUDED = (
    'mjpDecoder',
    'mjpDecoder_',
    'mjpPlugin',
    'mjpPlugin_',
    'mjpResourceProvider',
    'mjpResourceProvider_',
    'mjResource',
    'mjResource_',
)

_ARRAY_COMMENT_PATTERN = re.compile(r'(.+?)\s+\(([^\(\)]+) x ([^\(\)]+)\)\Z')


def traverse(node, visitor):
  visitor.visit(node)
  children = node.get('inner', [])
  for child in children:
    traverse(child, visitor)


class _AnonymousTypePlaceholder(ast_nodes.ValueType):

  def __init__(self, anonymous_key: str):
    self.name = anonymous_key
    self.is_const = False
    self.is_volatile = False


class MjStructVisitor:
  """A Clang AST JSON node visitor for MuJoCo API struct declarations."""

  def __init__(self):
    self._structs = {}
    self._anonymous = {}
    self._typedefs = {}

  def _normalize_type(
      self, declname: str
  ) -> Union[
      ast_nodes.ValueType, ast_nodes.PointerType, ast_nodes.ArrayType]:
    """Resolves anonymous structs/unions and looks up existing typedefs."""
    # Check for anonymous struct/union.
    if '(unnamed ' in declname:
      m = _ANONYMOUS_KEY_PATTERN.search(declname)
      if not m:
        raise RuntimeError('cannot parse anonymous key from {m!r}')
      return _AnonymousTypePlaceholder(m.group(0))

    # Lookup typedef name and use it instead if one exists.
    for k, v in self._typedefs.items():
      if declname == v.declname:
        return type_parsing.parse_type(k)

    # No valid normalization, just parse the declname.
    return type_parsing.parse_type(declname)

  def _make_comment(self, node: ClangJsonNode, strip: bool = True) -> str:
    """Makes a comment string from a Clang AST FullComment node."""
    kind = node.get('kind')
    if kind == 'TextComment':
      retval = node['text'].replace('\N{NO-BREAK SPACE}', '&nbsp;')
    else:
      strings = []
      for child in node['inner']:
        strings.append(self._make_comment(child, strip=False))
      retval = ''.join(strings)
    if strip:
      retval = retval.strip()
    return retval

  def _make_field(
      self, node: ClangJsonNode
  ) -> Union[ast_nodes.StructFieldDecl, _AnonymousTypePlaceholder]:
    """Makes a StructFieldDecl object from a Clang AST FieldDecl node."""
    doc = ''
    for child in node.get('inner', ()):
      if child['kind'] == 'FullComment':
        doc = self._make_comment(child)
    if 'name' in node:
      field_type = self._normalize_type(node['type']['qualType'])
      m = _ARRAY_COMMENT_PATTERN.match(doc)
      if m is None:
        array_extent = None
      else:
        doc = m.group(1)
        array_extent_0 = m.group(2)
        array_extent_1 = m.group(3)
        try:
          array_extent_1 = int(array_extent_1)
        except ValueError:
          pass
        if array_extent_1 == 1:
          array_extent = (array_extent_0,)
        else:
          array_extent = (array_extent_0, array_extent_1)
      return ast_nodes.StructFieldDecl(
          name=node['name'], type=field_type, doc=doc,
          array_extent=array_extent)
    else:
      return _AnonymousTypePlaceholder(self._make_anonymous_key(node))

  def _make_struct(
      self, node: ClangJsonNode
  ) -> Union[ast_nodes.AnonymousStructDecl, ast_nodes.StructDecl]:
    """Makes a Decl object from a Clang AST RecordDecl node."""
    name = f"{node['tagUsed']} {node['name']}" if 'name' in node else ''
    fields = []
    for child in node.get('inner', ()):
      child_kind = child.get('kind')
      if child_kind == 'FieldDecl':
        fields.append(self._make_field(child))

    if name:
      return ast_nodes.StructDecl(name=name, declname=name, fields=fields)
    elif node['tagUsed'] == 'union':
      return ast_nodes.AnonymousUnionDecl(fields=fields)
    else:
      return ast_nodes.AnonymousStructDecl(fields=fields)

  def _is_mujoco_type(self, node: ClangJsonNode) -> bool:
    node_name = node.get('name', '')
    included_from = os.path.basename(
        node['loc'].get('includedFrom', {}).get('file', '')
    )
    return node_name not in _EXCLUDED and (
        node_name.startswith('mj')
        or included_from == 'mujoco.h'
        or included_from.startswith('mj')
    )

  def _make_anonymous_key(self, node: ClangJsonNode) -> str:
    line = node['loc']['line']
    col = node['loc']['col']
    return f'{line}:{col}'

  def visit(self, node: ClangJsonNode) -> None:
    """Visits a JSON node."""
    if node.get('kind') == 'RecordDecl' and self._is_mujoco_type(node):
      struct_decl = self._make_struct(node)
      if hasattr(struct_decl, 'name'):
        self._structs[struct_decl.name] = struct_decl
      else:
        anonymous_key = self._make_anonymous_key(node)
        if anonymous_key in self._anonymous:
          raise RuntimeError(
              f'duplicate key for anonymous struct: {anonymous_key}')
        self._anonymous[anonymous_key] = struct_decl
    elif (node.get('kind') == 'TypedefDecl' and
          node['type']['qualType'].startswith('struct mj') and
          node['name'] not in _EXCLUDED):
      declname = node['type']['qualType']
      try:
        struct = self._structs[declname]
      except KeyError:
        self._typedefs[node['name']] = ast_nodes.StructDecl(
            name=node['name'], declname=declname, fields=())
      else:
        self._typedefs[node['name']] = ast_nodes.StructDecl(
            name=node['name'], declname=struct.declname, fields=struct.fields)

  def resolve_all_anonymous(self) -> None:
    """Replaces anonymous struct placeholders with corresponding decl."""
    for struct in itertools.chain(
        self._structs.values(), self._typedefs.values()
    ):
      fields = []
      for field in struct.fields:
        if isinstance(field, _AnonymousTypePlaceholder):
          fields.append(self._anonymous[field.name])
        elif isinstance(field.type, _AnonymousTypePlaceholder):
          fields.append(
              ast_nodes.StructFieldDecl(
                  name=field.name,
                  type=self._anonymous[field.type.name],
                  doc=field.doc,
              )
          )
        else:
          fields.append(field)
      struct.fields = tuple(fields)

  @property
  def structs(self) -> Mapping[str, ast_nodes.StructDecl]:
    return self._structs

  @property
  def typedefs(self) -> Mapping[str, ast_nodes.StructDecl]:
    return self._typedefs


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  with open(_JSON_PATH.value, 'r', encoding='utf-8') as f:
    root = json.load(f)

  visitor = MjStructVisitor()

  traverse(root, visitor)
  visitor.resolve_all_anonymous()

  structs_str = formatter.format_as_python_code(visitor.typedefs)

  print(f'''
# Copyright 2023 DeepMind Technologies Limited
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
"""Provides information about MuJoCo API structs.

DO NOT EDIT. THIS FILE IS AUTOMATICALLY GENERATED.
"""

from typing import Mapping

from .ast_nodes import AnonymousStructDecl
from .ast_nodes import AnonymousUnionDecl
from .ast_nodes import ArrayType
from .ast_nodes import PointerType
from .ast_nodes import StructDecl
from .ast_nodes import StructFieldDecl
from .ast_nodes import ValueType

STRUCTS: Mapping[str, StructDecl] = {structs_str}
'''.strip())  # `print` adds a trailing newline


if __name__ == '__main__':
  app.run(main)
