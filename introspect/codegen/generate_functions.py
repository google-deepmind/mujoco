# Copyright 2022 DeepMind Technologies Limited
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
"""Generates functions.py.

The JSON input can be generated via:
  clang -Xclang -ast-dump=json -fsyntax-only -fparse-all-comments -x c mujoco.h
"""

import json
from typing import Any, Mapping, Sequence

from absl import app
from absl import flags

from introspect import ast_nodes
from introspect import type_parsing
from . import formatter

_HEADER_PATH = flags.DEFINE_string(
    'header_path', None, 'Path to the original mujoco.h')
_JSON_PATH = flags.DEFINE_string(
    'json_path', None,
    'Path to the JSON file representing the Clang AST for mujoco.h')

ClangJsonNode = Mapping[str, Any]


def traverse(node, visitor):
  visitor.visit(node)
  children = node.get('inner', [])
  for child in children:
    traverse(child, visitor)


class MjFunctionVisitor:
  """A Clang AST JSON node visitor for MuJoCo API function declarations."""

  def __init__(self, raw_header):
    self._raw_header = raw_header
    self._functions = {}

  def _make_function(self, node: ClangJsonNode) -> ast_nodes.FunctionDecl:
    """Makes a FunctionDecl from a Clang AST FunctionDecl node."""
    name = node['name']
    return_type = type_parsing.parse_function_return_type(
        node['type']['qualType'])
    parameters = []
    comments = []
    for child in node['inner']:
      child_kind = child.get('kind')
      if child_kind == 'ParmVarDecl':
        parameters.append(self._make_parameter(child))
      if child_kind == 'FullComment':
        comments.append(self._make_comment(child))
    comment = ' '.join(comments).strip()
    return ast_nodes.FunctionDecl(
        name=name, return_type=return_type, parameters=parameters, doc=comment)

  def _make_parameter(
      self, node: ClangJsonNode) -> ast_nodes.FunctionParameterDecl:
    """Makes a ParameterDecl from a Clang AST ParmVarDecl node."""
    name = node['name']
    type_name = node['type']['qualType']

    # For a pointer parameters, look up in the original header to see if
    # n array extent was declared there.
    if type_name.endswith('*'):
      decl_begin = node['range']['begin']['offset']
      decl_end = node['range']['end']['offset'] + node['range']['end']['tokLen']
      decl = self._raw_header[decl_begin:decl_end]
      name_begin = node['loc']['offset'] - decl_begin
      name_end = name_begin + node['loc']['tokLen']
      type_name = decl[:name_begin] + decl[name_end:]

    return ast_nodes.FunctionParameterDecl(
        name=name, type=type_parsing.parse_type(type_name))

  def _make_comment(self, node: ClangJsonNode) -> str:
    """Makes a comment string from a Clang AST FullComment node."""
    kind = node.get('kind')
    if kind == 'TextComment':
      return node['text'].replace('\N{NO-BREAK SPACE}', '&nbsp;')
    else:
      strings = []
      for child in node['inner']:
        strings.append(self._make_comment(child))
      return ''.join(strings)

  def visit(self, node: ClangJsonNode) -> None:
    if (node.get('kind') == 'FunctionDecl' and
        node.get('name', '').startswith('mj')):
      func_decl = self._make_function(node)
      self._functions[func_decl.name] = func_decl

  @property
  def functions(self) -> Mapping[str, ast_nodes.FunctionDecl]:
    return self._functions


def main(argv: Sequence[str]) -> None:
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')

  with open(_JSON_PATH.value, 'r', encoding='utf-8') as f:
    root = json.load(f)

  with open(_HEADER_PATH.value, 'r') as f:
    visitor = MjFunctionVisitor(f.read())

  traverse(root, visitor)

  functions_str = formatter.format_as_python_code(visitor.functions)

  print(f'''
# Copyright 2022 DeepMind Technologies Limited
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
"""Provides information about MuJoCo API functions.

DO NOT EDIT. THIS FILE IS AUTOMATICALLY GENERATED.
"""

from typing import Mapping

from .ast_nodes import ArrayType
from .ast_nodes import FunctionDecl
from .ast_nodes import FunctionParameterDecl
from .ast_nodes import PointerType
from .ast_nodes import ValueType

FUNCTIONS: Mapping[str, FunctionDecl] = {functions_str}
'''.strip())  # `print` adds a trailing newline


if __name__ == '__main__':
  app.run(main)
