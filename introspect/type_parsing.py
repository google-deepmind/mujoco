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
"""Functions for parsing C type declarations."""

import collections
import re
from typing import Mapping, MutableSequence, Optional, Sequence, Tuple, Union

from . import ast_nodes

ARRAY_EXTENTS_PATTERN = re.compile(r'(\[[^\]]+\]\s*)+\Z')
ARRAY_N_PATTERN = re.compile(r'\[([^\]]+)\]')
STARTS_WITH_CONST_PATTERN = re.compile(r'\Aconst(?![A-Za-z0-9_])')
ENDS_WITH_CONST_PATTERN = re.compile(r'(?<![A-Za-z0-9_])const\Z')


def _parse_qualifiers(
    type_name: str,
    qualifiers: Sequence[str]) -> Tuple[str, Mapping[str, bool]]:
  """Separates qualifiers from the rest of the type name."""
  parts = re.split(r'\s+', type_name)
  counter = collections.defaultdict(lambda: 0)
  non_qualifiers = []
  for part in parts:
    if part in qualifiers:
      counter[part] += 1
      if counter[part] > 1:
        raise ValueError('duplicate qualifier: {part!r}')
    else:
      non_qualifiers.append(part)
  is_qualifier = dict()
  for qualifier in qualifiers:
    is_qualifier[f'is_{qualifier}'] = bool(counter[qualifier])
  return ' '.join(non_qualifiers), is_qualifier


def _parse_maybe_array(
    type_name: str, innermost_type: Optional[Union[ast_nodes.ValueType,
                                                   ast_nodes.PointerType]]
) -> Union[ast_nodes.ValueType, ast_nodes.PointerType, ast_nodes.ArrayType]:
  """Internal-only helper that parses a type that may be an array type."""
  array_match = ARRAY_EXTENTS_PATTERN.search(type_name)
  if array_match:
    extents = tuple(
        int(s.strip()) for s in ARRAY_N_PATTERN.findall(array_match.group(0)))
    inner_type_str = type_name[:array_match.start()]
    return ast_nodes.ArrayType(
        inner_type=_parse_maybe_pointer(inner_type_str.strip(), innermost_type),
        extents=extents)
  else:
    return _parse_maybe_pointer(type_name, innermost_type)


def _parse_maybe_pointer(
    type_name: str, innermost_type: Optional[Union[ast_nodes.ValueType,
                                                   ast_nodes.PointerType]]
) -> Union[ast_nodes.ValueType, ast_nodes.PointerType, ast_nodes.ArrayType]:
  """Internal-only helper that parses a type that may be a pointer type."""
  if type_name == 'void *(*)(void *)':
    return ast_nodes.ValueType(name=type_name)
  p = type_name.rfind('*')
  if p != -1:
    leftover, is_qualifier = _parse_qualifiers(
        type_name[p + 1:].strip(), ('const', 'volatile', 'restrict'))
    if leftover:
      raise ValueError('invalid qualifier for pointer: {leftover!r}')

    inner_type_str = type_name[:p].strip()
    if inner_type_str:
      inner_type = _parse_maybe_pointer(inner_type_str, innermost_type)
    else:
      assert innermost_type is not None
      inner_type = innermost_type
    return ast_nodes.PointerType(inner_type=inner_type, **is_qualifier)
  else:
    assert innermost_type is None  # value type should be innermost
    type_name, is_qualifier = _parse_qualifiers(
        type_name.strip(), ('const', 'volatile'))
    return ast_nodes.ValueType(name=type_name, **is_qualifier)


def _peel_nested_parens(input_str: str) -> MutableSequence[str]:
  """Extracts substrings from a string with nested parentheses.

  The returned sequence starts from the substring enclosed in the innermost
  parentheses and moves subsequently outwards. The contents of the inner
  substrings are removed from the outer ones. For example, given the string
  'lorem ipsum(dolor sit (consectetur adipiscing) amet)sed do eiusmod',
  this function produces the sequence
  ['consectetur adipiscing', 'dolor sit  amet', 'lorem ipsumsed do eiusmod'].

  Args:
    input_str: An input_str string consisting of zero or more nested
      parentheses.

  Returns:
    A sequence of substrings enclosed with in respective parentheses. See the
    description above for the precise detail of the output.
  """
  if input_str == 'void *(*)(void *)':
    return ['void *(*)(void *)']

  start = input_str.find('(')
  end = input_str.rfind(')')

  if start == -1 and end == -1:
    return [input_str]
  else:
    # Assertions to be re-raised into a meaningful error by the caller.
    assert start != -1  # '(' should be present if there is a ')'
    assert end != -1  # ')' should be present if there is a '('
    assert start < end  # '(' should come before ')'
    out = _peel_nested_parens(input_str[start + 1:end])
    out.append(input_str[:start] + input_str[end + 1:])
    return out


def parse_type(
    type_name: str
) -> Union[ast_nodes.ValueType, ast_nodes.PointerType, ast_nodes.ArrayType]:
  """Parses a string that represents a C type into an AST node."""
  try:
    type_str_stack = _peel_nested_parens(type_name.strip())
  except AssertionError as e:
    raise ValueError(f'{type_name!r} contains incorrectly nested '
                     f'parentheses') from e

  result = None
  while type_str_stack:
    try:
      result = _parse_maybe_array(type_str_stack.pop(), result)
    except AssertionError as e:
      raise ValueError(f'invalid type name {type_name!r}') from e

  assert result  # hint for pytype that `result` isn't None
  return result


def parse_function_return_type(
    type_name: str
) -> Union[ast_nodes.ValueType, ast_nodes.PointerType, ast_nodes.ArrayType]:
  return parse_type(type_name[:type_name.find('(')])
