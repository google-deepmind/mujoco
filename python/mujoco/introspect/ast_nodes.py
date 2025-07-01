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
"""Classes that roughly correspond to Clang AST node types."""

import collections
import dataclasses
import re
from typing import Dict, Optional, Sequence, Tuple, Union

# We are relying on Clang to do the actual source parsing and are only doing
# a little bit of extra parsing of function parameter type declarations here.
# These patterns are here for sanity checking rather than actual parsing.
VALID_TYPE_NAME_PATTERN = re.compile('(struct )?[A-Za-z_][A-Za-z0-9_]*')
C_INVALID_TYPE_NAMES = frozenset([
    'auto', 'break', 'case', 'const', 'continue', 'default', 'do', 'else',
    'enum', 'extern', 'for', 'goto', 'if', 'inline', 'register', 'restrict',
    'return', 'sizeof', 'static', 'struct', 'switch', 'typedef', 'union',
    'volatile', 'while', '_Alignas', '_Atomic', '_Generic', '_Imaginary',
    '_Noreturn', '_Static_assert', '_Thread_local', '__attribute__', '_Pragma'])


def _is_valid_integral_type(type_str: str):
  """Checks if a string is a valid integral type."""
  parts = re.split(r'\s+', type_str)
  counter = collections.defaultdict(lambda: 0)
  wildcard_counter = 0
  for part in parts:
    if part in ('signed', 'unsigned', 'short', 'long', 'int', 'char'):
      counter[part] += 1
    elif VALID_TYPE_NAME_PATTERN.fullmatch(part):
      # a non-keyword can be a typedef for int
      wildcard_counter += 1
    else:
      return False

  if (counter['signed'] + counter['unsigned'] > 1 or
      counter['short'] > 1 or counter['long'] > 2 or
      (counter['short'] and counter['long']) or
      ((counter['short'] or counter['long']) and counter['char']) or
      counter['char'] + counter['int'] + wildcard_counter > 1):
    return False
  else:
    return True


@dataclasses.dataclass
class ValueType:
  """Represents a C type that is neither a pointer type nor an array type."""

  name: str
  is_const: bool = False
  is_volatile: bool = False

  def __init__(self, name: str, is_const: bool = False,
               is_volatile: bool = False):
    is_valid_type_name = (
        name == 'void *(*)(void *)' or
        VALID_TYPE_NAME_PATTERN.fullmatch(name) or
        _is_valid_integral_type(name)) and name not in C_INVALID_TYPE_NAMES
    if not is_valid_type_name:
      raise ValueError(f'{name!r} is not a valid value type name')
    self.name = name
    self.is_const = is_const
    self.is_volatile = is_volatile

  def decl(self, name_or_decl: Optional[str] = None) -> str:
    parts = []
    if self.is_const:
      parts.append('const')
    if self.is_volatile:
      parts.append('volatile')
    parts.append(self.name)
    if name_or_decl:
      parts.append(name_or_decl)
    return ' '.join(parts)

  def __str__(self):
    return self.decl()


@dataclasses.dataclass
class ArrayType:
  """Represents a C array type."""

  inner_type: Union[ValueType, 'PointerType']
  extents: Tuple[int, ...]

  def __init__(self, inner_type: Union[ValueType, 'PointerType'],
               extents: Sequence[int]):
    self.inner_type = inner_type
    self.extents = tuple(extents)

  @property
  def _extents_str(self) -> str:
    return ''.join(f'[{n}]' for n in self.extents)

  def decl(self, name_or_decl: Optional[str] = None) -> str:
    name_or_decl = name_or_decl or ''
    return self.inner_type.decl(f'{name_or_decl}{self._extents_str}')

  def __str__(self):
    return self.decl()


@dataclasses.dataclass
class PointerType:
  """Represents a C pointer type."""

  inner_type: Union[ValueType, ArrayType, 'PointerType']
  is_const: bool = False
  is_volatile: bool = False
  is_restrict: bool = False

  def decl(self, name_or_decl: Optional[str] = None) -> str:
    """Creates a string that declares an object of this type."""
    parts = ['*']
    if self.is_const:
      parts.append('const')
    if self.is_volatile:
      parts.append('volatile')
    if self.is_restrict:
      parts.append('restrict')
    if name_or_decl:
      parts.append(name_or_decl)
    ptr_decl = ' '.join(parts)
    if isinstance(self.inner_type, ArrayType):
      ptr_decl = f'({ptr_decl})'
    return self.inner_type.decl(ptr_decl)

  def __str__(self):
    return self.decl()


@dataclasses.dataclass
class FunctionParameterDecl:
  """Represents a parameter in a function declaration.

  Note that according to the C language rule, a function parameter of array
  type undergoes array-to-pointer decay, and therefore appears as a pointer
  parameter in an actual C AST. We retain the arrayness of a parameter here
  since the array's extents are informative.
  """

  name: str
  type: Union[ValueType, ArrayType, PointerType]

  def __str__(self):
    return self.type.decl(self.name)

  @property
  def decltype(self) -> str:
    return self.type.decl()


@dataclasses.dataclass
class FunctionDecl:
  """Represents a function declaration."""

  name: str
  return_type: Union[ValueType, ArrayType, PointerType]
  parameters: Tuple[FunctionParameterDecl, ...]
  doc: str

  def __init__(self, name: str,
               return_type: Union[ValueType, ArrayType, PointerType],
               parameters: Sequence[FunctionParameterDecl],
               doc: str):
    self.name = name
    self.return_type = return_type
    self.parameters = tuple(parameters)
    self.doc = doc

  def __str__(self):
    param_str = ', '.join(str(p) for p in self.parameters)
    return f'{self.return_type} {self.name}({param_str})'

  @property
  def decltype(self) -> str:
    param_str = ', '.join(str(p.decltype) for p in self.parameters)
    return f'{self.return_type} ({param_str})'


class _EnumDeclValues(Dict[str, int]):
  """A dict with modified stringified representation.

  The __repr__ method of this class adds a trailing comma to the list of values.
  This is done as a hint for code formatters to place one item per line when
  the stringified OrderedDict is used in generated Python code.
  """

  def __repr__(self):
    out = super().__repr__()
    if self:
      out = re.sub(r'\(\[(.+)\]\)\Z', r'([\1,])', out)
    return re.sub(r'\A_EnumDeclValues', 'dict', out)


@dataclasses.dataclass
class EnumDecl:
  """Represents an enum declaration."""

  name: str
  declname: str
  values: Dict[str, int]

  def __init__(self, name: str, declname: str, values: Dict[str, int]):
    self.name = name
    self.declname = declname
    self.values = _EnumDeclValues(values)


@dataclasses.dataclass
class StructFieldDecl:
  """Represents a field in a struct or union declaration."""

  name: str
  type: Union[
      ValueType,
      ArrayType,
      PointerType,
      'AnonymousStructDecl',
      'AnonymousUnionDecl',
  ]
  doc: str
  array_extent: Optional[Tuple[Union[str, int], ...]] = None

  def __str__(self):
    return self.type.decl(self.name)

  @property
  def decltype(self) -> str:
    return self.type.decl()


@dataclasses.dataclass
class AnonymousStructDecl:
  """Represents an anonymous struct declaration."""

  fields: Tuple[Union[StructFieldDecl, 'AnonymousUnionDecl'], ...]

  def __init__(self, fields: Sequence[StructFieldDecl]):
    self.fields = tuple(fields)

  def __str__(self):
    return self.decl()

  def _inner_decl(self):
    return '; '.join(str(field) for field in self.fields) + ';'

  def decl(self, name_or_decl: Optional[str] = None):
    parts = ['struct', f'{{{self._inner_decl()}}}']
    if name_or_decl:
      parts.append(name_or_decl)
    return ' '.join(parts)


class AnonymousUnionDecl(AnonymousStructDecl):
  """Represents an anonymous union declaration."""

  def decl(self, name_or_decl: Optional[str] = None):
    parts = ['union', f'{{{self._inner_decl()}}}']
    if name_or_decl:
      parts.append(name_or_decl)
    return ' '.join(parts)


@dataclasses.dataclass
class StructDecl:
  """Represents a struct declaration."""

  name: str
  declname: str
  fields: Tuple[Union[StructFieldDecl, AnonymousUnionDecl], ...]

  def __init__(self, name: str,
               declname: str,
               fields: Sequence[Union[StructFieldDecl, AnonymousUnionDecl]]):
    self.name = name
    self.declname = declname
    self.fields = tuple(fields)

  def decl(self, name_or_decl: Optional[str] = None) -> str:
    parts = [self.name]
    if name_or_decl:
      parts.append(name_or_decl)
    return ' '.join(parts)
