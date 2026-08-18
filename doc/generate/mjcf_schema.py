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
"""Parser for the MJCF schema definition language.

The MJCF grammar is defined in a single hand-edited file (src/xml/mjcf.schema)
written in a small declarative language. This module parses and validates that
language and exposes the result as plain dataclasses, consumed by the
generators in this directory.

Grammar (see designs doc for rationale):

  schema    := { decl }
  decl      := enum | group | element
  enum      := "enum" IDENT [":" IDENT] "{" { key "=" value } "}"
  key       := IDENT | STRING             # XML keyword ("2d" needs quoting)
  value     := IDENT | NUMBER             # C constant name or literal value
  group     := "group" IDENT ["variant"] "{" { attr | use | constraint } "}"
  element   := "element" IDENT [":" IDENT] ["(" facet {"," facet} ")"]
               "{" { attr | use | child | const | constraint } "}"
  const     := "set" IDENT "=" IDENT
               (field takes the C constant when the element is read; states
               what the element's identity implies, e.g. a sensor's type)
  attr      := IDENT ":" type ["=" default] ["(" facet {"," facet} ")"]
  type      := scalar ["[" [arity] "]"] | "enum" "<" IDENT ">"
             | "flags" "<" IDENT ">" | "id" "<" IDENT ">" | "ref" "<" IDENT ">"
  constraint:= verb bundle bundle { bundle }
               verb := "exclusive" | "together" | "requires" | "oneof"
               bundle := IDENT { "+" IDENT }
               (presence constraints over attributes: exclusive = at most one
               bundle present, together = all-or-none, requires a b = a needs
               b, oneof = at least one bundle complete; a bundle is complete
               when all its attributes appear)
  scalar    := "double" | "float" | "int" | "bool" | "string" | "file"
             | "chars"
  arity     := NUMBER | NUMBER ".." NUMBER | NUMBER ".." IDENT
  default   := NUMBER | STRING | IDENT | "{" NUMBER {"," NUMBER} "}"
  facet     := IDENT ["=" (IDENT | STRING | NUMBER)]
  child     := "child" IDENT card
  card      := "?" | "!" | "*" | "R"
  use       := "use" IDENT

Names and references (the dm_control model, namespaces explicit): an
attribute of type id<ns> declares a name in namespace `ns` (e.g. geom name);
an attribute of type ref<ns> holds the name of an object in that namespace
(e.g. an actuator's site). Namespaces exist by virtue of id declarations;
a ref into a namespace nothing declares into is an error.
"""

import dataclasses
import re
import sys

from typing import Any, Optional, Union

# Facets accepted on attributes. Unknown facets are an error: forward
# compatibility is explicit, not silent.
KNOWN_FACETS = frozenset({'field', 'required', 'nodefault', 'pattern',
                          'reading', 'writing', 'min', 'max', 'positive'})

# Facets accepted on elements. Element names must be unique, but the same XML
# tag means different elements in different contexts (joint under body,
# equality, composite, tendon/fixed); 'xml' gives the tag when it differs from
# the declaration name. 'alias' records the mjXSchema::NameMatch behavior of
# tags validated against another element's row (worldbody, frame, replicate
# all match body): the MJCF[] emitter skips aliased elements, the XSD emitter
# declares them fully. 'field' names the sub-struct of the bound spec that
# this element's attributes live in (the mjVisual sub-sections).
ELEMENT_FACETS = frozenset({'xml', 'alias', 'field'})

# 'file' is a string resolved against asset directories and the VFS
# (the reader's ReadAttrFile); kept distinct for XSD/tooling and bindings.
# 'bool' is a primitive whose XML keywords are exactly "true" and "false".
# 'chars' is text bound to a fixed char array; its arity counts characters,
# not space-separated tokens, and must be bounded.
SCALAR_TYPES = frozenset({'double', 'float', 'int', 'bool', 'string',
                          'file', 'chars'})

CARDINALITIES = frozenset({'?', '!', '*', 'R'})


class SchemaError(Exception):
  """Parse or validation error, formatted as path:line: message."""

  def __init__(self, path: str, line: int, message: str):
    super().__init__(f'{path}:{line}: {message}')
    self.path = path
    self.line = line
    self.message = message


@dataclasses.dataclass
class Arity:
  """Token count of a vector attribute: exact, ranged, or unbounded.

  `hi` is an int, a symbolic C constant name (e.g. 'mjNREF'), or None for
  unbounded. Scalar attributes have arity (1, 1).
  """
  lo: int
  hi: Union[int, str, None]

  def is_scalar(self) -> bool:
    return self.lo == 1 and self.hi == 1


@dataclasses.dataclass
class Attr:
  """An attribute declaration in the MJCF schema."""
  name: str
  type: str                    # scalar name, 'enum' or 'ref'
  target: Optional[str]        # enum or ref target name
  arity: Arity
  default: Union[None, float, str, tuple[float, ...]]
  facets: dict[str, Union[bool, str, float]]
  doc: Optional[str]
  line: int


@dataclasses.dataclass
class Use:
  """A `use` directive referencing a group in the MJCF schema."""
  group: str
  line: int


@dataclasses.dataclass
class Child:
  """A child element declaration in the MJCF schema."""
  name: str
  card: str
  doc: Optional[str]
  line: int


@dataclasses.dataclass
class Const:
  """A constant assignment (`set field = CONST`) in an element declaration."""
  field: str      # bound C field
  value: str      # C constant it takes
  doc: Optional[str]
  line: int


@dataclasses.dataclass
class Constraint:
  """A presence constraint over attributes in the MJCF schema."""
  kind: str                     # exclusive | together | requires | oneof
  bundles: list[tuple[str, ...]]  # attribute bundles ('+'-joined in source)
  doc: Optional[str]
  line: int


@dataclasses.dataclass
class Group:
  """An attribute group declaration in the MJCF schema."""
  name: str
  variant: bool
  members: list[Union[Attr, Use, Constraint]]
  doc: Optional[str]
  line: int


@dataclasses.dataclass
class Element:
  """An XML element declaration in the MJCF schema."""
  name: str
  spec: Optional[str]          # bound mjs struct name, e.g. 'mjsGeom'
  facets: dict[str, Union[bool, str, float]]
  members: list[Union[Attr, Use, Child, Const, Constraint]]
  doc: Optional[str]
  line: int

  def children(self) -> list[Child]:
    return [m for m in self.members if isinstance(m, Child)]

  def consts(self) -> list[Const]:
    return [m for m in self.members if isinstance(m, Const)]

  def constraints(self) -> list[Constraint]:
    return [m for m in self.members if isinstance(m, Constraint)]

  def xml_name(self) -> str:
    return str(self.facets.get('xml', self.name))


@dataclasses.dataclass
class Enum:
  """An enum declaration in the MJCF schema."""
  name: str
  ctype: Optional[str]         # bound C enum type, e.g. 'mjtGeom'
  items: list[tuple[str, str]]  # (xml keyword, C constant or literal)
  doc: Optional[str]
  line: int

  def keywords(self) -> list[str]:
    return [key for key, _ in self.items]


@dataclasses.dataclass
class Schema:
  """Top-level parsed representation of an MJCF schema."""
  enums: dict[str, Enum]
  groups: dict[str, Group]
  elements: dict[str, Element]
  path: str

  def expanded_attrs(self, element: Element) -> list[Attr]:
    """Element's attributes with `use` groups expanded, in declaration order."""
    out = []
    for member in element.members:
      if isinstance(member, Attr):
        out.append(member)
      elif isinstance(member, Use):
        out.extend(self._group_attrs(member.group))
    return out

  def _group_attrs(self, name: str) -> list[Attr]:
    out = []
    for member in self.groups[name].members:
      if isinstance(member, Attr):
        out.append(member)
      elif isinstance(member, Use):
        out.extend(self._group_attrs(member.group))
    return out


#--------------------------------- lexer ---------------------------------------

# NUMBER must not swallow the first dot of a '..' range: 0..3 lexes as
# NUMBER(0) DOTDOT NUMBER(3) via the (?!\.) lookahead.
_TOKEN_RE = re.compile(r"""
    (?P<ws>[ \t]+)
  | (?P<comment>\#[^\n]*)
  | (?P<newline>\n)
  | (?P<string>"[^"\n]*")
  | (?P<number>-?(?:\d+(?:\.(?!\.)\d*)?|\.\d+)(?:[eE][+-]?\d+)?)
  | (?P<dotdot>\.\.)
  | (?P<ident>[A-Za-z_][A-Za-z0-9_]*)
  | (?P<punct>[{}()\[\]<>:=,?!*+])
""", re.VERBOSE)


@dataclasses.dataclass
class _Token:
  kind: str    # 'string' | 'number' | 'dotdot' | 'ident' | punct char | 'eof'
  value: str
  line: int


def _lex(text: str, path: str) -> tuple[list[_Token], dict[int, str]]:
  """Returns tokens and a map of line number -> trailing comment text."""
  tokens = []
  comments = {}
  line = 1
  pos = 0
  while pos < len(text):
    match = _TOKEN_RE.match(text, pos)
    if not match:
      raise SchemaError(path, line, f'unexpected character {text[pos]!r}')
    kind = match.lastgroup
    value = match.group()
    if kind == 'newline':
      line += 1
    elif kind == 'comment':
      comments[line] = value[1:].strip()
    elif kind == 'punct':
      tokens.append(_Token(value, value, line))
    elif kind != 'ws':
      tokens.append(_Token(kind, value, line))  # pyrefly: ignore[bad-argument-type]
    pos = match.end()
  tokens.append(_Token('eof', '', line))
  return tokens, comments


#--------------------------------- parser --------------------------------------


class _Parser:
  """Recursive-descent parser over the token stream."""

  def __init__(self, text: str, path: str):
    self.path = path
    self.tokens, self.comments = _lex(text, path)
    self.pos = 0

  def error(self, message: str, line: Optional[int] = None) -> SchemaError:
    return SchemaError(self.path, line or self.peek().line, message)

  def peek(self) -> _Token:
    return self.tokens[self.pos]

  def next(self) -> _Token:
    token = self.tokens[self.pos]
    self.pos += 1
    return token

  def expect(self, kind: str) -> _Token:
    token = self.next()
    if token.kind != kind:
      raise self.error(f'expected {kind!r}, got {token.value!r}',
                       line=token.line)
    return token

  def accept(self, kind: str, value: Optional[str] = None) -> Optional[_Token]:
    token = self.peek()
    if token.kind == kind and (value is None or token.value == value):
      return self.next()
    return None

  def doc_for(self, line: int) -> Optional[str]:
    return self.comments.get(line)

  def parse(self) -> Schema:
    """Parses the token stream into a Schema dataclass."""
    schema = Schema(enums={}, groups={}, elements={},
                    path=self.path)
    while self.peek().kind != 'eof':
      token = self.expect('ident')
      if token.value == 'enum':
        enum = self.parse_enum(token.line)
        self.declare(schema.enums, enum.name, 'enum', token.line)
        schema.enums[enum.name] = enum
      elif token.value == 'group':
        group = self.parse_group(token.line)
        self.declare(schema.groups, group.name, 'group', token.line)
        schema.groups[group.name] = group
      elif token.value == 'element':
        element = self.parse_element(token.line)
        self.declare(schema.elements, element.name, 'element', token.line)
        schema.elements[element.name] = element
      else:
        raise self.error(
            f"expected 'enum', 'group' or 'element', "
            f'got {token.value!r}', line=token.line)
    return schema

  def declare(self, table: dict[str, Any], name: str, what: str, line: int):
    """Ensures declaration names are unique within a table."""
    if name in table:
      raise self.error(f'duplicate {what} {name!r} '
                       f'(first declared on line {table[name].line})',
                       line=line)

  def parse_enum(self, line: int) -> Enum:
    """Parses an enum declaration."""
    name = self.expect('ident').value
    ctype = self.expect('ident').value if self.accept(':') else None
    doc = self.doc_for(line)
    self.expect('{')
    items = []
    seen = {}
    while not self.accept('}'):
      key_token = self.next()
      if key_token.kind == 'string':
        key = key_token.value.strip('"')
      elif key_token.kind == 'ident':
        key = key_token.value
      else:
        raise self.error(f'expected enum keyword, got {key_token.value!r}',
                         line=key_token.line)
      if key in seen:
        raise self.error(f'duplicate enum keyword {key!r}',
                         line=key_token.line)
      seen[key] = key_token.line
      self.expect('=')
      value_token = self.next()
      if value_token.kind not in ('ident', 'number'):
        raise self.error(f'expected C constant or number, '
                         f'got {value_token.value!r}', line=value_token.line)
      items.append((key, value_token.value))
    if not items:
      raise self.error(f'enum {name!r} is empty', line=line)
    return Enum(name=name, ctype=ctype, items=items, doc=doc, line=line)

  def parse_group(self, line: int) -> Group:
    """Parses a group declaration."""
    name = self.expect('ident').value
    variant = bool(self.accept('ident', 'variant'))
    doc = self.doc_for(line)
    self.expect('{')
    members = []
    while not self.accept('}'):
      members.append(self.parse_member(allow_child=False))
    if not members:
      raise self.error(f'group {name!r} is empty', line=line)
    return Group(name=name, variant=variant, members=members, doc=doc,
                 line=line)

  def parse_element(self, line: int) -> Element:
    """Parses an element declaration."""
    name = self.expect('ident').value
    spec = self.expect('ident').value if self.accept(':') else None
    facets = self.parse_facets(ELEMENT_FACETS) if self.accept('(') else {}
    doc = self.doc_for(line)
    self.expect('{')
    members = []
    while not self.accept('}'):
      members.append(self.parse_member(allow_child=True))
    return Element(name=name, spec=spec, facets=facets, members=members,
                   doc=doc, line=line)

  CONSTRAINT_VERBS = frozenset({'exclusive', 'together', 'requires', 'oneof'})

  def parse_member(self, allow_child: bool) -> Union[Attr, Use, Child, Const,
                                                     Constraint]:
    """Parses a member of a group or element."""
    token = self.expect('ident')
    if token.value == 'use':
      return Use(group=self.expect('ident').value, line=token.line)
    if token.value in self.CONSTRAINT_VERBS and self.peek().kind == 'ident':
      # constraints are single-line constructs
      bundles = []
      while self.peek().kind == 'ident' and self.peek().line == token.line:
        bundle = [self.expect('ident').value]
        while self.accept('+'):
          bundle.append(self.expect('ident').value)
        bundles.append(tuple(bundle))
      if len(bundles) < 2:
        raise self.error(f'{token.value!r} needs at least two attributes',
                         line=token.line)
      return Constraint(kind=token.value, bundles=bundles,
                        doc=self.doc_for(token.line), line=token.line)
    if token.value == 'set':
      if not allow_child:
        raise self.error("'set' is not allowed in a group", line=token.line)
      field = self.expect('ident').value
      self.expect('=')
      value = self.expect('ident').value
      return Const(field=field, value=value, doc=self.doc_for(token.line),
                   line=token.line)
    if token.value == 'child':
      if not allow_child:
        raise self.error("'child' is not allowed in a group", line=token.line)
      name = self.expect('ident').value
      card = self.next()
      if card.value not in CARDINALITIES:
        raise self.error(f'expected cardinality (? ! * R), '
                         f'got {card.value!r}', line=card.line)
      return Child(name=name, card=card.value, doc=self.doc_for(token.line),
                   line=token.line)
    return self.parse_attr(token)

  def parse_attr(self, name_token: _Token) -> Attr:
    """Parses an attribute declaration."""
    line = name_token.line
    self.expect(':')
    attr_type, target, arity = self.parse_type()
    default = self.parse_default() if self.accept('=') else None
    facets = self.parse_facets() if self.accept('(') else {}
    return Attr(name=name_token.value, type=attr_type, target=target,
                arity=arity, default=default, facets=facets,
                doc=self.doc_for(line), line=line)

  def parse_type(self) -> tuple[str, Optional[str], Arity]:
    """Parses an attribute type declaration."""
    token = self.expect('ident')
    if token.value in ('enum', 'flags', 'ref', 'id'):
      self.expect('<')
      target = self.expect('ident').value
      self.expect('>')
      return token.value, target, Arity(1, 1)
    if token.value not in SCALAR_TYPES:
      raise self.error(f'unknown type {token.value!r}', line=token.line)
    return token.value, None, self.parse_arity()

  def parse_arity(self) -> Arity:
    """Parses attribute arity specification."""
    if not self.accept('['):
      return Arity(1, 1)
    if self.accept(']'):
      return Arity(0, None)  # unbounded
    lo_token = self.expect('number')
    lo = self.parse_int(lo_token)
    if not self.accept('dotdot'):
      self.expect(']')
      return Arity(lo, lo)
    hi_token = self.next()
    if hi_token.kind == 'number':
      hi = self.parse_int(hi_token)
      if hi <= lo:
        raise self.error(f'arity range [{lo}..{hi}] is not increasing',
                         line=hi_token.line)
    elif hi_token.kind == 'ident':
      hi = hi_token.value  # symbolic bound, e.g. mjNREF
    else:
      raise self.error(f'expected arity bound, got {hi_token.value!r}',
                       line=hi_token.line)
    self.expect(']')
    return Arity(lo, hi)

  def parse_int(self, token: _Token) -> int:
    """Parses an integer value from a token."""
    try:
      value = int(token.value)
    except ValueError:
      raise self.error(f'expected integer, got {token.value!r}',
                       line=token.line) from None
    if value < 0:
      raise self.error('arity may not be negative', line=token.line)
    return value

  def parse_default(self) -> Union[float, str, tuple[float, ...]]:
    """Parses default value for an attribute."""
    token = self.next()
    if token.kind == 'number':
      return float(token.value)
    if token.kind == 'string':
      return token.value.strip('"')
    if token.kind == 'ident':
      return token.value  # enum keyword
    if token.kind == '{':
      values = [float(self.expect('number').value)]
      while self.accept(','):
        values.append(float(self.expect('number').value))
      self.expect('}')
      return tuple(values)
    raise self.error(f'expected default value, got {token.value!r}',
                     line=token.line)

  def parse_facets(self, known=KNOWN_FACETS) -> dict[str, Union[bool, str,
                                                                float]]:
    """Parses attribute or element facets."""
    facets = {}
    while True:
      token = self.expect('ident')
      if token.value not in known:
        raise self.error(f'unknown facet {token.value!r}', line=token.line)
      if token.value in facets:
        raise self.error(f'duplicate facet {token.value!r}', line=token.line)
      if self.accept('='):
        value_token = self.next()
        if value_token.kind == 'string':
          facets[token.value] = value_token.value.strip('"')
        elif value_token.kind == 'ident':
          facets[token.value] = value_token.value
        elif value_token.kind == 'number':
          facets[token.value] = float(value_token.value)
        else:
          raise self.error(f'expected facet value, got {value_token.value!r}',
                           line=value_token.line)
      else:
        facets[token.value] = True
      if self.accept(')'):
        return facets
      self.expect(',')


#------------------------------- validation ------------------------------------


def _validate(schema: Schema):
  """Semantic checks; raises SchemaError on the first violation."""
  path = schema.path

  def err(line: int, message: str):
    raise SchemaError(path, line, message)

  # group use graph: dangling targets and cycles
  for group in schema.groups.values():
    _check_group_cycle(schema, group.name, [], group.line)
  for group in schema.groups.values():
    member_names = {m.name for m in group.members
                    if isinstance(m, Attr)}
    for con in [m for m in group.members if isinstance(m, Constraint)]:
      for bundle in con.bundles:
        for name in bundle:
          if name not in member_names:
            err(con.line, f'constraint references unknown attribute {name!r}')
    if group.variant:
      for member in group.members:
        if isinstance(member, Use):
          err(member.line,
              f"variant group {group.name!r} may not contain 'use'")
        elif isinstance(member, Attr) and member.facets.get('required'):
          err(member.line, f'attribute {member.name!r} in variant group '
              f'{group.name!r} may not be required')

  containers = list(schema.groups.values()) + list(schema.elements.values())
  for container in containers:
    for member in container.members:
      if isinstance(member, Use) and member.group not in schema.groups:
        err(member.line, f'use of undeclared group {member.group!r}')

  # namespaces exist by virtue of id<ns> declarations
  namespaces = set()
  for container in containers:
    for member in container.members:
      if isinstance(member, Attr) and member.type == 'id':
        namespaces.add(member.target)

  for element in schema.elements.values():
    # element facets
    for facet in ('xml', 'alias'):
      if facet in element.facets and not isinstance(element.facets[facet], str):
        err(element.line, f'element facet {facet!r} requires a name')
    alias = element.facets.get('alias')
    if alias is not None and alias not in schema.elements:
      err(element.line,
          f'alias references undeclared element {alias!r}')

    # children: dangling targets and duplicates
    seen_children = set()
    for child in element.children():
      if child.name not in schema.elements:
        err(child.line, f'child references undeclared element {child.name!r}')
      if child.name in seen_children:
        err(child.line, f'duplicate child {child.name!r}')
      seen_children.add(child.name)

    # attributes: duplicates across direct and use-expanded members
    seen_attrs = {}
    for attr in schema.expanded_attrs(element):
      if attr.name in seen_attrs:
        err(attr.line if attr.line > seen_attrs[attr.name] else element.line,
            f'duplicate attribute {attr.name!r} in element {element.name!r} '
            f'(directly or via use)')
      seen_attrs[attr.name] = attr.line

    # constraints reference the element's own (expanded) attributes
    for con in element.constraints():
      for bundle in con.bundles:
        for name in bundle:
          if name not in seen_attrs:
            err(con.line, f'constraint references unknown attribute {name!r}')
      if con.kind == 'requires' and (
          len(con.bundles) != 2 or any(len(b) != 1 for b in con.bundles)):
        err(con.line, "'requires' takes exactly two attributes")

  # per-attribute checks, wherever the attribute is declared
  for container in containers:
    for attr in container.members:
      if isinstance(attr, Attr):
        _validate_attr(schema, attr, namespaces)


def _check_group_cycle(schema: Schema, name: str, stack: list[str], line: int):
  """Recursively checks for cycles in group `use` references."""
  if name in stack:
    cycle = ' -> '.join(stack + [name])
    raise SchemaError(schema.path, line, f'group use cycle: {cycle}')
  group = schema.groups.get(name)
  if group is None:
    return  # dangling use is reported separately with its own line
  for member in group.members:
    if isinstance(member, Use):
      _check_group_cycle(schema, member.group, stack + [name], member.line)


def _validate_attr(schema: Schema, attr: Attr, namespaces: set[str]):
  """Validates semantic constraints for a single attribute."""
  path = schema.path

  def err(message: str):
    raise SchemaError(path, attr.line, message)

  # target existence
  if attr.type in ('enum', 'flags') and attr.target not in schema.enums:
    err(f'attribute {attr.name!r} references undeclared enum {attr.target!r}')
  if attr.type == 'ref' and attr.target not in namespaces:
    err(f'attribute {attr.name!r} references namespace {attr.target!r}, '
        f'which no id<{attr.target}> declares')

  # arity restrictions
  if attr.type in ('file', 'bool') and not attr.arity.is_scalar():
    err(f'{attr.type} attribute {attr.name!r} may not be a vector')
  if attr.type == 'chars' and not isinstance(attr.arity.hi, int):
    err(f'chars attribute {attr.name!r} must declare a bounded length')

  # facet payloads
  if 'pattern' in attr.facets and attr.type not in ('string', 'chars'):
    err("facet 'pattern' requires a text attribute")
  numeric = attr.type in ('double', 'float', 'int')
  for facet in ('min', 'max'):
    if facet in attr.facets and not (
        numeric and isinstance(attr.facets[facet], (int, float))):
      err(f'facet {facet!r} requires a numeric attribute and value')
  if 'min' in attr.facets and 'max' in attr.facets:
    if attr.facets['min'] > attr.facets['max']:  # pyrefly: ignore[unsupported-operation]
      err("facet 'min' cannot be greater than 'max'")
  if attr.facets.get('positive') and not numeric:
    err("facet 'positive' requires a numeric attribute")
  if attr.facets.get('required') and attr.default is not None:
    err(f'attribute {attr.name!r} is required and has a default')

  # defaults
  if attr.default is None:
    return
  if attr.type == 'enum':
    if not isinstance(attr.default, str):
      err(f'default for enum attribute {attr.name!r} must be a keyword')
    keywords = schema.enums[attr.target].keywords()  # pyrefly: ignore[bad-index]
    if attr.default not in keywords:
      err(f'default {attr.default!r} is not a keyword of enum '
          f'{attr.target!r}')
    return
  if attr.type in ('ref', 'id', 'chars'):
    err(f'{attr.type} attribute {attr.name!r} may not have a default')
  if attr.type == 'bool':
    if attr.default not in ('true', 'false'):
      err(f'default for bool attribute {attr.name!r} must be true or false')
    return
  if attr.type in ('string', 'file'):
    if not isinstance(attr.default, str):
      err(f'default for {attr.type} attribute {attr.name!r} must be a string')
    return
  # numeric scalars and vectors
  if isinstance(attr.default, str):
    err(f'default for numeric attribute {attr.name!r} must be numeric')
  n = len(attr.default) if isinstance(attr.default, tuple) else 1
  lo, hi = attr.arity.lo, attr.arity.hi
  if isinstance(attr.default, tuple) and attr.arity.is_scalar():
    err(f'vector default for scalar attribute {attr.name!r}')
  if n < lo:
    err(f'default for {attr.name!r} has {n} values, arity requires '
        f'at least {lo}')
  if isinstance(hi, int) and n > hi:
    err(f'default for {attr.name!r} has {n} values, arity allows '
        f'at most {hi}')


#--------------------------------- entry points --------------------------------


def parse_string(text: str, path: str = '<string>') -> Schema:
  """Parses an MJCF schema from a string."""
  schema = _Parser(text, path).parse()
  _validate(schema)
  return schema


def parse_file(path: str) -> Schema:
  """Parses an MJCF schema from a file path."""
  with open(path, 'r', encoding='utf-8') as file:
    return parse_string(file.read(), path)


def main() -> int:
  """CLI entry point for checking an MJCF schema file."""
  if len(sys.argv) != 2:
    sys.exit(f'usage: {sys.argv[0]} <mjcf.schema>')
  try:
    schema = parse_file(sys.argv[1])
  except SchemaError as error:
    sys.exit(str(error))
  n_attrs = sum(len(schema.expanded_attrs(e))
                for e in schema.elements.values())
  print(f'{sys.argv[1]}: '
        f'{len(schema.elements)} elements, {len(schema.groups)} groups, '
        f'{len(schema.enums)} enums, {n_attrs} attributes')
  return 0


if __name__ == '__main__':
  sys.exit(main())
