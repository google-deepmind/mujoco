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
"""Utility for formatting AST node as Python code."""

import contextlib
import dataclasses
from typing import Any, Iterable, Mapping, Sequence

INDENT_WIDTH = 4
MAX_LINE_WIDTH = 80
SIMPLE_TYPES = frozenset([int, float, str, bool, bytes, type(None)])


def format_as_python_code(obj: Any) -> str:
  """Formats an AST node object as well-indented Python code."""
  formatter = _Formatter()
  formatter.add(obj)
  return str(formatter)


def _is_all_simple(seq: Iterable[Any]) -> bool:
  return all(type(obj) in SIMPLE_TYPES for obj in seq)


class _Formatter:
  """A helper for pretty-printing AST nodes as Python code."""

  def __init__(self):
    self._line_prefix = ''
    self._lines = []
    self._add_to_last_line = False

  @contextlib.contextmanager
  def _indent(self, width: int = INDENT_WIDTH):
    self._line_prefix += ' ' * width
    yield
    self._line_prefix = self._line_prefix[:-width]

  @contextlib.contextmanager
  def _append_at_end(self, s):
    yield
    self._lines[-1] += s

  def _add_line(self, line: str, no_break: bool = False):
    if self._add_to_last_line:
      self._lines[-1] += line
    else:
      self._lines.append(self._line_prefix + line)
    self._add_to_last_line = no_break

  def _add_dict(self, obj: Mapping[Any, Any]):
    """Adds a dict to the formatted output."""
    self._add_line('dict([')
    with self._indent():
      for k, v in obj.items():

        # Try to fit everything into a single line first.
        if _is_all_simple((k, v)):
          single_line = f'({k!r}, {v!r}),'
          if len(self._line_prefix) + len(single_line) <= MAX_LINE_WIDTH:
            self._add_line(single_line)
            continue

        self._add_line(f"('{k}',")
        with self._append_at_end('),'):
          with self._indent(1):
            self.add(v)

    self._add_line('])')

  def _add_dataclass(self, obj: Any):
    """Adds a dataclass object to the formatted output."""
    # Filter out default values.
    kv_pairs = []
    for k in dataclasses.fields(obj):
      v = getattr(obj, k.name)
      if v != k.default:
        kv_pairs.append((k, v))

    # Try to fit everything into a single line first.
    if _is_all_simple(v for _, v in kv_pairs):
      single_line = ', '.join(f'{k.name}={v!r}' for k, v in kv_pairs)
      single_line = f'{obj.__class__.__name__}({single_line})'
      if len(self._line_prefix) + len(single_line) <= MAX_LINE_WIDTH:
        self._add_line(single_line)
        return

    self._add_line(obj.__class__.__name__ + '(')
    with self._indent():
      for k, v in kv_pairs:
        self._add_line(k.name + '=', no_break=True)
        with self._append_at_end(','):
          self.add(v)
    self._add_line(')')

  def _add_sequence(self, obj: Sequence[Any]) -> None:
    """Adds a sequence to the formatted output."""
    default_str = repr(obj)
    open_token, close_token = default_str[0], default_str[-1]
    # Try to fit everything into a single line first.
    if _is_all_simple(obj):
      single_line = (
          f"{open_token}{', '.join(repr(o) for o in obj)}{close_token}")
      if close_token == ')' and len(obj) == 1:
        single_line = f'{single_line[:-1]},)'
      if len(self._line_prefix) + len(single_line) <= MAX_LINE_WIDTH:
        self._add_line(single_line)
        return

    self._add_line(open_token)
    with self._indent():
      for v in obj:
        with self._append_at_end(','):
          self.add(v)
    self._add_line(close_token)

  def add(self, obj: Any) -> None:
    """Adds an object to the formatted output."""
    if _is_all_simple((obj,)):
      self._add_line(repr(obj))
    elif dataclasses.is_dataclass(obj):
      self._add_dataclass(obj)
    elif isinstance(obj, Mapping):
      self._add_dict(obj)
    elif isinstance(obj, Sequence):
      self._add_sequence(obj)
    else:
      raise NotImplementedError

  def __str__(self):
    lines = []
    for line in self._lines:
      if len(line) > MAX_LINE_WIDTH:
        lines.append(f'{line}  # pylint: disable=line-too-long')
      else:
        lines.append(line)
    return '\n'.join(lines)
