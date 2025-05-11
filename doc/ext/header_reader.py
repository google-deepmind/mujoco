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
"""Reads MuJoCo header files and generates a doc-friendly data structure."""

import dataclasses
import re

from typing import Optional, List, Dict

# Precompiled regex for matching a section.
_SECTION_REGEX = re.compile(r'^//-+ (?P<section>.+) -+$')

# Precompiled regex for matching a C function definition.
_FUNCTION_REGEX = re.compile(r'(?P<token>mj\w+)\s*\(')

# Precompiled regex for matching a C function ending.
_FUNCTION_ENDING_REGEX = re.compile(r'\);\s$')

# Precompiled regex for matching a C struct ending.
_STRUCT_END_REGEX_1 = re.compile(r'^typedef\s+struct\s+\w+\s+(?P<token>mj\w+);')

# Precompiled regex for matching a C struct ending (version 2).
_STRUCT_END_REGEX_2 = re.compile(r'^}\s+(?P<token>mj\w+);')

# Precompiled regex for matching a C enum ending.
_ENUM_END_REGEX = re.compile(r'^}\s+(?P<token>mj\w+);')


@dataclasses.dataclass
class ApiDefinition:
  """Defines a C reference parsed from a C header file."""
  token: str
  c_type: str
  code: str
  start: int
  end: int
  section: str
  doc: str


class ApiState:
  """Internal state of the reader used for parsing header files."""

  def __init__(self):
    self.token = ''
    self.section = ''
    self.code = ''
    self.doc = ''

    self._state = None
    self._start = 0
    self._end = 0

  @property
  def state(self):
    return self._state

  def export_definition(self):
    return ApiDefinition(self.token, self._state, self.code, self._start,
                         self._end, self.section, self.doc)

  def start(self, state):
    self._state = state
    self._start = self._end

  def iterate(self):
    self._end += 1

  def end(self):
    self.token = ''
    self._state = None
    self.code = ''
    self.doc = ''


def read(lines: List[str]) -> Dict[str, ApiDefinition]:
  """Reads  header lines and returns a maps of ApiDefinition's."""

  api = {}
  stripped_functions = False
  s = ApiState()

  for line in lines:
    s.iterate()
    section = _find_section(line)
    if section is not None:
      if 'MJAPI FUNCTIONS' in section:
        # Stripped functions do not begin with MJAPI, and must be under the
        # predefined section 'MJAPI FUNCTIONS'.  This is because the docs don't
        # include this prefix, and so we need to read such functions from the
        # reference header.
        stripped_functions = True
      s.section = section
      s.end()
      continue

    if s.state == 'DOC':
      token = _find_function_start(line, stripped_functions)
      if token is not None:
        if stripped_functions:
          s.code = f'{s.code}{line}'
        else:
          s.code = f'{s.code}{line[6:]}'
        s.token = token
        s.start('FUNCTION')
        if _is_function_end(line):
          api[token] = s.export_definition()
          s.end()
        continue
      elif line.startswith('//'):
        s.doc = f'{s.doc}{line[3:]}'
      else:
        s.end()
    if s.state == 'FUNCTION':
      if stripped_functions:
        s.code = f'{s.code}{line}'
      else:
        s.code = f'{s.code}{line[6:]}'
      if _is_function_end(line):
        api[s.token] = s.export_definition()
        s.end()
    elif s.state == 'ENUM':
      match = _ENUM_END_REGEX.search(line)
      if match is not None:
        s.code = f'{s.code}{line}'
        s.token = match.group('token')
        api[s.token] = s.export_definition()
        s.end()
      else:
        s.code = f'{s.code}{line}'
    elif s.state == 'STRUCT':
      match = _STRUCT_END_REGEX_1.search(line)
      if match is None:
        match = _STRUCT_END_REGEX_2.search(line)

      if match is not None:
        s.code = f'{s.code}{line}'
        s.token = match.group('token')
        api[s.token] = s.export_definition()
        s.end()
      else:
        s.code = f'{s.code}{line}'
    elif s.state is None:
      if line.startswith('typedef enum'):
        s.start('ENUM')
        s.code = f'{s.code}{line}'

      if line.startswith('struct') or line.startswith('typedef struct'):
        s.start('STRUCT')
        s.code = f'{s.code}{line}'

      if line.startswith('//'):
        s.doc = f'{s.doc}{line[3:]}'
        s.start('DOC')

      token = _find_function_start(line, stripped_functions)
      if token is not None:
        if stripped_functions:
          s.code = f'{s.code}{line}'
        else:
          s.code = f'{s.code}{line[6:]}'
        s.token = token
        s.start('FUNCTION')
        if _is_function_end(line):
          api[token] = s.export_definition()
          s.end()

  return api


def _find_section(line) -> Optional[str]:
  match = _SECTION_REGEX.search(line)
  if match is not None:
    return match.group('section').strip()
  return None


def _find_function_start(line, stripped) -> Optional[str]:
  if (line.startswith('MJAPI') and 'extern' not in line) or stripped:
    match = _FUNCTION_REGEX.search(line)
    if match is not None:
      return match.group('token')
  return None


def _is_function_end(line):
  match = _FUNCTION_ENDING_REGEX.search(line)
  return match is not None
