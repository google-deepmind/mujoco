# Copyright 2025 DeepMind Technologies Limited
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
"""Update types.py with valid attribute types.

Instances like

@dataclasses.dataclass
class Option:
  ...
  timestep: array("*", float)
  ...

that do not have valid attribute types are not properly parsed during
documentation generation.

This script updates such instances with valid types

@dataclasses.dataclass
class Option:
  ...
  timestep: wp.array(dtype=float)
  ...
"""

import re
import sys


def replace_array_calls(match):
  """Replaces array() calls with wp.array, wp.array2d, etc."""
  args_str = match.group(1)
  if '...' in args_str:
    return match.group(0)  # Do not replace call in docstring

  args = [a.strip() for a in args_str.split(',')]
  n_args = len(args)
  dtype = args[-1]

  if n_args == 2:
    return f'wp.array(dtype={dtype})'
  elif n_args == 3:
    return f'wp.array2d(dtype={dtype})'
  elif n_args == 4:
    return f'wp.array3d(dtype={dtype})'
  elif n_args == 5:
    return f'wp.array4d(dtype={dtype})'
  else:
    return match.group(0)


def process_file(filepath):
  """Reads types.py, replaces array() calls, and writes back to file."""
  try:
    with open(filepath, 'r') as f:
      content = f.read()
  except FileNotFoundError:
    print(f'Error: Could not find {filepath}', file=sys.stderr)
    return

  new_content = re.sub(r'array\(([^)]+)\)', replace_array_calls, content)

  with open(filepath, 'w') as f:
    f.write(new_content)
  print(f'Processed {filepath}')


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print('Usage: python update_types.py <path_to_types.py>', file=sys.stderr)
    sys.exit(1)
  process_file(sys.argv[1])
