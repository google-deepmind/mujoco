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
"""Script to automatically extract and insert MuJoCo source file paths."""

import argparse
import os
import re
import sys


def main():
  """Extracts MuJoCo source file paths and updates linenumbers.js."""
  parser = argparse.ArgumentParser(description='Update SRCS in linenumbers.js')
  parser.add_argument(
      '--src_dir', required=True, help='Path to mujoco src directory')
  parser.add_argument(
      '--js_file', required=True, help='Path to linenumbers.js file')
  parser.add_argument(
      '--ref_file', required=True, help='Path to references.h')
  parser.add_argument(
      '--check', action='store_true',
      help='Check if SRCS matches, without updating')
  args = parser.parse_args()

  with open(args.ref_file, 'r', encoding='utf-8') as f:
    ref_content = f.read()

  # Extract all mj_... symbols that might be function names
  valid_funcs = set(re.findall(r'\bmj[a-zA-Z0-9_]*\b', ref_content))

  srcs = []
  pattern = re.compile(r'^(const )?[a-zA-Z0-9_*]+\s+(.+)\(.+[{,]$')

  for root, _, files in os.walk(args.src_dir):
    for f in files:
      if f.endswith('.c') or f.endswith('.cc'):
        path = os.path.relpath(os.path.join(root, f), args.src_dir)
        # Ensure we use forward slashes for Javascript array
        path = path.replace('\\', '/')

        # Check if it has any function defined in references.h
        filepath = os.path.join(root, f)
        found = False
        with open(filepath, 'r', encoding='utf-8') as cf:
          for line in cf:
            line = line.strip('\n')
            match = pattern.match(line)
            key = None
            if match:
              key = match.group(2).strip()

            # edge cases
            if 'user_api.cc' in filepath and line.startswith(
                '[[nodiscard]] int mj_recompile('):
              key = 'mj_recompile'
            elif 'engine_io.c' in filepath:
              if line.startswith('void mj_freeStack('):
                key = 'mj_freeStack'
              elif line.startswith('void mj_markStack('):
                key = 'mj_markStack'

            if key and key in valid_funcs:
              found = True
              break

        if found:
          srcs.append(path)

  srcs = sorted(srcs)

  with open(args.js_file, 'r', encoding='utf-8') as f:
    content = f.read()

  # Find the const SRCS = [ ... ]; block
  js_pattern = re.compile(r'const SRCS = \[\n(.*?)\n\];', re.DOTALL)
  match = js_pattern.search(content)
  if not match:
    sys.exit('Could not find const SRCS = [ in linenumbers.js')

  current_srcs_str = match.group(1)

  # Generate the new string
  new_srcs_str = '\n'.join([f"  '{s}'," for s in srcs])

  if current_srcs_str == new_srcs_str:
    print('SRCS is up to date.')
    sys.exit(0)

  if args.check:
    print('SRCS is not up to date. Please run update_docs to update it.')
    sys.exit(1)

  # Otherwise, update the file
  new_content = content[:match.start(1)] + new_srcs_str + content[match.end(1):]
  with open(args.js_file, 'w', encoding='utf-8') as f:
    f.write(new_content)
  print(f'Updated SRCS in {args.js_file} with {len(srcs)} files.')

if __name__ == '__main__':
  main()
