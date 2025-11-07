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

"""Utility functions for code generation."""

import os


def write_to_file(filepath: str, content: str) -> None:
  """Writes content to a file."""
  output_dir = os.path.dirname(filepath)

  try:
    if output_dir:
      os.makedirs(output_dir, exist_ok=True)
    with open(filepath, "w") as f:
      chars = f.write(content)
      print(f"wrote {chars} characters to file '{filepath}'")
  except IOError as e:
    print(f"Error writing to output file: {filepath} - {e}")


def lowercase_first_letter(input_string: str) -> str:
  """Lowercases the first letter of a string."""
  return input_string[:1].lower() + input_string[1:]


def uppercase_first_letter(input_string: str) -> str:
  """Uppercases the first letter of a string."""
  return input_string[:1].upper() + input_string[1:]


def replace_lines_containing_marker(
    lines: list[str],
    marker: str,
    content: list[str],
) -> list[str]:
  """Replaces lines containing a specific marker with new content."""
  for i, line in enumerate(lines):
    if marker in line:
      indent = line[: len(line) - len(line.lstrip(" "))]
      replacement_lines = []
      for text in content:
        if text.strip():
          # Prepend indent to ensure the first replacement line matches the
          # indentation of the marker and also ensure that text containing
          # newlines is also indented correctly.
          # TODO(matijak): This is working around an upstream problem, we should
          # make it a precondition that content elements do not contain newlines
          # and fix callers to ensure that.
          replacement_lines.append(
              indent + text.replace("\n", f"\n{indent}") + "\n"
          )
      return lines[:i] + replacement_lines + lines[i + 1 :]
  return lines
