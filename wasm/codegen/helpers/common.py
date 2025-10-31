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
import pathlib

from wasm.codegen.helpers import constants

Path = pathlib.Path


def get_default_output_dir() -> str:
  """Gets the default output directory (sibling of 'generated' folder)."""
  # Get the directory of the current file (generator/base.py)
  current_dir = Path(__file__).parent
  # Go up one level to the project root and then down to 'generated'
  default_output_dir = str(current_dir.parent / "generated")
  return default_output_dir


def get_file_path(
    template_dir: str, output_dir: str, filename: str
) -> tuple[str, str]:
  """Constructs the template and output file paths.

  Args:
      template_dir: The directory containing the template files.
      output_dir: The directory where the generated files will be saved.
      filename: The name of the file.

  Returns:
      A tuple containing the template file path and the output file path.
  """
  template_file = f"wasm/codegen/{template_dir}/{filename}"
  output_file = f"wasm/codegen/{output_dir}/{filename}"
  return template_file, output_file


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


def uppercase_first_letter(input_string: str) -> str:
  """Uppercases the first letter of a string."""
  if input_string:
    return input_string[0].upper() + input_string[1:]
  return input_string


def try_cast_to_scalar_type(value: str) -> int | float | str:
  """Tries to cast a string to an integer, then a float, otherwise returns the original string."""
  for type_ in [int, float]:
    try:
      return type_(value)
    except ValueError:
      continue
  return value


def debug_print(msg: str):
  """Prints a message to the console if STRUCT_DEBUG_MODE is enabled."""
  if constants.STRUCT_DEBUG_MODE:
    print(msg)


def replace_lines_containing_marker(
    lines: list[str],
    marker_to_replace: str,
    replacement_content: str | list[str],
) -> list[str]:
  """Replaces lines containing a specific marker with new content."""

  new_lines = []
  replaced = False
  for line in lines:
    if not replaced and marker_to_replace in line:
      indentation = _get_indentation(line)
      if isinstance(replacement_content, str):
        new_lines.append(indentation + replacement_content)
      elif isinstance(replacement_content, list):
        for content_line in replacement_content:
          if not content_line.strip():
            continue
          indented_line = (
              indentation
              + content_line.replace("\n", "\n" + indentation)
              + "\n"
          )
          new_lines.append(indented_line)
      replaced = True
    else:
      new_lines.append(line)
  return new_lines


def _get_indentation(line: str) -> str:
  """Returns the indentation of the given line as a string of spaces."""

  indentation = ""
  for char in line:
    if char == " ":
      indentation += " "
    else:
      break
  return indentation
