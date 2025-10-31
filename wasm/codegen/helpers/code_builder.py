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

"""Helper class to build code string line by line with indentation."""

INDENT = "  "


class CodeBuilder:
  """Helper class to build code string line by line with indentation."""

  def __init__(self, indent_str: str = INDENT):
    self._lines = []
    self._indent_level = 0
    self._indent_str = indent_str

  def line(self, line_content: str) -> None:
    """Adds a line with indentation, special-casing "private:" and "public:"."""
    indent = self._indent_str * self._indent_level
    content = line_content.strip()
    if content == "private:" or content == "public:":
      self._lines.append(indent[:-1] + line_content)
    elif content:
      self._lines.append(indent + line_content)
    else:
      self._lines.append("")

  def to_string(self) -> str:
    """Returns the complete code string."""
    return "\n".join(self._lines)

  class IndentBlock:
    """Helper class to manage indentation within a `with` statement."""

    def __init__(self, builder: "CodeBuilder", header_line=""):
      self._builder = builder
      self._header_line = header_line

    def __enter__(self):
      line = self._header_line
      line += " {" if line else "{"
      self._builder.line(line)
      self._builder._indent_level += 1
      return self._builder

    def __exit__(self, exc_type, exc_val, exc_tb):
      if self._builder._indent_level > 0:
        self._builder._indent_level -= 1
      self._builder.line("}")

  def block(self, header_line="") -> IndentBlock:
    """Creates a block including braces and an optional header before the opening brace.

    Use via a `with` statement.

    Args:
        header_line: Optional header line to add before the opening brace.

    Returns:
        An IndentBlock instance that manages the indentation.
    """
    return self.IndentBlock(self, header_line)
