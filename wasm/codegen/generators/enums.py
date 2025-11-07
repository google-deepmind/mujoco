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

"""Generates Embind bindings for MuJoCo enums."""

from typing import Mapping

from introspect import ast_nodes

from wasm.codegen.generators import code_builder


class Generator:
  """Generates Embind code for MuJoCo enums."""

  def __init__(self, enums: Mapping[str, ast_nodes.EnumDecl]):
    self.enums = enums

  def _generate_enum_binding(self, enum: ast_nodes.EnumDecl) -> str:
    """Generates the Embind code for a single enum."""

    code = f'{code_builder.INDENT}enum_<{enum.name}>("{enum.name}")'

    for value_name in enum.values:
      code += f'\n{2*code_builder.INDENT}.value("{value_name}", {value_name})'

    code += ";"
    return code

  def generate(self) -> list[tuple[str, list[str]]]:
    """Generates all Embind code for the provided enums."""

    code = []
    for enum in self.enums.values():
      code.append(self._generate_enum_binding(enum))

    content = "\n\n".join(code)
    marker = "// {{ ENUM_BINDINGS }}"
    return [(marker, [content])]
