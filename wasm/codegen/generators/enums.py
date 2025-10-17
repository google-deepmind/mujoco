"""Generates Embind bindings for MuJoCo enums."""

from typing import Mapping

from introspect import ast_nodes

from helpers import code_builder


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

  def generate(self) -> str:
    """Generates all Embind code for the provided enums."""

    code = []
    for enum in self.enums.values():
      code.append(self._generate_enum_binding(enum))
    return "\n\n".join(code) + "\n"
