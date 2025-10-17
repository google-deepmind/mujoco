"""Generates Embind bindings for MuJoCo functions."""

import pathlib
from typing import List, Mapping, TypeAlias

from introspect import ast_nodes

from helpers import code_builder
from helpers import function_utils

FunctionDecl: TypeAlias = ast_nodes.FunctionDecl
FunctionParameterDecl: TypeAlias = ast_nodes.FunctionParameterDecl
PointerType: TypeAlias = ast_nodes.PointerType
ValueType: TypeAlias = ast_nodes.ValueType
Path: TypeAlias = pathlib.Path


class Generator:
  """Generates Embind bindings for MuJoCo functions."""

  def __init__(self, functions: Mapping[str, FunctionDecl]):
    self.direct_bind_functions: List[FunctionDecl] = []
    self.wrapper_bind_functions: List[FunctionDecl] = []

    for func in functions.values():
      if function_utils.should_be_wrapped(func):
        self.wrapper_bind_functions.append(func)
      else:
        self.direct_bind_functions.append(func)

  def _generate_wrappers(self) -> str:
    """Generates Embind bindings for all functions that need wrappers."""

    code = []
    for func in self.wrapper_bind_functions:
      wrapper_code = function_utils.generate_function_wrapper(func)
      code.append(wrapper_code)

    return "\n\n".join(code)

  def _generate_direct_bindable_functions(self) -> str:
    """Generates Embind bindings for all directly bindable functions."""

    result = ""
    for func in self.direct_bind_functions:
      result += code_builder.INDENT
      result += self._generate_function_binding(func)

    return result

  def _generate_function_binding(
      self, func: FunctionDecl, is_wrapper=False
  ) -> str:
    """Generates the Embind code for a single function."""

    js_name, cpp_func = func.name, func.name
    if is_wrapper:
      cpp_func += "_wrapper"

    return f'function("{js_name}", &{cpp_func});\n'

  def _generate_wrapper_bindable_functions(self) -> str:
    """Generates Embind bindings for all functions that need wrappers."""

    result = ""
    for func in self.wrapper_bind_functions:
      result += code_builder.INDENT
      result += self._generate_function_binding(func, True)

    return result

  def generate(self) -> tuple[str, str]:
    """Generates the bindings file for all functions."""

    wrapper_functions = self._generate_wrappers()
    function_bindings = self._generate_direct_bindable_functions()
    function_bindings += self._generate_wrapper_bindable_functions()

    return wrapper_functions, function_bindings
