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

"""Helper functions for processing and generating bindings for MuJoCo functions."""

from typing import List, Mapping, Set, Tuple, cast

from introspect import ast_nodes

from wasm.codegen.generators import code_builder
from wasm.codegen.generators import common
from wasm.codegen.generators import constants


PRIMITIVE_TYPES = constants.PRIMITIVE_TYPES
uppercase_first_letter = common.uppercase_first_letter


def param_is_primitive_value(param: ast_nodes.FunctionParameterDecl) -> bool:
  """Checks if param is a primitive value type."""
  if isinstance(param.type, ast_nodes.ValueType):
    return param.type.name in PRIMITIVE_TYPES
  return False


def param_is_pointer_to_primitive_value(
    param: ast_nodes.FunctionParameterDecl,
) -> bool:
  """Checks if param is a pointer to a primitive value."""
  return (
      isinstance(param.type, ast_nodes.PointerType)
      or isinstance(param.type, ast_nodes.ArrayType)
  ) and (
      isinstance(param.type.inner_type, ast_nodes.ValueType)
      and param.type.inner_type.name in PRIMITIVE_TYPES
  )


def param_is_pointer_to_struct(param: ast_nodes.FunctionParameterDecl) -> bool:
  """Checks if param is a pointer to a struct."""
  return (
      isinstance(param.type, ast_nodes.PointerType)
      or isinstance(param.type, ast_nodes.ArrayType)
  ) and (
      isinstance(param.type.inner_type, ast_nodes.ValueType)
      and param.type.inner_type.name not in PRIMITIVE_TYPES
  )


def return_is_value_of_type(
    func: ast_nodes.FunctionDecl, allowed_types: Set[str]
) -> bool:
  """Checks if func returns an allowed value type."""
  return (
      isinstance(func.return_type, ast_nodes.ValueType)
      and func.return_type.name in allowed_types
  )


def return_is_pointer_to_struct(func: ast_nodes.FunctionDecl) -> bool:
  """Checks if func returns a pointer to a struct."""
  return (
      isinstance(func.return_type, ast_nodes.PointerType)
      and isinstance(func.return_type.inner_type, ast_nodes.ValueType)
      and func.return_type.inner_type.name not in PRIMITIVE_TYPES
  )


def return_is_pointer_to_primitive(func: ast_nodes.FunctionDecl) -> bool:
  """Checks if func returns a pointer to a primitive value."""
  return (
      isinstance(func.return_type, ast_nodes.PointerType)
      and isinstance(func.return_type.inner_type, ast_nodes.ValueType)
      and func.return_type.inner_type.name in PRIMITIVE_TYPES
  )


def get_const_qualifier(func: ast_nodes.FunctionDecl) -> str:
  """Returns the const qualifier of func's return type."""
  if (
      isinstance(func.return_type, ast_nodes.PointerType)
      and isinstance(func.return_type.inner_type, ast_nodes.ValueType)
      and func.return_type.inner_type.is_const
  ):
    return "const "
  return ""


def should_be_wrapped(func: ast_nodes.FunctionDecl) -> bool:
  """Checks if a MuJoCo function needs a wrapper function."""
  return (
      return_is_pointer_to_primitive(func)
      or return_is_pointer_to_struct(func)
      or any(
          param_is_pointer_to_primitive_value(param)
          or isinstance(param.type, ast_nodes.ArrayType)
          or param_is_pointer_to_struct(param)
          for param in func.parameters
      )
  )


def generate_function_wrapper(func: ast_nodes.FunctionDecl) -> str:
  """Generates C++ code for a wrapper function."""

  params_unpack_statements = get_params_unpack_statements(func.parameters)
  wrapper_params_list = get_params_string(func.parameters)
  not_nullable_params = get_params_notnullable(func.parameters)
  wrapper_params = ", ".join(wrapper_params_list)
  ret_type = get_compatible_return_type(func)

  builder = code_builder.CodeBuilder()
  with builder.function(f"{ret_type} {func.name}_wrapper({wrapper_params})"):
    invoker_params_list = get_params_string_maybe_with_conversion(
        func.parameters
    )
    invoker_params_str = ", ".join(invoker_params_list)
    invoker_call = f"{func.name}({invoker_params_str})"
    invoker_statement = get_compatible_return_call(func, invoker_call)
    for p in not_nullable_params:
      builder.line(f"CHECK_VAL({p});")
    for unpack_statement in params_unpack_statements:
      builder.line(unpack_statement)
    builder.line(f"{invoker_statement};")

  return builder.to_string()


def get_params_notnullable(
    ast_params: Tuple[ast_nodes.FunctionParameterDecl, ...],
) -> List[str]:
  """Generates list of param names for checking if they aren't null/undefined."""

  not_nullable_params = []
  for p in ast_params:
    if (
        isinstance(p.type, (ast_nodes.PointerType, ast_nodes.ArrayType))
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        # We only check for char because others are checked in the unpacker
        # and we don't want to check twice.
        and p.type.inner_type.name == "char"
        and not p.nullable
    ):
      not_nullable_params.append(p.name)
  return not_nullable_params


def get_params_unpack_statements(
    ast_params: Tuple[ast_nodes.FunctionParameterDecl, ...],
) -> List[str]:
  """Generates C++ statements to unpack JS values for pointer/array parameters."""

  params_unpack_statements = []
  for p in ast_params:
    if (
        isinstance(p.type, (ast_nodes.PointerType, ast_nodes.ArrayType))
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        and p.type.inner_type.name in PRIMITIVE_TYPES
    ):
      if p.type.inner_type.name == "char":
        # param is Javascript string
        continue

      if p.type.inner_type.is_const:
        # param is Javascript number[]
        params_unpack_statements.append(
            f"UNPACK_ARRAY({p.type.inner_type.name}, {p.name});"
        )
      else:
        # param is TypedArray or a WasmBuffer
        params_unpack_statements.append(
            f"UNPACK_VALUE({p.type.inner_type.name}, {p.name});"
        )
  return params_unpack_statements


def get_params_string(
    parameters: Tuple[ast_nodes.FunctionParameterDecl, ...]
) -> List[str]:
  """Generates a list of C++ parameter declarations as strings."""

  result = []
  for p in parameters:
    if (
        isinstance(p.type, ast_nodes.PointerType)
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        and p.type.inner_type.name not in PRIMITIVE_TYPES
    ):
      # Pointer to struct parameters
      const_qualifier = "const " if p.type.inner_type.is_const else ""
      result.append(
          f"{const_qualifier}{uppercase_first_letter(p.type.inner_type.name)}&"
          f" {p.name}"
      )
    elif (
        isinstance(p.type, ast_nodes.ValueType)
        and p.type.name in PRIMITIVE_TYPES
    ):
      # Primitive value parameters
      const_qualifier = "const " if p.type.is_const else ""
      result.append(f"{const_qualifier}{p.type} {p.name}")
    elif (
        isinstance(p.type, (ast_nodes.PointerType, ast_nodes.ArrayType))
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        and p.type.inner_type.name in PRIMITIVE_TYPES
    ):
      # Pointer to primitive value parameters or arrays
      if p.type.inner_type.name == "char":
        if p.nullable:
          result.append(f"const NullableString& {p.name}")
        else:
          result.append(f"const String& {p.name}")
      elif (
          p.type.inner_type.name
          in ["int", "float", "double", "mjtNum", "mjtByte"]
          and p.type.inner_type.is_const
      ):
        result.append(f"const NumberArray& {p.name}")
      else:
        result.append(f"const val& {p.name}")
    else:
      # This case should ideally not be reached if AST is well-formed
      # and types are categorized by the helper booleans correctly.
      raise TypeError(
          "Unable to generate param string. Unhandled parameter type:"
          f" {p.type} for param '{p.name}'"
      )
  return result


def get_params_string_maybe_with_conversion(
    ast_params: Tuple[ast_nodes.FunctionParameterDecl, ...],
) -> List[str]:
  """Generates C++ expressions for passing compatible params from JS to MuJoCo C-API functions."""

  native_params = []
  for p in ast_params:
    if param_is_pointer_to_struct(p):
      native_params.append(f"{p.name}.get()")
    elif param_is_primitive_value(p):
      native_params.append(p.name)
    elif (
        isinstance(p.type, (ast_nodes.PointerType, ast_nodes.ArrayType))
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        and p.type.inner_type.name in PRIMITIVE_TYPES
        and p.type.inner_type.name != "char"
    ):
      native_params.append(f"{p.name}_.data()")
    elif (
        isinstance(p.type, (ast_nodes.PointerType, ast_nodes.ArrayType))
        and isinstance(p.type.inner_type, ast_nodes.ValueType)
        and p.type.inner_type.name == "char"
    ):
      const_qualifier = "const " if p.type.inner_type.is_const else ""
      native_params.append(
          f"{p.name}.as<{const_qualifier}std::string>().data()"
      )
    else:
      raise TypeError(
          f"Unhandled parameter type for conversion: {p.type} for param"
          f" '{p.name}'"
      )
  return native_params


def get_compatible_return_call(
    func: ast_nodes.FunctionDecl, invoker: str
) -> str:
  """Generates embind compatible return value conversion."""

  if return_is_value_of_type(func, {"void"}):
    return invoker
  if isinstance(func.return_type, ast_nodes.PointerType) and isinstance(
      func.return_type.inner_type, ast_nodes.ValueType
  ):
    if func.return_type.inner_type.name == "char":
      return f"return std::string({invoker})"
    elif func.return_type.inner_type.name == "mjString":
      return f"return *{invoker}"
  if return_is_pointer_to_struct(func):
    return get_converted_struct_to_class(func, invoker)
  if return_is_value_of_type(func, PRIMITIVE_TYPES):
    return f"return {invoker}"
  raise RuntimeError(
      "Failed to calculate return value conversion for function"
      f" {func.name} that returns '{func.return_type}'"
  )


def get_compatible_return_type(func: ast_nodes.FunctionDecl) -> str:
  """Creates embind compatible return type."""

  if (
      isinstance(func.return_type, ast_nodes.PointerType)
      and isinstance(func.return_type.inner_type, ast_nodes.ValueType)
      and func.return_type.inner_type.name in ["char", "mjString"]
  ):
    return "std::string"
  if (
      isinstance(func.return_type, ast_nodes.PointerType)
      and isinstance(func.return_type.inner_type, ast_nodes.ValueType)
      and func.return_type.inner_type.name not in PRIMITIVE_TYPES
  ):
    const_qualifier = get_const_qualifier(func)
    return f"""{const_qualifier}std::optional<{uppercase_first_letter(func.return_type.inner_type.name)}>"""
  if (
      isinstance(func.return_type, ast_nodes.ValueType)
      and func.return_type.name in PRIMITIVE_TYPES
  ):
    return f"{func.return_type.name}"
  return "val"


def get_converted_struct_to_class(
    func: ast_nodes.FunctionDecl, invoker: str
) -> str:
  """Generates a C++ function invocation for a struct return-type function."""

  const_qualifier = get_const_qualifier(func)
  return_type = cast(ast_nodes.PointerType, func.return_type)
  struct_name = cast(ast_nodes.ValueType, return_type.inner_type).name
  class_constructor = uppercase_first_letter(struct_name)
  return_str = f"{class_constructor}(result)"
  return f"""{const_qualifier}{struct_name}* result = {invoker};
  if (result == nullptr) {{
    return std::nullopt;
  }}
  return {return_str}"""


def is_excluded_function_name(func_name: str) -> bool:
  """Checks if a function name should be excluded from direct binding."""
  return (
      func_name.startswith("mjr_")
      or func_name.startswith("mjui_")
      or func_name in constants.SKIPPED_FUNCTIONS
  )


class Generator:
  """Generates Embind bindings for MuJoCo functions."""

  def __init__(self, functions: Mapping[str, ast_nodes.FunctionDecl]):
    self.direct_bind_functions: List[ast_nodes.FunctionDecl] = []
    self.wrapper_bind_functions: List[ast_nodes.FunctionDecl] = []

    for func in functions.values():
      if should_be_wrapped(func):
        self.wrapper_bind_functions.append(func)
      else:
        self.direct_bind_functions.append(func)

  def _generate_wrappers(self) -> str:
    """Generates Embind bindings for all functions that need wrappers."""

    code = []
    for func in self.wrapper_bind_functions:
      wrapper_code = generate_function_wrapper(func)
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
      self, func: ast_nodes.FunctionDecl, is_wrapper=False
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

  def generate(self) -> list[tuple[str, list[str]]]:
    """Generates the bindings file for all functions."""

    wrapper_functions = self._generate_wrappers()
    function_bindings = self._generate_direct_bindable_functions()
    function_bindings += self._generate_wrapper_bindable_functions()

    return [
        ("// {{ WRAPPER_FUNCTIONS }}", [wrapper_functions]),
        ("// {{ FUNCTION_BINDINGS }}", [function_bindings]),
    ]
