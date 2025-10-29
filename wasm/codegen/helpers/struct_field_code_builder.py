"""Class to build the C++ code for a struct field wrapper."""

from introspect import ast_nodes
from wasm.codegen.helpers import code_builder

StructFieldDecl = ast_nodes.StructFieldDecl
ValueType = ast_nodes.ValueType


def build_primitive_type_definition(field: StructFieldDecl) -> str:
  """Builds the C++ code for a primitive type field wrapper."""
  if not isinstance(field.type, ValueType):
    raise ValueError(f"{field.type} must be ValueType.")
  builder = code_builder.CodeBuilder()
  # build getter for primitive type field
  with builder.block(f"{field.type.name} {field.name}() const"):
    builder.line(f"return ptr_->{field.name};")
  # build setter for primitive type field
  with builder.block(f"void set_{field.name}({field.type.name} value)"):
    builder.line(f"ptr_->{field.name} = value;")
  return builder.to_string()


def build_memory_view_definition(
    field: StructFieldDecl, array_size_str: str, ptr_expr: str
) -> str:
  """Builds the C++ code for a pointer type field wrapper."""
  builder = code_builder.CodeBuilder()
  with builder.block(f"emscripten::val {field.name}() const"):
    builder.line(
        "return"
        f" emscripten::val(emscripten::typed_memory_view({array_size_str},"
        f" {ptr_expr}));"
    )
  return builder.to_string()


def build_string_field_definition(field: StructFieldDecl) -> str:
  """Builds the C++ code for a string type field wrapper."""
  builder = code_builder.CodeBuilder()
  with builder.block(f"mjString {field.name}() const"):
    builder.line(
        f'return (ptr_ && ptr_->{field.name}) ? *(ptr_->{field.name}) : "";'
    )
  with builder.block(f"void set_{field.name}(const mjString& value)"):
    with builder.block(f"if (ptr_ && ptr_->{field.name})"):
      builder.line(f"*(ptr_->{field.name}) = value;")
  return builder.to_string()


def build_mjvec_pointer_definition(
    field: StructFieldDecl, vector_type: str
) -> str:
  """Builds the C++ code for a mjVec type field wrapper."""
  ptr_field_expr = f"*(ptr_->{field.name})"
  if vector_type == "mjByteVec":
    vector_type = "std::vector<uint8_t>"
    ptr_field_expr = (
        f"*(reinterpret_cast<std::vector<uint8_t>*>(ptr_->{field.name}))"
    )
  builder = code_builder.CodeBuilder()
  with builder.block(f"{vector_type} &{field.name}() const"):
    builder.line(f"return {ptr_field_expr};")
  return builder.to_string()


def build_simple_property_binding(
    field: StructFieldDecl,
    struct_wrapper_name: str,
    add_setter: bool = False,
    add_return_value_policy_as_ref: bool = False,
) -> str:
  """Builds the C++ code for a simple property binding."""
  builder = code_builder.CodeBuilder()
  setter_txt = ""
  if add_setter:
    setter_txt = f", &{struct_wrapper_name}::set_{field.name}"
  if add_return_value_policy_as_ref:
    as_reference_txt = ", reference()"
  else:
    as_reference_txt = ""
  builder.line(
      f'.property("{field.name}",'
      f" &{struct_wrapper_name}::{field.name}{setter_txt}{as_reference_txt})"
  )
  return builder.to_string()
