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

"""Class to handle the different struct field types, and provide the c++ code for the wrappers and bindings."""

import math
from typing import Tuple, Union, cast
from introspect import ast_nodes
from wasm.codegen.helpers import common
from wasm.codegen.helpers import constants
from wasm.codegen.helpers import struct_field_code_builder
from wasm.codegen.helpers import structs_wrappers_data

AnonymousStructDecl = ast_nodes.AnonymousStructDecl
ArrayType = ast_nodes.ArrayType
PointerType = ast_nodes.PointerType
StructFieldDecl = ast_nodes.StructFieldDecl
ValueType = ast_nodes.ValueType
WrappedFieldData = structs_wrappers_data.WrappedFieldData

debug_print = common.debug_print


class StructFieldHandler:
  """Class to handle the different struct field types, and provide the c++ code for the definitions and bindings."""

  def __init__(
      self,
      field: StructFieldDecl,
      struct_wrapper_name: str,
  ):
    self.field = field
    self.struct_wrapper_name = struct_wrapper_name
    self.simple_property_binding = (
        struct_field_code_builder.build_simple_property_binding(
            self.field, self.struct_wrapper_name
        )
    )
    self.manually_added_fields = (
        constants.MANUALLY_ADDED_FIELDS_FROM_TEMPLATE.get(
            self.struct_wrapper_name, {}
        )
    )

  def generate(self) -> WrappedFieldData:
    """Generates the C++ definition and binding code for the struct field."""
    field_type = self.field.type
    if isinstance(field_type, ValueType) and (
        field_type.name in constants.PRIMITIVE_TYPES
        or field_type.name.startswith("mjt")
    ):
      return self._handle_primitive()
    elif isinstance(field_type, PointerType):
      return self._handle_pointer()
    elif isinstance(field_type, ArrayType):
      return self._handle_array()
    elif isinstance(field_type, ValueType) and field_type.name.startswith("mj"):
      return self._handle_mj_struct()
    elif isinstance(field_type, AnonymousStructDecl):
      return self._handle_anonymous_struct()
    return self._undefined()

  def _handle_primitive(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for primitive fields."""
    return WrappedFieldData(
        definition=(
            struct_field_code_builder.build_primitive_type_definition(
                self.field
            )
        ),
        binding=struct_field_code_builder.build_simple_property_binding(
            self.field,
            self.struct_wrapper_name,
            add_setter=True,
            add_return_value_policy_as_ref=True,
        ),
        is_primitive_or_fixed_size=True,
    )

  def _handle_pointer(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for pointer fields."""
    if not isinstance(self.field.type, PointerType):
      raise ValueError(
          f"Expected PointerType, got {type(self.field.type)} for field"
          f" {self.field.name}"
      )
    field_type: PointerType = self.field.type
    inner_type_name = (
        field_type.inner_type.name
        if isinstance(field_type.inner_type, ValueType)
        else ""
    )
    ptr_field_expr = f"ptr_->{self.field.name}"
    array_size_str = ""

    if self.field.array_extent:
      array_size_str = parse_array_extent(
          self.field.array_extent, self.struct_wrapper_name, self.field.name
      )
    elif self.field.name in constants.BYTE_FIELDS.keys():
      # for byte fields, we need to cast the pointer to uint8_t*
      # so embind can correctly interpret the memory view
      ptr_field_expr = (
          f"static_cast<uint8_t*>({ptr_field_expr})"
      )
      # for these byte fields, there is no array_extent, so we add the size of
      # in the config file based in the documentation
      extent = (constants.BYTE_FIELDS[self.field.name]["size"],)
      array_size_str = parse_array_extent(
          extent, self.struct_wrapper_name, self.field.name
      )
    elif inner_type_name == "mjString":
      return WrappedFieldData(
          definition=struct_field_code_builder.build_string_field_definition(
              self.field
          ),
          binding=struct_field_code_builder.build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=True,
              add_return_value_policy_as_ref=True,
          ),
      )
    elif inner_type_name.startswith("mj") and inner_type_name.endswith("Vec"):
      return WrappedFieldData(
          definition=struct_field_code_builder.build_mjvec_pointer_definition(
              self.field, inner_type_name
          ),
          binding=struct_field_code_builder.build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=False,
              add_return_value_policy_as_ref=True,
          ),
      )
    elif inner_type_name in constants.PRIMITIVE_TYPES:
      return self._get_manual_definition(
          comment_type="primitive pointer field with complex extents"
      )

    if (
        inner_type_name.startswith("mj")
        and inner_type_name not in constants.PRIMITIVE_TYPES
    ):
      debug_print(
          f"\tcomplex pointer type: needs manual wrapper: {self.field.name}"
      )
      # it's a pointer to a single struct,
      # like the `element` field in mjs structs
      # and the struct is not manually added
      if (
          not self.field.array_extent
          and self.struct_wrapper_name
          not in constants.MANUALLY_ADDED_FIELDS_FROM_TEMPLATE.keys()
      ):
        ptr_field = cast(PointerType, self.field.type)
        wrapper_field_name = common.uppercase_first_letter(
            cast(ValueType, ptr_field.inner_type).name
        )
        return WrappedFieldData(
            definition=f"{wrapper_field_name} {self.field.name};",
            binding=struct_field_code_builder.build_simple_property_binding(
                self.field,
                self.struct_wrapper_name,
                add_setter=False,
                add_return_value_policy_as_ref=True,
            ),
            initialization=f", {self.field.name}(ptr_->{self.field.name})",
        )
      else:
        debug_print(
            "\tcomplex pointer type with array extent: needs manual wrapper:"
            f" {self.field.name}"
        )
        return self._get_manual_definition(comment_type="complex pointer field")

    return WrappedFieldData(
        definition=(
            struct_field_code_builder.build_memory_view_definition(
                self.field, array_size_str, ptr_field_expr
            )
        ),
        binding=self.simple_property_binding,
    )

  def _handle_array(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for array fields."""
    field_type = self.field.type
    if not isinstance(field_type, ArrayType):
      raise ValueError(
          f"Expected ArrayType, got {type(field_type)} for field"
          f" {self.field.name}"
      )
    inner_type = field_type.inner_type
    size = math.prod(field_type.extents)

    if isinstance(inner_type, ValueType):
      if inner_type.name in constants.PRIMITIVE_TYPES:
        ptr_expr = f"ptr_->{self.field.name}"
        if len(field_type.extents) > 1:
          # for multi-dimensional arrays, we need to cast the field
          # to a pointer, so embind can correctly interpret the memory
          # view
          ptr_expr = f"reinterpret_cast<{inner_type.name}*>({ptr_expr})"
        return WrappedFieldData(
            definition=(
                struct_field_code_builder.build_memory_view_definition(
                    self.field, str(size), ptr_expr
                )
            ),
            binding=self.simple_property_binding,
            is_primitive_or_fixed_size=True,
        )
      elif inner_type.name.startswith("mj") and not inner_type.name.startswith(
          "mjt"
      ):
        debug_print(f"\tarray to vector wrapper needed: {self.field.name}")
        return self._get_manual_definition(comment_type="array field")

    debug_print(f"\tNOT IMPLEMENTED ARRAY field: {self.field.name}")
    return WrappedFieldData(
        definition=(
            f"// TODO: NOT IMPLEMENTED ARRAY wrapper for {self.field.name}"
        ),
        binding=f"// TODO: NOT IMPLEMENTED ARRAY binding for {self.field.name}",
    )

  def _handle_mj_struct(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for mj struct fields."""
    if (
        isinstance(self.field.type, ValueType)
        and self.field.name not in self.manually_added_fields
        and self.field.type.name in constants.STRUCTS_TO_BIND
    ):
      # TODO(manevi): Find a better way to do this instead of checking the
      # struct wrapper name.
      definition = ""
      if self.struct_wrapper_name not in constants.HARDCODED_WRAPPER_STRUCTS:
        wrapper_field_name = common.uppercase_first_letter(self.field.type.name)
        definition = f"{wrapper_field_name} {self.field.name};"
      return WrappedFieldData(
          definition=definition,
          binding=struct_field_code_builder.build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=False,
              add_return_value_policy_as_ref=True,
          ),
          initialization=f", {self.field.name}(&ptr_->{self.field.name})",
          ptr_copy_reset=f"{self.field.name}.set(&ptr_->{self.field.name});",
          is_primitive_or_fixed_size=True,
      )
    return self._get_manual_definition(comment_type="struct field")

  def _handle_anonymous_struct(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for anonymous struct fields."""

    anonymous_struct_name = ""
    for name, value in constants.ANONYMOUS_STRUCTS.items():
      if (
          common.uppercase_first_letter(value["parent"])
          == self.struct_wrapper_name
          and value["field_name"] == self.field.name
      ):
        anonymous_struct_name = name
        break

    if (
        isinstance(self.field.type, AnonymousStructDecl)
        and self.field.name not in self.manually_added_fields
        and anonymous_struct_name in constants.STRUCTS_TO_BIND
    ):
      return WrappedFieldData(
          binding=struct_field_code_builder.build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=False,
              add_return_value_policy_as_ref=True,
          ),
          initialization=f", {self.field.name}(&ptr_->{self.field.name})",
          ptr_copy_reset=f"{self.field.name}.set(&ptr_->{self.field.name});",
          is_primitive_or_fixed_size=True,
      )
    return self._get_manual_definition(comment_type="anonymous struct field")

  def _undefined(self) -> WrappedFieldData:
    """This function adds a TODO comment for fields that are not handled by this class yet."""
    return WrappedFieldData(
        definition=f"// TODO: UNDEFINED definition for {self.field.name}",
        binding=f"// TODO: UNDEFINED binding for {self.field.name}",
    )

  def _get_manual_definition(self, comment_type: str = "") -> WrappedFieldData:
    """Helper method to generate a comment as a definition for manually added fields."""
    if self.field.name in self.manually_added_fields:
      return WrappedFieldData(
          definition=(
              f"// {comment_type} is defined manually. {self.field.name}"
          ),
          binding=self.simple_property_binding,
      )

    return WrappedFieldData(
        definition=(
            f"// TODO: Define {comment_type} manually for {self.field.name}"
        ),
        binding=f"// TODO: {self.simple_property_binding}",
    )


def parse_array_extent(
    extents: Tuple[Union[str, int], ...], wrapper_name: str, field_name: str
) -> str:
  """Parses the array extent of a field, returning a string representing the resolved extents."""
  if not extents:
    return ""
  return " * ".join(
      resolve_extent(extent, wrapper_name, field_name) for extent in extents
  )


def resolve_extent(
    extent: Union[str, int], wrapper_name: str, field_name: str
) -> str:
  """Resolves the extent of an array, handling integers and references to other struct fields.

  Args:
      extent: The extent to resolve, can be an int or a string referencing a
        field.
      wrapper_name: The name of the struct wrapper.
      field_name: The name of the field being processed.

  Returns:
      A string representing the resolved extent, either as a number or a field
      reference.
  """
  if isinstance(extent, int):
    return str(extent)
  # if starts with mj, it's a mujoco constant,
  # so we don't need to get a parent struct ptr
  if extent.startswith("mj"):
    return str(extent)
  if wrapper_name == "MjData" and field_name not in constants.MJDATA_SIZES:
    var_name = "model"
  else:
    var_name = "ptr_"
  return f"{var_name}->{extent}"
