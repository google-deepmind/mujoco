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

"""Parser for MuJoCo structs."""

import collections
import dataclasses
import math
from typing import Dict, List, Tuple, Union, cast

from introspect import ast_nodes
from introspect import structs

from wasm.codegen.helpers import code_builder
from wasm.codegen.helpers import common
from wasm.codegen.helpers import constants


debug_print = common.debug_print

introspect_structs = structs.STRUCTS


@dataclasses.dataclass
class WrappedFieldData:
  """Data class for struct field definition and binding."""

  # Line for struct field binding
  binding: str

  # Line for struct field definition
  definition: str | None = None

  # Initialization code for fields that require it
  initialization: str | None = None

  # Statement to reset the inner pointer when copying the field
  ptr_copy_reset: str | None = None

  # Whether the field is a primitive or fixed size
  is_primitive_or_fixed_size: bool = False

  # Underlying type of the field
  typename: str | None = None


@dataclasses.dataclass
class WrappedStructData:
  """Data class for struct wrapper definition and binding."""

  # Name of wrapper struct
  wrap_name: str

  # List of WrappedFieldData for this struct
  wrapped_fields: List[WrappedFieldData]

  # Struct header code
  wrapped_header: str

  # Struct source code
  wrapped_source: str

  # Whether to use shallow copy for this struct
  use_shallow_copy: bool = True


def build_primitive_type_definition(field: ast_nodes.StructFieldDecl) -> str:
  """Builds the C++ getter/setter code for a primitive type field wrapper."""
  if not isinstance(field.type, ast_nodes.ValueType):
    raise ValueError(f"{field.type} must be ValueType.")

  builder = code_builder.CodeBuilder()
  with builder.function(f"{field.type.name} {field.name}() const"):
    builder.line(f"return ptr_->{field.name};")
  with builder.function(f"void set_{field.name}({field.type.name} value)"):
    builder.line(f"ptr_->{field.name} = value;")
  return builder.to_string()


def build_memory_view_definition(
    field: ast_nodes.StructFieldDecl, array_size_str: str, ptr_expr: str
) -> str:
  """Builds the C++ code for a pointer type field wrapper."""
  builder = code_builder.CodeBuilder()
  with builder.function(f"emscripten::val {field.name}() const"):
    builder.line(
        "return"
        f" emscripten::val(emscripten::typed_memory_view({array_size_str},"
        f" {ptr_expr}));"
    )
  return builder.to_string()


def build_string_field_definition(field: ast_nodes.StructFieldDecl) -> str:
  """Builds the C++ code getter/setter for a string type field wrapper."""
  builder = code_builder.CodeBuilder()
  with builder.function(f"mjString {field.name}() const"):
    builder.line(
        f'return (ptr_ && ptr_->{field.name}) ? *(ptr_->{field.name}) : "";'
    )
  with builder.function(f"void set_{field.name}(const mjString& value)"):
    with builder.block(f"if (ptr_ && ptr_->{field.name})"):
      builder.line(f"*(ptr_->{field.name}) = value;")
  return builder.to_string()


def build_mjvec_pointer_definition(
    field: ast_nodes.StructFieldDecl, vector_type: str
) -> str:
  """Builds the C++ code for a mjVec type field wrapper."""
  ptr_field_expr = f"*(ptr_->{field.name})"
  if vector_type == "mjByteVec":
    vector_type = "std::vector<uint8_t>"
    ptr_field_expr = (
        f"*(reinterpret_cast<std::vector<uint8_t>*>(ptr_->{field.name}))"
    )
  builder = code_builder.CodeBuilder()
  with builder.function(f"{vector_type} &{field.name}() const"):
    builder.line(f"return {ptr_field_expr};")
  return builder.to_string()


def build_simple_property_binding(
    field: ast_nodes.StructFieldDecl,
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


class StructFieldHandler:
  """Class to handle the different struct field types, and provide the c++ code for the definitions and bindings."""

  def __init__(
      self,
      field: ast_nodes.StructFieldDecl,
      struct_wrapper_name: str,
  ):
    self.field = field
    self.struct_wrapper_name = struct_wrapper_name
    self.simple_property_binding = build_simple_property_binding(
        self.field, self.struct_wrapper_name
    )
    self.manually_added_fields = (
        constants.MANUALLY_ADDED_FIELDS_FROM_TEMPLATE.get(
            self.struct_wrapper_name, {}
        )
    )

  def generate(self) -> WrappedFieldData:
    """Generates the C++ definition and binding code for the struct field."""
    field_type = self.field.type
    if isinstance(field_type, ast_nodes.ValueType) and (
        field_type.name in constants.PRIMITIVE_TYPES
        or field_type.name.startswith("mjt")
    ):
      return self._handle_primitive()
    elif isinstance(field_type, ast_nodes.PointerType):
      return self._handle_pointer()
    elif isinstance(field_type, ast_nodes.ArrayType):
      return self._handle_array()
    elif isinstance(
        field_type, ast_nodes.ValueType
    ) and field_type.name.startswith("mj"):
      return self._handle_mj_struct()
    elif isinstance(field_type, ast_nodes.AnonymousStructDecl):
      return self._handle_anonymous_struct()
    return self._undefined()

  def _handle_primitive(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for primitive fields."""
    return WrappedFieldData(
        definition=(build_primitive_type_definition(self.field)),
        typename=_get_field_struct_type(self.field.type),
        binding=build_simple_property_binding(
            self.field,
            self.struct_wrapper_name,
            add_setter=True,
            add_return_value_policy_as_ref=True,
        ),
        is_primitive_or_fixed_size=True,
    )

  def _handle_pointer(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for pointer fields."""
    if not isinstance(self.field.type, ast_nodes.PointerType):
      raise ValueError(
          f"Expected PointerType, got {type(self.field.type)} for field"
          f" {self.field.name}"
      )
    field_type: ast_nodes.PointerType = self.field.type
    inner_type_name = (
        field_type.inner_type.name
        if isinstance(field_type.inner_type, ast_nodes.ValueType)
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
      ptr_field_expr = f"static_cast<uint8_t*>({ptr_field_expr})"
      # for these byte fields, there is no array_extent, so we add the size of
      # in the config file based in the documentation
      extent = (constants.BYTE_FIELDS[self.field.name]["size"],)
      array_size_str = parse_array_extent(
          extent, self.struct_wrapper_name, self.field.name
      )
    elif inner_type_name == "mjString":
      return WrappedFieldData(
          definition=build_string_field_definition(self.field),
          typename=_get_field_struct_type(self.field.type),
          binding=build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=True,
              add_return_value_policy_as_ref=True,
          ),
      )
    elif inner_type_name.startswith("mj") and inner_type_name.endswith("Vec"):
      return WrappedFieldData(
          definition=build_mjvec_pointer_definition(
              self.field, inner_type_name
          ),
          typename=_get_field_struct_type(self.field.type),
          binding=build_simple_property_binding(
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
        ptr_field = cast(ast_nodes.PointerType, self.field.type)
        wrapper_field_name = common.uppercase_first_letter(
            cast(ast_nodes.ValueType, ptr_field.inner_type).name
        )
        return WrappedFieldData(
            definition=f"{wrapper_field_name} {self.field.name};",
            typename=_get_field_struct_type(self.field.type),
            binding=build_simple_property_binding(
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
        typename=_get_field_struct_type(self.field.type),
        definition=(
            build_memory_view_definition(
                self.field, array_size_str, ptr_field_expr
            )
        ),
        binding=self.simple_property_binding,
    )

  def _handle_array(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for array fields."""
    field_type = self.field.type
    if not isinstance(field_type, ast_nodes.ArrayType):
      raise ValueError(
          f"Expected ArrayType, got {type(field_type)} for field"
          f" {self.field.name}"
      )
    inner_type = field_type.inner_type
    size = math.prod(field_type.extents)

    if isinstance(inner_type, ast_nodes.ValueType):
      if inner_type.name in constants.PRIMITIVE_TYPES:
        ptr_expr = f"ptr_->{self.field.name}"
        if len(field_type.extents) > 1:
          # for multi-dimensional arrays, we need to cast the field
          # to a pointer, so embind can correctly interpret the memory
          # view
          ptr_expr = f"reinterpret_cast<{inner_type.name}*>({ptr_expr})"
        return WrappedFieldData(
            definition=(
                build_memory_view_definition(self.field, str(size), ptr_expr)
            ),
            typename=_get_field_struct_type(self.field.type),
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
        typename=_get_field_struct_type(self.field.type),
        binding=f"// TODO: NOT IMPLEMENTED ARRAY binding for {self.field.name}",
    )

  def _handle_mj_struct(self) -> WrappedFieldData:
    """Handles the generation of C++ definition and binding code for mj struct fields."""
    if (
        isinstance(self.field.type, ast_nodes.ValueType)
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
          typename=_get_field_struct_type(self.field.type),
          binding=build_simple_property_binding(
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
        isinstance(self.field.type, ast_nodes.AnonymousStructDecl)
        and self.field.name not in self.manually_added_fields
        and anonymous_struct_name in constants.STRUCTS_TO_BIND
    ):
      return WrappedFieldData(
          binding=build_simple_property_binding(
              self.field,
              self.struct_wrapper_name,
              add_setter=False,
              add_return_value_policy_as_ref=True,
          ),
          typename=_get_field_struct_type(self.field.type),
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
          typename=_get_field_struct_type(self.field.type),
          definition=(
              f"// {comment_type} is defined manually. {self.field.name}"
          ),
          binding=self.simple_property_binding,
      )
    return WrappedFieldData(
        typename=_get_field_struct_type(self.field.type),
        definition=(
            f"// TODO: Define {comment_type} manually for {self.field.name}"
        ),
        binding=f"// TODO: {self.simple_property_binding}",
    )


def _has_nested_wrapper_members(struct_info: ast_nodes.StructDecl) -> bool:
  """Checks if the struct contains other wrapped structs as direct members."""
  for field in struct_info.fields:
    struct_field = cast(ast_nodes.StructFieldDecl, field)
    if isinstance(struct_field.type, ast_nodes.ValueType):
      if struct_field.type.name in constants.STRUCTS_TO_BIND:
        return True
    if isinstance(struct_field.type, ast_nodes.ArrayType):
      if isinstance(struct_field.type.inner_type, ast_nodes.ValueType):
        if struct_field.type.inner_type.name in constants.STRUCTS_TO_BIND:
          return True
    if isinstance(struct_field.type, ast_nodes.PointerType):
      if isinstance(struct_field.type.inner_type, ast_nodes.ValueType):
        if struct_field.type.inner_type.name in constants.STRUCTS_TO_BIND:
          return True
  return False


def _build_struct_header_internal(
    struct_name: str,
    wrapped_fields: List[WrappedFieldData],
    fields_with_init: List[WrappedFieldData],
    is_mjs: bool = False,
):
  """Builds the C++ header file code for a struct."""
  s = struct_name
  w = common.uppercase_first_letter(s)

  shallow_copy = use_shallow_copy(wrapped_fields)

  builder = code_builder.CodeBuilder()
  with builder.struct(f"{w}"):
    builder.line(f"explicit {w}({s} *ptr);")
    builder.line(f"~{w}();")

    if not is_mjs:
      builder.line(f"{w}();")

    if shallow_copy and not is_mjs:
      builder.line(f"{w}(const {w} &);")
      builder.line(f"{w} &operator=(const {w} &);")
      builder.line(f"std::unique_ptr<{w}> copy();")

    builder.line(f"{s}* get() const;")
    builder.line(f"void set({s}* ptr);")

    for field in wrapped_fields:
      if field.definition and field not in fields_with_init:
        for line in field.definition.splitlines():
          builder.line(line)

    builder.private()
    builder.line(f"{s}* ptr_;")
    if not is_mjs:
      builder.line("bool owned_ = false;")

    if is_mjs and fields_with_init:
      builder.public()
      for field in fields_with_init:
        if field.definition:
          builder.line(f"{field.definition}")

  return builder.to_string() + ";"


def _default_function_statement(struct_name: str) -> str:
  """Returns the default function name for the given struct."""
  if (
      struct_name in constants.ANONYMOUS_STRUCTS.keys()
      or struct_name in constants.NO_DEFAULT_CONSTRUCTORS
      or (
          common.uppercase_first_letter(struct_name)
          in constants.MANUALLY_ADDED_FIELDS_FROM_TEMPLATE.keys()
      )
  ):
    return ""
  elif struct_name.startswith("mjs"):
    return f"mjs_default{struct_name.removeprefix('mjs')}(ptr_);"
  elif struct_name == "mjvGeom":
    return (
        "mjv_initGeom(ptr_, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);"
    )
  elif struct_name.startswith("mjv"):
    return f"mjv_default{struct_name.removeprefix('mjv')}(ptr_);"
  else:
    return f"mj_default{struct_name.removeprefix('mj')}(ptr_);"


def _delete_ptr_statement(struct_name: str) -> str:
  """Returns the delete function name for the given struct."""
  if struct_name == "mjVFS":
    return "mj_deleteVFS(ptr_);"
  else:
    return "delete ptr_;"


def _find_fields_with_init(
    wrapped_fields: List[WrappedFieldData],
) -> List[WrappedFieldData]:
  """Finds the fields with initialization in the wrapped fields list."""
  fields_with_init = []
  for field in wrapped_fields:
    if field.initialization:
      fields_with_init.append(field)
  return fields_with_init


def use_shallow_copy(
    wrapped_fields: List[WrappedFieldData],
) -> bool:
  """Returns true if the struct fields can be shallow copied."""
  for field in wrapped_fields:
    if not field.is_primitive_or_fixed_size:
      return False
  return True


def build_struct_header(
    struct_name: str,
    wrapped_fields: List[WrappedFieldData],
):
  """Builds the C++ header file code for a struct."""
  struct_info = introspect_structs.get(struct_name)

  if struct_name.startswith("mjs"):
    fields_with_init = _find_fields_with_init(wrapped_fields)
    return _build_struct_header_internal(
        struct_name,
        wrapped_fields,
        fields_with_init,
        is_mjs=True,
    )

  if (
      (
          common.uppercase_first_letter(struct_name)
          not in constants.HARDCODED_WRAPPER_STRUCTS
      )
      and struct_info
      and not _has_nested_wrapper_members(struct_info)
  ):
    return _build_struct_header_internal(
        struct_name, wrapped_fields, [], is_mjs=False
    )
  return ""


def build_struct_source(
    struct_name: str,
    wrapped_fields: List[WrappedFieldData],
):
  """Builds the C++ .cc file code for a struct."""
  # These structs require specific function calls for creation and/or deletion
  # which, for now, are hardcoded in the template file.
  if struct_name in [
      "mjData",
      "mjModel",
      "mjvScene",
      "mjSpec",
  ]:
    return ""

  s = struct_name
  w = common.uppercase_first_letter(s)
  is_mjs = "Mjs" in w

  fields_with_init = _find_fields_with_init(wrapped_fields)
  shallow_copy = use_shallow_copy(wrapped_fields)

  fields_init = ""
  if fields_with_init:
    fields_init = "".join(
        field_with_init.initialization for field_with_init in fields_with_init
    )

  builder = code_builder.CodeBuilder()

  # constructor passing native ptr
  with builder.function(f"{w}::{w}({s} *ptr) : ptr_(ptr){fields_init}"):
    pass

  # destructor
  with builder.function(f"{w}::~{w}()"):
    if not is_mjs:
      with builder.block("if (owned_ && ptr_)"):
        delete_ptr = _delete_ptr_statement(s)
        builder.line(delete_ptr)

  if not is_mjs:
    # default constructor
    with builder.function(f"{w}::{w}() : ptr_(new {s}){fields_init}"):
      builder.line("owned_ = true;")
      default_func = _default_function_statement(s)
      if default_func:
        builder.line(default_func)

  if shallow_copy and not is_mjs:
    # copy constructor
    with builder.function(f"{w}::{w}(const {w} &other) : {w}()"):
      builder.line("*ptr_ = *other.get();")
      for field_with_init in fields_with_init:
        if field_with_init.ptr_copy_reset is not None:
          builder.line(field_with_init.ptr_copy_reset)

    # assignment operator
    with builder.function(f"{w}& {w}::operator=(const {w} &other)"):
      with builder.block("if (this == &other)"):
        builder.line("return *this;")
      builder.line("*ptr_ = *other.get();")
      for field_with_init in fields_with_init:
        if field_with_init.ptr_copy_reset is not None:
          builder.line(field_with_init.ptr_copy_reset)
      builder.line("return *this;")

    # explicit copy function
    with builder.function(f"std::unique_ptr<{w}> {w}::copy()"):
      builder.line(f"return std::make_unique<{w}>(*this);")

  # C struct getter/setter
  with builder.function(f"{s}* {w}::get() const"):
    builder.line("return ptr_;")
  with builder.function(f"void {w}::set({s}* ptr)"):
    builder.line("ptr_ = ptr;")

  return builder.to_string()


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


def generate_wasm_bindings(
    structs_to_bind: List[str],
) -> Dict[str, WrappedStructData]:
  """Generates WASM bindings for MuJoCo structs."""

  wrapped_structs: Dict[str, WrappedStructData] = {}
  for struct_name in structs_to_bind:
    wrapped_name = common.uppercase_first_letter(struct_name)

    if struct_name in introspect_structs:
      struct_fields = introspect_structs[struct_name].fields
    elif struct_name in constants.ANONYMOUS_STRUCTS:
      anonymous_struct = _get_anonymous_struct_field(struct_name)
      if not anonymous_struct or not isinstance(
          anonymous_struct.type, ast_nodes.AnonymousStructDecl
      ):
        raise RuntimeError(f"Anonymous struct not found: {struct_name}")
      struct_fields = anonymous_struct.type.fields
    else:
      raise RuntimeError(f"Struct not found: {struct_name}")

    debug_print(f"Wrapping struct: {struct_name}")

    wrapped_fields: List[WrappedFieldData] = []
    for field in struct_fields:
      wrapped_field = StructFieldHandler(field, wrapped_name).generate()
      wrapped_fields.append(wrapped_field)

    wrapped_header = build_struct_header(
        struct_name,
        wrapped_fields,
    )
    wrapped_source = build_struct_source(
        struct_name,
        wrapped_fields,
    )
    wrap_data = WrappedStructData(
        wrap_name=wrapped_name,
        wrapped_fields=wrapped_fields,
        wrapped_header=wrapped_header,
        wrapped_source=wrapped_source,
        use_shallow_copy=use_shallow_copy(wrapped_fields),
    )

    wrapped_structs[struct_name] = wrap_data

  return wrapped_structs


def _get_anonymous_struct_field(
    anonymous_structs_key: str,
) -> ast_nodes.StructFieldDecl | None:
  """Looks up the given key in the anonymous_structs dict and generates bindings for its fields."""
  info = constants.ANONYMOUS_STRUCTS[anonymous_structs_key]
  parent_decl = introspect_structs[info["parent"]]
  target_field = next(
      (
          f
          for f in parent_decl.fields
          if hasattr(f, "name")
          and f.name == info["field_name"]
          and hasattr(f, "type")
          and isinstance(f.type, ast_nodes.AnonymousStructDecl)
      ),
      None,
  )
  return target_field


def _get_field_struct_type(field_type):
  """Extracts the base struct name if the field type is a struct or pointer to a struct."""
  if isinstance(field_type, ast_nodes.ValueType):
    return field_type.name
  if isinstance(field_type, ast_nodes.PointerType):
    if isinstance(field_type.inner_type, ast_nodes.ValueType):
      return field_type.inner_type.name
  return None


def sort_structs_by_dependency(
    struct_wrappers: dict[str, WrappedStructData],
) -> List[str]:
  """Sorts structs based on their field dependencies using topological sort.

  Structs with no dependencies on other structs in the list come first.
  Struct A has a dependency on struct B if struct A has a field where the
  underlying_type is B. Note that this definition is stricter than the C++
  struct dependency criterion where forward declarations can be used to
  eliminate dependencies A and B if A only has a pointer to B.

  Args:
    struct_wrappers: A dictionary mapping struct names to their
      WrappedStructData.

  Returns:
    A new list of struct names sorted by dependency.

  Raises:
    RuntimeError: If a cyclic dependency is detected.
  """
  adj = collections.defaultdict(list)
  in_degree = collections.defaultdict(int)
  struct_names = struct_wrappers.keys()
  struct_set = set(struct_names)
  sorted_struct_names = sorted(struct_names)

  for struct_name in sorted_struct_names:
    for field in struct_wrappers[struct_name].wrapped_fields:

      field_type_name = field.typename
      if (
          field_type_name
          and field_type_name != struct_name
          and field_type_name in struct_set
      ):
        if struct_name not in adj[field_type_name]:
          adj[field_type_name].append(struct_name)
          in_degree[struct_name] += 1

  queue = collections.deque(
      [name for name in sorted_struct_names if in_degree[name] == 0]
  )
  sorted_list = []

  while queue:
    u = queue.popleft()
    sorted_list.append(u)
    for v in adj[u]:
      in_degree[v] -= 1
      if in_degree[v] == 0:
        queue.append(v)

  if len(sorted_list) == len(struct_names):
    return sorted_list
  else:
    remaining = set(struct_names) - set(sorted_list)
    raise RuntimeError(
        "Cycle detected in struct dependencies, involving: "
        f"{', '.join(sorted(list(remaining)))}"
    )
