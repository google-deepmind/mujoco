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

"""Generates Embind bindings for MuJoCo structs."""

import collections
import dataclasses
import math
from typing import Dict, List, Tuple, Union, cast

from introspect import ast_nodes
from introspect import structs as introspect_structs

from wasm.codegen.generators import code_builder
from wasm.codegen.generators import common
from wasm.codegen.generators import constants


@dataclasses.dataclass
class WrappedFieldData:
  """Data class for struct field definition and binding."""

  # Line for struct field binding
  binding: str = ""

  # Line for struct field definition
  definition: str = ""

  # Initialization code for fields that require it
  initialization: str = ""

  # Statement to reset the inner pointer when copying the field
  ptr_copy_reset: str = ""

  # Whether the field is a primitive or fixed size
  is_primitive_or_fixed_size: bool = False

  # Underlying type of the field. If non-empty, used to determine the order in
  # which structs are written in the bindings.h file.
  typename: str = ""


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

  # Struct bindings code
  bindings: str = ""


def _simple_property_binding(
    field: ast_nodes.StructFieldDecl,
    struct_wrapper_name: str,
    setter: bool = False,
    reference: bool = False,
) -> str:
  """Builds the C++ code for a simple property binding."""
  f = field
  w = struct_wrapper_name
  setter_txt = f", &{w}::set_{f.name}" if setter else ""
  reference_txt = ", reference()" if reference else ""
  return f'.property("{f.name}", &{w}::{f.name}{setter_txt}{reference_txt})'


def _generate_field_data(
    field: ast_nodes.StructFieldDecl, struct_wrapper_name: str
) -> WrappedFieldData:
  """Generates the C++ definition and binding code for the struct field."""
  f = field
  w = struct_wrapper_name
  s = common.lowercase_first_letter(w)

  if f.name in constants.MANUAL_FIELDS.get(w, []):
    # Note: Manually handled MjModel fields are special cased so that a
    # by-reference embind return value policy is used.
    return WrappedFieldData(
        typename=_get_field_struct_type(f.type),
        definition=f"// {f.name} field is handled manually in template file struct declaration",  # pylint: disable=line-too-long
        binding=_simple_property_binding(f, w, reference=(w == "MjModel")),
    )

  if f.name in constants.SKIPPED_FIELDS.get(w, []):
    return WrappedFieldData(
        typename="",
        definition=f"// {f.name} field is skipped.",
        binding=f"// {f.name} field is skipped.",
    )

  if isinstance(f.type, ast_nodes.ValueType) and (
      f.type.name in constants.PRIMITIVE_TYPES or f.type.name.startswith("mjt")
  ):

    builder = code_builder.CodeBuilder()
    with builder.function(f"{f.type.name} {f.name}() const"):
      builder.line(f"return ptr_->{f.name};")
    with builder.function(f"void set_{f.name}({f.type.name} value)"):
      builder.line(f"ptr_->{f.name} = value;")

    return WrappedFieldData(
        definition=builder.to_string(),
        typename=_get_field_struct_type(f.type),
        binding=_simple_property_binding(f, w, setter=True, reference=True),
        is_primitive_or_fixed_size=True,
    )

  elif isinstance(f.type, ast_nodes.ValueType) and f.type.name.startswith("mj"):

    return WrappedFieldData(
        definition=f"{common.uppercase_first_letter(f.type.name)} {f.name};",
        typename=_get_field_struct_type(f.type),
        binding=_simple_property_binding(f, w, setter=False, reference=True),
        initialization=f", {f.name}(&ptr_->{f.name})",
        ptr_copy_reset=f"{f.name}.set(&ptr_->{f.name});",
        is_primitive_or_fixed_size=True,
    )

  elif isinstance(f.type, ast_nodes.AnonymousStructDecl):

    anonymous_struct_name = ""
    for name, value in constants.ANONYMOUS_STRUCTS.items():
      if value["parent"] == s and value["field_name"] == f.name:
        anonymous_struct_name = name
        break

    if anonymous_struct_name in constants.STRUCTS_TO_BIND:
      return WrappedFieldData(
          binding=_simple_property_binding(f, w, setter=False, reference=True),
          typename=_get_field_struct_type(f.type),
          initialization=f", {f.name}(&ptr_->{f.name})",
          ptr_copy_reset=f"{f.name}.set(&ptr_->{f.name});",
          is_primitive_or_fixed_size=True,
      )

  elif isinstance(f.type, ast_nodes.ArrayType):

    inner_type = f.type.inner_type
    size = math.prod(f.type.extents)

    if (
        isinstance(inner_type, ast_nodes.ValueType)
        and inner_type.name in constants.PRIMITIVE_TYPES
    ):
      ptr_expr = f"ptr_->{f.name}"
      if len(f.type.extents) > 1:
        # for multi-dimensional arrays, we need to cast the field
        # to a pointer, so embind can correctly interpret the memory
        # view
        ptr_expr = f"reinterpret_cast<{inner_type.name}*>({ptr_expr})"

      builder = code_builder.CodeBuilder()
      with builder.function(f"emscripten::val {f.name}() const"):
        builder.line(
            "return"
            f" emscripten::val(emscripten::typed_memory_view({str(size)},"
            f" {ptr_expr}));"
        )

      return WrappedFieldData(
          definition=builder.to_string(),
          typename=_get_field_struct_type(f.type),
          binding=_simple_property_binding(f, w),
          is_primitive_or_fixed_size=True,
      )

  elif isinstance(f.type, ast_nodes.PointerType):

    inner_type_name = (
        f.type.inner_type.name
        if isinstance(f.type.inner_type, ast_nodes.ValueType)
        else ""
    )
    ptr_field_expr = f"ptr_->{f.name}"
    array_size_str = ""

    if f.array_extent:
      array_size_str = parse_array_extent(f.array_extent, w, f.name)
    elif f.name in constants.BYTE_FIELDS.keys():
      # for byte fields, we need to cast the pointer to uint8_t*
      # so embind can correctly interpret the memory view
      ptr_field_expr = f"static_cast<uint8_t*>({ptr_field_expr})"
      # for these byte fields, there is no array_extent, so we add the size of
      # in the config file based in the documentation
      extent = (constants.BYTE_FIELDS[f.name]["size"],)
      array_size_str = parse_array_extent(extent, w, f.name)
    elif inner_type_name == "mjString":

      builder = code_builder.CodeBuilder()
      with builder.function(f"mjString {f.name}() const"):
        builder.line(
            f'return (ptr_ && ptr_->{f.name}) ? *(ptr_->{f.name}) : "";'
        )
      with builder.function(f"void set_{f.name}(const mjString& value)"):
        with builder.block(f"if (ptr_ && ptr_->{f.name})"):
          builder.line(f"*(ptr_->{f.name}) = value;")

      return WrappedFieldData(
          definition=builder.to_string(),
          typename=_get_field_struct_type(f.type),
          binding=_simple_property_binding(f, w, setter=True, reference=True),
      )
    elif inner_type_name.startswith("mj") and inner_type_name.endswith("Vec"):
      ptr_field_expr_vec = f"*(ptr_->{f.name})"
      vector_type = inner_type_name
      if vector_type == "mjByteVec":
        vector_type = "std::vector<uint8_t>"
        ptr_field_expr_vec = (
            f"*(reinterpret_cast<std::vector<uint8_t>*>(ptr_->{f.name}))"
        )

      builder = code_builder.CodeBuilder()
      with builder.function(f"{vector_type} &{f.name}() const"):
        builder.line(f"return {ptr_field_expr_vec};")

      return WrappedFieldData(
          definition=builder.to_string(),
          typename=_get_field_struct_type(f.type),
          binding=_simple_property_binding(f, w, setter=False, reference=True),
      )

    if (
        inner_type_name.startswith("mj")
        and inner_type_name not in constants.PRIMITIVE_TYPES
        and not f.array_extent
        and w not in constants.MANUAL_FIELDS.keys()
    ):
      ptr_field = cast(ast_nodes.PointerType, f.type)
      wrapper_field_name = common.uppercase_first_letter(
          cast(ast_nodes.ValueType, ptr_field.inner_type).name
      )
      return WrappedFieldData(
          definition=f"{wrapper_field_name} {f.name};",
          typename=_get_field_struct_type(f.type),
          binding=_simple_property_binding(f, w, setter=False, reference=True),
          initialization=f", {f.name}(ptr_->{f.name})",
      )

    builder = code_builder.CodeBuilder()
    with builder.function(f"emscripten::val {f.name}() const"):
      builder.line(
          "return"
          f" emscripten::val(emscripten::typed_memory_view({array_size_str},"
          f" {ptr_field_expr}));"
      )

    return WrappedFieldData(
        definition=builder.to_string(),
        typename=_get_field_struct_type(f.type),
        binding=_simple_property_binding(f, w),
    )

  # SHOULD NOT OCCUR
  print("Error: field {f.name} not properly handled")
  return WrappedFieldData(
      definition=f"// Error: field {f.name} not properly handled.",
      typename=_get_field_struct_type(f.type),
      binding=f"// Error: field {f.name} not properly handled.",
  )


def _has_nested_wrapper_members(struct_info: ast_nodes.StructDecl) -> bool:
  """Checks if the struct contains other wrapped structs as direct members."""
  for field in struct_info.fields:
    member_type = cast(ast_nodes.StructFieldDecl, field).type
    if isinstance(member_type, (ast_nodes.ArrayType, ast_nodes.PointerType)):
      member_type = member_type.inner_type
    if isinstance(member_type, ast_nodes.ValueType):
      return member_type.name in constants.STRUCTS_TO_BIND
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
  if struct_name == "mjvGeom":
    f = "mjv_initGeom(ptr_, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);"
    return f
  elif struct_name in constants.ANONYMOUS_STRUCTS.keys():
    return ""
  elif struct_name in constants.NO_DEFAULT_CONSTRUCTORS:
    return ""
  elif struct_name.startswith("mjs"):
    return f"mjs_default{struct_name.removeprefix('mjs')}(ptr_);"
  elif struct_name.startswith("mjv"):
    return f"mjv_default{struct_name.removeprefix('mjv')}(ptr_);"
  elif struct_name.startswith("mj"):
    return f"mj_default{struct_name.removeprefix('mj')}(ptr_);"

  return ""


def _delete_ptr_statement(struct_name: str) -> str:
  """Returns the delete function name for the given struct."""
  if struct_name == "mjVFS":
    return "mj_deleteVFS(ptr_);"

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
  struct_info = introspect_structs.STRUCTS.get(struct_name)

  if struct_name.startswith("mjs"):
    fields_with_init = _find_fields_with_init(wrapped_fields)
    return _build_struct_header_internal(
        struct_name,
        wrapped_fields,
        fields_with_init,
        is_mjs=True,
    )

  is_anonymous_struct = struct_name in constants.ANONYMOUS_STRUCTS.keys()
  is_hardcoded_wrapper_struct = (
      common.uppercase_first_letter(struct_name) in constants.MANUAL_STRUCTS
  )

  if (
      not is_hardcoded_wrapper_struct
      and struct_info
      and not _has_nested_wrapper_members(struct_info)
      or is_anonymous_struct
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
  is_mjs = w.startswith("Mjs")

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


def _build_struct_bindings(
    struct_name: str,
    wrapped_fields: List[WrappedFieldData],
):
  """Builds the C++ bindings for a struct."""
  w = common.uppercase_first_letter(struct_name)
  is_mjs = w.startswith("Mjs")

  builder = code_builder.CodeBuilder()
  with builder.block(
      header_line=f'emscripten::class_<{w}>("{w}")', braces=False
  ):
    if w == "MjData":
      builder.line(".constructor<MjModel *>()")
      builder.line(".constructor<const MjModel &, const MjData &>()")
    elif w == "MjModel":
      builder.line(
          '.class_function("loadFromXML", &loadFromXML, take_ownership())'
      )
      builder.line(".constructor<const MjModel &>()")
    elif w == "MjSpec":
      builder.line(".constructor<const MjSpec &>()")
    elif w == "MjvScene":
      builder.line(".constructor<MjModel *, int>()")
      builder.line(".constructor<>()")
    elif not is_mjs:
      builder.line(".constructor<>()")

    shallow_copy = use_shallow_copy(wrapped_fields)
    if shallow_copy and not is_mjs:
      builder.line(f'.function("copy", &{w}::copy, take_ownership())')

    for field in wrapped_fields[:-1]:
      if field.binding:
        builder.line(field.binding)
    if wrapped_fields:
      builder.line(f"{wrapped_fields[-1].binding};")

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
    s = struct_name
    w = common.uppercase_first_letter(s)

    if s in introspect_structs.STRUCTS:
      struct_fields = introspect_structs.STRUCTS[s].fields
    elif s in constants.ANONYMOUS_STRUCTS:
      anonymous_struct = _get_anonymous_struct_field(s)
      if not anonymous_struct or not isinstance(
          anonymous_struct.type, ast_nodes.AnonymousStructDecl
      ):
        raise RuntimeError(f"Anonymous struct not found: {s}")
      struct_fields = anonymous_struct.type.fields
    else:
      raise RuntimeError(f"Struct not found: {s}")

    wrapped_fields: List[WrappedFieldData] = []
    for field in struct_fields:
      wrapped_fields.append(_generate_field_data(field, w))

    wrap_data = WrappedStructData(
        wrap_name=w,
        wrapped_fields=wrapped_fields,
        wrapped_header=build_struct_header(s, wrapped_fields),
        wrapped_source=build_struct_source(s, wrapped_fields),
        bindings=_build_struct_bindings(s, wrapped_fields),
    )

    wrapped_structs[s] = wrap_data

  return wrapped_structs


def _get_anonymous_struct_field(
    anonymous_structs_key: str,
) -> ast_nodes.StructFieldDecl | None:
  """Looks up the given key in the anonymous_structs dict and generates bindings for its fields."""
  info = constants.ANONYMOUS_STRUCTS[anonymous_structs_key]
  parent_decl = introspect_structs.STRUCTS[info["parent"]]
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


class Generator:
  """Generates C++ code for binding and wrapping MuJoCo structs."""

  def generate(self) -> list[tuple[str, list[str]]]:
    """Generates C++ header file for binding and wrapping MuJoCo structs."""

    # Traverse the introspect dictionary to get the field
    # wrapper/bindings statements set up for each struct
    self.structs_to_bind_data = generate_wasm_bindings(
        constants.STRUCTS_TO_BIND
    )

    autogenned_struct_definitions = []
    markers_and_content = []

    # Sort by struct name by dependency to ensure deterministic output order
    sorted_struct_names = sort_structs_by_dependency(self.structs_to_bind_data)

    for struct_name in sorted_struct_names:
      struct_data = self.structs_to_bind_data[struct_name]
      if struct_data.wrapped_header:
        autogenned_struct_definitions.append(struct_data.wrapped_header + "\n")
      else:
        markers_and_content.append((
            f"// INSERT-GENERATED-{struct_data.wrap_name}-DEFINITIONS",
            [
                l.definition if l.definition else ""
                for l in struct_data.wrapped_fields
            ],
        ))
    markers_and_content.append((
        "// {{ AUTOGENNED_STRUCTS_HEADER }}",
        autogenned_struct_definitions,
    ))

    autogenned_struct_source = []
    autogenned_struct_bindings = []
    for struct_name in sorted_struct_names:
      struct_data = self.structs_to_bind_data[struct_name]
      if struct_data.wrapped_source:
        autogenned_struct_source.append(struct_data.wrapped_source + "\n")
      autogenned_struct_bindings.append(struct_data.bindings)

    markers_and_content.append((
        "// {{ AUTOGENNED_STRUCTS_SOURCE }}",
        autogenned_struct_source,
    ))
    markers_and_content.append((
        "// {{ AUTOGENNED_STRUCTS_BINDINGS }}",
        autogenned_struct_bindings,
    ))
    return markers_and_content
