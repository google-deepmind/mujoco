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

"""Code builder for struct constructor code."""

from typing import List, cast

from introspect import ast_nodes
from introspect import structs as introspect_structs

from wasm.codegen.helpers import code_builder
from wasm.codegen.helpers import common
from wasm.codegen.helpers import constants
from wasm.codegen.helpers import structs_wrappers_data


def _has_nested_wrapper_members(
    struct_info: ast_nodes.StructDecl
) -> bool:
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
    wrapped_fields: List[structs_wrappers_data.WrappedFieldData],
    fields_with_init: List[structs_wrappers_data.WrappedFieldData],
    use_shallow_copy: bool = False,
    is_mjs: bool = False,
):
  """Builds the C++ header file code for a struct."""
  wrapper_name = common.uppercase_first_letter(struct_name)
  builder = code_builder.CodeBuilder()
  with builder.block(f"struct {wrapper_name}"):
    if not is_mjs:
      builder.line(f"{wrapper_name}();")
      builder.line(f"{wrapper_name}(const {wrapper_name} &);")
      builder.line(f"{wrapper_name} &operator=(const {wrapper_name} &);")

    builder.line(f"explicit {wrapper_name}({struct_name} *ptr);")
    builder.line(f"~{wrapper_name}();")

    if use_shallow_copy:
      builder.line(f"std::unique_ptr<{wrapper_name}> copy();")

    for field in wrapped_fields:
      if field.definition and field not in fields_with_init:
        for line in field.definition.splitlines():
          builder.line(line)

    builder.line(f"{struct_name}* get() const {{ return ptr_; }}")
    builder.line(f"void set({struct_name}* ptr) {{ ptr_ = ptr; }}")
    builder.line("")
    builder.line("private:")
    builder.line(f"{struct_name}* ptr_;")
    builder.line("bool owned_ = false;")

    if is_mjs and fields_with_init:
      builder.line("")
      builder.line("public:")
      for field in fields_with_init:
        if field.definition:
          builder.line(f"{field.definition}")
  return builder.to_string()+";"


def build_struct_header(
    struct_name: str,
    use_shallow_copy: bool = False,
    fields_with_init: List[structs_wrappers_data.WrappedFieldData] = [],
    wrapped_fields: List[structs_wrappers_data.WrappedFieldData] = [],
):
  """Builds the C++ header file code for a struct."""
  struct_info = introspect_structs.STRUCTS.get(struct_name)

  if struct_name.startswith("mjs"):
    return _build_struct_header_internal(
        struct_name,
        wrapped_fields,
        fields_with_init,
        use_shallow_copy,
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
        struct_name, wrapped_fields, [], use_shallow_copy, is_mjs=False
    )
  return ""


def build_struct_source(
    struct_name: str,
    mj_default_func: str | None = None,
    fields_with_init: List[structs_wrappers_data.WrappedFieldData] = [],
    use_shallow_copy: bool = False,
):
  """Builds the C++ .cc file code for a struct."""
  wrapper_name = common.uppercase_first_letter(struct_name)
  is_mjs_struct = "Mjs" in wrapper_name
  builder = code_builder.CodeBuilder()

  fields_init = ""
  if fields_with_init:
    fields_init = "".join(
        field_with_init.initialization
        for field_with_init in fields_with_init
    )
  # constructor passing native ptr
  builder.line(
      f"{wrapper_name}::{wrapper_name}({struct_name} *ptr) :"
      f" ptr_(ptr){fields_init} {{}}"
  )
  # constructor with default values
  if not is_mjs_struct:
    with builder.block(
        f"{wrapper_name}::{wrapper_name}() : ptr_(new"
        f" {struct_name}){fields_init}"
    ):
      builder.line("owned_ = true;")
      if mj_default_func:
        builder.line(f"{mj_default_func}(ptr_);")
  # copy constructor
  if use_shallow_copy and not is_mjs_struct:
    with builder.block(
        f"{wrapper_name}::{wrapper_name}(const"
        f" {wrapper_name} &other)"
        + (f" : {wrapper_name}()" if not is_mjs_struct else "")
    ):
      builder.line("*ptr_ = *other.get();")
      if fields_with_init:
        for field_with_init in fields_with_init:
          if field_with_init.ptr_copy_reset is not None:
            builder.line(field_with_init.ptr_copy_reset)
    # assignment operator
    with builder.block(
        f"{wrapper_name}&"
        f" {wrapper_name}::operator=(const"
        f" {wrapper_name} &other)"
    ):
      with builder.block("if (this == &other)"):
        builder.line("return *this;")
      builder.line("*ptr_ = *other.get();")
      if fields_with_init:
        for field_with_init in fields_with_init:
          if field_with_init.ptr_copy_reset is not None:
            builder.line(field_with_init.ptr_copy_reset)
      builder.line("return *this;")
  # destructor
  if is_mjs_struct:
    builder.line(f"{wrapper_name}::~{wrapper_name}() {{}}")
  else:
    with builder.block(f"{wrapper_name}::~{wrapper_name}()"):
      builder.line("if (owned_ && ptr_) delete ptr_;")
  # copy function
  if use_shallow_copy:
    with builder.block(
        f"std::unique_ptr<{wrapper_name}>"
        f" {wrapper_name}::copy()"
    ):
      builder.line(
          f"return std::make_unique<{wrapper_name}>(*this);"
      )
  return builder.to_string()
