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
from typing import Dict, List

from introspect import ast_nodes
from introspect import structs

from wasm.codegen.helpers import common
from wasm.codegen.helpers import constants
from wasm.codegen.helpers import struct_constructor_code_builder
from wasm.codegen.helpers import struct_field_handler
from wasm.codegen.helpers import structs_wrappers_data


WrappedFieldData = structs_wrappers_data.WrappedFieldData
WrappedStructData = structs_wrappers_data.WrappedStructData
StructFieldHandler = struct_field_handler.StructFieldHandler

AnonymousStructDecl = ast_nodes.AnonymousStructDecl
StructFieldDecl = ast_nodes.StructFieldDecl
debug_print = common.debug_print

introspect_structs = structs.STRUCTS


def generate_wasm_bindings(
    wrapped_structs: Dict[str, WrappedStructData],
) -> Dict[str, WrappedStructData]:
  """Generates WASM bindings for MuJoCo structs."""

  for struct_name, wrap_data in wrapped_structs.items():
    if struct_name in introspect_structs:
      struct_fields = introspect_structs[struct_name].fields
    elif struct_name in constants.ANONYMOUS_STRUCTS:
      anonymous_struct = _get_anonymous_struct_field(struct_name)
      if not anonymous_struct or not isinstance(
          anonymous_struct.type, AnonymousStructDecl
      ):
        raise RuntimeError(f"Anonymous struct not found: {struct_name}")
      struct_fields = anonymous_struct.type.fields
    else:
      raise RuntimeError(f"Struct not found: {struct_name}")

    debug_print(f"Wrapping struct: {struct_name}")

    fields_with_init: List[WrappedFieldData] = []
    for field in struct_fields:
      field_gen = StructFieldHandler(field, wrap_data.wrap_name).generate()
      # If the struct has at least one non-primitive or fixed size field
      # we avoid shallow copy to avoid uninitialized memory.
      if not field_gen.is_primitive_or_fixed_size:
        wrap_data.use_shallow_copy = False
      if field_gen.initialization:
        fields_with_init.append(field_gen)
      wrap_data.wrapped_fields.append(field_gen)

    wrap_data.wrapped_header = (
        struct_constructor_code_builder.build_struct_header(
            struct_name,
            wrap_data.use_shallow_copy,
            fields_with_init,
            wrap_data.wrapped_fields,
        )
    )
    wrap_data.wrapped_source = (
        struct_constructor_code_builder.build_struct_source(
            struct_name,
            get_default_func_name(struct_name),
            fields_with_init,
            wrap_data.use_shallow_copy,
        )
    )
  return wrapped_structs


def _get_anonymous_struct_field(
    anonymous_structs_key: str,
) -> StructFieldDecl | None:
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
          and isinstance(f.type, AnonymousStructDecl)
      ),
      None,
  )
  return target_field


def get_default_func_name(struct_name: str) -> str:
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
    return f"mjs_default{struct_name.removeprefix('mjs')}"
  elif struct_name.startswith("mjv"):
    return f"mjv_default{struct_name.removeprefix('mjv')}"
  else:
    return f"mj_default{struct_name.removeprefix('mj')}"


def _get_field_struct_type(field_type):
  """Extracts the base struct name if the field type is a struct or pointer to a struct."""
  if isinstance(field_type, ast_nodes.ValueType):
    return field_type.name
  if isinstance(field_type, ast_nodes.PointerType):
    if isinstance(field_type.inner_type, ast_nodes.ValueType):
      return field_type.inner_type.name
  return None


def sort_structs_by_dependency(struct_names: List[str]) -> List[str]:
  """Sorts structs based on their field dependencies using topological sort.

  Structs with no dependencies on other structs in the list come first.
  If struct A has a field of type struct B, B must come before A in the
  sorted list.

  Args:
    struct_names: A list of struct names to sort.

  Returns:
    A new list of struct names sorted by dependency.

  Raises:
    RuntimeError: If a cyclic dependency is detected.
  """
  adj = collections.defaultdict(list)
  in_degree = collections.defaultdict(int)
  struct_set = set(struct_names)
  sorted_struct_names = sorted(struct_names)

  for struct_name in sorted_struct_names:
    if struct_name not in introspect_structs:
      # Skip anonymous or other structs not in the main introspect map
      continue

    struct_decl = introspect_structs[struct_name]
    for field in struct_decl.fields:
      if isinstance(field, ast_nodes.AnonymousStructDecl):
        continue

      field_type_name = _get_field_struct_type(field.type)
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
