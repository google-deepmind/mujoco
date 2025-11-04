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

from typing import Optional

from wasm.codegen.helpers import constants
from wasm.codegen.helpers import structs


class Generator:
  """Generates C++ code for binding and wrapping MuJoCo structs."""

  def generate(self) -> list[tuple[str, list[str]]]:
    """Generates C++ header file for binding and wrapping MuJoCo structs."""

    # Traverse the introspect dictionary to get the field
    # wrapper/bindings statements set up for each struct
    self.structs_to_bind_data = structs.generate_wasm_bindings(
        constants.STRUCTS_TO_BIND
    )

    autogenned_struct_definitions = []
    markers_and_content = []

    # Sort by struct name by dependency to ensure deterministic output order
    sorted_struct_names = structs.sort_structs_by_dependency(
        constants.STRUCTS_TO_BIND
    )

    for struct_name in sorted_struct_names:
      struct_data = self.structs_to_bind_data[struct_name]
      if struct_data.wrapped_header:
        autogenned_struct_definitions.append(
            struct_data.wrapped_header + "\n"
        )
      else:
        markers_and_content.append((
            f"// INSERT-GENERATED-{struct_data.wrap_name}-DEFINITIONS",
            [
                l.definition if l.definition else ""
                for l in struct_data.wrapped_fields
            ],
        ))
    markers_and_content.append((
        "// {{ AUTOGENNED_STRUCT_DEFINITIONS }}",
        autogenned_struct_definitions,
    ))

    for _, struct_data in self.structs_to_bind_data.items():
      # Bindings
      markers_and_content.append((
          f"// INSERT-GENERATED-{struct_data.wrap_name}-BINDINGS",
          [l.binding for l in struct_data.wrapped_fields],
      ))

      # Special member functions
      markers_and_content.append((
          f"// INSERT-GENERATED-{struct_data.wrap_name}-CONSTRUCTOR",
          [struct_data.wrapped_source],
      ))

    return markers_and_content
