"""Generates Embind bindings for MuJoCo structs."""

from typing import Optional

from wasm.codegen.helpers import constants
from wasm.codegen.helpers import structs_parser
from wasm.codegen.helpers import structs_wrappers_data


class Generator:
  """Generates C++ code for binding and wrapping MuJoCo structs."""

  def __init__(self):
    # Set up the correct input dict based on the structs we want to bind
    # and already have a wrapper manually created in the template/bindings.cc
    wrapped_structs = structs_wrappers_data.create_wrapped_structs_set_up_data(
        constants.STRUCTS_TO_BIND
    )

    # Traverse the introspect dictionary to get the field
    # wrapper/bindings statements set up for each struct
    self.structs_to_bind_data = structs_parser.generate_wasm_bindings(
        wrapped_structs
    )

  def generate_header(
      self
  ) -> list[tuple[str, list[Optional[str]]]]:
    """Generates C++ header file for binding and wrapping MuJoCo structs."""
    autogenned_struct_definitions = []
    markers_and_content = []

    # Sort by struct name by dependency to ensure deterministic output order
    sorted_struct_names = structs_parser.sort_structs_by_dependency(
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
    return markers_and_content

  def generate_source(self) -> list[tuple[str, list[str]]]:
    """Generates C++ source file for binding and wrapping MuJoCo structs."""
    constructors = [
        (
            f"// INSERT-GENERATED-{struct_data.wrap_name}-CONSTRUCTOR",
            [struct_data.wrapped_source],
        )
        for _, struct_data in self.structs_to_bind_data.items()
    ]
    properties = [
        (
            f"// INSERT-GENERATED-{struct_data.wrap_name}-BINDINGS",
            [l.binding for l in struct_data.wrapped_fields],
        )
        for _, struct_data in self.structs_to_bind_data.items()
    ]
    return constructors + properties
