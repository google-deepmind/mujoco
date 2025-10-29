"""Classes used by the parser to generate the structs wrappers and bindings statements."""

import dataclasses
from typing import Dict, List

from wasm.codegen.helpers import common


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


def create_wrapped_structs_set_up_data(
    struct_names: List[str],
) -> Dict[str, WrappedStructData]:
  """Creates a dictionary of WrappedStructData for the given struct names."""
  return {
      name: WrappedStructData(
          wrap_name=common.uppercase_first_letter(name),
          wrapped_fields=[],
          wrapped_header="",
          wrapped_source="",
      )
      for name in struct_names
  }
