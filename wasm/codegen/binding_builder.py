"""Builds WASM bindings for MuJoCo."""

from introspect import enums as introspect_enums
from introspect import functions as introspect_functions

from generators import enums
from generators import functions
from generators import structs
from helpers import common
from helpers import constants as _constants
from helpers import function_utils


class BindingBuilder:
  """Builds WASM bindings for MuJoCo from introspected definitions."""

  def __init__(
      self,
      template_path_h: str,
      template_path_cc: str,
      generated_path_h: str,
      generated_path_cc: str,
  ):
    self.generated_path_h = generated_path_h
    self.generated_path_cc = generated_path_cc
    with open(template_path_h, "r") as f:
      self.content_h = f.readlines()
    with open(template_path_cc, "r") as f:
      self.content_cc = f.readlines()

    filtered_functions = {
        name: func
        for name, func in introspect_functions.FUNCTIONS.items()
        if not function_utils.is_excluded_function_name(name)
        and name not in _constants.BOUNDCHECK_FUNCS
    }
    self.enums_generator = enums.Generator(introspect_enums.ENUMS)
    self.functions_generator = functions.Generator(filtered_functions)
    self.structs_generator = structs.Generator()

  def set_enums(self):
    """Generates and sets the enum bindings."""
    enum_bindings = self.enums_generator.generate()
    self.content_cc = common.replace_lines_containing_marker(
        self.content_cc,
        "// {{ ENUM_BINDINGS }}",
        enum_bindings,
    )
    return self

  def set_headers(self):
    """Generates and sets the struct definitions."""
    struct_hdr_markers_and_content = self.structs_generator.generate_header()

    for marker, content in struct_hdr_markers_and_content:
      self.content_h = common.replace_lines_containing_marker(
          self.content_h, marker, content
      )

    return self

  def set_structs(self):
    """Generates and sets the struct bindings."""

    struct_src_markers_and_content = (
        self.structs_generator.generate_source()
    )
    for marker, content in struct_src_markers_and_content:
      self.content_cc = common.replace_lines_containing_marker(
          self.content_cc, marker, content
      )
    self.content_cc = common.replace_lines_containing_marker(
        self.content_cc,
        '#include "third_party/mujoco/wasm/codegen/templates/bindings.h"',
        '#include "third_party/mujoco/wasm/codegen/generated/bindings.h"\n',
    )
    return self

  def set_functions(self):
    """Generates and sets the function wrappers and bindings."""
    wrapper_functions, function_bindings = (
        self.functions_generator.generate()
    )
    self.content_cc = common.replace_lines_containing_marker(
        self.content_cc,
        "// {{ WRAPPER_FUNCTIONS }}",
        wrapper_functions,
    )
    self.content_cc = common.replace_lines_containing_marker(
        self.content_cc,
        "// {{ FUNCTION_BINDINGS }}",
        function_bindings,
    )
    return self

  def build(self):
    """Writes the generated content to the output files."""
    common.write_to_file(self.generated_path_h, "".join(self.content_h))
    common.write_to_file(self.generated_path_cc, "".join(self.content_cc))

  def to_string_header(self) -> str:
    return "".join(self.content_h)

  def to_string_source(self) -> str:
    return "".join(self.content_cc)
