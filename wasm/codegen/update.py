"""Generates WASM bindings for MuJoCo's API.

This script leverages MuJoCo's introspect dicts to gather information
about its internal structures and then uses a code generation framework to
produce corresponding WASM bindings.
"""

from wasm.codegen import binding_builder
from wasm.codegen.helpers import common


def generate_all_bindings():
  """Generates WASM bindings for MuJoCo."""
  template_path_h, generated_path_h = common.get_file_path(
      "templates", "generated", "bindings.h"
  )
  template_path_cc, generated_path_cc = common.get_file_path(
      "templates", "generated", "bindings.cc"
  )
  builder = binding_builder.BindingBuilder(
      template_path_h, template_path_cc, generated_path_h, generated_path_cc
  )
  builder.set_enums().set_headers().set_structs().set_functions().build()


if __name__ == "__main__":
  generate_all_bindings()
