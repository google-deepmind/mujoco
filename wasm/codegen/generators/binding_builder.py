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

from introspect import ast_nodes
from introspect import enums as introspect_enums
from introspect import functions as introspect_functions

from wasm.codegen.generators import common
from wasm.codegen.generators import constants
from wasm.codegen.generators import enums
from wasm.codegen.generators import functions
from wasm.codegen.generators import structs


class BindingBuilder:
  """Builds WASM bindings for MuJoCo."""

  def __init__(
      self,
      template_path_cc: str,
  ):
    with open(template_path_cc, "r") as f:
      self.content_cc = f.readlines()

    self.markers_and_content = []

  def set_enums(self):
    """Generates and sets the enum bindings."""
    generator = enums.Generator(introspect_enums.ENUMS)
    self.markers_and_content += generator.generate()
    return self

  def set_structs(self):
    """Generates and sets the struct bindings."""
    generator = structs.Generator()
    self.markers_and_content += generator.generate()
    return self

  def set_functions(self):
    """Generates and sets the function wrappers and bindings."""

    functions_to_bind: dict[str, ast_nodes.FunctionDecl] = {}
    for name, func in introspect_functions.FUNCTIONS.items():
      if not functions.is_excluded_function_name(name):
        if name not in constants.BOUNDCHECK_FUNCS:
          functions_to_bind[name] = func

    generator = functions.Generator(functions_to_bind)
    self.markers_and_content += generator.generate()
    return self

  def to_string(self) -> str:
    for marker, content in self.markers_and_content:
      self.content_cc = common.replace_lines_containing_marker(
          self.content_cc, marker, content
      )
    return "".join(self.content_cc)

  def build(self, generated_path_cc: str):
    common.write_to_file(generated_path_cc, self.to_string())
