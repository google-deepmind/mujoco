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

"""Generates Javascript/TypeScript bindings for MuJoCo."""

from wasm.codegen.generators import binding_builder


if __name__ == "__main__":
  template_file = "wasm/codegen/templates/bindings.cc"
  generated_file = "wasm/codegen/generated/bindings.cc"

  builder = binding_builder.BindingBuilder(template_file)
  builder.set_enums()
  builder.set_structs()
  builder.set_functions()

  builder.build(generated_file)
