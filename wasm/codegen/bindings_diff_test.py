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

from pathlib import Path

from absl.testing import absltest

from wasm.codegen import binding_builder

ERROR_MESSAGE = """
The file '{}' needs to be updated, please run:
update.py as described in wasm/README.md""".lstrip()


class BindingsDiffTest(absltest.TestCase):

  def setUp(self):
    super().setUp()

    SCRIPT_DIR = Path(__file__).parent
    with open(SCRIPT_DIR / 'generated/bindings.h', 'r') as f:
      self.generated_hdr = f.read()
    with open(SCRIPT_DIR / 'generated/bindings.cc', 'r') as f:
      self.generated_src = f.read()
    self.template_path_h = SCRIPT_DIR / 'templates/bindings.h'
    self.template_path_cc = SCRIPT_DIR / 'templates/bindings.cc'
    self.generated_path_h = SCRIPT_DIR / 'generated/bindings.h'
    self.generated_path_cc = SCRIPT_DIR / 'generated/bindings.cc'

    self.builder = binding_builder.BindingBuilder(
        self.template_path_h,
        self.template_path_cc,
        self.generated_path_h,
        self.generated_path_cc,
    )

  def test_bindings_source(self):
    generator_output = (self.builder.set_enums().set_structs().set_functions().
                        to_string_source())
    self.assertEqual(
        generator_output,
        self.generated_src,
        msg=ERROR_MESSAGE.format('bindings.cc'),
    )

  def test_bindings_header(self):
    generator_output = (self.builder.set_headers().to_string_header())
    self.assertEqual(
        generator_output,
        self.generated_hdr,
        msg=ERROR_MESSAGE.format('bindings.h'),
    )


if __name__ == '__main__':
  absltest.main()
