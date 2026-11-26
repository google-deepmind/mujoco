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
from absl.testing import parameterized

from wasm.codegen.generators import binding_builder

ERROR_MESSAGE = """
The file '{}' needs to be updated, please run:
update.py as described in wasm/README.md""".lstrip()


class BindingsDiffTest(parameterized.TestCase):

  @parameterized.named_parameters(
      ('source', 'bindings.cc'),
      ('header', 'bindings.h'),
  )
  def test_bindings(self, filename):
    codegen_dir = Path(__file__).resolve().parents[1]
    generated_src = (codegen_dir / 'generated' / filename).read_text()
    template_path = codegen_dir / 'templates' / filename

    generator_output = (
        binding_builder.BindingBuilder()
        .set_enums()
        .set_structs()
        .set_functions()
        .build_string(template_path)
    )
    self.assertEqual(
        generator_output,
        generated_src,
        msg=ERROR_MESSAGE.format(filename),
    )


if __name__ == '__main__':
  absltest.main()
