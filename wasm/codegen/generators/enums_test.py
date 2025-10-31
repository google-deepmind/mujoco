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

from absl.testing import absltest

from introspect import ast_nodes

from wasm.codegen.generators import enums


class EnumsGeneratorTest(absltest.TestCase):

  def test_generate_enum_bindings(self):

    generator = enums.Generator({
        "TestEnum": ast_nodes.EnumDecl(
            name="TestEnum",
            declname="enum TestEnum_",
            values={"FIRST_VAL": 0, "SECOND_VAL": 1, "THIRD_VAL": 2},
        ),
        "AnotherEnum": ast_nodes.EnumDecl(
            name="AnotherEnum",
            declname="enum AnotherEnum_",
            values={"ALPHA": 100, "BETA": 200},
        ),
        "EmptyEnum": ast_nodes.EnumDecl(
            name="EmptyEnum",
            declname="enum EmptyEnum_",
            values={},
        ),
    })

    expected_code = """  enum_<TestEnum>("TestEnum")
    .value("FIRST_VAL", FIRST_VAL)
    .value("SECOND_VAL", SECOND_VAL)
    .value("THIRD_VAL", THIRD_VAL);

  enum_<AnotherEnum>("AnotherEnum")
    .value("ALPHA", ALPHA)
    .value("BETA", BETA);

  enum_<EmptyEnum>("EmptyEnum");
"""

    actual_code = generator.generate()

    self.assertEqual(actual_code, expected_code)


if __name__ == "__main__":
  absltest.main()
