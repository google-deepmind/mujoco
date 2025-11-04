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
from wasm.codegen.generators import functions


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

  enum_<EmptyEnum>("EmptyEnum");"""

    markers_and_content = generator.generate()
    actual_code = "\n\n".join(markers_and_content[0][1])

    self.assertEqual(actual_code, expected_code)


class FunctionsGeneratorTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.generator = functions.Generator({})
    self.int_type = ast_nodes.ValueType(name="int")

  def test_generate_function_binding_simple_case(self):
    func_simple_void = ast_nodes.FunctionDecl(
        name="do_nothing",
        return_type=ast_nodes.ValueType(name="void"),
        parameters=tuple(),
        doc="doc",
    )
    self.assertEqual(
        self.generator._generate_function_binding(func_simple_void),
        'function("do_nothing", &do_nothing);\n',
    )

  def test_generate_direct_bindable_functions_simple_filter(self):
    direct_bind = ast_nodes.FunctionDecl(
        name="direct_bind",
        return_type=self.int_type,
        parameters=(
            ast_nodes.FunctionParameterDecl(name="val", type=self.int_type),
        ),
        doc="doc",
    )
    needs_wrap = ast_nodes.FunctionDecl(
        name="needs_wrap",
        return_type=ast_nodes.PointerType(inner_type=self.int_type),
        parameters=tuple(),
        doc="doc",
    )
    self.generator = functions.Generator({
        "direct1": direct_bind,
        "wrapped1": needs_wrap,
    })

    generated_code = self.generator._generate_direct_bindable_functions()
    self.assertIn('function("direct_bind", &direct_bind);\n', generated_code)
    self.assertNotIn('function("needs_wrap", &needs_wrap);\n', generated_code)


if __name__ == "__main__":
  absltest.main()
