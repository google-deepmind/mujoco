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

"""Tests for structs_parser."""

from absl.testing import absltest
from introspect import ast_nodes
from introspect import structs as introspect_structs
from wasm.codegen.helpers import structs_parser
from wasm.codegen.helpers import structs_wrappers_data


class StructsParserTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.wrapped_structs = structs_parser.generate_wasm_bindings(
        structs_wrappers_data.create_wrapped_structs_set_up_data([
            "mjModel",
            "mjData",
            "mjVisualGlobal",
            "mjVisualQuality",
            "mjVisual",
        ])
    )

  def test_sort_structs_by_dependency(self):
    mock_introspect_structs = {
        "mjA": ast_nodes.StructDecl(
            name="mjA",
            declname="mjA",
            fields=[
                ast_nodes.StructFieldDecl(
                    doc="", name="b_field", type=ast_nodes.ValueType(name="mjB")
                )
            ],
        ),
        "mjB": ast_nodes.StructDecl(
            name="mjB",
            declname="mjB",
            fields=[
                ast_nodes.StructFieldDecl(
                    doc="", name="c_field", type=ast_nodes.ValueType(name="mjC")
                )
            ],
        ),
        "mjC": ast_nodes.StructDecl(name="mjC", declname="mjC", fields=[]),
        "mjD": ast_nodes.StructDecl(name="mjD", declname="mjD", fields=[]),
    }
    with absltest.mock.patch.dict(
        introspect_structs.STRUCTS, mock_introspect_structs
    ):
      struct_names = ["mjA", "mjB", "mjC", "mjD"]
      sorted_names = structs_parser.sort_structs_by_dependency(struct_names)
      self.assertEqual(sorted_names, ["mjC", "mjD", "mjB", "mjA"])

  def test_generate_wasm_bindings(self):
    self.assertEqual(self.wrapped_structs["mjModel"].wrap_name, "MjModel")
    self.assertEqual(self.wrapped_structs["mjData"].wrap_name, "MjData")
    self.assertEqual(
        self.wrapped_structs["mjVisualGlobal"].wrap_name, "MjVisualGlobal"
    )
    self.assertEqual(
        self.wrapped_structs["mjVisualQuality"].wrap_name, "MjVisualQuality"
    )
    self.assertEqual(self.wrapped_structs["mjVisual"].wrap_name, "MjVisual")
    self.assertNotEmpty(self.wrapped_structs["mjModel"].wrapped_fields)
    self.assertNotEmpty(self.wrapped_structs["mjData"].wrapped_fields)
    self.assertNotEmpty(self.wrapped_structs["mjVisualGlobal"].wrapped_fields)
    self.assertNotEmpty(self.wrapped_structs["mjVisualQuality"].wrapped_fields)
    self.assertNotEmpty(self.wrapped_structs["mjVisual"].wrapped_fields)

  def test_generate_wasm_bindings_with_error(self):
    with self.assertRaises(RuntimeError):
      structs_parser.generate_wasm_bindings(
          structs_wrappers_data.create_wrapped_structs_set_up_data(
              ["mjFakeStruct"]
          )
      )
    with self.assertRaises(RuntimeError):
      structs_parser.generate_wasm_bindings(
          structs_wrappers_data.create_wrapped_structs_set_up_data(
              ["mjFakeAnonymousStruct"]
          )
      )

  def test_get_default_func_name_mjv(self):
    self.assertEqual(
        structs_parser.get_default_func_name("mjvPerturb"), "mjv_defaultPerturb"
    )

  def test_get_default_func_name(self):
    self.assertEqual(
        structs_parser.get_default_func_name("mjOption"), "mj_defaultOption"
    )


if __name__ == "__main__":
  absltest.main()
