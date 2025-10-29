"""Tests for structs_parser."""

import unittest
from wasm.codegen.helpers import structs_parser
from wasm.codegen.helpers import structs_wrappers_data


class StructsParserTest(unittest.TestCase):

  def setUp(self):
    super().setUp()
    self.wrapped_structs = structs_parser.generate_wasm_bindings(
        structs_wrappers_data.create_wrapped_structs_set_up_data([
            "mjModel",
            "mjData",
            "mjVisualGlobal",
            "mjVisualQuality",
            "mjVisual",
        ]))

  def test_generate_wasm_bindings(self):
    self.assertEqual(self.wrapped_structs["mjModel"].wrap_name, "MjModel")
    self.assertEqual(self.wrapped_structs["mjData"].wrap_name, "MjData")
    self.assertEqual(self.wrapped_structs["mjVisualGlobal"].wrap_name,
                     "MjVisualGlobal")
    self.assertEqual(self.wrapped_structs["mjVisualQuality"].wrap_name,
                     "MjVisualQuality")
    self.assertEqual(self.wrapped_structs["mjVisual"].wrap_name, "MjVisual")
    self.assertIsNotNone(self.wrapped_structs["mjModel"].wrapped_fields)
    self.assertIsNotNone(self.wrapped_structs["mjData"].wrapped_fields)
    self.assertIsNotNone(self.wrapped_structs["mjVisualGlobal"].wrapped_fields)
    self.assertIsNotNone(self.wrapped_structs["mjVisualQuality"].wrapped_fields)
    self.assertIsNotNone(self.wrapped_structs["mjVisual"].wrapped_fields)

  def test_generate_wasm_bindings_with_error(self):
    with self.assertRaises(RuntimeError):
      structs_parser.generate_wasm_bindings(
          structs_wrappers_data.create_wrapped_structs_set_up_data(
              ["mjFakeStruct"]))
    with self.assertRaises(RuntimeError):
      structs_parser.generate_wasm_bindings(
          structs_wrappers_data.create_wrapped_structs_set_up_data(
              ["mjFakeAnonymousStruct"]))

  def test_get_default_func_name_mjv(self):
    self.assertEqual(structs_parser.get_default_func_name("mjvPerturb"),
                     "mjv_defaultPerturb")

  def test_get_default_func_name(self):
    self.assertEqual(structs_parser.get_default_func_name("mjOption"),
                     "mj_defaultOption")


if __name__ == "__main__":
  unittest.main()
