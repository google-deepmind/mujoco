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

from wasm.codegen.helpers import constants
from wasm.codegen.helpers import function_utils


class FunctionUtilsTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.struct_type = ast_nodes.ValueType("MyStruct")
    self.ptr_to_int = ast_nodes.PointerType(ast_nodes.ValueType("int"))
    self.func_ret_ptr_int = ast_nodes.FunctionDecl(
        "func_pi", ast_nodes.PointerType(ast_nodes.ValueType("int")), [], "doc"
    )
    self.func_ret_ptr_struct = ast_nodes.FunctionDecl(
        "func_ps",
        ast_nodes.PointerType(ast_nodes.ValueType("MyStruct")),
        [],
        "doc",
    )

  def test_return_is_value_of_type(self):
    self.assertTrue(
        function_utils.return_is_value_of_type(
            ast_nodes.FunctionDecl(
                "func_i", ast_nodes.ValueType("int"), [], "doc"
            ),
            constants.PRIMITIVE_TYPES,
        )
    )
    self.assertFalse(
        function_utils.return_is_value_of_type(
            ast_nodes.FunctionDecl(
                "func_s", ast_nodes.ValueType("MyStruct"), [], "doc"
            ),
            constants.PRIMITIVE_TYPES,
        )
    )

  def test_return_is_pointer_to_struct(self):
    self.assertTrue(
        function_utils.return_is_pointer_to_struct(self.func_ret_ptr_struct)
    )
    self.assertFalse(
        function_utils.return_is_pointer_to_struct(self.func_ret_ptr_int)
    )

  def test_return_is_pointer_to_primitive(self):
    self.assertTrue(
        function_utils.return_is_pointer_to_primitive(self.func_ret_ptr_int)
    )
    self.assertFalse(
        function_utils.return_is_pointer_to_primitive(self.func_ret_ptr_struct)
    )

  def test_param_is_primitive_value(self):
    param_prim_val = ast_nodes.FunctionParameterDecl(
        "prim_v", ast_nodes.ValueType("int")
    )
    param_arr = ast_nodes.FunctionParameterDecl(
        "arr_v", ast_nodes.ArrayType(ast_nodes.ValueType("int"), extents=(10,))
    )

    self.assertTrue(function_utils.param_is_primitive_value(param_prim_val))
    self.assertFalse(function_utils.param_is_primitive_value(param_arr))

  def test_param_is_pointer_to_primitive_value(self):
    param_ptr_to_prim = ast_nodes.FunctionParameterDecl(
        "p_prim", self.ptr_to_int
    )
    param_arr_of_prim = ast_nodes.FunctionParameterDecl(
        "a_prim", ast_nodes.ArrayType(ast_nodes.ValueType("int"), extents=(10,))
    )
    param_ptr_to_struct = ast_nodes.FunctionParameterDecl(
        name="p_struct", type=ast_nodes.PointerType(inner_type=self.struct_type)
    )
    self.assertTrue(
        function_utils.param_is_pointer_to_primitive_value(param_ptr_to_prim)
    )
    self.assertTrue(
        function_utils.param_is_pointer_to_primitive_value(param_arr_of_prim)
    )
    self.assertFalse(
        function_utils.param_is_pointer_to_primitive_value(param_ptr_to_struct)
    )

  def test_param_is_pointer_to_struct(self):
    param_arr_of_struct = ast_nodes.FunctionParameterDecl(
        "a_struct", ast_nodes.ArrayType(self.struct_type, extents=(5,))
    )
    param_ptr_to_struct = ast_nodes.FunctionParameterDecl(
        "p_struct", ast_nodes.PointerType(self.struct_type)
    )
    param_ptr_to_ptr = ast_nodes.FunctionParameterDecl(
        "p_ptr", ast_nodes.PointerType(self.ptr_to_int)
    )
    self.assertTrue(
        function_utils.param_is_pointer_to_struct(param_arr_of_struct)
    )
    self.assertTrue(
        function_utils.param_is_pointer_to_struct(param_ptr_to_struct)
    )
    self.assertFalse(
        function_utils.param_is_pointer_to_struct(param_ptr_to_ptr)
    )

  def test_should_be_wrapped_with_primitive_ptr_return(self):
    func = ast_nodes.FunctionDecl(
        name="get_data",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("int")),
        parameters=tuple(),
        doc="Returns int pointer",
    )
    self.assertTrue(function_utils.should_be_wrapped(func))

  def test_generate_function_wrapper_for_simple_func(self):
    func = ast_nodes.FunctionDecl(
        name="get_id",
        return_type=ast_nodes.ValueType("int"),
        parameters=tuple(),
        doc="Returns an integer ID",
    )
    result = function_utils.generate_function_wrapper(func)
    self.assertEqual(
        result,
        """int get_id_wrapper()
{
  return get_id();
}""",
    )

  def test_generate_function_wrapper_checking_param(self):
    parameters = (
        ast_nodes.FunctionParameterDecl(
            name="mat",
            type=ast_nodes.PointerType(
                inner_type=ast_nodes.ValueType(name="mjtNum", is_const=True),
            ),
        ),
        ast_nodes.FunctionParameterDecl(
            name="nr",
            type=ast_nodes.ValueType(name="int"),
        ),
    )
    func = ast_nodes.FunctionDecl(
        name="get_id",
        return_type=ast_nodes.ValueType("int"),
        parameters=parameters,
        doc="Returns an integer ID",
    )
    result = function_utils.generate_function_wrapper(func)
    self.assertEqual(
        result,
        """int get_id_wrapper(const NumberArray& mat, int nr)
{
  UNPACK_ARRAY(mjtNum, mat);
  return get_id(mat_.data(), nr);
}""",
    )

  def test_get_params_string_with_struct_ptr(self):
    param = ast_nodes.FunctionParameterDecl(
        name="my_struct",
        type=ast_nodes.PointerType(ast_nodes.ValueType("mystruct")),
    )
    result = function_utils.get_params_string((param,))
    self.assertEqual(result, ["Mystruct& my_struct"])

  def test_get_params_string_maybe_with_conversion_struct_ptr(self):
    param = ast_nodes.FunctionParameterDecl(
        name="s",
        type=ast_nodes.PointerType(ast_nodes.ValueType("customstruct")),
    )
    result = function_utils.get_params_string_maybe_with_conversion((param,))
    self.assertEqual(result, ["s.get()"])

  def test_get_compatible_return_call(self):
    func = ast_nodes.FunctionDecl(
        name="noop",
        return_type=ast_nodes.ValueType("void"),
        parameters=tuple(),
        doc="does nothing",
    )
    result = function_utils.get_compatible_return_call(func, "noop()")
    self.assertEqual(result, "noop()")

  def test_get_compatible_return_type(self):
    func = ast_nodes.FunctionDecl(
        name="get_name",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("char")),
        parameters=tuple(),
        doc="returns name",
    )
    result = function_utils.get_compatible_return_type(func)
    self.assertEqual(result.strip(), "std::string")

  def test_get_converted_struct_to_class(self):
    func = ast_nodes.FunctionDecl(
        name="get_struct",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("mystruct")),
        parameters=tuple(),
        doc="returns struct",
    )
    result = function_utils.get_converted_struct_to_class(func, "get_struct()")
    self.assertIn("mystruct* result = get_struct();", result)
    self.assertIn("return Mystruct(result)", result)

  def test_is_excluded_function_name(self):
    self.assertTrue(function_utils.is_excluded_function_name("mjr_function"))
    self.assertTrue(function_utils.is_excluded_function_name("mjui_function"))
    self.assertTrue(function_utils.is_excluded_function_name("mju_malloc"))
    self.assertTrue(function_utils.is_excluded_function_name("mj_makeData"))
    self.assertFalse(
        function_utils.is_excluded_function_name("mjv_updateScene")
    )
    self.assertFalse(
        function_utils.is_excluded_function_name("mj_normalFunction")
    )
    self.assertFalse(
        function_utils.is_excluded_function_name("mju_someOtherFunction")
    )


if __name__ == "__main__":
  absltest.main()
