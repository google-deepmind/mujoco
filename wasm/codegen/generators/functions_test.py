import unittest

from wasm.codegen.generators import functions

from introspect import ast_nodes



class FunctionsGeneratorTest(unittest.TestCase):

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
  unittest.main()
