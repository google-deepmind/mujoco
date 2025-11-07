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

"""Tests for helper modules."""

from absl.testing import absltest
from introspect import ast_nodes

from wasm.codegen.generators import code_builder
from wasm.codegen.generators import common
from wasm.codegen.generators import constants
from wasm.codegen.generators import enums
from wasm.codegen.generators import functions
from wasm.codegen.generators import structs


class CodeBuilderTest(absltest.TestCase):

  def test_code_builder_functionality(self):
    """Test nested indentation blocks."""
    builder = code_builder.CodeBuilder(indent_str="    ")
    builder.line("let a = 1")
    with builder.block("function myFunc()"):
      builder.line("let flag = true")
      with builder.block("while (flag)"):
        builder.line("a++")
        builder.line("flag = a < 10")
      builder.line("return a")
    builder.line("print('Done')")

    expected_lines = [
        "let a = 1",
        "function myFunc() {",
        "    let flag = true",
        "    while (flag) {",
        "        a++",
        "        flag = a < 10",
        "    }",
        "    return a",
        "}",
        "print('Done')",
    ]
    self.assertEqual(builder.to_string(), "\n".join(expected_lines))


class CommonUtilsTest(absltest.TestCase):

  def test_uppercase_first_letter(self):
    self.assertEqual(common.uppercase_first_letter(""), "")
    self.assertEqual(common.uppercase_first_letter("hello"), "Hello")
    self.assertEqual(common.uppercase_first_letter("1st place"), "1st place")
    self.assertEqual(common.uppercase_first_letter("!wow"), "!wow")
    self.assertEqual(
        common.uppercase_first_letter(" leading space"), " leading space"
    )


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
        functions.return_is_value_of_type(
            ast_nodes.FunctionDecl(
                "func_i", ast_nodes.ValueType("int"), [], "doc"
            ),
            constants.PRIMITIVE_TYPES,
        )
    )
    self.assertFalse(
        functions.return_is_value_of_type(
            ast_nodes.FunctionDecl(
                "func_s", ast_nodes.ValueType("MyStruct"), [], "doc"
            ),
            constants.PRIMITIVE_TYPES,
        )
    )

  def test_return_is_pointer_to_struct(self):
    self.assertTrue(
        functions.return_is_pointer_to_struct(self.func_ret_ptr_struct)
    )
    self.assertFalse(
        functions.return_is_pointer_to_struct(self.func_ret_ptr_int)
    )

  def test_return_is_pointer_to_primitive(self):
    self.assertTrue(
        functions.return_is_pointer_to_primitive(self.func_ret_ptr_int)
    )
    self.assertFalse(
        functions.return_is_pointer_to_primitive(self.func_ret_ptr_struct)
    )

  def test_param_is_primitive_value(self):
    param_prim_val = ast_nodes.FunctionParameterDecl(
        "prim_v", ast_nodes.ValueType("int")
    )
    param_arr = ast_nodes.FunctionParameterDecl(
        "arr_v", ast_nodes.ArrayType(ast_nodes.ValueType("int"), extents=(10,))
    )

    self.assertTrue(functions.param_is_primitive_value(param_prim_val))
    self.assertFalse(functions.param_is_primitive_value(param_arr))

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
        functions.param_is_pointer_to_primitive_value(param_ptr_to_prim)
    )
    self.assertTrue(
        functions.param_is_pointer_to_primitive_value(param_arr_of_prim)
    )
    self.assertFalse(
        functions.param_is_pointer_to_primitive_value(param_ptr_to_struct)
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
        functions.param_is_pointer_to_struct(param_arr_of_struct)
    )
    self.assertTrue(
        functions.param_is_pointer_to_struct(param_ptr_to_struct)
    )
    self.assertFalse(
        functions.param_is_pointer_to_struct(param_ptr_to_ptr)
    )

  def test_should_be_wrapped_with_primitive_ptr_return(self):
    func = ast_nodes.FunctionDecl(
        name="get_data",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("int")),
        parameters=tuple(),
        doc="Returns int pointer",
    )
    self.assertTrue(functions.should_be_wrapped(func))

  def test_generate_function_wrapper_for_simple_func(self):
    func = ast_nodes.FunctionDecl(
        name="get_id",
        return_type=ast_nodes.ValueType("int"),
        parameters=tuple(),
        doc="Returns an integer ID",
    )
    result = functions.generate_function_wrapper(func)
    self.assertEqual(
        result,
        """int get_id_wrapper() {
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
    result = functions.generate_function_wrapper(func)
    self.assertEqual(
        result,
        """int get_id_wrapper(const NumberArray& mat, int nr) {
  UNPACK_ARRAY(mjtNum, mat);
  return get_id(mat_.data(), nr);
}""",
    )

  def test_get_params_string_with_struct_ptr(self):
    param = ast_nodes.FunctionParameterDecl(
        name="my_struct",
        type=ast_nodes.PointerType(ast_nodes.ValueType("mystruct")),
    )
    result = functions.get_params_string((param,))
    self.assertEqual(result, ["Mystruct& my_struct"])

  def test_get_params_string_maybe_with_conversion_struct_ptr(self):
    param = ast_nodes.FunctionParameterDecl(
        name="s",
        type=ast_nodes.PointerType(ast_nodes.ValueType("customstruct")),
    )
    result = functions.get_params_string_maybe_with_conversion((param,))
    self.assertEqual(result, ["s.get()"])

  def test_get_compatible_return_call(self):
    func = ast_nodes.FunctionDecl(
        name="noop",
        return_type=ast_nodes.ValueType("void"),
        parameters=tuple(),
        doc="does nothing",
    )
    result = functions.get_compatible_return_call(func, "noop()")
    self.assertEqual(result, "noop()")

  def test_get_compatible_return_type(self):
    func = ast_nodes.FunctionDecl(
        name="get_name",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("char")),
        parameters=tuple(),
        doc="returns name",
    )
    result = functions.get_compatible_return_type(func)
    self.assertEqual(result.strip(), "std::string")

  def test_get_converted_struct_to_class(self):
    func = ast_nodes.FunctionDecl(
        name="get_struct",
        return_type=ast_nodes.PointerType(ast_nodes.ValueType("mystruct")),
        parameters=tuple(),
        doc="returns struct",
    )
    result = functions.get_converted_struct_to_class(func, "get_struct()")
    self.assertIn("mystruct* result = get_struct();", result)
    self.assertIn("return Mystruct(result)", result)

  def test_is_excluded_function_name(self):
    self.assertTrue(functions.is_excluded_function_name("mjr_function"))
    self.assertTrue(functions.is_excluded_function_name("mjui_function"))
    self.assertTrue(functions.is_excluded_function_name("mju_malloc"))
    self.assertTrue(functions.is_excluded_function_name("mj_makeData"))
    self.assertFalse(
        functions.is_excluded_function_name("mjv_updateScene")
    )
    self.assertFalse(
        functions.is_excluded_function_name("mj_normalFunction")
    )
    self.assertFalse(
        functions.is_excluded_function_name("mju_someOtherFunction")
    )


class StructConstructorCodeBuilderTest(absltest.TestCase):

  def test_constructor_code_with_default_function(self):
    wrapped_structs = structs.generate_wasm_bindings(["mjLROpt"])
    self.assertEqual(
        wrapped_structs["mjLROpt"].wrapped_source,
        """
MjLROpt::MjLROpt(mjLROpt *ptr) : ptr_(ptr) {}
MjLROpt::~MjLROpt() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjLROpt::MjLROpt() : ptr_(new mjLROpt) {
  owned_ = true;
  mj_defaultLROpt(ptr_);
}
MjLROpt::MjLROpt(const MjLROpt &other) : MjLROpt() {
  *ptr_ = *other.get();
}
MjLROpt& MjLROpt::operator=(const MjLROpt &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjLROpt> MjLROpt::copy() {
  return std::make_unique<MjLROpt>(*this);
}
mjLROpt* MjLROpt::get() const {
  return ptr_;
}
void MjLROpt::set(mjLROpt* ptr) {
  ptr_ = ptr;
}
""".strip(),
    )

  def test_constructor_code_without_default_function(self):
    wrapped_structs = structs.generate_wasm_bindings(["mjsElement"])
    self.assertEqual(
        wrapped_structs["mjsElement"].wrapped_source,
        """
MjsElement::MjsElement(mjsElement *ptr) : ptr_(ptr) {}
MjsElement::~MjsElement() {}
mjsElement* MjsElement::get() const {
  return ptr_;
}
void MjsElement::set(mjsElement* ptr) {
  ptr_ = ptr;
}
""".strip(),
    )

  def test_constructor_code_with_fields_with_init(self):
    field_with_init = ast_nodes.StructFieldDecl(
        name="element",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjsElement"),
        ),
        doc="",
    )
    wrapped_field_data = structs._generate_field_data(
        field_with_init, "MjsTexture"
    )
    self.assertEqual(
        structs.build_struct_source(
            "mjsTexture",
            [wrapped_field_data],
        ),
        """
MjsTexture::MjsTexture(mjsTexture *ptr) : ptr_(ptr), element(ptr_->element) {}
MjsTexture::~MjsTexture() {}
mjsTexture* MjsTexture::get() const {
  return ptr_;
}
void MjsTexture::set(mjsTexture* ptr) {
  ptr_ = ptr;
}
""".strip(),
    )

  def test_constructor_code_with_shallow_copy(self):
    self.assertEqual(
        structs.build_struct_source("mjvLight", []),
        """MjvLight::MjvLight(mjvLight *ptr) : ptr_(ptr) {}
MjvLight::~MjvLight() {
  if (owned_ && ptr_) {
    delete ptr_;
  }
}
MjvLight::MjvLight() : ptr_(new mjvLight) {
  owned_ = true;
}
MjvLight::MjvLight(const MjvLight &other) : MjvLight() {
  *ptr_ = *other.get();
}
MjvLight& MjvLight::operator=(const MjvLight &other) {
  if (this == &other) {
    return *this;
  }
  *ptr_ = *other.get();
  return *this;
}
std::unique_ptr<MjvLight> MjvLight::copy() {
  return std::make_unique<MjvLight>(*this);
}
mjvLight* MjvLight::get() const {
  return ptr_;
}
void MjvLight::set(mjvLight* ptr) {
  ptr_ = ptr;
}
""".strip(),
    )


def test_build_struct_header_with_nested_wrappers(self):
  self.assertEqual(
      structs.build_struct_header("mjData", []),
      "",
  )


def test_build_struct_header_basic_struct(self):
  self.assertEqual(
      structs.build_struct_header("mjLROpt", []),
      """
struct MjLROpt {
  MjLROpt();
  MjLROpt(const MjLROpt &);
  MjLROpt &operator=(const MjLROpt &);
  explicit MjLROpt(mjLROpt *ptr);
  ~MjLROpt();
  mjLROpt* get() const { return ptr_; }
  void set(mjLROpt* ptr) { ptr_ = ptr; }

 private:
  mjLROpt* ptr_;
  bool owned_ = false;
};
""".strip(),
  )


class StructFieldCodeBuilderTest(absltest.TestCase):

  def test_primitive_type_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(
        structs.build_primitive_type_definition(field),
        """
int ngeom() const {
  return ptr_->ngeom;
}
void set_ngeom(int value) {
  ptr_->ngeom = value;
}
""".strip(),
    )

  def test_memory_view_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="geom_rgba",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="float"),
        ),
        doc="rgba when material is omitted",
        array_extent=("ngeom", 4),
    )
    self.assertEqual(
        structs.build_memory_view_definition(
            field, "ptr_->ngeom * 4", "ptr_->geom_rgba"
        ),
        """
emscripten::val geom_rgba() const {
  return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_rgba));
}
""".strip(),
    )

  def test_string_field_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="string_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjString"),
        ),
        doc="rgba when material is omitted",
    )
    self.assertEqual(
        structs.build_string_field_definition(field),
        """
mjString string_field() const {
  return (ptr_ && ptr_->string_field) ? *(ptr_->string_field) : "";
}
void set_string_field(const mjString& value) {
  if (ptr_ && ptr_->string_field) {
    *(ptr_->string_field) = value;
  }
}
""".strip(),
    )

  def test_mjvec_pointer_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="vector_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjDoubleVec"),
        ),
        doc="",
    )
    self.assertEqual(
        structs.build_mjvec_pointer_definition(field, "mjDoubleVec"),
        """
mjDoubleVec &vector_field() const {
  return *(ptr_->vector_field);
}""".strip(),
    )

  def test_mjbyte_vec_pointer_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="vector_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjByteVec"),
        ),
        doc="",
    )
    self.assertEqual(
        structs.build_mjvec_pointer_definition(field, "mjByteVec"),
        """
std::vector<uint8_t> &vector_field() const {
  return *(reinterpret_cast<std::vector<uint8_t>*>(ptr_->vector_field));
}""".strip(),
    )

  def test_simple_property_binding(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(
        structs.build_simple_property_binding(field, "MjModel"),
        '.property("ngeom", &MjModel::ngeom)',
    )

  def test_simple_property_binding_with_setter(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        structs.build_simple_property_binding(field, "MjModel", True),
        '.property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom)',
    )

  def test_simple_property_binding_with_return_value_policy_as_ref(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        structs.build_simple_property_binding(
            field,
            "MjModel",
            add_setter=True,
            add_return_value_policy_as_ref=True,
        ),
        '.property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom, reference())',
    )


class StructFieldCodeBuilderTest(absltest.TestCase):

  def test_primitive_type_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(structs._generate_field_data(field, "ngeom").definition,
        """
int ngeom() const {
  return ptr_->ngeom;
}
void set_ngeom(int value) {
  ptr_->ngeom = value;
}
""".strip(),
    )

  def test_memory_view_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="geom_rgba",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="float"),
        ),
        doc="rgba when material is omitted",
        array_extent=("ngeom", 4),
    )
    self.assertEqual(
        structs._generate_field_data(field, "geom_rgba").definition,
        """
emscripten::val geom_rgba() const {
  return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_rgba));
}
""".strip(),
    )

  def test_string_field_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="string_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjString"),
        ),
        doc="rgba when material is omitted",
    )
    self.assertEqual(
        structs._generate_field_data(field, "MjString").definition,
        """
mjString string_field() const {
  return (ptr_ && ptr_->string_field) ? *(ptr_->string_field) : "";
}
void set_string_field(const mjString& value) {
  if (ptr_ && ptr_->string_field) {
    *(ptr_->string_field) = value;
  }
}
""".strip(),
    )

  def test_mjvec_pointer_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="vector_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjDoubleVec"),
        ),
        doc="",
    )
    self.assertEqual(
        structs._generate_field_data(field, "mjDoubleVec").definition,
        """
mjDoubleVec &vector_field() const {
  return *(ptr_->vector_field);
}""".strip(),
    )

  def test_mjbyte_vec_pointer_definition(self):
    field = ast_nodes.StructFieldDecl(
        name="vector_field",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjByteVec"),
        ),
        doc="",
    )
    self.assertEqual(
        structs._generate_field_data(field, "mjByteVec").definition,
        """
std::vector<uint8_t> &vector_field() const {
  return *(reinterpret_cast<std::vector<uint8_t>*>(ptr_->vector_field));
}""".strip(),
    )

  def test_simple_property_binding(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(
        structs._simple_property_binding(field, "MjModel"),
        '.property("ngeom", &MjModel::ngeom)',
    )

  def test_simple_property_binding_with_setter(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        structs._simple_property_binding(field, "MjModel", True),
        '.property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom)',
    )

  def test_simple_property_binding_with_return_value_policy_as_ref(self):
    field = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        structs._simple_property_binding(
            field,
            "MjModel",
            setter=True,
            reference=True,
        ),
        '.property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom, reference())',
    )


class StructFieldHandlerTest(absltest.TestCase):

  def test_scalar_field(self):
    """Test that a scalar type field is handled correctly."""
    field_scalar = ast_nodes.StructFieldDecl(
        name="ngeom",
        type=ast_nodes.ValueType(name="int"),
        doc="number of geoms",
    )

    wrapped_field_data = structs._generate_field_data(field_scalar, "MjModel")
    self.assertEqual(
        wrapped_field_data.definition,
        """
int ngeom() const {
  return ptr_->ngeom;
}
void set_ngeom(int value) {
  ptr_->ngeom = value;
}
""".strip(),
    )
    self.assertEqual(
        wrapped_field_data.binding,
        '.property("ngeom", &MjModel::ngeom, &MjModel::set_ngeom, reference())',
    )

  def test_pointer_type_field(self):
    """Test that a pointer type field is handled correctly."""
    field = ast_nodes.StructFieldDecl(
        name="geom_rgba",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="float"),
        ),
        doc="rgba when material is omitted",
        array_extent=("ngeom", 4),
    )
    wrapped_field_data = structs._generate_field_data(field, "MjModel")

    self.assertEqual(
        wrapped_field_data.definition,
        ("""
emscripten::val geom_rgba() const {
  return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_rgba));
}
""".strip()),
    )

    self.assertEqual(
        wrapped_field_data.binding,
        '.property("geom_rgba", &MjModel::geom_rgba)',
    )

  def test_pointer_type_field_for_byte_type(self):
    """Test that a pointer type field for a byte type is handled correctly."""
    field = ast_nodes.StructFieldDecl(
        name="buffer",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="void"),
        ),
        doc="main buffer; all pointers point in it (nbuffer bytes)",
    )
    wrapped_field_data = structs._generate_field_data(field, "MjData")

    self.assertEqual(
        wrapped_field_data.definition,
        ("""
emscripten::val buffer() const {
  return emscripten::val(emscripten::typed_memory_view(model->nbuffer, static_cast<uint8_t*>(ptr_->buffer)));
}
""".strip()),
    )
    assert wrapped_field_data.binding == '.property("buffer", &MjData::buffer)'

  def test_pointer_type_field_for_mj_struct(self):
    """Test that a pointer type field for a mj struct is handled correctly."""
    field = ast_nodes.StructFieldDecl(
        name="element",
        type=ast_nodes.PointerType(
            inner_type=ast_nodes.ValueType(name="mjsElement"),
        ),
        doc="",
    )
    wrapped_field_data = structs._generate_field_data(field, "MjsTexture")
    self.assertEqual(wrapped_field_data.definition, "MjsElement element;")

    self.assertEqual(
        wrapped_field_data.binding,
        '.property("element", &MjsTexture::element, reference())',
    )
    self.assertEqual(
        wrapped_field_data.initialization,
        ", element(ptr_->element)",
    )

  def test_array_type_field(self):
    """Test that an array type field is handled correctly."""
    field = ast_nodes.StructFieldDecl(
        name="gravity",
        type=ast_nodes.ArrayType(
            inner_type=ast_nodes.ValueType(name="mjtNum"),
            extents=(3,),
        ),
        doc="gravitational acceleration",
    )
    wrapped_field_data = structs._generate_field_data(field, "MjOption")

    self.assertEqual(
        wrapped_field_data.definition,
        ("""
emscripten::val gravity() const {
  return emscripten::val(emscripten::typed_memory_view(3, ptr_->gravity));
}
""".strip()),
    )
    self.assertEqual(
        wrapped_field_data.binding,
        '.property("gravity", &MjOption::gravity)',
    )

  def test_array_field_with_multi_dimensional_array(self):
    """Test that multi-dimensional arrays are handled correctly."""
    field = ast_nodes.StructFieldDecl(
        name="multi_dim_array",
        type=ast_nodes.ArrayType(
            inner_type=ast_nodes.ValueType(name="float"),
            extents=(3, 4),
        ),
        doc="description",
    )
    wrapped_field_data = structs._generate_field_data(field, "MjModel")
    self.assertEqual(
        wrapped_field_data.definition,
        """
emscripten::val multi_dim_array() const {
  return emscripten::val(emscripten::typed_memory_view(12, reinterpret_cast<float*>(ptr_->multi_dim_array)));
}
""".strip(),
    )
    self.assertEqual(
        wrapped_field_data.binding,
        '.property("multi_dim_array", &MjModel::multi_dim_array)',
    )

  def test_parse_array_extent(self):
    """Test that parse_array_extent handles various cases correctly."""
    self.assertEqual(
        structs.parse_array_extent((1, 2), "MjModel", "geom_rgba"),
        "1 * 2",
    )
    self.assertEqual(
        structs.parse_array_extent((1, "ngeom"), "MjModel", "geom_rgba"),
        "1 * ptr_->ngeom",
    )
    self.assertEqual(
        structs.parse_array_extent((1, "mjConstant"), "MjModel", "geom_rgba"),
        "1 * mjConstant",
    )

  def test_resolve_extent(self):
    """Test that resolve_extent handles various cases correctly."""
    # for integer just return the number
    self.assertEqual(structs.resolve_extent(1, "MjModel", "geom_rgba"), "1")

    # for string that does not start with mj, it's a member of the struct
    self.assertEqual(
        structs.resolve_extent("ngeom", "MjModel", "geom_rgba"),
        "ptr_->ngeom",
    )

    # when it's MjData and the field is in MJDATA_SIZES, it should use ptr_->
    self.assertEqual(
        structs.resolve_extent("size_value", "MjData", "efc_AR"),
        "ptr_->size_value",
    )
    # when it's MjData and the field is not in MJDATA_SIZES,
    # it should use model->
    self.assertEqual(
        structs.resolve_extent("size_value", "MjData", "data_field"),
        "model->size_value",
    )
    self.assertEqual(
        structs.resolve_extent("mjConstant", "MjModel", "model_field"),
        "mjConstant",
    )


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
