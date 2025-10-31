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

from wasm.codegen.helpers import struct_field_code_builder

StructFieldDecl = ast_nodes.StructFieldDecl
ValueType = ast_nodes.ValueType
PointerType = ast_nodes.PointerType
ArrayType = ast_nodes.ArrayType


class StructFieldCodeBuilderTest(absltest.TestCase):

  def test_primitive_type_definition(self):
    field = StructFieldDecl(
        name="ngeom",
        type=ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(
        struct_field_code_builder.build_primitive_type_definition(field),
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
    field = StructFieldDecl(
        name="geom_rgba",
        type=PointerType(
            inner_type=ValueType(name="float"),
        ),
        doc="rgba when material is omitted",
        array_extent=("ngeom", 4),
    )
    self.assertEqual(
        struct_field_code_builder.build_memory_view_definition(
            field, "ptr_->ngeom * 4", "ptr_->geom_rgba"
        ),
        """
emscripten::val geom_rgba() const {
  return emscripten::val(emscripten::typed_memory_view(ptr_->ngeom * 4, ptr_->geom_rgba));
}
""".strip(),
    )

  def test_string_field_definition(self):
    field = StructFieldDecl(
        name="string_field",
        type=PointerType(
            inner_type=ValueType(name="mjString"),
        ),
        doc="rgba when material is omitted",
    )
    self.assertEqual(
        struct_field_code_builder.build_string_field_definition(field),
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
    field = StructFieldDecl(
        name="vector_field",
        type=PointerType(
            inner_type=ValueType(name="mjDoubleVec"),
        ),
        doc="",
    )
    self.assertEqual(
        struct_field_code_builder.build_mjvec_pointer_definition(
            field, "mjDoubleVec"
        ),
        """
mjDoubleVec &vector_field() const {
  return *(ptr_->vector_field);
}""".strip(),
    )

  def test_mjbyte_vec_pointer_definition(self):
    field = StructFieldDecl(
        name="vector_field",
        type=PointerType(
            inner_type=ValueType(name="mjByteVec"),
        ),
        doc="",
    )
    self.assertEqual(
        struct_field_code_builder.build_mjvec_pointer_definition(
            field, "mjByteVec"
        ),
        """
std::vector<uint8_t> &vector_field() const {
  return *(reinterpret_cast<std::vector<uint8_t>*>(ptr_->vector_field));
}""".strip(),
    )

  def test_simple_property_binding(self):
    field = StructFieldDecl(
        name="ngeom",
        type=ValueType(name="int"),
        doc="number of geoms",
    )
    self.assertEqual(
        struct_field_code_builder.build_simple_property_binding(
            field, "MjModel"
        ),
        '.property("ngeom", &MjModel::ngeom)',
    )

  def test_simple_property_binding_with_setter(self):
    field = StructFieldDecl(
        name="ngeom",
        type=ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        struct_field_code_builder.build_simple_property_binding(
            field, "MjModel", True
        ),
        '.property("ngeom", &MjModel::ngeom,'
        " &MjModel::set_ngeom)",
    )

  def test_simple_property_binding_with_return_value_policy_as_ref(self):
    field = StructFieldDecl(
        name="ngeom",
        type=ValueType(name="int"),
        doc="",
    )
    self.assertEqual(
        struct_field_code_builder.build_simple_property_binding(
            field,
            "MjModel",
            add_setter=True,
            add_return_value_policy_as_ref=True,
        ),
        '.property("ngeom", &MjModel::ngeom,'
        " &MjModel::set_ngeom,"
        " reference())",
    )


if __name__ == "__main__":
  absltest.main()
