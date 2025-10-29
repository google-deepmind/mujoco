import unittest
from introspect import ast_nodes

from wasm.codegen.helpers import struct_field_handler


StructFieldDecl = ast_nodes.StructFieldDecl
ValueType = ast_nodes.ValueType
PointerType = ast_nodes.PointerType
ArrayType = ast_nodes.ArrayType


class StructFieldHandlerTest(unittest.TestCase):

  def test_scalar_field(self):
    """Test that a scalar type field is handled correctly."""
    field_scalar = StructFieldDecl(
        name='ngeom',
        type=ValueType(name='int'),
        doc='number of geoms',
    )

    field_handler_scalar = struct_field_handler.StructFieldHandler(
        field_scalar, 'MjModel'
    )
    wrapped_field_data = field_handler_scalar.generate()
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
    field = StructFieldDecl(
        name='geom_rgba',
        type=PointerType(
            inner_type=ValueType(name='float'),
        ),
        doc='rgba when material is omitted',
        array_extent=('ngeom', 4),
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field, 'MjModel'
    ).generate()

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
    field = StructFieldDecl(
        name='buffer',
        type=PointerType(
            inner_type=ValueType(name='void'),
        ),
        doc='main buffer; all pointers point in it (nbuffer bytes)',
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field, 'MjData'
    ).generate()

    self.assertEqual(
        wrapped_field_data.definition,
        ("""
emscripten::val buffer() const {
  return emscripten::val(emscripten::typed_memory_view(model->nbuffer, static_cast<uint8_t*>(ptr_->buffer)));
}
""".strip()),
    )
    assert (
        wrapped_field_data.binding
        == '.property("buffer", &MjData::buffer)'
    )

  def test_pointer_type_field_for_mj_struct(self):
    """Test that a pointer type field for a mj struct is handled correctly."""
    field = StructFieldDecl(
        name='element',
        type=PointerType(
            inner_type=ValueType(name='mjsElement'),
        ),
        doc='',
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field, 'MjsTexture'
    ).generate()
    self.assertEqual(wrapped_field_data.definition, "MjsElement element;")

    self.assertEqual(
        wrapped_field_data.binding,
        '.property("element", &MjsTexture::element, reference())',
    )
    self.assertEqual(
        wrapped_field_data.initialization,
        ', element(ptr_->element)',
    )

  def test_array_type_field(self):
    """Test that an array type field is handled correctly."""
    field = StructFieldDecl(
        name='gravity',
        type=ArrayType(
            inner_type=ValueType(name='mjtNum'),
            extents=(3,),
        ),
        doc='gravitational acceleration',
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field, 'MjOption'
    ).generate()

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
    field = StructFieldDecl(
        name='multi_dim_array',
        type=ArrayType(
            inner_type=ValueType(name='float'),
            extents=(3, 4),
        ),
        doc='description',
    )
    wrapped_field_data = struct_field_handler.StructFieldHandler(
        field, 'MjModel'
    ).generate()
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
        struct_field_handler.parse_array_extent((1, 2), 'MjModel', 'geom_rgba'),
        '1 * 2',
    )
    self.assertEqual(
        struct_field_handler.parse_array_extent(
            (1, 'ngeom'), 'MjModel', 'geom_rgba'
        ),
        '1 * ptr_->ngeom',
    )
    self.assertEqual(
        struct_field_handler.parse_array_extent(
            (1, 'mjConstant'), 'MjModel', 'geom_rgba'
        ),
        '1 * mjConstant',
    )

  def test_resolve_extent(self):
    """Test that resolve_extent handles various cases correctly."""
    # for integer just return the number
    self.assertEqual(
        struct_field_handler.resolve_extent(1, 'MjModel', 'geom_rgba'), '1'
    )

    # for string that does not start with mj, it's a member of the struct
    self.assertEqual(
        struct_field_handler.resolve_extent('ngeom', 'MjModel', 'geom_rgba'),
        'ptr_->ngeom',
    )

    # when it's MjData and the field is in MJDATA_SIZES, it should use ptr_->
    self.assertEqual(
        struct_field_handler.resolve_extent('size_value', 'MjData', 'efc_AR'),
        'ptr_->size_value',
    )
    # when it's MjData and the field is not in MJDATA_SIZES,
    # it should use model->
    self.assertEqual(
        struct_field_handler.resolve_extent(
            'size_value', 'MjData', 'data_field'
        ),
        'model->size_value',
    )
    self.assertEqual(
        struct_field_handler.resolve_extent(
            'mjConstant', 'MjModel', 'model_field'
        ),
        'mjConstant',
    )


if __name__ == '__main__':
  unittest.main()
