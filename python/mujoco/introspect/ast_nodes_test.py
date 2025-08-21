# Copyright 2022 DeepMind Technologies Limited
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
# ==============================================================================
"""Tests for ast_nodes.py."""

from absl.testing import absltest

from . import ast_nodes


class AstNodesTest(absltest.TestCase):

  def test_value_type(self):
    value_type = ast_nodes.ValueType('int')
    self.assertEqual(str(value_type), 'int')
    self.assertEqual(value_type.decl('var'), 'int var')

    const_value_type = ast_nodes.ValueType('double', is_const=True)
    self.assertEqual(str(const_value_type), 'const double')
    self.assertEqual(const_value_type.decl('var2'), 'const double var2')

  def test_pointer_type(self):
    pointer_type = ast_nodes.PointerType(ast_nodes.ValueType('int'))
    self.assertEqual(str(pointer_type), 'int *')
    self.assertEqual(pointer_type.decl('var'), 'int * var')

    const_pointer_type = ast_nodes.PointerType(
        ast_nodes.ValueType('double'), is_const=True)
    self.assertEqual(str(const_pointer_type), 'double * const')
    self.assertEqual(const_pointer_type.decl('var2'), 'double * const var2')

    pointer_to_const_type = ast_nodes.PointerType(
        ast_nodes.ValueType('float', is_const=True))
    self.assertEqual(str(pointer_to_const_type), 'const float *')
    self.assertEqual(pointer_to_const_type.decl('var3'), 'const float * var3')

    restrict_volatile_pointer_to_const_type = ast_nodes.PointerType(
        ast_nodes.ValueType('char', is_const=True),
        is_volatile=True, is_restrict=True)
    self.assertEqual(str(restrict_volatile_pointer_to_const_type),
                     'const char * volatile restrict')
    self.assertEqual(
        restrict_volatile_pointer_to_const_type.decl('var4'),
        'const char * volatile restrict var4')

    pointer_to_array_type = ast_nodes.PointerType(
        ast_nodes.ArrayType(ast_nodes.ValueType('long'), (3,)))
    self.assertEqual(str(pointer_to_array_type), 'long (*)[3]')
    self.assertEqual(pointer_to_array_type.decl('var5'), 'long (* var5)[3]')

    const_pointer_to_array_type = ast_nodes.PointerType(
        ast_nodes.ArrayType(ast_nodes.ValueType('unsigned int'), (4,)),
        is_const=True)
    self.assertEqual(
        str(const_pointer_to_array_type), 'unsigned int (* const)[4]')
    self.assertEqual(
        const_pointer_to_array_type.decl('var6'),
        'unsigned int (* const var6)[4]')

  def test_array_type(self):
    array_type = ast_nodes.ArrayType(ast_nodes.ValueType('int'), (4,))
    self.assertEqual(str(array_type), 'int [4]')
    self.assertEqual(array_type.decl('var'), 'int var[4]')

    array_2d_type = ast_nodes.ArrayType(
        ast_nodes.ValueType('double', is_const=True), (2, 3))
    self.assertEqual(str(array_2d_type), 'const double [2][3]')
    self.assertEqual(array_2d_type.decl('var2'), 'const double var2[2][3]')

    array_to_pointer_type = ast_nodes.ArrayType(
        ast_nodes.PointerType(ast_nodes.ValueType('char', is_const=True)), (5,))
    self.assertEqual(str(array_to_pointer_type), 'const char * [5]')
    self.assertEqual(array_to_pointer_type.decl('var3'), 'const char * var3[5]')

    array_to_const_pointer_type = ast_nodes.ArrayType(
        ast_nodes.PointerType(ast_nodes.ValueType('float'), is_const=True),
        (7,))
    self.assertEqual(str(array_to_const_pointer_type), 'float * const [7]')
    self.assertEqual(
        array_to_const_pointer_type.decl('var4'), 'float * const var4[7]')

  def test_complex_type(self):
    complex_type = ast_nodes.ArrayType(
        extents=[9],
        inner_type=ast_nodes.PointerType(
            ast_nodes.PointerType(
                is_const=True,
                inner_type=ast_nodes.ArrayType(
                    extents=[7],
                    inner_type=ast_nodes.PointerType(
                        is_const=True,
                        inner_type=ast_nodes.PointerType(
                            ast_nodes.ArrayType(
                                extents=(3, 4),
                                inner_type=ast_nodes.ValueType(
                                    'unsigned int', is_const=True)
                                )
                            )
                        )
                    )
                )
            )
        )
    self.assertEqual(str(complex_type),
                     'const unsigned int (* * const (* const * [9])[7])[3][4]')
    self.assertEqual(
        complex_type.decl('var'),
        'const unsigned int (* * const (* const * var[9])[7])[3][4]')

  def test_struct_decl(self):
    struct = ast_nodes.StructDecl(
        name='mystruct',
        declname='struct mystruct_',
        fields=[
            ast_nodes.StructFieldDecl(
                name='foo',
                type=ast_nodes.ValueType('int'),
                doc='',
            )
        ],
    )
    self.assertEqual(struct.decl('var'), 'mystruct var')

  def test_anonymous_struct_decl(self):
    struct = ast_nodes.AnonymousStructDecl(
        fields=[
            ast_nodes.StructFieldDecl(
                name='foo',
                type=ast_nodes.ValueType('int'),
                doc='',
            ),
            ast_nodes.StructFieldDecl(
                name='bar',
                type=ast_nodes.ArrayType(
                    inner_type=ast_nodes.ValueType('float'), extents=(3,)
                ),
                doc='',
            ),
        ],
    )
    self.assertEqual(str(struct), 'struct {int foo; float bar[3];}')
    self.assertEqual(struct.decl('var'), 'struct {int foo; float bar[3];} var')
    self.assertEqual(struct.fields[0].decltype, 'int')
    self.assertEqual(struct.fields[1].decltype, 'float [3]')

  def test_anonymous_union_decl(self):
    union = ast_nodes.AnonymousUnionDecl(
        fields=[
            ast_nodes.StructFieldDecl(
                name='foo',
                type=ast_nodes.ValueType('int'),
                doc='',
            ),
            ast_nodes.StructFieldDecl(
                name='bar',
                type=ast_nodes.ArrayType(
                    inner_type=ast_nodes.ValueType('float'), extents=(3,)
                ),
                doc='',
            ),
        ],
    )
    self.assertEqual(str(union), 'union {int foo; float bar[3];}')
    self.assertEqual(union.decl('var'), 'union {int foo; float bar[3];} var')

  def test_function_parameter_decl_nullable(self):
    param_ptr_nullable = ast_nodes.FunctionParameterDecl(
        name='ptr_param',
        type=ast_nodes.PointerType(ast_nodes.ValueType('int')),
        nullable=True
    )
    self.assertTrue(param_ptr_nullable.nullable)
    self.assertEqual(str(param_ptr_nullable), 'int * ptr_param')

    param_ptr_not_nullable = ast_nodes.FunctionParameterDecl(
        name='ptr_param',
        type=ast_nodes.PointerType(ast_nodes.ValueType('int')),
    )
    self.assertFalse(param_ptr_not_nullable.nullable)
    self.assertEqual(str(param_ptr_not_nullable), 'int * ptr_param')

    param_array_nullable = ast_nodes.FunctionParameterDecl(
        name='array_param',
        type=ast_nodes.ArrayType(ast_nodes.ValueType('float'), (10,)),
        nullable=True
    )
    self.assertTrue(param_array_nullable.nullable)
    self.assertEqual(str(param_array_nullable), 'float array_param[10]')

if __name__ == '__main__':
  absltest.main()
