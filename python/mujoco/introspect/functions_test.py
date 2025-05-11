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
"""Tests for functions.py."""

from absl.testing import absltest

from . import ast_nodes
from . import functions
from . import type_parsing


class FunctionsTest(absltest.TestCase):

  def test_mj_copyData(self):  # pylint: disable=invalid-name
    func_decl = functions.FUNCTIONS['mj_copyData']
    self.assertEqual(func_decl.name, 'mj_copyData')
    self.assertEqual(func_decl.return_type, type_parsing.parse_type('mjData*'))
    self.assertEqual(
        func_decl.parameters,
        (ast_nodes.FunctionParameterDecl(
            name='dest', type=type_parsing.parse_type('mjData*')),
         ast_nodes.FunctionParameterDecl(
             name='m', type=type_parsing.parse_type('const mjModel*')),
         ast_nodes.FunctionParameterDecl(
             name='src', type=type_parsing.parse_type('const mjData*'))))
    self.assertEqual(
        func_decl.doc, 'Copy mjData. '
        'm is only required to contain the size fields from MJMODEL_INTS.')

  def test_mju_transformSpatial(self):  # pylint: disable=invalid-name
    func_decl = functions.FUNCTIONS['mju_transformSpatial']
    self.assertEqual(func_decl.name, 'mju_transformSpatial')
    self.assertEqual(func_decl.return_type, type_parsing.parse_type('void'))
    self.assertEqual(
        func_decl.parameters,
        (ast_nodes.FunctionParameterDecl(
            name='res', type=type_parsing.parse_type('mjtNum[6]')),
         ast_nodes.FunctionParameterDecl(
             name='vec', type=type_parsing.parse_type('const mjtNum[6]')),
         ast_nodes.FunctionParameterDecl(
             name='flg_force', type=type_parsing.parse_type('int')),
         ast_nodes.FunctionParameterDecl(
             name='newpos', type=type_parsing.parse_type('const mjtNum[3]')),
         ast_nodes.FunctionParameterDecl(
             name='oldpos', type=type_parsing.parse_type('const mjtNum[3]')),
         ast_nodes.FunctionParameterDecl(
             name='rotnew2old',
             type=type_parsing.parse_type('const mjtNum[9]'))))
    self.assertEqual(
        func_decl.doc, 'Coordinate transform of 6D motion or force vector in ' +
        'rotation:translation format. rotnew2old is 3-by-3, ' +
        'NULL means no rotation; flg_force specifies force or motion type.')


if __name__ == '__main__':
  absltest.main()
