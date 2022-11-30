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
"""Tests for MuJoCo API header reader."""

from absl.testing import absltest
from absl.testing import parameterized

import header_reader

_EXAMPLE = """
//------- My favorite section --------

// My function
MJAPI void mj_function(int a, int b);

// My other function
// This one has multiple lines
MJAPI const char* mj_other_function(int a, int b,
                                    const char* a);

typedef enum mjEnum_ {
  mjVALUE1 // Some value
  mjVALUE2 // Some other value
} mjEnum;

struct mjStruct_ {
  int value1 // Some value
  int value2 // Some other value
};
typedef struct mjStruct_ mjStruct;
// My favorite struct
typedef struct mjStruct2_ {
  int value1 // Another value
  int value2 // More more value
} mjStruct2;

MJAPI void mj_no_doc(int a, void* b);

//------------ MJAPI FUNCTIONS --------------

void mj_stripped(int a, int b,
                 int c);
"""

_API = header_reader.read([f'{line}\n' for line in _EXAMPLE.split('\n')])


class MuJoCoApiGeneratorTest(parameterized.TestCase):

  def test_enums_line_numbers(self):
    self.assertEqual(_API['mjEnum'].start, 12)
    self.assertEqual(_API['mjEnum'].end, 15)

  def test_structs_line_numbers(self):
    self.assertEqual(_API['mjStruct'].start, 17)
    self.assertEqual(_API['mjStruct'].end, 21)

  def test_structs2_line_numbers(self):
    self.assertEqual(_API['mjStruct2'].start, 23)
    self.assertEqual(_API['mjStruct2'].end, 26)

  def test_function_line_numbers(self):
    self.assertEqual(_API['mj_function'].start, 5)
    self.assertEqual(_API['mj_function'].end, 5)

  def test_function_code(self):
    self.assertEqual(_API['mj_function'].code,
                     'void mj_function(int a, int b);\n')

  def test_function_section(self):
    self.assertEqual(_API['mj_function'].section, 'My favorite section')

  def test_function_doc(self):
    self.assertEqual(_API['mj_function'].doc, 'My function\n')

  def test_multi_line_doc(self):
    self.assertEqual(_API['mj_other_function'].doc,
                     'My other function\nThis one has multiple lines\n')

  def test_multi_line_function(self):
    self.assertEqual(_API['mj_other_function'].start, 9)
    self.assertEqual(_API['mj_other_function'].end, 10)

  def test_no_doc_function(self):
    self.assertEqual(_API['mj_no_doc'].start, 28)
    self.assertEqual(_API['mj_no_doc'].end, 28)

  def test_stripped_functions(self):
    self.assertEqual(_API['mj_stripped'].start, 32)


if __name__ == '__main__':
  absltest.main()
