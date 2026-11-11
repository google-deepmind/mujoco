# Copyright 2026 DeepMind Technologies Limited
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
"""Tests for generate_cs_bindings.py."""

from absl.testing import absltest
from absl.testing import parameterized

from . import generate_cs_bindings


class SafeEvalConstantExprTest(parameterized.TestCase):

  @parameterized.parameters(
      ('1<<1', 2),
      ('1 << 4', 16),
      ('0x10 >> 2', 4),
      ('6 & 3', 2),
      ('6 | 1', 7),
      ('6 ^ 3', 5),
      ('~0', -1),
      ('-(2 + 3) * 4', -20),
      ('+5', 5),
      ('1.5 * 2', 3.0),
  )
  def test_arithmetic_and_bitwise(self, expr, expected):
    self.assertEqual(
        generate_cs_bindings._safe_eval_constant_expr(expr, {}), expected
    )

  def test_resolves_constant_names(self):
    constants = {'mjSC_IN': 1, 'mjSC_OUT': 2}
    self.assertEqual(
        generate_cs_bindings._safe_eval_constant_expr(
            'mjSC_IN | mjSC_OUT', constants
        ),
        3,
    )

  @parameterized.parameters(
      ('7/2', 3),
      ('-7/2', -3),
      ('7/-2', -3),
      ('-7/-2', 3),
      ('7/2.0', 3.5),
      ('7%2', 1),
      ('-7%2', -1),
      ('7%-2', 1),
  )
  def test_c_division_semantics(self, expr, expected):
    result = generate_cs_bindings._safe_eval_constant_expr(expr, {})
    self.assertEqual(result, expected)
    self.assertIsInstance(result, type(expected))

  @parameterized.parameters(
      '__import__("os")',
      '2**3',
      '1 if 1 else 2',
      '1 < 2',
      '1 // 2',
      '"abc"',
      'unknown_name',
      'abs(-1)',
      '(1).real',
  )
  def test_rejects_unsupported_expressions(self, expr):
    with self.assertRaises(ValueError):
      generate_cs_bindings._safe_eval_constant_expr(expr, {})

  def test_rejects_syntax_errors(self):
    with self.assertRaises(SyntaxError):
      generate_cs_bindings._safe_eval_constant_expr('extern "C"', {})

  def test_division_by_zero_raises(self):
    with self.assertRaises(ZeroDivisionError):
      generate_cs_bindings._safe_eval_constant_expr('1/0', {})


class ParseConstantsFromHeadersTest(absltest.TestCase):

  def test_skips_unsupported_defines(self):
    header = self.create_tempfile(
        'header.h',
        content="""
#define mjSC_IN 1<<0
#define mjSC_OUT 1<<1
#define mjSC_INOUT mjSC_IN | mjSC_OUT
#define HALF (7/2)
#define BAD 2**3
#define MJ_CALL __import__("os")
#define mjEXTERNC extern "C"
""",
    )
    constants = generate_cs_bindings._parse_constants_from_headers(
        [header.full_path]
    )
    self.assertEqual(constants['mjSC_IN'], 1)
    self.assertEqual(constants['mjSC_OUT'], 2)
    self.assertEqual(constants['mjSC_INOUT'], 3)
    self.assertEqual(constants['HALF'], 3)
    self.assertNotIn('BAD', constants)
    self.assertNotIn('MJ_CALL', constants)
    self.assertNotIn('mjEXTERNC', constants)


if __name__ == '__main__':
  absltest.main()
