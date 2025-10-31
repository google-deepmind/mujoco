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
from wasm.codegen.helpers import common


class CommonUtilsTest(absltest.TestCase):

  def test_uppercase_first_letter(self):
    self.assertEqual(common.uppercase_first_letter(""), "")
    self.assertEqual(common.uppercase_first_letter("hello"), "Hello")
    self.assertEqual(common.uppercase_first_letter("1st place"), "1st place")
    self.assertEqual(common.uppercase_first_letter("!wow"), "!wow")
    self.assertEqual(
        common.uppercase_first_letter(" leading space"), " leading space"
    )

  def test_try_cast_to_scalar_type(self):
    self.assertEqual(common.try_cast_to_scalar_type("123"), 123)
    self.assertEqual(common.try_cast_to_scalar_type("123.456"), 123.456)
    self.assertEqual(common.try_cast_to_scalar_type("abc"), "abc")


if __name__ == "__main__":
  absltest.main()
