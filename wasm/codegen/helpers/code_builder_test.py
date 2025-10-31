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

"""Tests for the code_builder module."""

from absl.testing import absltest
from wasm.codegen.helpers import code_builder


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


if __name__ == "__main__":
  absltest.main()
