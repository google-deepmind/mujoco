"""Tests for the code_builder module."""

import unittest
from wasm.codegen.helpers import code_builder


class CodeBuilderTest(unittest.TestCase):

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
  unittest.main()
