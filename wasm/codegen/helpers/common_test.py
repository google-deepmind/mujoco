from absl.testing import absltest
from wasm.codegen.helpers import common


class CommonUtilsTest(absltest.TestCase):

  def test_uppercase_first_letter(self):
    self.assertEqual(common.uppercase_first_letter(""), "")
    self.assertEqual(common.uppercase_first_letter("hello"), "Hello")
    self.assertEqual(common.uppercase_first_letter("1st place"), "1st place")
    self.assertEqual(common.uppercase_first_letter("!wow"), "!wow")
    self.assertEqual(common.uppercase_first_letter(" leading space"), " leading space")

  def test_try_cast_to_scalar_type(self):
    self.assertEqual(common.try_cast_to_scalar_type("123"), 123)
    self.assertEqual(common.try_cast_to_scalar_type("123.456"), 123.456)
    self.assertEqual(common.try_cast_to_scalar_type("abc"), "abc")


if __name__ == "__main__":
  absltest.main()
