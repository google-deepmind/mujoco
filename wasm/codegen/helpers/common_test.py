from unittest.mock import mock_open, patch
import unittest
from helpers import common
from helpers import constants

patch = unittest.mock.patch
uppercase_first_letter = common.uppercase_first_letter


class CommonUtilsTest(unittest.TestCase):

    def test_uppercase_first_letter(self):
        self.assertEqual(uppercase_first_letter(""), "")
        self.assertEqual(uppercase_first_letter("hello"), "Hello")
        self.assertEqual(uppercase_first_letter("1st place"), "1st place")
        self.assertEqual(uppercase_first_letter("!wow"), "!wow")
        self.assertEqual(uppercase_first_letter(" leading space"),
                         " leading space")

    def test_try_cast_to_scalar_type(self):
        self.assertEqual(common.try_cast_to_scalar_type("123"), 123)
        self.assertEqual(common.try_cast_to_scalar_type("123.456"), 123.456)
        self.assertEqual(common.try_cast_to_scalar_type("abc"), "abc")

    @patch("builtins.print")
    def test_debug_print_enabled(self, mock_print):
        """Test that a message is printed when debug mode is enabled."""
        constants.STRUCT_DEBUG_MODE = True
        common.debug_print("test message")
        mock_print.assert_called_once_with("test message")

    @patch("builtins.print")
    def test_debug_print_disabled(self, mock_print):
        """Test that a message is not printed when debug mode is disabled."""
        constants.STRUCT_DEBUG_MODE = False
        common.debug_print("test message")
        mock_print.assert_not_called()

    def test_get_file_path(self):
        template_dir = "templates"
        output_dir = "generated"
        filename = "test.txt"
        template_file, output_file = common.get_file_path(
            template_dir, output_dir, filename)
        self.assertEqual(
            template_file,
            "codegen/templates/test.txt",
        )
        self.assertEqual(
            output_file,
            "codegen/generated/test.txt",
        )


if __name__ == "__main__":
    unittest.main()
