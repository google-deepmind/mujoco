import unittest
from helpers import binding_builder

ERROR_MESSAGE = """
The file '{}' needs to be updated, please run:
PYTHONPATH=../python/mujoco:./codegen python codegen/update.py""".lstrip()


class BindingsDiffTest(unittest.TestCase):

  def setUp(self):
    super().setUp()
    with open('./codegen/generated/bindings.h', 'r') as f:
      self.generated_hdr = f.read()
    with open('./codegen/generated/bindings.cc', 'r') as f:
      self.generated_src = f.read()

    self.template_path_h = './codegen/templates/bindings.h'
    self.template_path_cc = './codegen/templates/bindings.cc'
    self.generated_path_h = './codegen/generated/bindings.h'
    self.generated_path_cc = './codegen/generated/bindings.cc'

    self.builder = binding_builder.BindingBuilder(
        self.template_path_h,
        self.template_path_cc,
        self.generated_path_h,
        self.generated_path_cc,
    )

  def test_bindings_source(self):
    generator_output = (self.builder.set_enums().set_structs().set_functions().
                        to_string_source())
    self.assertEqual(
        generator_output,
        self.generated_src,
        msg=ERROR_MESSAGE.format('bindings.cc'),
    )

  def test_bindings_header(self):
    generator_output = (self.builder.set_headers().to_string_header())
    self.assertEqual(
        generator_output,
        self.generated_hdr,
        msg=ERROR_MESSAGE.format('bindings.h'),
    )


if __name__ == '__main__':
  unittest.main()
