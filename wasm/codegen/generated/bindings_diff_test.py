from absl.testing import absltest

from google3.pyglib import resources
from google3.third_party.mujoco.wasm.codegen import binding_builder

ERROR_MESSAGE = """
The file '{}' needs to be updated, please run:
blaze run //third_party/mujoco/wasm/codegen:update""".lstrip()


class BindingsDiffTest(absltest.TestCase):

  def setUp(self):
    super().setUp()
    self.generated_hdr = resources.GetResource(
        'google3/third_party/mujoco/wasm/codegen/generated/bindings.h'
    ).decode()
    self.generated_src = resources.GetResource(
        'google3/third_party/mujoco/wasm/codegen/generated/bindings.cc'
    ).decode()

    self.template_path_h = resources.GetResourceFilenameInDirectoryTree(
        'google3/third_party/mujoco/wasm/codegen/templates/bindings.h'
    )
    self.template_path_cc = resources.GetResourceFilenameInDirectoryTree(
        'google3/third_party/mujoco/wasm/codegen/templates/bindings.cc'
    )
    self.generated_path_h = resources.GetResourceFilenameInDirectoryTree(
        'google3/third_party/mujoco/wasm/codegen/generated/bindings.h'
    )
    self.generated_path_cc = resources.GetResourceFilenameInDirectoryTree(
        'google3/third_party/mujoco/wasm/codegen/generated/bindings.cc'
    )

    self.builder = binding_builder.BindingBuilder(
        self.template_path_h, self.template_path_cc,
        self.generated_path_h,
        self.generated_path_cc,
    )

  def test_bindings_source(self):
    generator_output = (
        self.builder.set_enums()
        .set_structs()
        .set_functions()
        .to_string_source()
    )
    self.assertEqual(
        generator_output,
        self.generated_src,
        msg=ERROR_MESSAGE.format('bindings.cc'),
    )

  def test_bindings_header(self):
    generator_output = (
        self.builder
        .set_headers()
        .to_string_header()
    )
    self.assertEqual(
        generator_output,
        self.generated_hdr,
        msg=ERROR_MESSAGE.format('bindings.h'),
    )


if __name__ == '__main__':
  absltest.main()
