"""Generator for the constants."""

from wasm.codegen.helpers import common


# TODO(manevi): Delete this file and use the genrule to handle the file copying
class Generator:
  """Generator for the constants."""

  def run(self):
    """Runs the generator."""
    template_cc_file, output_cc_file = common.get_file_path(
        "templates", "generated", "constants.cc"
    )

    with open(template_cc_file, "r") as f_template:
      template_content = f_template.read()

    with open(output_cc_file, "w") as f_output:
      f_output.write(template_content)
