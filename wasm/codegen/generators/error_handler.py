"""Generator for the error handler."""

import shutil

from wasm.codegen.helpers import common


# TODO(manevi): Delete this file and use the genrule to handle the file copying
class Generator:
  """Generator for the error handler."""

  def run(self):
    """Runs the generator."""
    template_cc_file, output_cc_file = common.get_file_path(
        "templates", "generated", "error_handler.cc"
    )

    shutil.copyfile(template_cc_file, output_cc_file)
