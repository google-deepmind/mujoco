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
