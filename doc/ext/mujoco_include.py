# Copyright 2022 DeepMind Technologies Limited
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
# ==============================================================================
"""Sphinx extension for the mujoco-include directive."""

import header_reader
from sphinx.application import Sphinx
from sphinx.directives.code import LiteralInclude
from sphinx.util.console import red

_FILENAME = 'includes/references.h'
_ERROR_LINE = 16


class MujocoInclude(LiteralInclude):
  """Extension to LiteralInclude directive for MuJoCo."""

  def run(self):
    mujoco_api = self.env.app.config['mujoco_include_header']
    token = self.arguments[0]
    source = mujoco_api.get(token)
    start_line = _ERROR_LINE
    end_line = _ERROR_LINE

    if source is None:
      print(red(f'Warning: C reference \'{token}\' not found.'))
    else:
      start_line = source.start
      end_line = source.end

    # Config arguments and options for LiteralInclude.
    self.arguments[0] = f'../{_FILENAME}'
    self.options['language'] = 'C'
    self.options['lines'] = f'{start_line}-{end_line}'

    return list(LiteralInclude.run(self))


def setup(app: Sphinx) -> None:
  api = {}
  with open(_FILENAME, 'r') as file:
    api = header_reader.read(file.readlines())

  app.add_config_value('mujoco_include_header', api, '')
  app.add_directive('mujoco-include', MujocoInclude)
