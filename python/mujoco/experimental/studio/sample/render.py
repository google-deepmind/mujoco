# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Render a MuJoCo model to an image."""

import os
import sys

from absl import app as _app
from absl import flags as _flags
import mujoco
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import renderer
from mujoco.experimental.studio import viewer_protocol
from PIL import Image

_MODEL = _flags.DEFINE_string('model', None, 'Path to model file.')
_OUTPUT = _flags.DEFINE_string('output', '', 'Output file to save.')
_GFX = _flags.DEFINE_enum(
    'gfx', None, viewer_protocol.GFX_MODES, 'Rendering graphics mode.'
)
_WIDTH = _flags.DEFINE_integer('width', 320, 'Width of the output image.')
_HEIGHT = _flags.DEFINE_integer('height', 240, 'Height of the output image.')
_STEPS = _flags.DEFINE_integer('steps', 1, 'Number of steps before render.')


def main(argv: list[str]) -> None:
  model_path = _MODEL.value or (
      argv[1] if len(argv) > 1 and not argv[1].startswith('--') else None
  )
  if not model_path:
    raise _app.UsageError('Please provide a model path argument or --model flag.')
  if not _OUTPUT.value:
    raise _app.UsageError('`output` flag is required.')

  try:
    data = parser.parse(model_path)
    model = data.model
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Error loading model from `{model_path}`: {ex}')
    sys.exit(-1)

  for _ in range(_STEPS.value):
    mujoco.mj_step(model, data)

  try:
    r = renderer.Renderer(_GFX.value or '')
    r.Init(model)
    pixels = r.Render(
        model, data, None, None, None, _WIDTH.value, _HEIGHT.value
    )
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Error rendering model: {ex}')
    sys.exit(-2)

  try:
    img = Image.frombytes('RGB', (_WIDTH.value, _HEIGHT.value), pixels)
    img.save(_OUTPUT.value, format=os.path.splitext(_OUTPUT.value)[1][1:])
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Error saving image to `{_OUTPUT.value}`: {ex}')
    sys.exit(-3)

  return 0  # pyrefly: ignore[bad-return]


if __name__ == '__main__':
  _app.run(main)
