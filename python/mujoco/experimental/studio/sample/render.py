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

from absl import app
from absl import flags
import mujoco
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import renderer
from PIL import Image

_MODEL = flags.DEFINE_string('model', '', 'Model file to load.')
_OUTPUT = flags.DEFINE_string('output', '', 'Output file to save.')
_GFX = flags.DEFINE_string('gfx', '', 'Renderer to use.')
_WIDTH = flags.DEFINE_integer('width', 320, 'Width of the output image.')
_HEIGHT = flags.DEFINE_integer('height', 240, 'Height of the output image.')
_STEPS = flags.DEFINE_integer('steps', 1, 'Number of steps before render.')


def main(argv):
  if len(argv) > 1:
    raise app.UsageError('Too many command-line arguments.')
  if not _MODEL.value:
    raise ValueError('`model` flag is required.')
  if not _OUTPUT.value:
    raise ValueError('`output flag is required.')

  try:
    data = parser.parse(_MODEL.value)
    model = data.model
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Error loading model from `{_MODEL.value}`: {ex}')
    sys.exit(-1)

  for _ in range(_STEPS.value):
    mujoco.mj_step(model, data)

  try:
    r = renderer.Renderer(_GFX.value)
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

  return 0


if __name__ == '__main__':
  app.run(main)
