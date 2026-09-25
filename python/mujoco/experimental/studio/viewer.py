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
"""Interactive Studio GUI viewer for MuJoCo."""

from absl import app as _app
from absl import flags as _flags
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import step_control
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol

vp = viewer_protocol

_MODEL = _flags.DEFINE_string('model', None, 'Path to model file.')
_GFX = _flags.DEFINE_enum(
    'gfx', None, vp.GFX_MODES, 'Graphics mode ("web" launches Web Viewer).'
)
_PORT = _flags.DEFINE_integer(
    'port', 0, 'Web Viewer port (0 picks first free port >= 8080).'
)
_WIDTH = _flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = _flags.DEFINE_integer('height', 800, 'Height of the output image')


def main(argv: list[str]) -> None:
  config = vp.ViewerConfig(
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      http_port=_PORT.value,
  )

  # Resolve model path, if provided.
  model_path = _MODEL.value or (
      argv[1] if len(argv) > 1 and not argv[1].startswith('--') else None
  )

  # Load model if path was provided.
  data, model = None, None
  if model_path and (data := parser.parse(model_path)):
    model = data.model

  launch_passive.run(
      config,
      model=model,
      data=data,
      model_path=model_path,
      viewer_plugins=[viewer_app.ViewerApp()],
      sim_plugins=[step_control.StepControl()],
  )


_app.run(main)
