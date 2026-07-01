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
from absl import flags
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import viewer_protocol

vp = viewer_protocol

_GFX = flags.DEFINE_enum('gfx', None, vp.GFX_MODES, 'Graphics mode.')
_WIDTH = flags.DEFINE_integer('width', 1200, 'Width of output window.')
_HEIGHT = flags.DEFINE_integer('height', 800, 'Height of output window.')
_MJCF_PATH = flags.DEFINE_string('mjcf', None, 'Path to MJCF file.')
_VIEWER = flags.DEFINE_enum_class(
    'viewer', vp.ViewerMode.NATIVE, vp.ViewerMode, 'Viewer mode.'
)


def main(argv: list[str]) -> None:
  config = vp.ViewerConfig(
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      viewer_mode=_VIEWER.value,
  )

  # Get the model path, if provided.
  model_path = None
  if _MJCF_PATH.value is not None:
    model_path = _MJCF_PATH.value
  elif len(argv) > 1 and not argv[1].startswith('--'):
    model_path = argv[1]

  with launch_passive.launch_passive(config) as handle:
    # Load the model if we have a path.
    model, data = None, None
    if model_path is not None:
      data = parser.parse(model_path)
      if data is not None:
        model = data.model

    # Send the model to the viewer, if we have a model.
    if model is not None:
      handle.send_to_viewer(messages.ModelEvent(model=model))

    # Run the simulation.
    step_control = sim.StepControl()
    while handle.is_running():
      step_control.advance(model, data)
      model, data, step_control = handle.sync(model, data, step_control)


_app.run(main)
