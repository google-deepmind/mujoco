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
from mujoco.experimental.studio import viewer_app
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

  # Resolve model path, if provided.
  model_path = _MJCF_PATH.value or (
      argv[1] if len(argv) > 1 and not argv[1].startswith('--') else None
  )

  # Load model if path was provided.
  data, model = None, None
  if model_path and (data := parser.parse(model_path)):
    model = data.model

  with launch_passive.launch_passive(
      config,
      viewer_handlers=[viewer_app.ViewerApp()],
  ) as handle:
    # Send the model to the viewer, if we have a model.
    if model is not None:
      handle.send_to_viewer(messages.ModelEvent(model=model, path=model_path))

    # Run the simulation.
    step_control = sim.StepControl()
    try:
      while handle.is_running():
        step_control.advance(model, data)
        model, data, step_control = handle.sync(model, data, step_control)
    except KeyboardInterrupt:
      # Ctrl+C is the documented way to quit; exit cleanly, no traceback.
      print('\nShutting down.', flush=True)


_app.run(main)
