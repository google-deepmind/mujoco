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
"""This script runs Studio from Python, visualized in a native viewer."""

from absl import app as absl_app
from absl import flags as absl_flags
from mujoco.experimental.studio import native_viewer
from mujoco.experimental.studio import studio_app

_GFX = absl_flags.DEFINE_string('gfx', '', 'Rendering graphics mode.')
_WIDTH = absl_flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = absl_flags.DEFINE_integer('height', 800, 'Height of the output image')


def main(argv: list[str]) -> None:
  app = studio_app.StudioApp.from_argv(argv)

  # Initialize the viewer.
  viewer = native_viewer.NativeViewer(
      app.model,
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value,
  )

  # Main viewer loop.
  while viewer.is_running():
    if not app.update_from_viewer(viewer):
      break

    app.build_gui(viewer.camera, viewer.vis_options, viewer.render_flags)

    viewer.sync(app.model, app.data)

  viewer.stop()


if __name__ == '__main__':
  absl_app.run(main)
