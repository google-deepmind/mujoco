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
"""Interactive Studio GUI viewer for MuJoCo.

Pass ``--plugins=a.py,b.py`` to load ``ViewerPlugin`` and ``SimPlugin``
subclasses from Python files (see ``plugin.load_plugins``). With
``--hot_reload=all``, saving those files, the model file, or its assets reloads
them without restarting (``plugins``, ``model``, or plugin class names narrow
that down).
"""

import os

from absl import app as _app
from absl import flags as _flags
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import plugin
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol

vp = viewer_protocol

_MODEL = _flags.DEFINE_string('model', None, 'Path to model file.')
_PLUGINS = _flags.DEFINE_list(
    'plugins',
    [],
    'Comma-separated list of Python plugin files (.py) to load.',
)
_GFX = _flags.DEFINE_enum(
    'gfx', None, vp.GFX_MODES, 'Graphics mode ("web" launches Web Viewer).'
)
_PORT = _flags.DEFINE_integer(
    'port', 0, 'Web Viewer port (0 picks first free port >= 8080).'
)
_WIDTH = _flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = _flags.DEFINE_integer('height', 800, 'Height of the output image')
_HOT_RELOAD = _flags.DEFINE_string(
    'hot_reload',
    '',
    "What is reloaded when modified. Options are '', 'plugins', 'model',"
    " 'all', 'Plugin1,Plugin2,...'. Default is '', meaning nothing.",
)


def _resolve(path: str) -> str:
  """Resolves a relative path against the directory the command was run from."""
  # Build tools such as Bazel run binaries from their output tree and export
  # the invoking directory as BUILD_WORKING_DIRECTORY; elsewhere it is unset.
  if path:
    path = os.path.join(os.environ.get('BUILD_WORKING_DIRECTORY', ''), path)
  return path


def main(argv: list[str]) -> None:
  config = vp.ViewerConfig(
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      http_port=_PORT.value,
      hot_reload=_HOT_RELOAD.value,
  )

  # Resolve model path, if provided.
  model_path = _MODEL.value or (
      argv[1] if len(argv) > 1 and not argv[1].startswith('--') else ''
  )
  user_plugins = plugin.load_plugins(map(_resolve, _PLUGINS.value))

  launch_passive.run(
      config,
      model_path=_resolve(model_path),
      plugins=[
          viewer_app.ViewerApp(),
          *viewer_app.SIM_PLUGINS,
          *user_plugins,
      ],
  )


_app.run(main)
