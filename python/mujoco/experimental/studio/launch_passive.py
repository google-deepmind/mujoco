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
"""Unified non-blocking viewer launcher (backward-compatible wrapper).

Launches the viewer in a daemon thread and returns a ``ViewerHandle``
that the calling thread uses to push simulation state into the viewer.
"""

from typing import Any

from mujoco.experimental.studio import viewer_handle
from mujoco.experimental.studio import viewer_protocol


def launch_passive(
    config: viewer_protocol.ViewerConfig,
    *,
    viewer_plugins: list[Any] | None = None,
    sim_plugins: list[Any] | None = None,
) -> viewer_handle.ViewerHandle:
  """Launches the viewer in a daemon thread without blocking.

  The viewer runs on the rendering thread and renders the scene along with any
  GUI built by registered plugins (such as ``ViewerApp``). The caller keeps
  running and pushes state via ``handle.sync()``.

  Args:
    config: Viewer window configuration.
    viewer_plugins: Optional list of viewer-side plugin instances, which are
      classes with methods decorated with ``@handler``.
    sim_plugins: Optional list of sim-side plugin instances, which are classes
      with methods decorated with ``@handler``. Include a stepping plugin
      (e.g. ``step_control.StepControl()``) to advance the physics on each
      ``sync``; without one nothing steps and the sim loop must pace itself.

  Returns:
    A ViewerHandle for interacting with the viewer.
  """
  if config.gfx in ('web', 'webgl'):
    from mujoco.experimental.studio import launch_web  # pylint: disable=g-import-not-at-top

    return launch_web.launch_web(
        config,
        viewer_plugins=viewer_plugins,
        sim_plugins=sim_plugins,
    )
  from mujoco.experimental.studio import launch_native  # pylint: disable=g-import-not-at-top

  return launch_native.launch_native(
      config,
      viewer_plugins=viewer_plugins,
      sim_plugins=sim_plugins,
  )
