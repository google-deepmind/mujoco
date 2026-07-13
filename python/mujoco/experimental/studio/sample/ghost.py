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
"""Example to run studio with a time-delayed ghost overlay of model geoms.

This script runs a Studio viewer that renders time-delayed semi-transparent
ghosts of the primitive mjvGeoms in the model.
"""

import collections
import os
import sys

from absl import app as _app
from absl import flags as _flags
import mujoco
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import studio_app_events
from mujoco.experimental.studio import ux
from mujoco.experimental.studio import viewer_protocol
from mujoco.experimental.studio import viewer_utils
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui

vp = viewer_protocol

_GFX = _flags.DEFINE_enum('gfx', None, vp.GFX_MODES, 'Graphics mode.')
_WIDTH = _flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = _flags.DEFINE_integer('height', 800, 'Height of the output image')
_VIEWER = _flags.DEFINE_enum_class(
    'viewer', vp.ViewerMode.NATIVE, vp.ViewerMode, 'Viewer mode.'
)


class GhostRenderer:
  """Handler that renders time-delayed semi-transparent ghost geoms."""

  def __init__(self) -> None:
    self._viewer: viewer_protocol.Viewer | None = None
    self._delay: float = 0.5
    self._ghost_rgba: list[float] = [0.4, 0.5, 0.9, 0.5]
    self._history: collections.deque[tuple[float, np.ndarray, np.ndarray]] = (
        collections.deque()
    )
    self._last_time: float | None = None

  def _is_fixed_body(self, body_id: int) -> bool:
    """Check if a body is fixed (welded to world, not attached to mocap)."""
    assert self._viewer is not None
    model = self._viewer.model
    is_weld = model.body_weldid[body_id] == 0
    root_id = model.body_rootid[body_id]
    return bool(is_weld and model.body_mocapid[root_id] < 0)

  @messages.handler
  def on_viewer_init(self, event: viewer_protocol.ViewerInitEvent) -> None:
    """Caches the Viewer reference for later access to model/data."""
    self._viewer = event.viewer

  @messages.handler
  def on_model(self, event: messages.ModelEvent) -> bool:
    """Resets ghost-specific state when the model changes."""
    del event  # Model/data are accessed via self._viewer.
    self._history.clear()
    self._last_time = None
    return False

  @messages.handler
  def on_build_gui(self, _: messages.BuildGuiEvent) -> None:
    """Renders the ghost configuration GUI panel."""
    if imgui.Begin('Ghost', flags=int(imgui.WindowFlags.AlwaysAutoResize)):
      changed_delay, delay = imgui.InputFloat(
          'Delay (s)', self._delay, 0.05, 0.5, '%.2f'
      )
      if changed_delay:
        self._delay = max(0.0, delay)

      changed_color, rgba = imgui.ColorEdit4('Color', self._ghost_rgba)
      if changed_color:
        self._ghost_rgba = rgba
    imgui.End()

  @messages.handler
  def on_update(self, _: messages.UpdateEvent) -> None:
    """Handles mouse events, applies perturbations, and updates ghost geoms."""
    assert self._viewer is not None
    model = self._viewer.model
    data = self._viewer.data

    studio_app_events.handle_mouse_events(
        model,
        data,
        self._viewer.camera,
        self._viewer.vis_options,
        self._viewer.perturb,
        ux.UxState(),
    )
    viewer_utils.apply_perturb(self._viewer, model, data)

    self._viewer.extra_geoms.clear()
    if self._last_time is None or data.time < self._last_time:
      self._history.clear()
    if not self._history or data.time > self._last_time:
      self._history.append((
          data.time,
          data.geom_xpos.copy(),
          data.geom_xmat.copy(),
      ))
      self._last_time = data.time

    target_time = data.time - self._delay
    while len(self._history) > 1 and self._history[1][0] <= target_time:
      self._history.popleft()

    _, xpos, xmat = self._history[0]
    if len(xpos) != model.ngeom or len(xmat) != model.ngeom:
      return

    ghost_rgba = np.array(self._ghost_rgba, dtype=np.float32)
    for i in range(model.ngeom):
      if (
          self._is_fixed_body(model.geom_bodyid[i])
          or model.geom_rgba[i, 3] == 0
          or model.geom_group[i] > 2
      ):
        continue

      geom = mujoco.MjvGeom()
      mujoco.mjv_initGeom(
          geom,
          int(model.geom_type[i]),
          model.geom_size[i],
          xpos[i],
          xmat[i].flatten(),
          ghost_rgba,
      )
      geom.dataid = model.geom_dataid[i]
      geom.objtype = int(mujoco.mjtObj.mjOBJ_GEOM)
      geom.objid = i
      self._viewer.extra_geoms.append(geom)


def main(argv: list[str]) -> None:
  if len(argv) != 2:
    raise _app.UsageError('Please provide exactly one MJCF path argument.')

  data = None
  try:
    if (data := parser.parse(argv[1])) is None:
      raise ValueError('parser returned None')
  except Exception as ex:  # pylint: disable=broad-except
    print(f'Failed to load model from {argv[1]!r}: {ex}')
    sys.exit(1)
  model = data.model

  config = viewer_protocol.ViewerConfig(
      title=os.path.basename(sys.argv[0]),
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      viewer_mode=_VIEWER.value,
  )

  ghost_renderer = GhostRenderer()

  with launch_passive.launch_passive(
      config,
      viewer_handlers=[ghost_renderer],
  ) as handle:
    handle.send_to_viewer(messages.ModelEvent(model=model))

    step_control = sim.StepControl()
    while handle.is_running():
      step_control.advance(model, data)
      model, data, step_control = handle.sync(model, data, step_control)


if __name__ == '__main__':
  _app.run(main)
