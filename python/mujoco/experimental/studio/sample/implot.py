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
"""Example to run studio in the native viewer with responsive ImPlot UI.

This script runs a Studio viewer and adds an 'Inspect Body' window using ImGui
and ImPlot bindings to visualize selected body data. The example demonstrates
how responsive UI layout rules are easily implemented.

Provide an MJCF model file via the first command-line argument to launch.
"""

import math
import os
import sys

from absl import app as _app
from absl import flags as _flags
import mujoco
from mujoco.experimental.studio import launch_passive
from mujoco.experimental.studio import messages
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import viewer_app
from mujoco.experimental.studio import viewer_protocol
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui
from mujoco.experimental.implot import implot

vp = viewer_protocol

_GFX = _flags.DEFINE_enum('gfx', None, vp.GFX_MODES, 'Graphics mode.')
_WIDTH = _flags.DEFINE_integer('width', 1200, 'Width of the output image.')
_HEIGHT = _flags.DEFINE_integer('height', 800, 'Height of the output image')
_VIEWER = _flags.DEFINE_enum_class(
    'viewer', vp.ViewerMode.NATIVE, vp.ViewerMode, 'Viewer mode.'
)


_N_HISTORY = 100

_PLOT_FLAGS = (
    implot.Flags.NoInputs.value  # Disable pan/zoom mouse interaction.
    | implot.Flags.NoMenus.value  # Disable right-click context menu.
    | implot.Flags.NoBoxSelect.value  # Disable drag-to-select regions.
)

_AXIS_FLAGS = (
    implot.AxisFlags.NoGridLines.value  # Hide background grid lines.
    | implot.AxisFlags.NoTickMarks.value  # Hide small tick marks on the axis.
)


def _setup_plot_flags(plot_size: imgui.Vec2) -> int:
  flags = _PLOT_FLAGS
  if min(plot_size.x, plot_size.y) < 300:
    flags |= implot.Flags.NoTitle.value
  if min(plot_size.x, plot_size.y) < 200:
    flags |= implot.Flags.NoLegend.value
  return flags


def _setup_time_axis(plot_size: imgui.Vec2) -> None:
  flags = _AXIS_FLAGS
  if plot_size.x < 300:
    flags |= implot.AxisFlags.NoTickLabels.value
  implot.SetupAxis(implot.Axis.X1, '', flags)
  implot.SetupAxisLimits(implot.Axis.X1, 0, _N_HISTORY)


def _setup_xpos_axis(centroid: list[np.ndarray], plot_size: imgui.Vec2) -> None:
  flags = _AXIS_FLAGS
  if plot_size.y < 300:
    flags |= implot.AxisFlags.NoTickLabels.value
  implot.SetupAxis(implot.Axis.Y1, '', flags)
  min_y = min(c[1] for c in centroid)
  max_y = max(c[1] for c in centroid)
  margin = max((max_y - min_y) * 0.1, 0.05)
  implot.SetupAxisLimits(
      implot.Axis.Y1,
      min_y - margin,
      max_y + margin,
      cond=implot.Cond.Always,
  )


def _setup_angle_axis(plot_size: imgui.Vec2) -> None:
  flags = _AXIS_FLAGS
  if plot_size.y < 300:
    flags |= implot.AxisFlags.NoTickLabels.value
  implot.SetupAxis(implot.Axis.Y1, '', flags)
  implot.SetupAxisLimits(implot.Axis.Y1, -185.0, 185.0)
  implot.SetupAxisTicks(
      implot.Axis.Y1,
      [-180.0, -90.0, 0.0, 90.0, 180.0],
      ['-180', '-90', '0', '90', '180'],
  )


class BodyInspector:
  """Handler that draws body-inspection plots using ImGui/ImPlot."""

  def __init__(self) -> None:
    self._app: viewer_app.ViewerApp | None = None
    self._centroid: list[np.ndarray] = [np.zeros(3) for _ in range(_N_HISTORY)]
    self._euler: list[np.ndarray] = [np.zeros(3) for _ in range(_N_HISTORY)]
    self._body_id: int = -1

  @messages.handler
  def on_viewer_app_init(self, event: viewer_app.ViewerAppInitEvent) -> None:
    """Caches the ViewerApp reference on startup."""
    assert isinstance(event.viewer_app, viewer_app.ViewerApp)
    self._app = event.viewer_app

  @messages.handler
  def inspect_body(self, _: messages.BuildGuiEvent) -> None:
    """Renders the body-inspection charts in ImGui/ImPlot."""
    app = self._app
    if app is None:
      return
    if app.viewer.perturb.select > 0:
      self._body_id = app.viewer.perturb.select

    # Display selected body information.
    if self._body_id > 0:
      body_name = mujoco.mj_id2name(
          app.model, int(mujoco.mjtObj.mjOBJ_BODY), self._body_id
      )

      io = imgui.GetIO()
      imgui.SetNextWindowPos(
          imgui.Vec2(io.DisplaySize.x * 0.5, io.DisplaySize.y * 0.5),
          imgui.Cond.FirstUseEver,
          imgui.Vec2(0.5, 0.5),
      )
      imgui.SetNextWindowSize(imgui.Vec2(1200, 600), imgui.Cond.FirstUseEver)

      # Note: The window title uses the special "###" markup to ensure the imgui
      # ID for the window is constant for all body names.  This is needed for
      # the window to retain its state for all bodies.
      window_title = (
          f'Inspect Body {body_name or "(???)"!r} ({self._body_id})###Plot'
      )
      if imgui.Begin(window_title):
        avail = imgui.GetContentRegionAvail()
        wide = avail.x > avail.y

        # Add a small padding factor to prevent scrollbars.
        plot_size = imgui.Vec2(
            avail.x * 0.5 - 4 if wide else avail.x,
            avail.y if wide else avail.y * 0.5 - 4,
        )

        plot_flags = _setup_plot_flags(plot_size)
        if implot.BeginPlot('Centroid vs Time', plot_size, flags=plot_flags):
          _setup_time_axis(plot_size)
          _setup_xpos_axis(self._centroid, plot_size)
          implot.PlotLine(
              'x', range(_N_HISTORY), [c[0] for c in self._centroid]
          )
          implot.PlotLine(
              'y', range(_N_HISTORY), [c[1] for c in self._centroid]
          )
          implot.PlotLine(
              'z', range(_N_HISTORY), [c[2] for c in self._centroid]
          )
          implot.EndPlot()

        if wide:
          imgui.SameLine()

        if implot.BeginPlot('Euler Angle vs Time', plot_size, flags=plot_flags):
          _setup_time_axis(plot_size)
          _setup_angle_axis(plot_size)
          implot.PlotLine(
              'roll', range(_N_HISTORY), [e[0] for e in self._euler]
          )
          implot.PlotLine(
              'pitch', range(_N_HISTORY), [e[1] for e in self._euler]
          )
          implot.PlotLine('yaw', range(_N_HISTORY), [e[2] for e in self._euler])
          implot.EndPlot()
      imgui.End()

    self._centroid.pop(0)
    self._euler.pop(0)
    if self._body_id > 0 and self._body_id < app.model.nbody:
      self._centroid.append(app.data.xpos[self._body_id].copy())
      quat = app.data.xquat[self._body_id]
      mat = np.zeros(9)
      mujoco.mju_quat2Mat(mat, quat)
      # mat is row-major 3x3: R[i,j] = mat[3*i + j].
      roll = math.atan2(mat[7], mat[8])
      pitch = math.atan2(-mat[6], math.sqrt(mat[7] ** 2 + mat[8] ** 2))
      yaw = math.atan2(mat[3], mat[0])
      self._euler.append(np.degrees(np.array([roll, pitch, yaw])))
    else:
      self._centroid.append(np.zeros(3))
      self._euler.append(np.zeros(3))


def main(argv: list[str]) -> None:
  if len(argv) < 2:
    print('Usage: implot <model_path.xml>')
    sys.exit(1)

  if (data := parser.parse(argv[1])) is None:
    print(f'Error loading model from {argv[1]!r}')
    sys.exit(1)
  model = data.model

  title = os.path.basename(sys.argv[0])

  config = viewer_protocol.ViewerConfig(
      title=title,
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      viewer_mode=_VIEWER.value,
  )

  with launch_passive.launch_passive(
      config,
      viewer_handlers=[viewer_app.ViewerApp(), BodyInspector()],
  ) as handle:
    handle.send_to_viewer(messages.ModelEvent(model=model))

    step_control = sim.StepControl()
    try:
      while handle.is_running():
        step_control.advance(model, data)
        model, data, step_control = handle.sync(model, data, step_control)
    except KeyboardInterrupt:
      # Ctrl+C is the documented way to quit; exit cleanly, no traceback.
      print('\nShutting down.', flush=True)


if __name__ == '__main__':
  _app.run(main)
