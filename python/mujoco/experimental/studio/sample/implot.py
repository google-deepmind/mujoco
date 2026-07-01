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
from mujoco.experimental.studio import messages as msg
from mujoco.experimental.studio import parser
from mujoco.experimental.studio import sim
from mujoco.experimental.studio import viewer_app as va
from mujoco.experimental.studio import viewer_protocol as vp
import numpy as np

from mujoco.experimental.dear_imgui import dear_imgui as imgui
from mujoco.experimental.implot import implot

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


class PlottingUi(va.ViewerGuiHook):
  """Custom GUI component maintaining history buffers for ImPlot charts."""

  def __init__(self):
    self.centroid = [np.zeros(3) for _ in range(_N_HISTORY)]
    self.euler = [np.zeros(3) for _ in range(_N_HISTORY)]
    self.body_id = -1

  def build_gui(self, app: va.ViewerApp) -> None:
    # Inspect the perturb.select body
    if app.viewer.perturb.select > 0:
      self.body_id = app.viewer.perturb.select

    # Display selected body information.
    if self.body_id > 0:
      body_name = mujoco.mj_id2name(
          app.model, int(mujoco.mjtObj.mjOBJ_BODY), self.body_id
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
          f'Inspect Body {body_name or "(???)"!r} ({self.body_id})###Plot'
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
          _setup_xpos_axis(self.centroid, plot_size)
          implot.PlotLine('x', range(_N_HISTORY), [c[0] for c in self.centroid])
          implot.PlotLine('y', range(_N_HISTORY), [c[1] for c in self.centroid])
          implot.PlotLine('z', range(_N_HISTORY), [c[2] for c in self.centroid])
          implot.EndPlot()

        if wide:
          imgui.SameLine()

        if implot.BeginPlot('Euler Angle vs Time', plot_size, flags=plot_flags):
          _setup_time_axis(plot_size)
          _setup_angle_axis(plot_size)
          implot.PlotLine('roll', range(_N_HISTORY), [e[0] for e in self.euler])
          implot.PlotLine(
              'pitch', range(_N_HISTORY), [e[1] for e in self.euler]
          )
          implot.PlotLine('yaw', range(_N_HISTORY), [e[2] for e in self.euler])
          implot.EndPlot()
      imgui.End()

    # Update plot data
    self.centroid.pop(0)
    self.euler.pop(0)
    if self.body_id > 0 and self.body_id < app.model.nbody:
      self.centroid.append(app.data.xpos[self.body_id].copy())
      # Convert quaternion to Euler angles via rotation matrix.
      quat = app.data.xquat[self.body_id]
      mat = np.zeros(9)
      mujoco.mju_quat2Mat(mat, quat)
      # mat is row-major 3x3: R[i,j] = mat[3*i + j].
      roll = math.atan2(mat[7], mat[8])
      pitch = math.atan2(-mat[6], math.sqrt(mat[7] ** 2 + mat[8] ** 2))
      yaw = math.atan2(mat[3], mat[0])
      self.euler.append(np.degrees(np.array([roll, pitch, yaw])))
    else:
      self.centroid.append(np.zeros(3))
      self.euler.append(np.zeros(3))


def main(argv: list[str]) -> None:
  if len(argv) < 2:
    print('Usage: implot <model_path.xml>')
    sys.exit(1)

  data = parser.parse(argv[1])
  if data is None:
    print(f'Error loading model from {argv[1]!r}')
    sys.exit(1)
  model = data.model

  title = os.path.basename(sys.argv[0])
  plot_ui = PlottingUi()

  config = vp.ViewerConfig(
      title=title,
      width=_WIDTH.value,
      height=_HEIGHT.value,
      gfx=_GFX.value or '',
      viewer_mode=_VIEWER.value,
  )

  with launch_passive.launch_passive(config, viewer_gui_hook=plot_ui) as handle:
    handle.send_to_viewer(msg.ModelEvent(model=model))

    step_control = sim.StepControl()
    while handle.is_running():
      step_control.advance(model, data)
      model, data, step_control = handle.sync(model, data, step_control)


if __name__ == '__main__':
  _app.run(main)
