# Copyright 2026 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Video report section for system identification results."""

from collections.abc import Callable
import os
import pathlib
from typing import Any

import mujoco
import mujoco.rollout
from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter
from mujoco.sysid._src.plotting import render_rollout
from mujoco.sysid._src.trajectory import SystemTrajectory
from mujoco.sysid.report.sections.base import ReportSection


def spec_apply(spec, attrs, values):
  def apply_to_geoms_recursive(body):
    for g in body.geoms:
      for attr, value in zip(attrs, values, strict=True):
        setattr(g, attr, value)
    for child_body in body.bodies:
      apply_to_geoms_recursive(child_body)

  for top_body in spec.worldbody.bodies:
    apply_to_geoms_recursive(top_body)


def generate_video_from_trajectories(
    initial_params: parameter.ParameterDict,
    opt_params: parameter.ParameterDict,
    _build_model: Callable[
        [parameter.ParameterDict, mujoco.MjSpec], mujoco.MjModel
    ],
    trajectories: list[SystemTrajectory],
    model_spec: mujoco.MjSpec,
    output_filepath: os.PathLike[str],
    render_initial: bool = True,
    render_nominal: bool = True,
    render_opt: bool = True,
    height: int = 480,
    width: int = 640,
    fovy: float = 60,
    camera: str | int = -1,
    fps: int = 60,
) -> pathlib.Path:
  """Render trajectories and concatenate into a single video.

  Each trajectory is rendered with initial/nominal/optimized parameters
  overlaid, then all frames are concatenated.

  Args:
    initial_params: Initial parameter values.
    opt_params: Optimized parameter values.
    _build_model: Callable to build a model from parameters and spec.
    trajectories: List of trajectories to render.
    model_spec: MuJoCo model specification.
    output_filepath: Path to save the output video.
    render_initial: Whether to render with initial parameters.
    render_nominal: Whether to render with nominal parameters.
    render_opt: Whether to render with optimized parameters.
    height: Frame height in pixels.
    width: Frame width in pixels.
    fovy: Vertical field of view in degrees.
    camera: Camera index or name.
    fps: Frames per second.

  Returns:
    Path to the saved video file.
  """
  import imageio

  all_frames = []
  for traj in trajectories:
    # Build models for this trajectory
    models = []
    datas = []

    nominal_params = initial_params.copy()
    nominal_params.reset()

    # initial
    if render_initial:
      initial_spec = model_spec.copy()
      initial_spec = model_modifier.apply_param_modifiers_spec(
          initial_params, initial_spec
      )
      spec_apply(initial_spec, ["rgba"], [[1, 0, 0, 0.5]])
      initial_model = initial_spec.compile()
      initial_data = mujoco.MjData(initial_model)
      models.append(initial_model)
      datas.append(initial_data)

    # nominal
    if render_nominal:
      nominal_spec = model_spec.copy()
      nominal_spec = model_modifier.apply_param_modifiers_spec(
          nominal_params, nominal_spec
      )
      spec_apply(nominal_spec, ["rgba"], [[0, 1, 0, 0.4]])
      nominal_model = nominal_spec.compile()
      nominal_data = mujoco.MjData(nominal_model)
      models.append(nominal_model)
      datas.append(nominal_data)

    # pred
    if render_opt:
      pred_spec = model_spec.copy()
      pred_spec = model_modifier.apply_param_modifiers_spec(
          opt_params, pred_spec
      )
      spec_apply(pred_spec, ["rgba"], [[0, 0, 1, 1.0]])
      pred_model = pred_spec.compile()
      pred_data = mujoco.MjData(pred_model)
      models.append(pred_model)
      datas.append(pred_data)

    control_ts = traj.control.resample(target_dt=models[0].opt.timestep)
    state, _ = mujoco.rollout.rollout(
        models, datas, traj.initial_state, control_ts.data
    )
    models[0].vis.global_.fovy = fovy
    models[0].vis.global_.offwidth = width
    models[0].vis.global_.offheight = height
    frames = render_rollout(
        models,
        datas[0],
        state,
        framerate=fps,
        height=height,
        width=width,
        camera=camera,
    )
    all_frames.extend(list(frames))

  output_filepath_str = str(output_filepath)
  writer = imageio.get_writer(output_filepath_str, fps=fps, quality=8)
  for frame in all_frames:
    writer.append_data(frame)
  writer.close()

  return pathlib.Path(output_filepath_str)


class VideoPlayer(ReportSection):
  """A report section to embed and display a video file."""

  def __init__(
      self,
      title: str,
      video_filepath: pathlib.Path,
      anchor: str = "",
      width: int | str = 800,
      height: int | None = 450,
      autoplay: bool = False,
      controls: bool = True,
      muted: bool = False,
      loop: bool = True,
      caption: str = "<b>Legend:</b> <span class='color-initial'>Initial</span>, <span class='color-nominal'>Nominal</span>, <span class='color-optimized'>Optimized</span>",
      collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self._anchor = anchor
    self._video_filepath = video_filepath
    self._width = width
    self._height = height
    self._autoplay = autoplay
    self._controls = controls
    self._muted = muted
    self._loop = loop
    self._caption = caption

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  @property
  def template_filename(self) -> str:
    """Tells the builder to look for 'video.html'."""
    return "video.html"

  def header_includes(self) -> set[str]:
    return set()

  def get_context(self) -> dict[str, Any]:
    """Returns the data needed to render the video player in the template."""
    return {
        "title": self._title,
        "video_filepath": self._video_filepath.name,
        "width": self._width,
        "height": self._height,
        "autoplay": "autoplay" if self._autoplay else "",
        "controls": "controls" if self._controls else "",
        "muted": "muted" if self._muted else "",
        "loop": "loop" if self._loop else "",
        "caption": self._caption,
    }
