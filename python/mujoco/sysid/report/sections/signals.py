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
"""Signal comparison report section."""

import math
from typing import Any

import mujoco
from mujoco.sysid._src import timeseries
from mujoco.sysid.report.sections.base import ReportSection
from mujoco.sysid.report.utils import plotly_script_tag
import numpy as np
from plotly import colors as plt_colors
from plotly import subplots as plt_subplots
import plotly.graph_objects as go


class SignalReport(ReportSection):
  """A report section comparing predicted vs measured observation data."""

  def __init__(
      self,
      title: str,
      model: mujoco.MjModel,
      size_factor: float = 1.0,
      title_prefix: str = "",
      max_datapoints: int | None = 300,
      resample_to_frequency: float | None = None,
      anchor: str = "",
      ts_dict=None,
      collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self._anchor = anchor
    self._model = model
    self._ts_dict = ts_dict if ts_dict is not None else {}
    self._size_factor = size_factor
    self._title_prefix = title_prefix
    self._max_datapoints = max_datapoints
    self._resample_to_frequency = resample_to_frequency
    self._figure: go.Figure | None = None

    if resample_to_frequency is not None and resample_to_frequency <= 0:
      raise ValueError(
          f"Invalid resample_to_frequency: {resample_to_frequency}"
      )

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  @property
  def template_filename(self) -> str:
    """Tells the builder to look for 'sensors.html'."""
    return "signals.html"

  def header_includes(self) -> set[str]:
    """Ensures the Plotly Javascript library is loaded in <head>."""
    return {plotly_script_tag()}

  def get_context(self) -> dict[str, Any]:
    if self._figure is None:
      self._figure = self._build_figure()

    config = {
        "displayModeBar": True,
        "displaylogo": False,
        "toImageButtonOptions": {
            "format": "svg",
            "filename": f"{self._title}_signals",
            "height": 800,
            "width": 1200,
            "scale": 1,
        },
        "responsive": True,
    }
    context = {"title": self._title}

    if self._figure:
      plot_html = self._figure.to_html(
          full_html=False, include_plotlyjs=False, config=config
      )
      context["plot_div"] = plot_html

    return context

  def _build_figure(self) -> go.Figure | None:
    """Internal helper to construct the Plotly figure."""
    mapping = None

    signal_dict = {}

    for ts_name in self._ts_dict:
      ts = self._ts_dict[ts_name]
      if ts.signal_mapping:
        mapping = ts.signal_mapping
      signal_dict[ts_name] = self._resample_if_needed(ts)

    if not mapping and self._ts_dict:
      # Fallback: create mapping from the first time series assuming identity
      first_ts_name = next(iter(self._ts_dict))
      first_ts = self._ts_dict[first_ts_name]
      n_dim = first_ts.data.shape[1]
      mapping = {
          f"{first_ts_name}_{i}": (first_ts_name, [i]) for i in range(n_dim)
      }

    if not mapping:
      return

    signal_names = []
    for name in mapping:
      signal_dim = len(mapping[name][1])
      if signal_dim > 1:
        for j in range(signal_dim):
          signal_names.append(f"{name}[{j}]")
      else:
        signal_names.append(name)

    n_plots = len(signal_names)
    n_cols = 3 if n_plots > 1 else 1
    n_rows = (n_plots + n_cols - 1) // n_cols
    n_rows += 1
    n_params = len(mapping)
    n_rows = math.ceil(n_params / n_cols)

    fig = plt_subplots.make_subplots(
        rows=n_rows,
        cols=n_cols,
        shared_xaxes=True,
        subplot_titles=signal_names,
        vertical_spacing=0.5 / n_rows if n_rows > 1 else 0.2,
    )

    colors = plt_colors.DEFAULT_PLOTLY_COLORS

    plot_idx = 0
    curr_row = 1
    curr_col = 1
    for key in mapping:
      indices = mapping[key][1]
      signal_dim = len(indices)

      for j in range(signal_dim):
        for color_id, ts_name in enumerate(signal_dict):
          predicted_signal = signal_dict[ts_name].data[:, indices]
          predicted_times = signal_dict[ts_name].times

          curr_row = (plot_idx // n_cols) + 1
          curr_col = (plot_idx % n_cols) + 1
          fig.add_trace(
              go.Scatter(
                  x=predicted_times,
                  y=predicted_signal[:, j],
                  mode="lines",
                  line=dict(width=2, color=colors[color_id]),
                  opacity=0.8,
                  name=ts_name,
                  legendgroup=ts_name,
                  showlegend=(plot_idx == 0),
              ),
              row=curr_row,
              col=curr_col,
          )
        fig.update_xaxes(title_text="Time (s)", row=curr_row, col=curr_col)
        plot_idx += 1

    fig.update_layout(
        title_text=f"{self._title_prefix} Signals",
        height=max(400, 220 * n_rows * self._size_factor),
        autosize=True,
        legend=dict(
            orientation="h", yanchor="bottom", y=1.15, xanchor="center", x=0.5
        ),
        margin=dict(l=60, r=60, t=150, b=60),
        template="plotly_white",
        hovermode="x unified",
    )
    fig.update_xaxes(
        showgrid=True,
        gridcolor="rgba(211, 211, 211, 0.7)",
        showspikes=True,
        spikemode="across",
        spikesnap="cursor",
        showline=True,
        linewidth=1,
        linecolor="black",
        matches="x",  # Critical for zooming all subplots together
    )
    fig.update_yaxes(
        showgrid=True,
        gridcolor="rgba(211, 211, 211, 0.7)",
        showline=True,
        linewidth=1,
        linecolor="black",
    )

    return fig

  def _resample_if_needed(
      self, data: timeseries.TimeSeries
  ) -> timeseries.TimeSeries:
    """Helper to downsample data for faster/lighter plotting."""
    if self._resample_to_frequency:
      new_times = np.arange(
          data.times[0], data.times[-1], 1.0 / self._resample_to_frequency
      )
      data = data.resample(new_times)
    if self._max_datapoints and len(data.times) > self._max_datapoints:
      new_times = np.linspace(
          data.times[0], data.times[-1], self._max_datapoints
      )
      data = data.resample(new_times)
    return data
