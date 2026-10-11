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
"""Parameter distribution report section."""

import math
from typing import Any

from mujoco.sysid._src import parameter
from mujoco.sysid.report.sections.base import ReportSection
from mujoco.sysid.report.utils import plotly_script_tag
import numpy as np
from plotly import subplots as plt_subplots
import plotly.graph_objects as go


class ParameterDistribution(ReportSection):
  """Displays the identified parameters relative to their bounds and nominal values.

  Visualization: One horizontal track per parameter with markers for Nominal,
  Identified, and Bounds.
  """

  def __init__(
      self,
      title: str,
      opt_params: parameter.ParameterDict,
      initial_params: parameter.ParameterDict,
      confidence_intervals: np.ndarray | None = None,
      height_per_param: int = 60,
      anchor: str = "",
      collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self._anchor = anchor

    self._opt_params = opt_params
    self._initial_params = initial_params
    self._nominal_params = initial_params.copy()
    self._nominal_params.reset()

    self._x_nominal = self._nominal_params.as_vector()
    self._x_hat = self._opt_params.as_vector()
    self._x_initial = self._initial_params.as_vector()
    self._confidence_intervals = confidence_intervals
    self._height_per_param = height_per_param

  @property
  def template_filename(self) -> str:
    return "plot_generic.html"

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  def header_includes(self) -> set[str]:
    return {plotly_script_tag()}

  def get_context(self) -> dict[str, Any]:
    fig = self._build_figure()

    config = {
        "displayModeBar": True,
        "displaylogo": False,
        "responsive": True,
        "toImageButtonOptions": {
            "format": "svg",
            "filename": f"{self._title}_distribution",
            "height": max(400, len(self._x_hat) * 100),
            "width": 1200,
            "scale": 1,
        },
    }

    return {
        "title": self._title,
        "plot_div": fig.to_html(
            full_html=False, include_plotlyjs=False, config=config
        ),
        "caption": (
            "<b>Visualization Guide:</b><br><b>Error Bars ( — )</b>: 95%"
            " Confidence Interval. Smaller is better.<br><span"
            " style='color:green'>◆ High Confidence</span>: Interval is &lt;"
            " 0.5% of the parameter range (error bar may be"
            " invisible).<br><span style='color:blue'>◆ Identified</span>:"
            " Standard confidence interval.<br><span style='color:red'>◆"
            " Unconstrained</span>: Interval is infinite or larger than range"
            " (Red error bar).<br><span style='color:red'>x Nominal</span>,"
            " <span style='color:blue'>- - Bounds</span>: Reference"
            " values.<br>* parameter is frozen."
        ),
    }

  def _build_figure(self) -> go.Figure:
    plot_items = []
    non_frozen_idx = 0

    for param_name, param in self._opt_params.parameters.items():
      # Get bounds for this specific parameter
      p_min, p_max = param.get_bounds()

      if param.size == 1:
        flat_value = (
            param.value if np.isscalar(param.value) else param.value.item()
        )
        lb = p_min.item()
        ub = p_max.item()

        if param.frozen:
          plot_items.append({
              "name": param_name,
              "value": flat_value,
              "is_frozen": True,
              "lb": lb,
              "ub": ub,
              "nominal": flat_value if self._x_nominal is not None else None,
          })
        else:
          conf = (
              self._confidence_intervals[non_frozen_idx]
              if self._confidence_intervals is not None
              else None
          )
          plot_items.append({
              "name": param_name,
              "value": self._x_hat[non_frozen_idx],
              "is_frozen": False,
              "lb": lb,
              "ub": ub,
              "nominal": (
                  self._x_nominal[non_frozen_idx]
                  if self._x_nominal is not None
                  else None
              ),
              "conf": conf,
          })
          non_frozen_idx += 1
      else:
        for i in range(param.size):
          if param.shape == (param.size,):
            element_name = f"{param_name}[{i}]"
            val_frozen = param.value[i]
            lb = p_min[i]
            ub = p_max[i]
          else:
            multi_idx = np.unravel_index(i, param.shape)
            idx_str = ",".join(str(x) for x in multi_idx)
            element_name = f"{param_name}[{idx_str}]"
            val_frozen = param.value[multi_idx]
            lb = p_min[i]
            ub = p_max[i]

          if param.frozen:
            plot_items.append({
                "name": element_name,
                "value": val_frozen,
                "is_frozen": True,
                "lb": lb,
                "ub": ub,
                "nominal": val_frozen if self._x_nominal is not None else None,
            })
          else:
            conf = (
                self._confidence_intervals[non_frozen_idx]
                if self._confidence_intervals is not None
                else None
            )
            plot_items.append({
                "name": element_name,
                "value": self._x_hat[non_frozen_idx],
                "is_frozen": False,
                "lb": lb,
                "ub": ub,
                "nominal": (
                    self._x_nominal[non_frozen_idx]
                    if self._x_nominal is not None
                    else None
                ),
                "conf": conf,
            })
            non_frozen_idx += 1

    n_params = len(plot_items)

    n_cols = 8 if n_params > 1 else 1
    n_rows = math.ceil(n_params / n_cols)

    # Prepare subplot titles with conditional formatting
    subplot_titles = []
    for p in plot_items:
      if p["is_frozen"]:
        subplot_titles.append(f"<span style='color: grey;'>{p['name']}*</span>")
      else:
        subplot_titles.append(p["name"])

    # Create subplots
    fig = plt_subplots.make_subplots(
        rows=n_rows,
        cols=n_cols,
        subplot_titles=subplot_titles,
        vertical_spacing=0.05,
        horizontal_spacing=0.02,
    )

    # Add dummy trace for Bounds legend
    fig.add_trace(
        go.Scatter(
            x=[None],
            y=[None],
            mode="lines",
            line=dict(color="blue", width=3, dash="dash"),
            name="Bounds",
        )
    )

    shown_legends = set()

    for i, item in enumerate(plot_items):
      row = (i // n_cols) + 1
      col = (i % n_cols) + 1

      val = item["value"]
      lb, ub = item["lb"], item["ub"]

      # 1. Bounds lines (For items with valid bounds)
      if lb is not None and ub is not None:
        # Lower Bound (Horizontal line at y=lb)
        fig.add_trace(
            go.Scatter(
                x=[-1, 1],
                y=[lb, lb],
                mode="lines",
                line=dict(color="blue", width=3, dash="dash"),
                showlegend=False,
                hoverinfo="skip",
            ),
            row=row,
            col=col,
        )
        # Upper Bound (Horizontal line at y=ub)
        fig.add_trace(
            go.Scatter(
                x=[-1, 1],
                y=[ub, ub],
                mode="lines",
                line=dict(color="blue", width=3, dash="dash"),
                showlegend=False,
                hoverinfo="skip",
            ),
            row=row,
            col=col,
        )

        # Calculate margin based on bounds
        dist = ub - lb
        if dist > 0:
          margin = dist * 0.2
          y_range_min = lb
          y_range_max = ub
        else:
          margin = abs(val) * 0.2 if val != 0 else 1.0
          y_range_min = val
          y_range_max = val
      else:
        # Fallback
        margin = abs(val) * 0.2 if val != 0 else 1.0
        y_range_min = val
        y_range_max = val

      # 2. Add Traces (Frozen vs Optimized)
      if item["is_frozen"]:
        # Frozen Parameter
        fig.add_trace(
            go.Scatter(
                x=[0],
                y=[val],
                mode="markers",
                marker=dict(
                    symbol="circle", size=10, color="gray", opacity=0.7
                ),
                name="Frozen",
                showlegend=False,
                hoverinfo="skip",
            ),
            row=row,
            col=col,
        )
      else:
        # Optimized Parameter

        # Determine Confidence Status
        trace_name = "Identified"
        marker_color = "blue"
        legend_group = "identified"

        error_y = None

        if item.get("conf") is not None:
          interval = item["conf"]
          min_bound = item["lb"]
          max_bound = item["ub"]

          error_val = interval

          rng = 0
          if min_bound is not None and max_bound is not None:
            rng = max_bound - min_bound

          if not np.isfinite(interval) or (
              rng > 0 and 2.0 * interval > 1.0 * rng
          ):
            # Unconstrained
            trace_name = "Unconstrained"
            marker_color = "red"
            legend_group = "unconstrained"

            if rng > 0:
              error_val = rng
            else:
              error_val = interval  # or large value call fallback

          elif rng > 0 and interval <= 0.005 * rng:
            trace_name = "High Confidence"
            marker_color = "green"
            legend_group = "high_conf"

          error_bar_color = marker_color
          error_y = dict(
              type="data",
              array=[error_val],
              visible=True,
              thickness=1.5,
              width=3,
              color=error_bar_color,
          )

        show_leg = trace_name not in shown_legends
        if show_leg:
          shown_legends.add(trace_name)

        fig.add_trace(
            go.Scatter(
                x=[0],
                y=[val],
                mode="markers",
                marker=dict(symbol="diamond", size=12, color=marker_color),
                name=trace_name,
                showlegend=show_leg,
                legendgroup=legend_group,
                hovertemplate=(
                    f"{trace_name}: %{{y}} ±"
                    f" {item.get('conf', 0):.4g}<extra></extra>"
                ),
                error_y=error_y,
            ),
            row=row,
            col=col,
        )

        # Nominal Value (if exists)
        if item["nominal"] is not None:
          show_leg_nom = "Nominal" not in shown_legends
          if show_leg_nom:
            shown_legends.add("Nominal")

          fig.add_trace(
              go.Scatter(
                  x=[0],
                  y=[item["nominal"]],
                  mode="markers",
                  marker=dict(symbol="x", size=10, color="red"),
                  name="Nominal",
                  showlegend=show_leg_nom,
                  legendgroup="nominal",
                  hovertemplate="Nominal: %{y}<extra></extra>",
              ),
              row=row,
              col=col,
          )

      fig.update_yaxes(
          range=[y_range_min - margin, y_range_max + margin],
          showgrid=True,
          zeroline=False,
          row=row,
          col=col,
      )
      fig.update_xaxes(
          range=[-1, 1],
          showgrid=False,
          zeroline=False,
          showticklabels=False,
          row=row,
          col=col,
      )

    # Global Layout
    total_height = max(
        400, n_rows * 180
    )  # Increased height per row for better spacing
    fig.update_layout(
        height=total_height,
        template="plotly_white",
        margin=dict(l=60, r=60, t=150, b=60),  # Increased side margins
        autosize=True,
        legend=dict(
            orientation="h", yanchor="bottom", y=1.02, xanchor="center", x=0.5
        ),
        hovermode="closest",
    )

    return fig
