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
"""Optimization trace report section."""

from collections.abc import Sequence
import math
from typing import Any

from mujoco.sysid.report.sections.base import ReportSection
from mujoco.sysid.report.utils import plotly_script_tag
import numpy as np
from numpy import typing as npt
from plotly import subplots as plt_subplots
import plotly.graph_objects as go


class OptimizationTrace(ReportSection):
  """Displays plots summarizing the optimization process."""

  def __init__(
      self,
      title: str,
      objective: Sequence[float],
      candidate: Sequence[npt.ArrayLike],
      bounds: (
          tuple[Sequence[float] | np.ndarray, Sequence[float] | np.ndarray]
          | None
      ) = None,
      param_names: Sequence[str] | None = None,
      log_diff: bool = True,
      bound_eps: float = 1e-3,
      dims_per_page: int = 6,  # For candidate plot paging
      anchor: str = "",
      collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self._objective = objective
    self._candidate = candidate  # List of vectors
    self._bounds = bounds
    self._param_names = param_names
    self._log_diff = log_diff
    self._bound_eps = bound_eps
    self._dims_per_page = dims_per_page
    self._anchor = anchor

  @property
  def template_filename(self) -> str:
    return "optimization_trace.html"

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  def header_sections(self) -> set[str]:
    return {plotly_script_tag()}

  def get_context(self) -> dict[str, Any]:
    objective_fig = self._get_objective_figure()
    candidate_figs = self._get_candidate_figures()
    candidate_heatmap_fig = self._get_candidate_heatmap_figure()

    config = {
        "displayModeBar": True,
        "displaylogo": False,
        "toImageButtonOptions": {
            "format": "svg",
            "filename": f"{self._title}_optimization",
            "height": 800,
            "width": 1200,
            "scale": 1,
        },
        "responsive": True,
    }

    return {
        "title": self._title,
        "objective_plot_html": (
            objective_fig.to_html(
                full_html=False, include_plotlyjs=False, config=config
            )
            if objective_fig
            else None
        ),
        "candidate_plots_html": (
            [
                fig.to_html(
                    full_html=False, include_plotlyjs=False, config=config
                )
                for fig in candidate_figs
            ]
            if candidate_figs
            else None
        ),
        "candidate_heatmap_html": (
            candidate_heatmap_fig.to_html(
                full_html=False, include_plotlyjs=False, config=config
            )
            if candidate_heatmap_fig
            else None
        ),
    }

  def _get_objective_figure(self) -> go.Figure | None:
    """Generates the objective function plot."""
    if not self._objective:
      return None

    fig = go.Figure()
    fig.add_trace(
        go.Scatter(
            y=self._objective,
            mode="lines+markers",
            marker=dict(size=4),
            line=dict(width=2),
            name="Objective",
        )
    )

    final_value = self._objective[-1]
    if abs(final_value) < 1e-3 or abs(final_value) > 1e3:
      final_str = f"{final_value:.2e}"
    else:
      final_str = f"{final_value:.4f}"

    fig.update_layout(
        title=f"Objective Over Time (Final: {final_str})",
        xaxis_title="Iteration",
        yaxis_title="Objective",
        hovermode="x unified",
        height=400,
        autosize=True,
        margin=dict(l=50, r=50, t=80, b=50),
        template="plotly_white",
    )
    fig.update_xaxes(
        showgrid=True, gridwidth=1, gridcolor="rgba(211, 211, 211, 0.7)"
    )
    fig.update_yaxes(
        showgrid=True, gridwidth=1, gridcolor="rgba(211, 211, 211, 0.7)"
    )
    return fig

  def _get_candidate_figures(self) -> list[go.Figure]:
    """Generates the parameter candidate plots (paged)."""
    if not self._candidate:
      return []

    values = np.array(self._candidate).T  # shape: (n_dim, n_iter)
    n_dim, n_iter = values.shape
    if n_iter <= 1:
      return []
    diffs = np.diff(values, axis=1)  # shape: (n_dim, n_iter-1)

    if self._bounds is not None:
      mins = np.array(self._bounds[0])
      maxs = np.array(self._bounds[1])
      if not (mins.shape == (n_dim,) and maxs.shape == (n_dim,)):
        raise ValueError("Bounds dimensions do not match parameter dimensions.")
    else:
      mins = np.full(n_dim, -np.inf)
      maxs = np.full(n_dim, np.inf)

    param_names = (
        self._param_names
        if self._param_names is not None
        else [f"Dim {i}" for i in range(n_dim)]
    )
    if len(param_names) != n_dim:
      raise ValueError(
          "Number of parameter names does not match parameter dimensions."
      )

    n_pages = math.ceil(n_dim / self._dims_per_page)
    figures = []
    iterations = np.arange(n_iter)
    iterations_diff = np.arange(1, n_iter)

    for page in range(n_pages):
      start_dim = page * self._dims_per_page
      end_dim = min((page + 1) * self._dims_per_page, n_dim)
      dims_in_page = end_dim - start_dim
      page_param_names = param_names[start_dim:end_dim]

      fig = plt_subplots.make_subplots(
          rows=dims_in_page,
          cols=2,
          shared_xaxes=True,
          subplot_titles=[
              title
              for name in page_param_names
              for title in (f"{name} Value", f"{name} Δ")
          ],
          vertical_spacing=max(0.02, 0.1 / dims_in_page),
      )

      for i, dim in enumerate(range(start_dim, end_dim)):
        row_idx = i + 1
        vals = values[dim, :]
        diff_vals = diffs[dim, :]
        lower, upper = mins[dim], maxs[dim]

        # --- Value Plot (Col 1) ---
        # Check for near-bound points
        near_lower = np.abs(vals - lower) < self._bound_eps
        near_upper = np.abs(vals - upper) < self._bound_eps
        near_bound = near_lower | near_upper

        # Plot segments with different colors if near bounds
        for t in range(1, n_iter):
          is_near_prev = near_bound[t - 1]
          is_near_curr = near_bound[t]
          color = (
              "#d62728" if (is_near_prev or is_near_curr) else "#1f77b4"
          )  # Red if current or prev near bound
          fig.add_trace(
              go.Scatter(
                  x=iterations[t - 1 : t + 1],
                  y=vals[t - 1 : t + 1],
                  mode="lines",
                  line=dict(color=color, width=2),
                  showlegend=False,
              ),
              row=row_idx,
              col=1,
          )

        fig.add_trace(
            go.Scatter(
                x=iterations,
                y=vals,
                mode="markers",
                marker=dict(
                    size=5,
                    color=["#d62728" if nb else "#1f77b4" for nb in near_bound],
                    symbol=[
                        "triangle-down"
                        if nl
                        else "triangle-up"
                        if nu
                        else "circle"
                        for nl, nu in zip(near_lower, near_upper, strict=True)
                    ],  # Triangles for bounds.
                ),
                name=f"{param_names[dim]}",
                showlegend=False,
                hoverinfo="x+y+name",
            ),
            row=row_idx,
            col=1,
        )

        # Add bound lines if finite.
        if np.isfinite(lower):
          fig.add_hline(
              y=lower,
              line_dash="dash",
              line_color="gray",
              row=row_idx,  # pyright: ignore[reportArgumentType]
              col=1,  # pyright: ignore[reportArgumentType]
              opacity=0.5,
          )
        if np.isfinite(upper):
          fig.add_hline(
              y=upper,
              line_dash="dash",
              line_color="gray",
              row=row_idx,  # pyright: ignore[reportArgumentType]
              col=1,  # pyright: ignore[reportArgumentType]
              opacity=0.5,
          )

        # Add final value annotation.
        final_val = vals[-1]
        final_str = (
            f"{final_val:.2e}"
            if abs(final_val) < 1e-3 or abs(final_val) > 1e3
            else f"{final_val:.4f}"
        )
        fig.add_annotation(
            x=iterations[-1],
            y=final_val,
            text=final_str,
            showarrow=True,
            arrowhead=1,
            ax=20,
            ay=-30,
            row=row_idx,
            col=1,
            font=dict(color="blue", size=9),
        )

        # --- Diff Plot (Col 2) ---
        if self._log_diff:
          eps = 1e-12
          plot_diff_vals = np.log10(np.abs(diff_vals) + eps)
          yaxis_title = "log |Δ|"
        else:
          plot_diff_vals = diff_vals
          yaxis_title = "Δ"

        fig.add_trace(
            go.Scatter(
                x=iterations_diff,
                y=plot_diff_vals,
                mode="lines+markers",
                marker=dict(symbol="x", size=5, color="orange"),
                line=dict(width=1.5, color="orange"),
                name=f"Δ {param_names[dim]}",
                showlegend=False,
                hoverinfo="x+y+name",
            ),
            row=row_idx,
            col=2,
        )
        fig.update_yaxes(
            title_text=yaxis_title, row=row_idx, col=2, title_font_size=10
        )

      # --- Layout Updates for the Page Figure ---
      fig.update_layout(
          title=(
              f"Candidate Values and Changes (Page {page + 1}/{n_pages}, Dims"
              f" {start_dim}-{end_dim - 1})"
          ),
          height=max(400, 200 * dims_in_page),  # Adjust height based on dims
          autosize=True,
          margin=dict(l=60, r=30, t=100, b=50),
          hovermode="x unified",
          template="plotly_white",
      )
      fig.update_xaxes(
          showgrid=True,
          gridwidth=1,
          gridcolor="rgba(211, 211, 211, 0.7)",
          zeroline=False,
      )
      fig.update_yaxes(
          showgrid=True,
          gridwidth=1,
          gridcolor="rgba(211, 211, 211, 0.7)",
          zeroline=False,
      )

      # Add common x-axis label to the bottom row.
      fig.update_xaxes(title_text="Iteration", row=dims_in_page, col=1)
      fig.update_xaxes(title_text="Iteration", row=dims_in_page, col=2)

      for annotation in fig.layout.annotations:
        annotation.font.size = 10

      figures.append(fig)

    return figures

  def _get_candidate_heatmap_figure(self) -> go.Figure | None:
    """Generates the parameter candidate heatmap."""
    if not self._candidate:
      return None

    data = np.array(self._candidate).T  # shape: (n_dim, n_iter)
    n_dim, n_iter = data.shape

    param_names = (
        self._param_names
        if self._param_names is not None
        else [f"Dim {i}" for i in range(n_dim)]
    )
    if len(param_names) != n_dim:
      raise ValueError(
          "Number of parameter names does not match parameter dimensions."
      )

    # Normalize data for heatmap colors if bounds are provided.
    heatmap_data = data.copy()
    normalize = self._bounds is not None
    if normalize and self._bounds is not None:
      min_bounds, max_bounds = self._bounds
      if not (len(min_bounds) == len(max_bounds) == n_dim):
        raise ValueError("Bounds dimensions do not match parameter dimensions.")
      for i in range(n_dim):
        min_val, max_val = min_bounds[i], max_bounds[i]
        denom = max_val - min_val if max_val > min_val else 1.0
        clipped_vals = np.clip(data[i], min_val, max_val)
        heatmap_data[i] = (
            (clipped_vals - min_val) / denom if denom != 0 else 0.5
        )  # Center if range is zero
    else:
      row_mins = np.min(data, axis=1, keepdims=True)
      row_maxs = np.max(data, axis=1, keepdims=True)
      row_ranges = row_maxs - row_mins
      row_ranges[row_ranges == 0] = 1.0  # Avoid division by zero
      heatmap_data = (data - row_mins) / row_ranges

    fig = go.Figure(
        data=go.Heatmap(
            z=heatmap_data[::-1],
            x=np.arange(n_iter),
            y=list(reversed(param_names)),
            colorscale="RdBu",
            colorbar=dict(
                title="Normalized Value"
                if normalize
                else "Row-Normalized Value"
            ),
            hovertemplate=(
                "Iter: %{x}<br>Param: %{y}<br>Value:"
                " %{customdata:.4f}<extra></extra>"
            ),
            customdata=data[::-1].tolist(),
        )
    )

    # Add markers for bound hits if bounds exist.
    bound_markers_x = []
    bound_markers_y = []
    if self._bounds is not None:
      min_bounds, max_bounds = self._bounds
      for dim in range(n_dim):
        min_val, max_val = min_bounds[dim], max_bounds[dim]
        for iter_idx, val in enumerate(data[dim]):
          if (
              abs(val - min_val) < self._bound_eps
              or abs(val - max_val) < self._bound_eps
          ):
            bound_markers_x.append(iter_idx)
            bound_markers_y.append(param_names[dim])

      if bound_markers_x:
        fig.add_trace(
            go.Scatter(
                x=bound_markers_x,
                y=bound_markers_y,
                mode="markers",
                marker=dict(color="black", size=6, symbol="x"),
                name="At Bound",
                showlegend=False,
                hoverinfo="skip",
            )
        )

    fig.update_layout(
        title="Candidate Parameter Heatmap",
        xaxis_title="Iteration",
        yaxis_title="Parameter",
        height=max(400, 30 * n_dim),
        autosize=True,
        margin=dict(l=150, r=50, t=80, b=50),
        yaxis=dict(
            tickmode="array", tickvals=param_names, ticktext=param_names
        ),
        template="plotly_white",
    )

    return fig
