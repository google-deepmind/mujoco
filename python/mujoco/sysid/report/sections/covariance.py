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

"""Covariance and correlation matrix report sections."""

from collections.abc import Callable
from typing import Any

import matplotlib
from matplotlib import cm
from matplotlib import colors as mpl_colors
from mujoco.sysid._src import parameter
from mujoco.sysid.report.sections.base import ReportSection
from mujoco.sysid.report.utils import get_text_color
import numpy as np
from numpy import typing as npt


def _compute_correlation(cov: npt.ArrayLike) -> np.ndarray:
  """Calculates the correlation matrix from a covariance matrix.

  Formula: A[i,j] = cov[i,j] / sqrt(cov[i,i] * cov[j,j])

  Args:
    cov: The covariance matrix.

  Returns:
    The correlation matrix.
  """
  cov = np.array(cov)
  sqrt_diag_cov = np.sqrt(np.diag(cov))

  # denom[i, j] = sqrt_diag_cov[i] * sqrt_diag_cov[j]
  denom = np.outer(sqrt_diag_cov, sqrt_diag_cov)

  # Safe division (handles division by zero by returning 0)
  return np.divide(
      cov, denom, out=np.zeros_like(cov, dtype=float), where=denom != 0
  )


class Covariance(ReportSection):
  """Displays values of the covariance and correlation matrices.

  Assumed to be symmetric.
  """

  def __init__(
      self,
      title: str,
      covariance: npt.ArrayLike,
      parameter_dict: parameter.ParameterDict,
      anchor: str = "",
      collapsible: bool = True,
  ):
    super().__init__(collapsible=collapsible)
    self._title = title
    self._anchor = anchor
    self._covariance = np.array(covariance)
    self._parameter_dict = parameter_dict

  @property
  def template_filename(self) -> str:
    return "covariance.html"

  @property
  def title(self):
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  def get_context(self) -> dict[str, Any]:
    dim_names = []
    for param_name, param in self._parameter_dict.parameters.items():
      if param.frozen:
        continue
      if param.size == 1:
        dim_names.append(param_name)
      else:
        for i in range(param.size):
          dim_names.append(f"{param_name}[{i}]")

    context = {
        "title": self._title,
        "covariance_data": self._covariance_table_data(),
        "correlation_data": self._correlation_table_data(),
        "dim_names": dim_names,
    }
    if self._covariance.size == 0:
      context["message"] = (
          "Covariance matrix is empty. This usually means there are no"
          " parameters to optimize or all parameters are frozen."
      )
    return context

  def header_includes(self) -> set[str]:
    return {""}

  def _create_table_data(
      self,
      matrix: np.ndarray,
      norm: mpl_colors.Normalize,
      cmap: mpl_colors.Colormap,
      # Function to get the value used for coloring based on (row, col, value)
      get_color_input_value: Callable[[int, int, float], float],
  ) -> list[list[dict[str, Any]]]:
    """Helper method to generate formatted table data with colors."""
    scalar_map = cm.ScalarMappable(norm=norm, cmap=cmap)
    table_data = []
    for i in range(matrix.shape[0]):
      row_data = []
      for j in range(matrix.shape[1]):
        value = matrix[i, j]
        color_value = get_color_input_value(i, j, value)

        # Get RGBA color, convert to HEX.
        rgba_color = scalar_map.to_rgba(color_value)  # pyright: ignore[reportArgumentType]  # type: ignore[arg-type]
        hex_color = mpl_colors.rgb2hex(rgba_color)  # pyright: ignore[reportArgumentType]  # type: ignore[arg-type]
        # Make sure the text is visible over the cell color.
        text_color = get_text_color(hex_color)
        row_data.append({
            "value": value,
            "bgcolor": hex_color,
            "textcolor": text_color,
        })
      table_data.append(row_data)
    return table_data

  def _covariance_table_data(self):
    # Use the positive side of bwr that goes from white to red
    cmap = matplotlib.colormaps["bwr"]

    if self._covariance.size == 0:
      max_abs_val = 1.0  # Default value to avoid errors
    else:
      max_abs_val = np.max(np.abs(self._covariance))

    if max_abs_val == 0:
      max_abs_val = 1

    norm = mpl_colors.Normalize(vmin=-max_abs_val, vmax=max_abs_val)

    # Color based on the actual value
    def get_color_input(_i, _j, val):
      return np.abs(val)

    return self._create_table_data(
        self._covariance, norm, cmap, get_color_input
    )

  def _correlation_table_data(self):
    correlation = _compute_correlation(self._covariance)
    # Use the positive side of bwr that goes from white to red
    cmap = matplotlib.colormaps["bwr"]
    norm = mpl_colors.Normalize(vmin=-1, vmax=1)

    # Color based on the abs value, but use 0 for the diagonal (white)
    def get_color_input(i, j, val):
      if i == j:
        return 0.0
      return np.abs(val)

    return self._create_table_data(correlation, norm, cmap, get_color_input)
