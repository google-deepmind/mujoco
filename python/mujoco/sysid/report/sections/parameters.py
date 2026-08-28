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

"""Parameter table report section."""

from typing import Any

from mujoco.sysid._src import parameter
from mujoco.sysid.report.sections.base import ReportSection
import numpy as np


class ParametersTable(ReportSection):
  """Displays a table of identified, initial, and nominal parameters."""

  def __init__(
      self,
      title: str,
      opt_params: parameter.ParameterDict,
      initial_params: parameter.ParameterDict,
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

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  @property
  def template_filename(self) -> str:
    """Tells the builder to look for 'parameters_table.html'."""
    return "parameters_table.html"

  def header_includes(self) -> set[str]:
    return {""}

  def get_context(self) -> dict[str, Any]:
    # Get the vector of non-frozen parameters
    non_frozen_vector = self._opt_params.as_vector()

    if len(self._x_hat) != non_frozen_vector.size:
      raise ValueError(
          "Parameter vector lengths don't match. Expected"
          f" {non_frozen_vector.size}, got {len(self._x_hat)}"
      )

    if self._x_nominal is not None:
      if len(self._x_nominal) != non_frozen_vector.size:
        raise ValueError(
            "Parameter vector lengths don't match. Expected"
            f" {non_frozen_vector.size}, got {len(self._x_nominal)}"
        )

    if self._x_initial is not None:
      if len(self._x_initial) != non_frozen_vector.size:
        raise ValueError(
            "Parameter vector lengths don't match. Expected"
            f" {non_frozen_vector.size}, got {len(self._x_initial)}"
        )

    # Compute error metrics if nominal exists
    if self._x_nominal is not None:
      if self._x_hat.size > 0:
        rel_errors = np.abs(self._x_hat - self._x_nominal) / (
            np.abs(self._x_nominal) + 1e-8
        )
        overall_rmse = np.sqrt(np.mean((self._x_hat - self._x_nominal) ** 2))
        abs_errors = np.abs(self._x_hat - self._x_nominal)
      else:
        rel_errors = np.array([])
        overall_rmse = 0.0
        abs_errors = np.array([])
    else:
      rel_errors = None
      overall_rmse = None
      abs_errors = None

    def create_table_row(param_name, val, lb, ub, idx=None, is_frozen=False):
      """Create a dict for a table row."""
      error_class = ""

      # Bounds check
      if not is_frozen:
        # Check if on boundary
        if (abs(val - lb) < 1e-8 + 1e-3 * abs(lb)) or (
            abs(val - ub) < 1e-8 + 1e-3 * abs(ub)
        ):
          error_class = "pt_on_boundary"

        # If not on boundary, check errors if nominal exists
        elif rel_errors is not None and idx is not None:
          rel_error = rel_errors[idx]
          if rel_error < 0.02:
            error_class = "pt_small_error"
          elif rel_error < 0.1:
            error_class = "pt_medium_error"
          else:
            error_class = "pt_large_error"
      else:
        error_class = "pt_frozen"

      row = {
          "name": param_name + ("*" if is_frozen else ""),
          "pred": val,
          "lower_bound": lb,
          "upper_bound": ub,
          "error_class": error_class,
          "is_frozen": is_frozen,
      }

      if self._x_initial is not None:
        if is_frozen:
          # Initial for frozen is assumed same as val
          row["initial"] = val
        elif idx is not None:
          row["initial"] = self._x_initial[idx]

      if self._x_nominal is not None:
        if is_frozen:
          # Nominal for frozen is assumed same as val
          row.update({
              "nominal": val,
              "abs_err": 0.0,
              "rel_err": 0.0,
          })
        elif (
            idx is not None
            and abs_errors is not None
            and rel_errors is not None
        ):
          row.update({
              "nominal": self._x_nominal[idx],
              "abs_err": abs_errors[idx],
              "rel_err": rel_errors[idx],
          })

      return row

    # Build table data.
    table_data = []
    non_frozen_idx = 0  # Index for non-frozen parameters in the arrays

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
          table_data.append(
              create_table_row(param_name, flat_value, lb, ub, is_frozen=True)
          )
        else:
          val = self._x_hat[non_frozen_idx]
          table_data.append(
              create_table_row(param_name, val, lb, ub, idx=non_frozen_idx)
          )
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
            lb = p_min[multi_idx]
            ub = p_max[multi_idx]

          if param.frozen:
            table_data.append(
                create_table_row(
                    element_name, val_frozen, lb, ub, is_frozen=True
                )
            )
          else:
            val = self._x_hat[non_frozen_idx]
            table_data.append(
                create_table_row(element_name, val, lb, ub, idx=non_frozen_idx)
            )
            non_frozen_idx += 1

    headers = ["Parameter"]

    if self._x_initial is not None:
      headers.append("Initial")

    if self._x_nominal is not None:
      headers.append("Nominal")

    headers.append("Identified")
    headers.extend(["Lower Bound", "Upper Bound"])

    if self._x_nominal is not None:
      headers.extend(["Absolute Change", "Relative Change"])

    return {
        "title": self._title,
        "headers": headers,
        "table_data": table_data,
        "overall_rmse": overall_rmse,
        "has_initial": self._x_initial is not None,
        "has_nominal": self._x_nominal is not None,
    }
