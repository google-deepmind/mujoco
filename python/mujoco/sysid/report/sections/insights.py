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
"""Automated insights report section."""

from typing import Any

from mujoco.sysid._src import parameter
from mujoco.sysid.report.sections.base import ReportSection


class AutomatedInsights(ReportSection):
  """Analyzes identification results and generates automated insights/suggestions.

  Currently checks for parameters stuck at boundaries.
  """

  def __init__(
      self,
      title: str,
      parameter_dict: parameter.ParameterDict,
      anchor: str = "insights",
      collapsible: bool = True,
  ):
    # Default to collapsed
    super().__init__(collapsible=collapsible, is_open=False)
    self._anchor = anchor
    self._parameter_dict = parameter_dict
    self._x_hat = parameter_dict.as_vector()

    self._insights = self._generate_insights()
    self._n_warnings = len(
        [i for i in self._insights if i["type"] == "warning"]
    )

    self._title = f"Log - {self._n_warnings} warnings"

  def _generate_insights(self) -> list[dict[str, Any]]:
    insights = []

    # Logic: Check for boundary hits
    param_names = self._parameter_dict.get_non_frozen_parameter_names()
    bounds = self._parameter_dict.get_bounds()
    lower_bounds, upper_bounds = bounds

    if len(self._x_hat) != len(param_names):
      raise ValueError("Parameter count mismatch")

    for i, name in enumerate(param_names):
      val = self._x_hat[i]
      lb = lower_bounds[i]
      ub = upper_bounds[i]

      rng = max(ub - lb, 1e-9)

      # Threshold: 0.1% of range or 1e-6 absolute
      threshold = rng * 1e-3
      if threshold < 1e-8:
        threshold = 1e-8

      if abs(val - lb) < threshold:
        insights.append({
            "type": "warning",
            "title": "Lower Bound Hit",
            "message": (
                f"Parameter <b>{name}</b> ({val:.4g}) is at its lower bound"
                f" ({lb:.4g})."
            ),
        })
      elif abs(val - ub) < threshold:
        insights.append({
            "type": "warning",
            "title": "Upper Bound Hit",
            "message": (
                f"Parameter <b>{name}</b> ({val:.4g}) is at its upper bound"
                f" ({ub:.4g})."
            ),
        })

    if not insights:
      insights.append({
          "type": "success",
          "title": "No Issues Detected",
          "message": "All parameters are within their bounds.",
      })

    return insights

  @property
  def template_filename(self) -> str:
    return "insights.html"

  @property
  def title(self) -> str:
    return self._title

  @property
  def anchor(self) -> str:
    return self._anchor

  def get_context(self) -> dict[str, Any]:
    return {"title": self._title, "insights": self._insights}
