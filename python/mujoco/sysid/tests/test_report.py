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
"""Tests for sysid report sections."""

import mujoco
from mujoco.sysid._src import parameter
from mujoco.sysid._src import timeseries
from mujoco.sysid.report.sections import parameter_distribution
from mujoco.sysid.report.sections import signals
import numpy as np


def test_signal_report_multidimensional_sensor():
  """SignalReport allocates enough rows for multi-dimensional sensors."""
  m = mujoco.MjModel.from_xml_string("<mujoco/>")
  times = np.array([0.0, 0.1, 0.2])
  data = np.zeros((3, 4))
  mapping = {"quat": (timeseries.SignalType.MjSensor, np.array([0, 1, 2, 3]))}
  ts = timeseries.TimeSeries(data=data, times=times, signal_mapping=mapping)

  report = signals.SignalReport(
      title="Signals",
      model=m,
      ts_dict={"measured": ts, "predicted": ts},
  )
  context = report.get_context()
  assert "plot_div" in context


def test_parameter_distribution_none_confidence():
  """ParameterDistribution renders cleanly when confidence_intervals is None."""
  param = parameter.Parameter("mass", 1.0, 0.5, 2.0)
  pdict = parameter.ParameterDict({"mass": param})
  opt_pdict = pdict.copy()
  opt_pdict.update_from_vector(np.array([1.2]))

  dist = parameter_distribution.ParameterDistribution(
      title="Parameters",
      opt_params=opt_pdict,
      initial_params=pdict,
      confidence_intervals=None,
  )
  context = dist.get_context()
  assert "plot_div" in context
  assert "±" not in dist._build_figure().data[3].hovertemplate
