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
"""Tests for residual functions."""

from mujoco.sysid._src import residual
from mujoco.sysid._src import timeseries
import numpy as np


def test_construct_ts_from_defaults_reordered_and_subset_sensors():
  """Predicted sensors align by name rather than measured column index."""
  times = np.array([0.0, 0.1])
  state_mapping = {
      "j1_qpos": (timeseries.SignalType.MjStateQPos, np.array([0])),
  }
  state_ts = timeseries.TimeSeries(
      times, np.array([[0.5], [0.6]]), signal_mapping=state_mapping
  )

  # Predicted has extra leading "unused", then "pos" (3-D), then "vel" (1-D).
  pred_mapping = {
      "unused": (timeseries.SignalType.MjSensor, np.array([0])),
      "pos": (timeseries.SignalType.MjSensor, np.array([1, 2, 3])),
      "vel": (timeseries.SignalType.MjSensor, np.array([4])),
  }
  pred_data = np.array([
      [99.0, 1.0, 2.0, 3.0, 10.0],
      [99.0, 4.0, 5.0, 6.0, 20.0],
  ])
  pred_ts = timeseries.TimeSeries(times, pred_data, signal_mapping=pred_mapping)

  # Measured data has "vel" in column 0 and "pos" in columns 1..3.
  measured_mapping = {
      "vel": (timeseries.SignalType.MjSensor, np.array([0])),
      "pos": (timeseries.SignalType.MjSensor, np.array([1, 2, 3])),
  }
  measured_data = np.array([
      [10.0, 1.0, 2.0, 3.0],
      [20.0, 4.0, 5.0, 6.0],
  ])
  measured_ts = timeseries.TimeSeries(
      times, measured_data, signal_mapping=measured_mapping
  )

  sel_measured, sel_predicted = residual.construct_ts_from_defaults(
      state_ts, pred_ts, measured_ts
  )

  np.testing.assert_allclose(sel_measured.data, measured_data)
  np.testing.assert_allclose(sel_predicted.data, measured_data)
