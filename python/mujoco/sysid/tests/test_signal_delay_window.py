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
"""Regression tests for the common window of delayed signal columns."""

import mujoco
from mujoco.sysid._src.parameter import Parameter
from mujoco.sysid._src.parameter import ParameterDict
from mujoco.sysid._src.signal_transform import SignalTransform
from mujoco.sysid._src.timeseries import SignalType
from mujoco.sysid._src.timeseries import TimeSeries
import numpy as np
import pytest


@pytest.mark.parametrize("delay", [1.0, -1.0])
@pytest.mark.parametrize("map_second_column", [True, False])
def test_undelayed_columns_bound_the_common_window(delay, map_second_column):
  """A delayed channel must not permit extrapolation of another channel."""
  times = np.arange(5, dtype=float)
  measured_times = np.arange(-1, 6, dtype=float)
  mapping = {"a": (SignalType.CustomObs, np.array([0]))}
  if map_second_column:
    mapping["b"] = (SignalType.CustomObs, np.array([1]))
  predicted = TimeSeries(times, np.column_stack([times, 2 * times]), mapping)
  measured = TimeSeries(
      measured_times,
      np.column_stack([measured_times - delay, 2 * measured_times]),
      mapping,
  )
  param = Parameter("delay", delay, delay, delay)
  transform = SignalTransform(normalize=False)
  transform.delay("a", param)
  residual, pred, observed = transform.apply(
      ParameterDict({"delay": param}),
      predicted,
      measured,
      mujoco.MjModel.from_xml_string("<mujoco/>"),
      return_pred_all=True,
  )
  expected_times = np.arange(1, 5) if delay > 0 else np.arange(4)
  np.testing.assert_array_equal(observed.times, expected_times)
  np.testing.assert_array_equal(pred.times, expected_times)
  np.testing.assert_allclose(residual, 0, atol=1e-14)


@pytest.mark.parametrize("overridden", [False, True])
def test_inactive_delay_bounds_do_not_discard_observations(overridden):
  """Unmatched and fully overridden delays do not constrain interpolation."""
  times = np.arange(5, dtype=float)
  ts = TimeSeries.from_custom_map(times, times[:, None], ["a"])
  unused = Parameter("unused", 10.0, 9.0, 11.0)
  zero = Parameter("zero", 0.0, 0.0, 0.0)
  transform = SignalTransform(normalize=False)
  transform.delay("*" if overridden else "missing", unused)
  if overridden:
    transform.delay("a", zero)
  residual, pred, observed = transform.apply(
      ParameterDict({"unused": unused, "zero": zero}),
      ts,
      ts,
      mujoco.MjModel.from_xml_string("<mujoco/>"),
      return_pred_all=True,
  )
  np.testing.assert_array_equal(observed.times, times)
  np.testing.assert_array_equal(pred.data, ts.data)
  np.testing.assert_array_equal(residual, np.zeros((5, 1)))


def test_fully_delayed_series_keeps_its_shifted_endpoint():
  """Zero is not included when every column has the same positive delay."""
  times = np.arange(5, dtype=float)
  predicted = TimeSeries.from_custom_map(times, times[:, None], ["a"])
  measured = TimeSeries.from_custom_map(times + 1, times[:, None], ["a"])
  delay = Parameter("delay", 1.0, 1.0, 1.0)
  transform = SignalTransform(normalize=False)
  transform.delay("*", delay)
  residual, _, observed = transform.apply(
      ParameterDict({"delay": delay}),
      predicted,
      measured,
      mujoco.MjModel.from_xml_string("<mujoco/>"),
      return_pred_all=True,
  )
  np.testing.assert_array_equal(observed.times, times + 1)
  np.testing.assert_array_equal(residual, np.zeros((5, 1)))


def test_shared_column_bounds_follow_the_applied_alias():
  """An alias overwritten at column expansion cannot narrow the window."""
  times = np.arange(5, dtype=float)
  ts = TimeSeries(
      times,
      times[:, None],
      {
          "first": (SignalType.CustomObs, np.array([0])),
          "last": (SignalType.CustomObs, np.array([0])),
      },
  )
  unused = Parameter("unused", 10.0, 10.0, 10.0)
  zero = Parameter("zero", 0.0, 0.0, 0.0)
  transform = SignalTransform(normalize=False)
  transform.delay("first", unused)
  transform.delay("last", zero)
  residual, _, observed = transform.apply(
      ParameterDict({"unused": unused, "zero": zero}),
      ts,
      ts,
      mujoco.MjModel.from_xml_string("<mujoco/>"),
      return_pred_all=True,
  )
  np.testing.assert_array_equal(observed.times, times)
  np.testing.assert_array_equal(residual, np.zeros((5, 1)))
