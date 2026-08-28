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
"""Tests for signal_modifier and SignalTransform."""

from mujoco.sysid._src import parameter
from mujoco.sysid._src import signal_modifier
from mujoco.sysid._src import timeseries
from mujoco.sysid._src.parameter import Parameter
from mujoco.sysid._src.parameter import ParameterDict
from mujoco.sysid._src.signal_transform import SignalTransform
import numpy as np
import pytest


# ===========================================================================
# Helpers
# ===========================================================================


def _make_pdict(*params: Parameter) -> ParameterDict:
  pdict = ParameterDict()
  for p in params:
    pdict.add(p)
  return pdict


def _make_arm_sensor_ts(arm_model):
  """Create a synthetic TimeSeries with signal_mapping matching arm sensors."""
  n_steps = 10
  n_sensors = arm_model.nsensordata
  times = np.linspace(0, 1, n_steps)
  data = np.random.default_rng(42).standard_normal((n_steps, n_sensors))

  mapping = {}
  for i in range(arm_model.nsensor):
    name = arm_model.sensor(i).name
    adr = arm_model.sensor_adr[i]
    dim = arm_model.sensor_dim[i]
    mapping[name] = (timeseries.SignalType.MjSensor, np.arange(adr, adr + dim))

  return timeseries.TimeSeries(times, data, signal_mapping=mapping)


def _make_resample_ts(n_steps, n_cols, seed=0):
  """Deterministic TimeSeries for resample tests."""
  rng = np.random.default_rng(seed)
  times = np.linspace(0, 1, n_steps)
  data = rng.standard_normal((n_steps, n_cols))
  mapping = {
      f"s{i}": (timeseries.SignalType.MjSensor, np.array([i]))
      for i in range(n_cols)
  }
  return timeseries.TimeSeries(times, data, signal_mapping=mapping)


def _make_transform_sensor_ts(n_steps=50, n_sensors=15, seed=0):
  """Deterministic TimeSeries with named MjSensor columns for transforms."""
  rng = np.random.default_rng(seed)
  times = np.linspace(0, 1, n_steps)
  data = rng.standard_normal((n_steps, n_sensors))
  mapping = {
      f"joint{i + 1}_pos": (timeseries.SignalType.MjSensor, np.array([i]))
      for i in range(5)
  }
  mapping.update({
      f"joint{i - 4}_vel": (timeseries.SignalType.MjSensor, np.array([i]))
      for i in range(5, 10)
  })
  mapping.update({
      f"joint{i - 9}_torque": (timeseries.SignalType.MjSensor, np.array([i]))
      for i in range(10, 15)
  })
  return timeseries.TimeSeries(times, data, signal_mapping=mapping)


def _run_both(ts, times, default_delay, sensor_delays, predicted_data):
  """Run grouped and column-wise implementations, return both."""
  delays = signal_modifier._build_per_column_delays(
      ts, default_delay, sensor_delays, predicted_data
  )
  reference = signal_modifier._apply_resample_and_delay_columnwise(
      ts, times, delays
  )
  result = signal_modifier.apply_resample_and_delay(
      ts,
      times,
      default_delay,
      sensor_delays=sensor_delays,
      predicted_data=predicted_data,
  )
  return result.data, reference


def _run_gains_biases_both(transform, ts, target_label, params):
  """Run new and reference implementations, return both results."""
  new_result = transform._apply_gains_biases(ts, target_label, params)
  ref_result = transform._apply_gains_biases_reference(ts, target_label, params)
  return new_result, ref_result


# ===========================================================================
# signal_modifier: get_sensor_indices
# ===========================================================================


def test_get_sensor_indices(arm_model):
  """Sensor name lookup returns the right data column indices for one or many sensors."""
  indices = signal_modifier.get_sensor_indices(arm_model, "joint1_pos")
  assert isinstance(indices, list)
  assert len(indices) == 1

  indices = signal_modifier.get_sensor_indices(
      arm_model, ["joint1_pos", "joint2_pos"]
  )
  assert len(indices) == 2


# ===========================================================================
# signal_modifier: apply_gain / apply_bias
# ===========================================================================


def test_apply_gain(arm_model):
  """Gain scaling affects only the named sensor's columns, leaving others untouched."""
  ts = _make_arm_sensor_ts(arm_model)
  gain = parameter.Parameter("gain", 2.0, 0.5, 3.0)

  result = signal_modifier.apply_gain(ts, "joint1_torque", gain)

  idx = ts.get_indices("joint1_torque")[1]
  np.testing.assert_allclose(result.data[:, idx], ts.data[:, idx] * 2.0)
  other = [i for i in range(ts.data.shape[1]) if i not in idx]
  np.testing.assert_array_equal(result.data[:, other], ts.data[:, other])


def test_apply_bias(arm_model):
  """Bias offset affects only the named sensor's columns, leaving others untouched."""
  ts = _make_arm_sensor_ts(arm_model)
  bias = parameter.Parameter("bias", 0.5, -1.0, 1.0)

  result = signal_modifier.apply_bias(ts, "joint1_pos", bias)

  idx = ts.get_indices("joint1_pos")[1]
  np.testing.assert_allclose(result.data[:, idx], ts.data[:, idx] + 0.5)
  other = [i for i in range(ts.data.shape[1]) if i not in idx]
  np.testing.assert_array_equal(result.data[:, other], ts.data[:, other])


# ===========================================================================
# signal_modifier: apply_delayed_ts_window
# ===========================================================================


def test_apply_delayed_ts_window(arm_model):
  """Time-windowing crops timestamps to the overlapping region between two series."""
  ts = _make_arm_sensor_ts(arm_model)
  ts_delayed = _make_arm_sensor_ts(arm_model)

  result = signal_modifier.apply_delayed_ts_window(
      ts, ts_delayed, min_delay=0.0, max_delay=0.0
  )
  assert result.times[0] >= ts_delayed.times[0]
  assert result.times[-1] <= ts_delayed.times[-1]


# ===========================================================================
# signal_modifier: weighted_diff / normalize_residual
# ===========================================================================


def test_weighted_diff_basic():
  """Without weights, the residual is simply measured minus predicted."""
  predicted = np.array([[1.0, 2.0], [3.0, 4.0]])
  measured = np.array([[1.1, 2.2], [3.3, 4.4]])
  result = signal_modifier.weighted_diff(predicted, measured)
  np.testing.assert_allclose(result, measured - predicted)


def test_weighted_diff_with_weights(arm_model):
  """Sensor weights let you emphasize or de-emphasize specific channels in the residual."""
  n = arm_model.nsensordata
  predicted = np.ones((5, n))
  measured = np.ones((5, n)) * 2.0
  weights = {"joint1_pos": 0.5}
  result = signal_modifier.weighted_diff(
      predicted, measured, arm_model, weights
  )
  idx = signal_modifier.get_sensor_indices(arm_model, "joint1_pos")
  np.testing.assert_allclose(result[:, idx], 0.5)
  other = [i for i in range(n) if i not in idx]
  np.testing.assert_allclose(result[:, other], 1.0)


def test_normalize_residual():
  """Normalization makes residuals comparable across sensors with different scales."""
  residual = np.array([[2.0, 4.0], [6.0, 8.0]])
  measured = np.array([[1.0, 2.0], [3.0, 4.0]])
  result = signal_modifier.normalize_residual(residual, measured)
  norm = np.linalg.norm(measured, axis=0) / np.sqrt(2)
  np.testing.assert_allclose(result, residual / norm)


# ===========================================================================
# signal_modifier: resample_and_delay grouped vs columnwise equivalence
# ===========================================================================


def test_resample_delay_mixed_delays():
  """Optimized grouped resampling gives identical results to naive per-column resampling."""
  ts = _make_resample_ts(200, 8, seed=42)
  out_times = np.linspace(0.05, 0.95, 150)
  sensor_delays = {
      "s0": 0.01,
      "s1": 0.01,
      "s2": 0.01,
      "s3": 0.03,
      "s4": 0.03,
  }
  result, reference = _run_both(ts, out_times, 0.0, sensor_delays, True)
  np.testing.assert_array_equal(result, reference)


# ===========================================================================
# SignalTransform: pattern matching
# ===========================================================================


class TestPatternMatching:
  """Tests for signal transform pattern matching."""

  def test_basic_glob(self):
    """Wildcard patterns select the right sensors (e.g.

    '*_pos' matches positions only).
    """
    transform = SignalTransform()
    delay_param = Parameter("delay", [0.01], [0.0], [0.05])
    transform.delay("*_pos", delay_param)
    pdict = _make_pdict(delay_param)

    resolved = transform._resolve_delays(
        ["joint1_pos", "joint2_pos", "joint1_vel"], pdict
    )
    assert "joint1_pos" in resolved
    assert "joint2_pos" in resolved
    assert "joint1_vel" not in resolved
    assert resolved["joint1_pos"] == pytest.approx(0.01)

  def test_last_match_wins(self):
    """When patterns overlap, the last one registered takes priority."""
    transform = SignalTransform()
    general_delay = Parameter("delay_general", [0.01], [0.0], [0.05])
    specific_delay = Parameter("delay_specific", [0.05], [0.0], [0.1])
    transform.delay("*_torque", general_delay)
    transform.delay("joint5_torque", specific_delay)
    pdict = _make_pdict(general_delay, specific_delay)

    resolved = transform._resolve_delays(
        ["joint1_torque", "joint5_torque"], pdict
    )
    assert resolved["joint1_torque"] == pytest.approx(0.01)
    assert resolved["joint5_torque"] == pytest.approx(0.05)

  def test_no_match(self):
    """Patterns that don't match any sensor names produce no delay entries."""
    transform = SignalTransform()
    delay_param = Parameter("delay", [0.01], [0.0], [0.05])
    transform.delay("*_pos", delay_param)
    pdict = _make_pdict(delay_param)

    resolved = transform._resolve_delays(["joint1_vel", "joint2_vel"], pdict)
    assert not resolved


# ===========================================================================
# SignalTransform: delay bounds
# ===========================================================================


class TestDelayBounds:
  """Tests for delay bound computation."""

  def test_single_param(self):
    """The min/max delay window is derived from a parameter's declared bounds."""
    transform = SignalTransform()
    delay_param = Parameter("delay", [0.01], [-0.02], [0.05])
    transform.delay("*_pos", delay_param)

    min_d, max_d = transform._compute_delay_bounds()
    assert min_d == pytest.approx(-0.02)
    assert max_d == pytest.approx(0.05)

  def test_dedup_by_name(self):
    """Reusing one delay param across patterns doesn't double-count its bounds."""
    transform = SignalTransform()
    delay_param = Parameter("delay", [0.01], [-0.01], [0.05])
    transform.delay("*_pos", delay_param)
    transform.delay("*_vel", delay_param)

    min_d, max_d = transform._compute_delay_bounds()
    assert min_d == pytest.approx(-0.01)
    assert max_d == pytest.approx(0.05)


# ===========================================================================
# SignalTransform: edge cases
# ===========================================================================


class TestEdgeCases:
  """Tests for edge cases in signal transforms."""

  def test_enable_sensors_stores_copy(self):
    """The sensor list is defensively copied so callers can't mutate it after the fact."""
    transform = SignalTransform()
    sensors = ["a", "b"]
    transform.enable_sensors(sensors)
    sensors.append("c")
    assert transform._enabled_sensors == ["a", "b"]

  def test_invalid_target(self):
    """Typos in the target argument ('predicted'/'measured'/'both') are caught early."""
    transform = SignalTransform()
    param = Parameter("gain", [1.0], [0.5], [2.0])
    with pytest.raises(ValueError, match="target must be"):
      transform.gain("*", param, target="invalid")

    bias_param = Parameter("bias", [0.0], [-1.0], [1.0])
    with pytest.raises(ValueError, match="target must be"):
      transform.bias("*", bias_param, target="invalid")


# ===========================================================================
# SignalTransform: _apply_gains_biases equivalence
# ===========================================================================


class TestApplyGainsBiasesEquivalence:
  """Tests for gains/biases application equivalence."""

  def test_gains_and_biases_mixed(self):
    """Applying gains and biases together produces the same result as the reference path."""
    ts = _make_transform_sensor_ts()
    gain = Parameter("torque_scale", [1.5], [0.5], [3.0])
    bias = Parameter("torque_bias", [0.3], [-1.0], [1.0])
    pdict = _make_pdict(gain, bias)

    transform = SignalTransform()
    transform.gain("*_torque", gain, target="both")
    transform.bias("*_torque", bias, target="both")

    new, ref = _run_gains_biases_both(transform, ts, "predicted", pdict)
    np.testing.assert_array_equal(new.data, ref.data)

  def test_target_filtering(self):
    """A gain meant for measured data doesn't accidentally affect the predicted side."""
    ts = _make_transform_sensor_ts()
    gain = Parameter("gain", [2.0], [0.5], [3.0])
    pdict = _make_pdict(gain)

    transform = SignalTransform()
    transform.gain("*_torque", gain, target="measured")

    new, ref = _run_gains_biases_both(transform, ts, "predicted", pdict)
    np.testing.assert_array_equal(new.data, ref.data)
    np.testing.assert_array_equal(new.data, ts.data)

  def test_original_ts_not_mutated(self):
    """Signal transforms produce new data without mutating the input TimeSeries."""
    ts = _make_transform_sensor_ts()
    original_data = ts.data.copy()
    gain = Parameter("gain", [2.0], [0.5], [3.0])
    pdict = _make_pdict(gain)

    transform = SignalTransform()
    transform.gain("*_torque", gain, target="predicted")

    transform._apply_gains_biases(ts, "predicted", pdict)
    np.testing.assert_array_equal(ts.data, original_data)
