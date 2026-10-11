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

"""Declarative signal transformation for system identification residuals."""

from __future__ import annotations

from collections.abc import Mapping
from fnmatch import fnmatch

import mujoco
from mujoco.sysid._src import parameter
from mujoco.sysid._src import signal_modifier
from mujoco.sysid._src import timeseries
import numpy as np


class SignalTransform:
  """Declarative signal transformation replacing boilerplate modify_residual callbacks.

  Usage::

    transform = SignalTransform()
    transform.delay("*_pos", params["delay_pos"])
    transform.delay("*_torque", params["delay_torque"])
    transform.gain("*_torque", params["torque_scale"], target="predicted")
    transform.enable_sensors(cfg.sensors_enabled)

  The ``apply`` method has the same signature as ``ModifyResidualFn`` and can
  be passed directly to ``build_residual_fn(signal_transform=transform)``.
  """

  def __init__(self, normalize: bool = True):
    self._delays: list[tuple[str, str, parameter.Parameter]] = []
    self._gains: list[tuple[str, str, str]] = []
    self._biases: list[tuple[str, str, str]] = []
    self._enabled_sensors: list[str] | None = None
    self._sensor_weights: Mapping[str, float] | None = None
    self.normalize = normalize

  def delay(self, pattern: str, param: parameter.Parameter) -> None:
    """Register a delay for sensors matching *pattern* (fnmatch).

    Args:
      pattern: fnmatch pattern matched against sensor names.
      param: Parameter whose ``.value`` is the delay in seconds.
    """
    self._delays.append((pattern, param.name, param))

  def gain(
      self,
      pattern: str,
      param: parameter.Parameter,
      target: str = "both",
  ) -> None:
    """Register a multiplicative gain for sensors matching *pattern*.

    Args:
      pattern: fnmatch pattern matched against sensor names.
      param: Parameter whose ``.value`` is the gain factor.
      target: One of ``"predicted"``, ``"measured"``, or ``"both"``.
    """
    if target not in ("predicted", "measured", "both"):
      raise ValueError(
          f"target must be 'predicted', 'measured', or 'both', got {target!r}"
      )
    self._gains.append((pattern, param.name, target))

  def bias(
      self,
      pattern: str,
      param: parameter.Parameter,
      target: str = "both",
  ) -> None:
    """Register an additive bias for sensors matching *pattern*.

    Args:
      pattern: fnmatch pattern matched against sensor names.
      param: Parameter whose ``.value`` is the additive bias.
      target: One of ``"predicted"``, ``"measured"``, or ``"both"``.
    """
    if target not in ("predicted", "measured", "both"):
      raise ValueError(
          f"target must be 'predicted', 'measured', or 'both', got {target!r}"
      )
    self._biases.append((pattern, param.name, target))

  def enable_sensors(self, sensor_names: list[str]) -> None:
    """Only include these sensors in the returned residual/timeseries.

    Args:
      sensor_names: Sensor names to keep in the output.
    """
    self._enabled_sensors = list(sensor_names)

  def set_sensor_weights(self, weights: Mapping[str, float]) -> None:
    """Set per-sensor weights for the weighted diff.

    Args:
      weights: Mapping from sensor name to weight.
    """
    self._sensor_weights = weights

  # Private methods.

  def _resolve_delays(
      self,
      sensor_names: list[str],
      params: parameter.ParameterDict,
  ) -> dict[str, float]:
    """Resolve delay patterns to concrete sensor name -> delay value (last match wins)."""
    resolved: dict[str, float] = {}
    for pattern, param_name, _ in self._delays:
      delay_value = params[param_name].value[0]
      for name in sensor_names:
        if fnmatch(name, pattern):
          resolved[name] = delay_value
    return resolved

  def _compute_delay_bounds(self) -> tuple[float, float]:
    """Compute min/max delay across all registered delay params (deduplicated by name)."""
    if not self._delays:
      return 0.0, 0.0
    seen: set[str] = set()
    min_vals: list[float] = []
    max_vals: list[float] = []
    for _, param_name, param in self._delays:
      if param_name in seen:
        continue
      seen.add(param_name)
      min_vals.append(float(param.min_value[0]))
      max_vals.append(float(param.max_value[0]))
    return min(min_vals), max(max_vals)

  def _get_sensor_names(self, ts: timeseries.TimeSeries) -> list[str]:
    """Extract sensor names from a TimeSeries signal_mapping."""
    if ts.signal_mapping is None:
      return []
    return list(ts.signal_mapping.keys())

  def _apply_gains_biases_reference(
      self,
      ts: timeseries.TimeSeries,
      target_label: str,
      params: parameter.ParameterDict,
  ) -> timeseries.TimeSeries:
    """Reference implementation: one full copy per gain/bias application."""
    sensor_names = self._get_sensor_names(ts)
    for pattern, param_name, target in self._gains:
      if target != target_label and target != "both":
        continue
      for name in sensor_names:
        if fnmatch(name, pattern):
          ts = signal_modifier.apply_gain(ts, name, params[param_name])
    for pattern, param_name, target in self._biases:
      if target != target_label and target != "both":
        continue
      for name in sensor_names:
        if fnmatch(name, pattern):
          ts = signal_modifier.apply_bias(ts, name, params[param_name])
    return ts

  _VERIFY_GAINS_BIASES = False

  def _apply_gains_biases(
      self,
      ts: timeseries.TimeSeries,
      target_label: str,
      params: parameter.ParameterDict,
  ) -> timeseries.TimeSeries:
    """Apply matching gains and biases to a timeseries for the given target label."""
    sensor_names = self._get_sensor_names(ts)
    data = ts.data.copy()

    for pattern, param_name, target in self._gains:
      if target != target_label and target != "both":
        continue
      for name in sensor_names:
        if fnmatch(name, pattern):
          indices = ts.get_indices(name)[1]
          data[..., indices] *= params[param_name].value

    for pattern, param_name, target in self._biases:
      if target != target_label and target != "both":
        continue
      for name in sensor_names:
        if fnmatch(name, pattern):
          indices = ts.get_indices(name)[1]
          data[..., indices] += params[param_name].value

    result = timeseries.TimeSeries(ts.times, data, ts.signal_mapping)

    if self._VERIFY_GAINS_BIASES:
      ref = self._apply_gains_biases_reference(ts, target_label, params)
      np.testing.assert_array_equal(result.data, ref.data)

    return result

  def apply(
      self,
      params: parameter.ParameterDict,
      sensordata_predicted: timeseries.TimeSeries,
      sensordata_measured: timeseries.TimeSeries,
      model: mujoco.MjModel,
      return_pred_all: bool,
      state: np.ndarray | None = None,
      sensor_weights: Mapping[str, float] | None = None,
  ) -> tuple[np.ndarray, timeseries.TimeSeries, timeseries.TimeSeries]:
    """Apply all registered transforms and compute the residual.

    Signature matches :data:`ModifyResidualFn` so this method can be passed
    directly as ``modify_residual`` to :func:`model_residual`.

    Pipeline: window measured data, resample + delay predicted data, apply
    gains/biases, weighted diff, normalise, slice to enabled sensors.

    Args:
      params: Current parameter dictionary.
      sensordata_predicted: Predicted sensor TimeSeries from rollout.
      sensordata_measured: Measured sensor TimeSeries (ground truth).
      model: MuJoCo model for sensor metadata lookup.
      return_pred_all: Whether to return all predicted signals.
      state: Unused; part of the ModifyResidualFn signature.
      sensor_weights: Per-sensor weighting for the residual.

    Returns:
      ``(residual_array, predicted_ts, measured_ts)`` — the residual matrix
      and the (possibly sliced) predicted/measured TimeSeries.
    """
    del state  # Part of ModifyResidualFn signature but unused here.
    sensor_names = self._get_sensor_names(sensordata_predicted)

    # 1. Resolve delays and compute bounds.
    sensor_delays = self._resolve_delays(sensor_names, params)
    min_delay, max_delay = self._compute_delay_bounds()

    # 2. Window measured data.
    sensordata_measured = signal_modifier.apply_delayed_ts_window(
        sensordata_measured, sensordata_predicted, min_delay, max_delay
    )

    # 3. Resample and delay predicted data.
    if sensor_delays:
      sensordata_predicted = signal_modifier.apply_resample_and_delay(
          sensordata_predicted,
          sensordata_measured.times,
          0.0,
          sensor_delays=sensor_delays,
      )
    else:
      sensordata_predicted = sensordata_predicted.resample(
          sensordata_measured.times
      )

    # 4. Apply gains and biases.
    sensordata_predicted = self._apply_gains_biases(
        sensordata_predicted, "predicted", params
    )
    sensordata_measured = self._apply_gains_biases(
        sensordata_measured, "measured", params
    )

    # 5. Weighted diff.
    weights = sensor_weights or self._sensor_weights
    res = signal_modifier.weighted_diff(
        predicted_data=sensordata_predicted.data,
        measured_data=sensordata_measured.data,
        model=model,
        sensor_weights=weights,
    )

    # 6. Normalize.
    if self.normalize:
      res = signal_modifier.normalize_residual(res, sensordata_measured.data)

    # 7. Slice to enabled sensors.
    if not return_pred_all and self._enabled_sensors is not None:
      indices = signal_modifier.get_sensor_indices(model, self._enabled_sensors)
      sensordata_predicted = timeseries.TimeSeries(
          sensordata_predicted.times,
          sensordata_predicted.data[:, indices],
      )
      sensordata_measured = timeseries.TimeSeries(
          sensordata_measured.times,
          sensordata_measured.data[:, indices],
      )
      res = res[:, indices]

    return res, sensordata_predicted, sensordata_measured
