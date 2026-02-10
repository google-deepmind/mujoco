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

"""Residual computation for system identification."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
import copy
import os
from typing import TypeAlias

import mujoco
from mujoco.sysid._src import model_modifier
from mujoco.sysid._src import parameter
from mujoco.sysid._src import signal_modifier
from mujoco.sysid._src import timeseries
from mujoco.sysid._src.trajectory import ModelSequences
from mujoco.sysid._src.trajectory import SystemTrajectory
from mujoco.sysid._src.trajectory import sysid_rollout
import numpy as np

_NUM_CPUS: int = os.cpu_count() or 1

BuildModelFn: TypeAlias = Callable[
    [parameter.ParameterDict, mujoco.MjSpec], mujoco.MjModel
]

CustomRolloutFn: TypeAlias = Callable[..., Sequence[SystemTrajectory]]
"""Replaces the default sysid_rollout. Called with keyword arguments:
models, datas, control_signal, initial_states, param_dicts,
rollout_signal_mapping, rollout_state_mapping, ctrl_mapping."""

ModifyResidualFn: TypeAlias = Callable[
    ..., tuple[np.ndarray, timeseries.TimeSeries, timeseries.TimeSeries]
]
"""Custom residual computation. Called as:
modify_residual(params, sensordata_predicted, sensordata_measured,
model, return_pred_all, state=..., sensor_weights=...)."""


def construct_ts_from_defaults(
    state_ts: timeseries.TimeSeries,
    pred_sensordata: timeseries.TimeSeries,
    measured_sensordata: timeseries.TimeSeries,
    enabled_observations: (
        Sequence[tuple[str, timeseries.SignalType]] | None
    ) = None,
):
  """Assemble predicted observations to match the measured signal layout.

  For each enabled observation, copies the predicted values from either
  ``pred_sensordata`` (for MjSensor signals) or ``state_ts`` (for state
  signals like qpos/qvel/act) into a new array whose columns align with
  the measured data.

  Args:
    state_ts: Predicted state TimeSeries (time column already stripped).
    pred_sensordata: Raw predicted sensor TimeSeries from rollout.
    measured_sensordata: Measured sensor TimeSeries (defines the target layout).
    enabled_observations: Subset of observations to include.  If None, all
      observations in ``measured_sensordata`` are used.

  Returns:
    A ``(measured, predicted)`` tuple of TimeSeries with matching signal
    mappings, sliced to the enabled observations.
  """
  assert measured_sensordata.signal_mapping is not None

  # Trim measured data enabled observations
  if enabled_observations:
    enabled_observations_names = [i[0] for i in enabled_observations]
    enabled_observations_types = [i[1] for i in enabled_observations]
  else:
    enabled_observations_names = list(measured_sensordata.signal_mapping.keys())
    enabled_observations_types = [
        v[0] for v in measured_sensordata.signal_mapping.values()
    ]

  selected_measured_sensordata = timeseries.TimeSeries.slice_by_name(
      measured_sensordata, enabled_observations_names
  )
  assert selected_measured_sensordata.signal_mapping is not None
  selected_measured_signal_mapping = selected_measured_sensordata.signal_mapping

  shape = (
      pred_sensordata.data.shape[0],
      selected_measured_sensordata.data.shape[1],
  )
  predicted_data_out = np.zeros(shape)

  measured_signal_mapping = measured_sensordata.signal_mapping
  for enabled_obs_name, enabled_obs_type in zip(
      enabled_observations_names, enabled_observations_types, strict=True
  ):
    assert state_ts.signal_mapping is not None
    if (
        enabled_obs_name not in measured_signal_mapping
        and enabled_obs_name not in state_ts.signal_mapping
    ):
      raise ValueError(f"{enabled_obs_name} is missing.")

    obs_type, indices = measured_signal_mapping[enabled_obs_name]

    if obs_type != enabled_obs_type:
      raise ValueError(
          f"Observation type error: {enabled_obs_name} is of type {obs_type}"
          f" but declared as {enabled_obs_type}."
      )

    if obs_type == timeseries.SignalType.CustomObs:
      raise ValueError(
          "You are attempting to use the default SysID's modify_residual with"
          f" a custom observation of name {enabled_obs_name}. This is not"
          " supported. You must implement your own modify_residual. See"
          " documentation at ..."
      )

    elif obs_type == timeseries.SignalType.MjSensor:
      target_indices = selected_measured_signal_mapping[enabled_obs_name][1]
      predicted_data_out[:, ..., target_indices] = pred_sensordata.data[
          :, ..., indices
      ]

    elif (
        obs_type == timeseries.SignalType.MjStateQPos
        or obs_type == timeseries.SignalType.MjStateQVel
        or obs_type == timeseries.SignalType.MjStateAct
    ):
      state_indices = state_ts.signal_mapping[enabled_obs_name][1]

      values = state_ts.data[:, ..., state_indices]
      target_indices = selected_measured_signal_mapping[enabled_obs_name][1]
      predicted_data_out[:, ..., target_indices] = values

  ts_predicted_data = timeseries.TimeSeries(
      pred_sensordata.times,
      predicted_data_out,
      selected_measured_sensordata.signal_mapping,
  )

  return selected_measured_sensordata, ts_predicted_data


# Lowest level residual function, works on one model
def model_residual(
    x: np.ndarray,
    params: parameter.ParameterDict,
    build_model: Callable[[parameter.ParameterDict], mujoco.MjModel],
    traj_measured: Sequence[SystemTrajectory] | SystemTrajectory,
    modify_residual: ModifyResidualFn | None = None,
    custom_rollout: CustomRolloutFn | None = None,
    n_threads: int = _NUM_CPUS,
    return_pred_all: bool = False,
    resample_true: bool = True,
    sensor_weights: Mapping[str, float] | None = None,
    enabled_observations: Sequence[tuple[str, timeseries.SignalType]] = (),
):
  """Compute residuals for a single model against measured trajectories.

  Builds the model from *x*, rolls out each trajectory, and computes the
  weighted difference between predicted and measured sensor data.

  Args:
    x: Decision variable vector (flat, or 2-D for batched finite-difference).
    params: Parameter dictionary — updated in-place from *x*.
    build_model: ``(ParameterDict) -> MjModel`` factory.
    traj_measured: Ground-truth trajectory or sequence of trajectories.
    modify_residual: Optional custom residual callback (replaces the default
      resampling / differencing logic).
    custom_rollout: Optional replacement for :func:`sysid_rollout`.
    n_threads: Number of ``MjData`` scratch objects for parallel rollout.
    return_pred_all: If True, return full predicted/measured TimeSeries.
    resample_true: Whether to resample the measured data at simulation timesteps
      (ignored when *modify_residual* is provided).
    sensor_weights: Per-sensor weights for the weighted diff.
    enabled_observations: Subset of ``(name, SignalType)`` pairs to include.

  Returns:
    A 3-tuple ``(residuals, pred_sensordatas, measured_sensordatas)``.
  """
  # Convert single trajectory to list for consistent handling.
  if isinstance(traj_measured, SystemTrajectory):
    traj_measured = [traj_measured]
  n_chunks = len(traj_measured)

  # Handle finite difference columns if present.
  initial_ndim = x.ndim
  n_fd = 1
  if x.ndim > 1:
    n_fd = x.shape[1]
    x_reshaped = x
  else:
    x_reshaped = x.reshape(-1, 1)

  # Process each finite difference column.
  models = []
  models_x = []
  model_0 = None
  for i in range(n_fd):
    params.update_from_vector(x_reshaped[:, i])
    model = build_model(params)
    if not model_0:
      model_0 = model
    models_x.extend([x_reshaped[:, i]] * n_chunks)
    models.extend([model] * n_chunks)

  assert model_0 is not None
  qpos_map, qvel_map, act_map, rollout_ctrl_map = (
      timeseries.TimeSeries.compute_all_state_mappings(model_0)
  )
  rollout_state_mapping = qpos_map | qvel_map | act_map
  rollout_signal_mapping = timeseries.TimeSeries.compute_all_sensor_mapping(
      model_0
  )

  # Create data objects for parallel computation.
  datas = [mujoco.MjData(models[0]) for _ in range(n_threads)]

  # Interpolate control signal.
  if resample_true:
    control_chunks = [
        traj.control.resample(target_dt=models[0].opt.timestep)
        for traj in traj_measured
    ]
  else:
    control_chunks = [traj.control for traj in traj_measured]

  # Rollout trajectories in parallel.
  if custom_rollout is None:
    pred_trajectories = sysid_rollout(
        models=models[: n_fd * n_chunks],
        datas=datas,
        control_signal=[control for control in control_chunks] * n_fd,
        initial_states=[chunk.initial_state for chunk in traj_measured] * n_fd,
        rollout_signal_mapping=rollout_signal_mapping,
        rollout_state_mapping=rollout_state_mapping,
        ctrl_mapping=rollout_ctrl_map,
    )
  else:
    param_dicts = [copy.deepcopy(params) for i in range(x_reshaped.shape[1])]
    for i in range(x_reshaped.shape[1]):
      param_dicts[i].update_from_vector(x_reshaped[:, i])
    pred_trajectories = custom_rollout(
        models=models[: n_fd * n_chunks],
        datas=datas,
        control_signal=[control for control in control_chunks] * n_fd,
        initial_states=[chunk.initial_state for chunk in traj_measured] * n_fd,
        param_dicts=param_dicts,
        rollout_signal_mapping=rollout_signal_mapping,
        rollout_state_mapping=rollout_state_mapping,
        ctrl_mapping=rollout_ctrl_map,
    )

  # Compute residuals for each trajectory chunk.
  all_residuals = []
  pred_sensordatas = []
  measured_sensordatas = []

  for i in range(len(models)):
    model = models[i]
    pred_traj = pred_trajectories[i]
    assert pred_traj.state is not None
    pred_state = pred_traj.state.data

    rollout_state_ts = timeseries.TimeSeries(
        times=pred_state[:, 0],
        data=pred_state[:, 1:],
        signal_mapping=rollout_state_mapping,
    )

    measuredidx = i % n_chunks
    measuredtraj = traj_measured[measuredidx]

    pred_sensordata = pred_traj.sensordata
    measured_sensordata = measuredtraj.sensordata

    # If the user passes a residual function allow them to
    # handle all resampling, etc.
    if modify_residual is not None:
      params.update_from_vector(models_x[i])
      res, pred_sensordata, measured_sensordata = modify_residual(
          params,
          pred_sensordata,
          measured_sensordata,
          model,
          return_pred_all,
          state=pred_state,
      )

    # If the user does not pass a residual function, resample
    # the ground truth data to match the sime times if requested.
    else:
      measured_sensordata, pred_sensordata = construct_ts_from_defaults(
          rollout_state_ts,
          pred_sensordata,
          measured_sensordata,
          enabled_observations,
      )
      if resample_true:
        # Window the true data so that times in it correspond
        # to times spanned by predicted data.
        measured_sensordata = signal_modifier.apply_delayed_ts_window(
            measured_sensordata, pred_sensordata, 0.0, 0.0
        )
        # Sample the predicted signal at the true times.
        pred_sensordata = pred_sensordata.resample(measured_sensordata.times)

      else:
        # Do not include difference in first sensor outputs in residual vector.
        # It corresponds to the initial condition and so provides little new
        # information. Additionally the semantics of rollout
        # make it difficult to
        # simulate the sensor output corresponding to the initial condition.
        measured_sensordata = timeseries.TimeSeries(
            measured_sensordata.times[1:],
            measured_sensordata.data[1:, :],
            measured_sensordata.signal_mapping,
        )

      res = signal_modifier.weighted_diff(
          predicted_data=pred_sensordata.data,
          measured_data=measured_sensordata.data,
          model=model,
          sensor_weights=sensor_weights,
      )
      res = signal_modifier.normalize_residual(res, measured_sensordata.data)

    if pred_sensordata.signal_mapping != measured_sensordata.signal_mapping:
      raise ValueError(
          "The observation mapping between the measured data and predicted"
          " rollout data is not the same. You have not modified the observation"
          " data in TimeSeries in modify_residual to correctly reflect the"
          " measured data."
      )

    all_residuals.append(res)
    pred_sensordatas.append(pred_sensordata)
    measured_sensordatas.append(measured_sensordata)

  res_array = np.stack(all_residuals, axis=0)
  if initial_ndim == 1:
    res_array = res_array.ravel()
  else:
    res_array = res_array.reshape(res_array.shape[0], -1)

  return res_array.T, pred_sensordatas, measured_sensordatas


def build_residual_fn(**captured_kwargs):
  """Create a residual closure with pre-bound keyword arguments.

  Returns a function ``fn(x, params, **overrides)`` that calls
  :func:`residual` with the captured kwargs merged in.  This is the
  recommended way to construct the callable passed to :func:`optimize`.

  Example::

      residual_fn = build_residual_fn(
          models_sequences=seqs,
          signal_transform=transform,
      )
      opt_params, result = optimize(params, residual_fn)
  """

  def built_residual_fn(x, params, **kwargs):
    return residual(
        x,
        params,
        **captured_kwargs,
        **kwargs,
    )

  return built_residual_fn


def residual(
    x: np.ndarray,
    params: parameter.ParameterDict,
    models_sequences: list[ModelSequences],
    build_model: BuildModelFn = model_modifier.apply_param_modifiers,
    modify_residual: ModifyResidualFn | None = None,
    custom_rollout: CustomRolloutFn | None = None,
    n_threads: int = _NUM_CPUS,
    return_pred_all: bool = False,
    resample_true: bool = True,
    sensor_weights: Mapping[str, float] | None = None,
    enabled_observations: Sequence[tuple[str, timeseries.SignalType]] = (),
):
  """Top-level residual: iterate over all model-sequence groups.

  Calls :func:`model_residual` for every measured rollout in every
  :class:`ModelSequences` entry and collects the results.

  Args:
    x: Current parameter vector.
    params: Parameter dictionary with bounds and metadata.
    models_sequences: List of model-sequence groups.
    build_model: Callable to compile a model from parameters.
    modify_residual: Optional post-processing for the residual.
    custom_rollout: Optional custom rollout function.
    n_threads: Number of threads for parallel rollout.
    return_pred_all: Whether to return all predicted signals.
    resample_true: Whether to resample ground-truth to predicted times.
    sensor_weights: Per-sensor weighting for the residual.
    enabled_observations: Additional observation signals to include.

  Returns:
    A 3-tuple ``(residuals, preds, records)`` — lists with one entry per
    measured rollout across all groups.
  """
  residuals = []
  preds = []
  records = []
  for model_sequences in models_sequences:
    for measured_rollout in model_sequences.measured_rollout:
      res = model_residual(
          x,
          params,
          lambda p, _spec=model_sequences.spec: build_model(p, _spec),
          measured_rollout,
          modify_residual,
          custom_rollout,
          n_threads,
          return_pred_all,
          resample_true,
          sensor_weights,
          enabled_observations,
      )
      if isinstance(res, np.ndarray):
        residuals.append(res)
      else:
        residuals.append(res[0])
        preds.append(res[1])
        records.append(res[2])

  return residuals, preds, records
