# Copyright 2022 DeepMind Technologies Limited
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
"""Roll out open-loop trajectories from initial states, get subsequent states and sensor values."""

from mujoco import _rollout
import numpy as np


def rollout(model, data, initial_state=None, ctrl=None,
            *,  # require following arguments to be named
            skip_checks=False,
            nstate=None,
            nstep=None,
            initial_time=None,
            initial_warmstart=None,
            qfrc_applied=None,
            xfrc_applied=None,
            mocap=None,
            state=None,
            sensordata=None):
  """Roll out open-loop trajectories from initial states, get subsequent states and sensor values.

    This function serves as a Python wrapper for the C++ functionality in
    `rollout.cc`, please see documentation therein. This python funtion will
    infer `nstate` and `nstep`, tile input arguments with singleton dimensions,
    and allocate output arguments if none are given.
  """
  # don't infer nstate/nstep, don't support singleton expansion, don't allocate
  # output arrays, just call rollout
  if skip_checks:
    _rollout.rollout(model, data, nstate, nstep, initial_state, initial_time,
                     initial_warmstart, ctrl, qfrc_applied, xfrc_applied, mocap,
                     state, sensordata)
    return state, sensordata

  # check types
  if nstate and not isinstance(nstate, int):
    raise ValueError('nstate must be an integer')
  if nstep and not isinstance(nstep, int):
    raise ValueError('nstep must be an integer')
  _check_must_be_numeric(
      initial_state=initial_state,
      initial_time=initial_time,
      initial_warmstart=initial_warmstart,
      ctrl=ctrl,
      qfrc_applied=qfrc_applied,
      xfrc_applied=xfrc_applied,
      mocap=mocap,
      state=state,
      sensordata=sensordata)

  # check number of dimensions
  _check_number_of_dimensions(2,
                              initial_state=initial_state,
                              initial_time=initial_time,
                              initial_warmstart=initial_warmstart)
  _check_number_of_dimensions(3,
                              ctrl=ctrl,
                              qfrc_applied=qfrc_applied,
                              xfrc_applied=xfrc_applied,
                              mocap=mocap,
                              state=state,
                              sensordata=sensordata)

  # ensure 2D, make contiguous, row-major (C ordering)
  initial_state = _ensure_2d(initial_state)
  initial_time = _ensure_2d(initial_time)
  initial_warmstart = _ensure_2d(initial_warmstart)

  # ensure 3D, make contiguous, row-major (C ordering)
  ctrl = _ensure_3d(ctrl)
  qfrc_applied = _ensure_3d(qfrc_applied)
  xfrc_applied = _ensure_3d(xfrc_applied)
  mocap = _ensure_3d(mocap)
  state = _ensure_3d(state)
  sensordata = _ensure_3d(sensordata)

  # check trailing dimensions
  _check_trailing_dimension(model.nq + model.nv + model.na,
                            initial_state=initial_state, state=state)
  _check_trailing_dimension(1, initial_time=initial_time)
  _check_trailing_dimension(model.nu, ctrl=ctrl)
  _check_trailing_dimension(model.nv, qfrc_applied=qfrc_applied)
  _check_trailing_dimension(model.nbody*6, xfrc_applied=xfrc_applied)
  _check_trailing_dimension(model.nmocap*7, mocap=mocap)
  _check_trailing_dimension(model.nsensordata, sensordata=sensordata)

  # infer nstate, check for incompatibilities
  nstate = _infer_dimension(0, nstate or 1,
                            initial_state=initial_state,
                            initial_time=initial_time,
                            initial_warmstart=initial_warmstart,
                            ctrl=ctrl,
                            qfrc_applied=qfrc_applied,
                            xfrc_applied=xfrc_applied,
                            mocap=mocap,
                            state=state,
                            sensordata=sensordata)

  # infer nstep, check for incompatibilities
  nstep = _infer_dimension(1, nstep or 1,
                           ctrl=ctrl,
                           qfrc_applied=qfrc_applied,
                           xfrc_applied=xfrc_applied,
                           mocap=mocap,
                           state=state,
                           sensordata=sensordata)

  # tile input arrays if required (singleton expansion)
  initial_state = _tile_if_required(initial_state, nstate)
  initial_time = _tile_if_required(initial_time, nstate)
  initial_warmstart = _tile_if_required(initial_warmstart, nstate)
  ctrl = _tile_if_required(ctrl, nstate, nstep)
  qfrc_applied = _tile_if_required(qfrc_applied, nstate, nstep)
  xfrc_applied = _tile_if_required(xfrc_applied, nstate, nstep)
  mocap = _tile_if_required(mocap, nstate, nstep)

  # allocate output if not provided
  if state is None:
    state = np.empty((nstate, nstep, model.nq + model.nv + model.na))
  if sensordata is None:
    sensordata = np.empty((nstate, nstep, model.nsensordata))

  # call rollout
  _rollout.rollout(model, data, nstate, nstep, initial_state, initial_time,
                   initial_warmstart, ctrl, qfrc_applied, xfrc_applied, mocap,
                   state, sensordata)

  # return squeezed outputs
  return state.squeeze(), sensordata.squeeze()

def _check_must_be_numeric(**kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if not isinstance(value, np.ndarray) and not isinstance(value, float):
      raise ValueError(f'{key} must be a numpy array or float')

def _check_number_of_dimensions(ndim, **kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if value.ndim > ndim:
      raise ValueError(f'{key} can have at most {ndim} dimensions')

def _check_trailing_dimension(dim, **kwargs):
  for key, value in kwargs.items():
    if value is None:
      continue
    if value.shape[-1] != dim:
      raise ValueError(f'trailing dimension of {key} must be {dim}, got {value.shape[-1]}')

def _ensure_2d(arg):
  if arg is None:
    return None
  else:
    return np.ascontiguousarray(np.atleast_2d(arg), dtype=np.float64)

def _ensure_3d(arg):
  if arg is None:
    return None
  else:
    # np.atleast_3d adds both leading and trailing dims, we want only leading
    if arg.ndim == 0:
      arg = arg[np.newaxis, np.newaxis, np.newaxis, ...]
    elif arg.ndim == 1:
      arg = arg[np.newaxis, np.newaxis, ...]
    elif arg.ndim == 2:
      arg = arg[np.newaxis, ...]
    return np.ascontiguousarray(arg, dtype=np.float64)

def _infer_dimension(dim, value, **kwargs):
  for name, array in kwargs.items():
    if array is None:
      continue
    if array.shape[dim] != value:
      if value == 1:
        value = array.shape[dim]
      elif array.shape[dim] != 1:
        raise ValueError(
            f'dimension {dim} inferred as {value} but {name} has {array.shape[dim]}'
        )
  return value

def _tile_if_required(array, dim0, dim1=None):
  if array is None:
    return
  reps = np.ones(array.ndim, dtype=int)
  if array.shape[0] == 1:
    reps[0] = dim0
  if dim1 is not None and array.shape[1] == 1:
    reps[1] = dim1
  return np.tile(array, reps)
