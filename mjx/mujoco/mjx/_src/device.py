# Copyright 2023 DeepMind Technologies Limited
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
"""Get and put mujoco data on/off device."""

import copy
import dataclasses
from typing import Any, Dict, Iterable, List, Union, overload
import warnings

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import mesh
from mujoco.mjx._src import types
import numpy as np

_MJ_TYPE_ATTR = {
    mujoco.mjtBias: (mujoco.MjModel.actuator_biastype,),
    mujoco.mjtDyn: (mujoco.MjModel.actuator_dyntype,),
    mujoco.mjtEq: (mujoco.MjModel.eq_type,),
    mujoco.mjtGain: (mujoco.MjModel.actuator_gaintype,),
    mujoco.mjtTrn: (mujoco.MjModel.actuator_trntype,),
    mujoco.mjtCone: (
        mujoco.MjModel.opt,
        mujoco.MjOption.cone,
    ),
    mujoco.mjtIntegrator: (
        mujoco.MjModel.opt,
        mujoco.MjOption.integrator,
    ),
    mujoco.mjtSolver: (
        mujoco.MjModel.opt,
        mujoco.MjOption.solver,
    ),
}

_TYPE_MAP = {
    mujoco._structs._MjContactList: types.Contact,  # pylint: disable=protected-access
    mujoco.MjData: types.Data,
    mujoco.MjModel: types.Model,
    mujoco.MjOption: types.Option,
    mujoco.MjStatistic: types.Statistic,
    mujoco.mjtBias: types.BiasType,
    mujoco.mjtCone: types.ConeType,
    mujoco.mjtDisableBit: types.DisableBit,
    mujoco.mjtDyn: types.DynType,
    mujoco.mjtEq: types.EqType,
    mujoco.mjtGain: types.GainType,
    mujoco.mjtIntegrator: types.IntegratorType,
    mujoco.mjtSolver: types.SolverType,
    mujoco.mjtTrn: types.TrnType,
}

_TRANSFORMS = {
    (types.Data, 'ximat'): lambda x: x.reshape(x.shape[:-1] + (3, 3)),
    (types.Data, 'xmat'): lambda x: x.reshape(x.shape[:-1] + (3, 3)),
    (types.Data, 'geom_xmat'): lambda x: x.reshape(x.shape[:-1] + (3, 3)),
    (types.Data, 'site_xmat'): lambda x: x.reshape(x.shape[:-1] + (3, 3)),
    (types.Contact, 'frame'): (
        lambda x: x.reshape(x.shape[:-1] + (3, 3))  # pylint: disable=g-long-lambda
        if x is not None and x.shape[0] else jp.zeros((0, 3, 3))
    ),
}

_INVERSE_TRANSFORMS = {
    (types.Data, 'ximat'): lambda x: x.reshape(x.shape[:-2] + (9,)),
    (types.Data, 'xmat'): lambda x: x.reshape(x.shape[:-2] + (9,)),
    (types.Data, 'geom_xmat'): lambda x: x.reshape(x.shape[:-2] + (9,)),
    (types.Data, 'site_xmat'): lambda x: x.reshape(x.shape[:-2] + (9,)),
    (types.Contact, 'frame'): (
        lambda x: x.reshape(x.shape[:-2] + (9,))  # pylint: disable=g-long-lambda
        if x is not None and x.shape[0] else jp.zeros((0, 9))
    ),
}

_DERIVED = mesh.DERIVED.union(
    # efc_J is dense in MJX, sparse in MJ. ignore for now.
    {(types.Data, 'efc_J'), (types.Option, 'has_fluid_params')}
)


def _model_derived(value: mujoco.MjModel) -> Dict[str, Any]:
  return {k: jax.device_put(v) for k, v in mesh.get(value).items()}


def _data_derived(value: mujoco.MjData) -> Dict[str, Any]:
  return {'efc_J': jax.device_put(value.efc_J)}


def _option_derived(value: types.Option) -> Dict[str, Any]:
  has_fluid = (
      value.density > 0 or value.viscosity > 0 or (value.wind != 0.0).any()
  )
  return {'has_fluid_params': has_fluid}


def _validate(m: mujoco.MjModel):
  """Validates that an mjModel is compatible with MJX."""

  # check enum types
  for mj_type, attrs in _MJ_TYPE_ATTR.items():
    val = m
    for attr in attrs:
      val = attr.fget(val)  # pytype: disable=attribute-error

    typs = set(val) if isinstance(val, Iterable) else {val}
    unsupported_typs = typs - set(_TYPE_MAP[mj_type])
    unsupported = [mj_type(t) for t in unsupported_typs]  # pylint: disable=too-many-function-args
    if unsupported:
      raise NotImplementedError(f'{unsupported} not implemented.')

  # check condim
  if any(dim != 3 for dim in m.geom_condim) or any(
      dim != 3 for dim in m.pair_dim
  ):
    raise NotImplementedError('Only condim=3 is supported.')

  if m.ntendon:
    raise NotImplementedError('Tendons are not supported.')

  # check collision geom types
  candidate_set = collision_driver.collision_candidates(m)
  for g1, g2, *_ in candidate_set:
    g1, g2 = mujoco.mjtGeom(g1), mujoco.mjtGeom(g2)
    if g1 == mujoco.mjtGeom.mjGEOM_PLANE and g2 in (
        mujoco.mjtGeom.mjGEOM_PLANE,
        mujoco.mjtGeom.mjGEOM_HFIELD,
    ):
      # MuJoCo does not collide planes with other planes or hfields
      continue
    if collision_driver.get_collision_fn((g1, g2)) is None:
      raise NotImplementedError(f'({g1}, {g2}) collisions not implemented.')

  # TODO(erikfrey): warn for high solver iterations, nefc, etc.

  # mjNDISABLE is not a DisableBit flag, so must be explicitly ignored
  disablebit_members = set(mujoco.mjtDisableBit.__members__.values()) - {
      mujoco.mjtDisableBit.mjNDISABLE}
  unsupported_disable = disablebit_members - {
      mujoco.mjtDisableBit(t.value) for t in types.DisableBit
  }
  for f in unsupported_disable:
    if f & m.opt.disableflags:
      warnings.warn(f'Ignoring disable flag {f.name}.')

  # mjNENABLE is not an EnableBit flag, so must be explicitly ignored
  unsupported_enable = set(mujoco.mjtEnableBit.__members__.values()) - {
      mujoco.mjtEnableBit.mjNENABLE
  }
  for f in unsupported_enable:
    if f & m.opt.enableflags:
      warnings.warn(f'Ignoring enable flag {f.name}.')

  if not np.allclose(m.dof_frictionloss, 0):
    raise NotImplementedError('dof_frictionloss is not implemented.')


@overload
def device_put(value: mujoco.MjData) -> types.Data:
  ...


@overload
def device_put(value: mujoco.MjModel) -> types.Model:
  ...


def device_put(value):
  """Places mujoco data onto a device.

  Args:
    value: a mujoco struct to transfer

  Returns:
    on-device MJX struct reflecting the input value
  """
  warnings.warn(
      'device_put is deprecated, use put_model and put_data instead',
      category=DeprecationWarning,
  )

  clz = _TYPE_MAP.get(type(value))
  if clz is None:
    raise NotImplementedError(f'{type(value)} is not supported for device_put.')

  if isinstance(value, mujoco.MjModel):
    _validate(value)  # type: ignore

  init_kwargs = {}
  for f in dataclasses.fields(clz):  # type: ignore
    if (clz, f.name) in _DERIVED:
      continue

    field_value = getattr(value, f.name)
    if (clz, f.name) in _TRANSFORMS:
      field_value = _TRANSFORMS[(clz, f.name)](field_value)

    if f.type is jax.Array:
      field_value = jax.device_put(field_value)
    elif type(field_value) in _TYPE_MAP.keys():
      field_value = device_put(field_value)

    init_kwargs[f.name] = copy.copy(field_value)

  derived_kwargs = {}
  if isinstance(value, mujoco.MjModel):
    derived_kwargs = _model_derived(value)
  elif isinstance(value, mujoco.MjData):
    derived_kwargs = _data_derived(value)
  elif isinstance(value, mujoco.MjOption):
    derived_kwargs = _option_derived(value)

  return clz(**init_kwargs, **derived_kwargs)  # type: ignore


@overload
def device_get_into(
    result: Union[mujoco.MjData, List[mujoco.MjData]], value: types.Data
):
  ...


def device_get_into(result, value):
  """Transfers data off device into a mujoco MjData.

  Data on device often has a batch dimension which adds (N,) to the beginning
  of each array shape where N = batch size.

  If result is a single MjData, arrays are copied over with the batch dimension
  intact. If result is a list, the list must be length N and will be populated
  with distinct MjData structs where the batch dimension is stripped.

  Args:
    result: struct (or list of structs) to transfer into
    value: device value to transfer

  Raises:
    RuntimeError: if result length doesn't match data batch size
  """
  warnings.warn(
      'device_get_into is deprecated, use get_data instead',
      category=DeprecationWarning,
  )

  value = jax.device_get(value)

  if isinstance(result, list):
    array_shapes = [s.shape for s in jax.tree_util.tree_flatten(value)[0]]

    if any(len(s) < 1 or s[0] != array_shapes[0][0] for s in array_shapes):
      raise ValueError('unrecognizable batch dimension in value')

    batch_size = array_shapes[0][0]

    if len(result) != batch_size:
      raise ValueError(
          f"result length ({len(result)}) doesn't match value batch size"
          f' ({batch_size})'
      )

    for i in range(batch_size):
      value_i = jax.tree_map(lambda x, i=i: x[i], value)
      device_get_into(result[i], value_i)

  else:
    if isinstance(result, mujoco.MjData):
      ncon = value.contact.dist.shape[0]
      nefc = value.efc_J.shape[0]
      mujoco._functions._realloc_con_efc(  # pylint: disable=protected-access
          result, ncon=ncon, nefc=nefc
      )
      result.ncon = ncon
      result.nefc = nefc
      efc_start = nefc - ncon * 4
      result.contact.efc_address[:] = np.arange(efc_start, nefc, 4)
      result.contact.dim[:] = 3

    for f in dataclasses.fields(value):  # type: ignore
      if (type(value), f.name) in _DERIVED:
        continue

      field_value = getattr(value, f.name)

      if (type(value), f.name) in _INVERSE_TRANSFORMS:
        field_value = _INVERSE_TRANSFORMS[(type(value), f.name)](field_value)

      if type(field_value) in _TYPE_MAP.values():
        device_get_into(getattr(result, f.name), field_value)
        continue

      try:
        setattr(result, f.name, field_value)
      except AttributeError:
        getattr(result, f.name)[:] = field_value
