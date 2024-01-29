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
"""Functions to initialize, load, or save data."""

import copy
from typing import List, Union

import jax
from jax import numpy as jp
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import constraint
from mujoco.mjx._src import mesh
from mujoco.mjx._src import support
from mujoco.mjx._src import types
import numpy as np
import scipy


def _put_option(o: mujoco.MjOption, device=None) -> types.Option:
  """Puts mujoco.MjOption onto a device, resulting in mjx.Option."""
  if o.integrator not in set(types.IntegratorType):
    raise NotImplementedError(f'{mujoco.mjtIntegrator(o.integrator)}')

  if o.cone not in set(types.ConeType):
    raise NotImplementedError(f'{mujoco.mjtCone(o.cone)}')

  if o.jacobian not in set(types.JacobianType):
    raise NotImplementedError(f'{mujoco.mjtJacobian(o.jacobian)}')

  if o.solver not in set(types.SolverType):
    raise NotImplementedError(f'{mujoco.mjtSolver(o.solver)}')

  for i in range(mujoco.mjtEnableBit.mjNENABLE):
    if o.enableflags & 2**i:
      raise NotImplementedError(f'{mujoco.mjtEnableBit(2 ** i)}')

  static_fields = {
      f.name: copy.copy(getattr(o, f.name))
      for f in types.Option.fields()
      if f.type in (int, bytes, np.ndarray)
  }
  static_fields['integrator'] = types.IntegratorType(o.integrator)
  static_fields['cone'] = types.ConeType(o.cone)
  static_fields['jacobian'] = types.JacobianType(o.jacobian)
  static_fields['solver'] = types.SolverType(o.solver)
  static_fields['disableflags'] = types.DisableBit(o.disableflags)

  device_fields = {
      f.name: copy.copy(getattr(o, f.name))
      for f in types.Option.fields()
      if f.type is jax.Array
  }
  device_fields = jax.device_put(device_fields, device=device)

  has_fluid_params = o.density > 0 or o.viscosity > 0 or o.wind.any()

  return types.Option(
      has_fluid_params=has_fluid_params,
      **static_fields,
      **device_fields,
  )


def _put_statistic(s: mujoco.MjStatistic, device=None) -> types.Statistic:
  """Puts mujoco.MjStatistic onto a device, resulting in mjx.Statistic."""
  return types.Statistic(
      meaninertia=jax.device_put(s.meaninertia, device=device)
  )


def put_model(m: mujoco.MjModel, device=None) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""

  if m.ntendon:
    raise NotImplementedError('tendons are not supported')

  if (m.geom_condim != 3).any() or (m.pair_dim != 3).any():
    raise NotImplementedError('only condim=3 is supported')

  # check collision geom types
  for g1, g2, *_ in collision_driver.collision_candidates(m):
    if collision_driver.get_collision_fn((g1, g2)) is None:
      g1, g2 = mujoco.mjtGeom(g1), mujoco.mjtGeom(g2)
      raise NotImplementedError(f'({g1}, {g2}) has no collision function')

  for enum_field, enum_type, mj_type in (
      (m.actuator_biastype, types.BiasType, mujoco.mjtBias),
      (m.actuator_dyntype, types.DynType, mujoco.mjtDyn),
      (m.actuator_gaintype, types.GainType, mujoco.mjtGain),
      (m.actuator_trntype, types.TrnType, mujoco.mjtTrn),
      (m.eq_type, types.EqType, mujoco.mjtEq),
  ):
    missing = set(enum_field) - set(enum_type)
    if missing:
      raise NotImplementedError(
          f'{[mj_type(m) for m in missing]} not supported'
      )

  if not np.allclose(m.dof_frictionloss, 0):
    raise NotImplementedError('dof_frictionloss is not implemented.')

  opt = _put_option(m.opt, device=device)
  stat = _put_statistic(m.stat, device=device)

  static_fields = {
      f.name: getattr(m, f.name)
      for f in types.Model.fields()
      if f.type in (int, bytes, np.ndarray)
  }
  static_fields['geom_rgba'] = static_fields['geom_rgba'].reshape((-1, 4))
  static_fields['mat_rgba'] = static_fields['mat_rgba'].reshape((-1, 4))

  device_fields = {
      f.name: copy.copy(getattr(m, f.name))  # copy because device_put is async
      for f in types.Model.fields()
      if f.type is jax.Array
  }
  device_fields.update(mesh.get(m))
  device_fields = jax.device_put(device_fields, device=device)

  return types.Model(
      opt=opt,
      stat=stat,
      **static_fields,
      **device_fields,
  )


def make_data(m: Union[types.Model, mujoco.MjModel]) -> types.Data:
  """Allocate and initialize Data."""

  ncon = collision_driver.ncon(m)
  ne, nf, nl, nc = constraint.count_constraints(m)
  nefc = ne + nf + nl + nc

  zero_0 = jp.zeros(0, dtype=jp.float32)
  zero_nv = jp.zeros(m.nv, dtype=jp.float32)
  zero_nv_6 = jp.zeros((m.nv, 6), dtype=jp.float32)
  zero_nv_nv = jp.zeros((m.nv, m.nv), dtype=jp.float32)
  zero_nbody_3 = jp.zeros((m.nbody, 3), dtype=jp.float32)
  zero_nbody_6 = jp.zeros((m.nbody, 6), dtype=jp.float32)
  zero_nbody_10 = jp.zeros((m.nbody, 10), dtype=jp.float32)
  zero_nbody_3_3 = jp.zeros((m.nbody, 3, 3), dtype=jp.float32)
  zero_nefc = jp.zeros(nefc, dtype=jp.float32)
  zero_na = jp.zeros(m.na, dtype=jp.float32)
  zero_nu = jp.zeros(m.nu, dtype=jp.float32)
  zero_njnt_3 = jp.zeros((m.njnt, 3), dtype=jp.float32)
  zero_nm = jp.zeros(m.nM, dtype=jp.float32)

  # create first d to get num contacts and nc
  d = types.Data(
      solver_niter=jp.array(0, dtype=jp.int32),
      time=jp.array(0.0),
      qpos=jp.array(m.qpos0),
      qvel=zero_nv,
      act=zero_na,
      qacc_warmstart=zero_nv,
      ctrl=zero_nu,
      qfrc_applied=zero_nv,
      xfrc_applied=zero_nbody_6,
      eq_active=jp.zeros(m.neq, dtype=jp.int32),
      qacc=zero_nv,
      act_dot=zero_na,
      xpos=zero_nbody_3,
      xquat=jp.zeros((m.nbody, 4), dtype=jp.float32),
      xmat=zero_nbody_3_3,
      xipos=zero_nbody_3,
      ximat=zero_nbody_3_3,
      xanchor=zero_njnt_3,
      xaxis=zero_njnt_3,
      geom_xpos=jp.zeros((m.ngeom, 3), dtype=jp.float32),
      geom_xmat=jp.zeros((m.ngeom, 3, 3), dtype=jp.float32),
      site_xpos=jp.zeros((m.nsite, 3), dtype=jp.float32),
      site_xmat=jp.zeros((m.nsite, 3, 3), dtype=jp.float32),
      subtree_com=zero_nbody_3,
      cdof=zero_nv_6,
      cinert=zero_nbody_10,
      actuator_length=zero_nu,
      actuator_moment=jp.zeros((m.nu, m.nv), dtype=jp.float32),
      crb=zero_nbody_10,
      qM=zero_nm if support.is_sparse(m) else zero_nv_nv,
      qLD=zero_nm if support.is_sparse(m) else zero_nv_nv,
      qLDiagInv=zero_nv if support.is_sparse(m) else zero_0,
      contact=types.Contact.zero(ncon),
      efc_J=jp.zeros((nefc, m.nv), dtype=jp.float32),
      efc_frictionloss=zero_nefc,
      efc_D=zero_nefc,
      actuator_velocity=zero_nu,
      cvel=zero_nbody_6,
      cdof_dot=zero_nv_6,
      qfrc_bias=zero_nv,
      qfrc_passive=zero_nv,
      efc_aref=zero_nefc,
      qfrc_actuator=zero_nv,
      qfrc_smooth=zero_nv,
      qacc_smooth=zero_nv,
      qfrc_constraint=zero_nv,
      qfrc_inverse=zero_nv,
      efc_force=zero_nefc,
  )

  return d


def _get_contact(
    c: mujoco._structs._MjContactList,
    cx: types.Contact,
    efc_start: int,
):
  """Converts mjx.Contact to mujoco._structs._MjContactList."""
  con_id = np.nonzero(cx.dist <= 0)[0]
  for field in types.Contact.fields():
    value = getattr(cx, field.name)[con_id]
    if field.name == 'frame':
      value = value.reshape((-1, 9))
    getattr(c, field.name)[:] = value

  ncon = cx.dist.shape[0]
  c.efc_address[:] = np.arange(efc_start, efc_start + ncon * 4, 4)[con_id]


def get_data(
    m: mujoco.MjModel, d: types.Data
) -> Union[mujoco.MjData, List[mujoco.MjData]]:
  """Gets mjx.Data from a device, resulting in mujoco.MjData or List[MjData]."""
  batched = len(d.qpos.shape) > 1
  batch_size = d.qpos.shape[0] if batched else 1

  if batched:
    result = [mujoco.MjData(m) for _ in range(batch_size)]
  else:
    result = mujoco.MjData(m)

  get_data_into(result, m, d)

  return result


def get_data_into(
    result: Union[mujoco.MjData, List[mujoco.MjData]],
    m: mujoco.MjModel,
    d: types.Data,
):
  """Gets mjx.Data from a device into an existing mujoco.MjData or list."""
  batched = isinstance(result, list)
  if batched and len(d.qpos.shape) < 2:
    raise ValueError('dst is a list, but d is not batched.')
  if not batched and len(d.qpos.shape) >= 2:
    raise ValueError('dst is a an MjData, but d is batched.')

  d = jax.device_get(d)

  batch_size = d.qpos.shape[0] if batched else 1
  ne, nf, nl, nc = constraint.count_constraints(m)
  efc_type = np.array([
      mujoco.mjtConstraint.mjCNSTR_EQUALITY,
      mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF,
      mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT,
      mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL,
  ]).repeat([ne, nf, nl, nc])

  dof_i, dof_j = [], []
  for i in range(m.nv):
    j = i
    while j > -1:
      dof_i.append(i)
      dof_j.append(j)
      j = m.dof_parentid[j]

  for i in range(batch_size):
    d_i = jax.tree_map(lambda x, i=i: x[i], d) if batched else d
    result_i = result[i] if batched else result
    ncon = (d_i.contact.dist <= 0).sum()
    efc_active = (d_i.efc_J != 0).any(axis=1)
    efc_con = efc_type == mujoco.mjtConstraint.mjCNSTR_CONTACT_PYRAMIDAL
    nefc, nc = efc_active.sum(), (efc_active & efc_con).sum()
    result_i.nnzJ = nefc * m.nv
    mujoco._functions._realloc_con_efc(result_i, ncon=ncon, nefc=nefc)  # pylint: disable=protected-access
    result_i.efc_J_rownnz[:] = np.repeat(m.nv, nefc)
    result_i.efc_J_rowadr[:] = np.arange(0, nefc * m.nv, m.nv)
    result_i.efc_J_colind[:] = np.tile(np.arange(m.nv), nefc)

    for field in types.Data.fields():
      if field.name == 'contact':
        _get_contact(result_i.contact, d_i.contact, nefc - nc)
        continue

      value = getattr(d_i, field.name)

      if field.name in ('xmat', 'ximat', 'geom_xmat', 'site_xmat'):
        value = value.reshape((-1, 9))

      if field.name in ('efc_frictionloss', 'efc_D', 'efc_aref', 'efc_force'):
        value = value[efc_active]

      if field.name == 'efc_J':
        value = value[efc_active].reshape(-1)

      if field.name == 'qM' and not support.is_sparse(m):
        value = value[dof_i, dof_j]

      if field.name == 'qLD' and not support.is_sparse(m):
        value = value[dof_i, dof_j]

      if field.name == 'qLDiagInv' and not support.is_sparse(m):
        value = np.ones(m.nv)

      if value.shape:
        getattr(result_i, field.name)[:] = value
      else:
        setattr(result_i, field.name, value)

    result_i.efc_type[:] = efc_type[efc_active]


def _put_contact(
    c: mujoco._structs._MjContactList, ncon: int, device=None
) -> types.Contact:
  """Puts mujoco.structs._MjContactList onto a device, resulting in mjx.Contact."""
  fields = {
      f.name: copy.copy(getattr(c, f.name)) for f in types.Contact.fields()
  }
  fields['frame'] = fields['frame'].reshape((-1, 3, 3))
  pad_size = ncon - c.dist.shape[0]
  pad_fn = lambda x: np.concatenate(
      (x, np.zeros((pad_size,) + x.shape[1:], dtype=x.dtype))
  )
  fields = jax.tree_map(pad_fn, fields)
  fields['dist'][-pad_size:] = np.inf
  fields = jax.device_put(fields, device=device)

  return types.Contact(**fields)


def put_data(m: mujoco.MjModel, d: mujoco.MjData, device=None) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data."""
  ncon = collision_driver.ncon(m)
  ne, nf, nl, nc = constraint.count_constraints(m)
  nefc = ne + nf + nl + nc

  for d_val, val, name in (
      (d.ncon, ncon, 'ncon'),
      (d.ne, ne, 'ne'),
      (d.nf, nf, 'nf'),
      (d.nl, nl, 'nl'),
      (d.nefc, nefc, 'nefc'),
  ):
    if d_val > val:
      raise ValueError(f'd.{name} too high, d.{name} = {d_val}, model = {val}')

  fields = {
      f.name: copy.copy(getattr(d, f.name))  # copy because device_put is async
      for f in types.Data.fields()
      if f.type is jax.Array
  }

  for fname in ('xmat', 'ximat', 'geom_xmat', 'site_xmat'):
    fields[fname] = fields[fname].reshape((-1, 3, 3))

  # pad efc fields: MuJoCo efc arrays are sparse for inactive constraints.
  # efc_J is also optionally column-sparse (typically for large nv).  MJX is
  # neither: it contains zeros for inactive constraints, and efc_J is always
  # (nefc, nv).  this may change in the future.
  if mujoco.mj_isSparse(m):
    nr = d.efc_J_rownnz.shape[0]
    efc_j = np.zeros((nr, m.nv))
    for i in range(nr):
      rowadr = d.efc_J_rowadr[i]
      for j in range(d.efc_J_rownnz[i]):
        efc_j[i, d.efc_J_colind[rowadr + j]] = fields['efc_J'][rowadr + j]
    fields['efc_J'] = efc_j
  else:
    fields['efc_J'] = fields['efc_J'].reshape((-1 if m.nv else 0, m.nv))

  for fname in ('efc_J', 'efc_frictionloss', 'efc_D', 'efc_aref', 'efc_force'):
    value = np.zeros((nefc, m.nv)) if fname == 'efc_J' else np.zeros(nefc)
    for i in range(4):
      value_beg = sum([ne, nf, nl][:i])
      d_beg = sum([d.ne, d.nf, d.nl][:i])
      size = [d.ne, d.nf, d.nl, d.nefc - d.nl - d.nf - d.ne][i]
      value[value_beg:value_beg+size] = fields[fname][d_beg:d_beg+size]
    fields[fname] = value

  # convert qM and qLD if jacobian is dense
  if not support.is_sparse(m):
    fields['qM'] = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, fields['qM'], d.qM)
    # TODO(erikfrey): derive L*L' from L'*D*L instead of recomputing
    try:
      fields['qLD'], _ = scipy.linalg.cho_factor(fields['qM'])
    except scipy.linalg.LinAlgError:
      # this happens when qM is empty or unstable simulation
      fields['qLD'] = np.zeros((m.nv, m.nv))
    fields['qLDiagInv'] = np.zeros(0)

  fields = jax.device_put(fields, device=device)
  fields['contact'] = _put_contact(d.contact, ncon, device=device)

  return types.Data(**fields)
