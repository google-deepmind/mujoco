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
from mujoco.mjx._src import support
from mujoco.mjx._src import types
import numpy as np
import scipy


def _make_option(o: mujoco.MjOption) -> types.Option:
  """Returns mjx.Option given mujoco.MjOption."""
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

  fields = {f.name: getattr(o, f.name, None) for f in types.Option.fields()}
  fields['integrator'] = types.IntegratorType(o.integrator)
  fields['cone'] = types.ConeType(o.cone)
  fields['jacobian'] = types.JacobianType(o.jacobian)
  fields['solver'] = types.SolverType(o.solver)
  fields['disableflags'] = types.DisableBit(o.disableflags)
  fields['has_fluid_params'] = o.density > 0 or o.viscosity > 0 or o.wind.any()

  return types.Option(**fields)


def _make_statistic(s: mujoco.MjStatistic) -> types.Statistic:
  """Puts mujoco.MjStatistic onto a device, resulting in mjx.Statistic."""
  return types.Statistic(meaninertia=s.meaninertia)


def put_model(m: mujoco.MjModel, device=None) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""

  if m.ntendon:
    raise NotImplementedError('tendons are not supported')

  for g1, g2, ip in collision_driver.geom_pairs(m):
    t1, t2 = m.geom_type[[g1, g2]]
    # check collision function exists for type pair
    if not collision_driver.has_collision_fn(t1, t2):
      t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
      raise NotImplementedError(f'({t1}, {t2}) collisions not implemented.')
    # margin/gap not supported for geoms
    if mujoco.mjtGeom.mjGEOM_MESH in (t1, t2):
      if ip != -1:
        margin = m.pair_margin[ip]
      else:
        margin = m.geom_margin[g1] + m.geom_margin[g2]
      if margin.any():
        t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
        raise NotImplementedError(f'({t1}, {t2}) margin/gap not implemented.')

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

  fields = {f.name: getattr(m, f.name) for f in types.Model.fields()}
  fields['geom_rgba'] = fields['geom_rgba'].reshape((-1, 4))
  fields['mat_rgba'] = fields['mat_rgba'].reshape((-1, 4))
  fields['cam_mat0'] = fields['cam_mat0'].reshape((-1, 3, 3))
  fields['opt'] = _make_option(m.opt)
  fields['stat'] = _make_statistic(m.stat)
  model = types.Model(**{k: copy.copy(v) for k, v in fields.items()})

  return jax.device_put(model, device=device)


def make_data(m: Union[types.Model, mujoco.MjModel]) -> types.Data:
  """Allocate and initialize Data."""
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(efc_type, dim)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc

  zero_0 = jp.zeros(0, dtype=float)
  zero_nv = jp.zeros(m.nv, dtype=float)
  zero_nv_6 = jp.zeros((m.nv, 6), dtype=float)
  zero_nv_nv = jp.zeros((m.nv, m.nv), dtype=float)
  zero_nbody_3 = jp.zeros((m.nbody, 3), dtype=float)
  zero_nbody_6 = jp.zeros((m.nbody, 6), dtype=float)
  zero_nbody_10 = jp.zeros((m.nbody, 10), dtype=float)
  zero_nbody_3_3 = jp.zeros((m.nbody, 3, 3), dtype=float)
  zero_nefc = jp.zeros(nefc, dtype=float)
  zero_na = jp.zeros(m.na, dtype=float)
  zero_nu = jp.zeros(m.nu, dtype=float)
  zero_njnt_3 = jp.zeros((m.njnt, 3), dtype=float)
  zero_nm = jp.zeros(m.nM, dtype=float)

  contact = types.Contact(
      dist=jp.zeros(ncon),
      pos=jp.zeros((ncon, 3)),
      frame=jp.zeros((ncon, 3, 3)),
      includemargin=jp.zeros(ncon),
      friction=jp.zeros((ncon, 5)),
      solref=jp.zeros((ncon, mujoco.mjNREF)),
      solreffriction=jp.zeros((ncon, mujoco.mjNREF)),
      solimp=jp.zeros((ncon, mujoco.mjNIMP)),
      dim=dim,
      geom1=jp.zeros(ncon, dtype=int) - 1,
      geom2=jp.zeros(ncon, dtype=int) - 1,
      geom=jp.zeros((ncon, 2), dtype=int) - 1,
      efc_address=efc_address,
  )

  d = types.Data(
      ne=ne,
      nf=nf,
      nl=nl,
      nefc=nefc,
      ncon=ncon,
      solver_niter=jp.array(0, dtype=int),
      time=jp.array(0.0),
      qpos=jp.array(m.qpos0),
      qvel=zero_nv,
      act=zero_na,
      qacc_warmstart=zero_nv,
      ctrl=zero_nu,
      qfrc_applied=zero_nv,
      xfrc_applied=zero_nbody_6,
      eq_active=jp.zeros(m.neq, dtype=jp.uint8),
      qacc=zero_nv,
      act_dot=zero_na,
      xpos=zero_nbody_3,
      xquat=jp.zeros((m.nbody, 4), dtype=float),
      xmat=zero_nbody_3_3,
      xipos=zero_nbody_3,
      ximat=zero_nbody_3_3,
      xanchor=zero_njnt_3,
      xaxis=zero_njnt_3,
      geom_xpos=jp.zeros((m.ngeom, 3), dtype=float),
      geom_xmat=jp.zeros((m.ngeom, 3, 3), dtype=float),
      site_xpos=jp.zeros((m.nsite, 3), dtype=float),
      site_xmat=jp.zeros((m.nsite, 3, 3), dtype=float),
      cam_xpos=jp.zeros((m.ncam, 3), dtype=float),
      cam_xmat=jp.zeros((m.ncam, 3, 3), dtype=float),
      subtree_com=zero_nbody_3,
      cdof=zero_nv_6,
      cinert=zero_nbody_10,
      actuator_length=zero_nu,
      actuator_moment=jp.zeros((m.nu, m.nv), dtype=float),
      crb=zero_nbody_10,
      qM=zero_nm if support.is_sparse(m) else zero_nv_nv,
      qLD=zero_nm if support.is_sparse(m) else zero_nv_nv,
      qLDiagInv=zero_nv if support.is_sparse(m) else zero_0,
      contact=contact,
      efc_type=efc_type,
      efc_J=jp.zeros((nefc, m.nv), dtype=float),
      efc_frictionloss=zero_nefc,
      efc_D=zero_nefc,
      actuator_velocity=zero_nu,
      cvel=zero_nbody_6,
      cdof_dot=zero_nv_6,
      qfrc_bias=zero_nv,
      qfrc_gravcomp=zero_nv,
      qfrc_passive=zero_nv,
      efc_aref=zero_nefc,
      qfrc_actuator=zero_nv,
      qfrc_smooth=zero_nv,
      qacc_smooth=zero_nv,
      qfrc_constraint=zero_nv,
      qfrc_inverse=zero_nv,
      efc_force=zero_nefc,
      userdata=jp.zeros(m.nuserdata, dtype=float),
  )

  return d


def _get_contact(c: mujoco._structs._MjContactList, cx: types.Contact):
  """Converts mjx.Contact to mujoco._structs._MjContactList."""
  con_id = np.nonzero(cx.dist <= 0)[0]
  for field in types.Contact.fields():
    value = getattr(cx, field.name)[con_id]
    if field.name == 'frame':
      value = value.reshape((-1, 9))
    getattr(c, field.name)[:] = value


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

  dof_i, dof_j = [], []
  for i in range(m.nv):
    j = i
    while j > -1:
      dof_i.append(i)
      dof_j.append(j)
      j = m.dof_parentid[j]

  for i in range(batch_size):
    d_i = jax.tree_util.tree_map(lambda x, i=i: x[i], d) if batched else d
    result_i = result[i] if batched else result
    ncon = (d_i.contact.dist <= 0).sum()
    efc_active = (d_i.efc_J != 0).any(axis=1)
    nefc = int(efc_active.sum())
    result_i.nnzJ = nefc * m.nv
    if ncon != result_i.ncon or nefc != result_i.nefc:
      mujoco._functions._realloc_con_efc(result_i, ncon=ncon, nefc=nefc)  # pylint: disable=protected-access
    result_i.efc_J_rownnz[:] = np.repeat(m.nv, nefc)
    result_i.efc_J_rowadr[:] = np.arange(0, nefc * m.nv, m.nv)
    result_i.efc_J_colind[:] = np.tile(np.arange(m.nv), nefc)

    for field in types.Data.fields():
      if field.name == 'contact':
        _get_contact(result_i.contact, d_i.contact)
        # efc_address must be updated because rows were deleted above:
        efc_map = np.cumsum(efc_active) - 1
        result_i.contact.efc_address[:] = efc_map[result_i.contact.efc_address]
        continue

      value = getattr(d_i, field.name)

      if field.name in ('nefc', 'ncon'):
        value = {'nefc': nefc, 'ncon': ncon}[field.name]
      elif field.name.endswith('xmat') or field.name == 'ximat':
        value = value.reshape((-1, 9))
      elif field.name.startswith('efc_'):
        value = value[efc_active]
        if field.name == 'efc_J':
          value = value.reshape(-1)
      elif field.name == 'qM' and not support.is_sparse(m):
        value = value[dof_i, dof_j]
      elif field.name == 'qLD' and not support.is_sparse(m):
        value = value[dof_i, dof_j]
      elif field.name == 'qLDiagInv' and not support.is_sparse(m):
        value = np.ones(m.nv)

      if isinstance(value, np.ndarray) and value.shape:
        getattr(result_i, field.name)[:] = value
      else:
        setattr(result_i, field.name, value)


def _make_contact(
    c: mujoco._structs._MjContactList,
    dim: np.ndarray,
    efc_address: np.ndarray,
) -> types.Contact:
  """Converts mujoco.structs._MjContactList into mjx.Contact."""
  fields = {f.name: getattr(c, f.name) for f in types.Contact.fields()}
  fields['frame'] = fields['frame'].reshape((-1, 3, 3))
  # reorder contacts so that their condims match those specified in dim.
  # if we have fewer Contacts for a condim range, pad the range with zeros

  # build a map for where to find a dim-matching contact, or -1 if none
  contact_map = np.zeros_like(dim) - 1
  for i, di in enumerate(fields['dim']):
    space = [j for j, dj in enumerate(dim) if di == dj and contact_map[j] == -1]
    if not space:
      # this can happen if max_geom_pairs or max_contact_points is too low
      raise ValueError(f'unable to place Contact[{i}], no space in condim {di}')
    contact_map[space[0]] = i

  if contact_map.size > 0:
    # reorganize contact according, with a zero contact at the end for -1
    zero = jax.tree_util.tree_map(
        lambda x: np.zeros((1,) + x.shape[1:], dtype=x.dtype), fields
    )
    zero['dist'][:] = np.finfo(float).max
    fields = jax.tree_util.tree_map(lambda *x: np.concatenate(x), fields, zero)
    fields = jax.tree_util.tree_map(lambda x: x[contact_map], fields)

  fields['dim'] = dim
  fields['efc_address'] = efc_address

  return types.Contact(**fields)


def put_data(m: mujoco.MjModel, d: mujoco.MjData, device=None) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data."""
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(efc_type, dim)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc

  for d_val, val, name in (
      (d.ncon, ncon, 'ncon'),
      (d.ne, ne, 'ne'),
      (d.nf, nf, 'nf'),
      (d.nl, nl, 'nl'),
      (d.nefc, nefc, 'nefc'),
  ):
    if d_val > val:
      raise ValueError(f'd.{name} too high, d.{name} = {d_val}, model = {val}')

  fields = {f.name: getattr(d, f.name) for f in types.Data.fields()}

  # MJX prefers square matrices for these fields:
  for fname in ('xmat', 'ximat', 'geom_xmat', 'site_xmat', 'cam_xmat'):
    fields[fname] = fields[fname].reshape((-1, 3, 3))

  # MJX does not support islanding, so only transfer the first solver_niter
  fields['solver_niter'] = fields['solver_niter'][0]

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
      value[value_beg : value_beg + size] = fields[fname][d_beg : d_beg + size]
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

  fields['contact'] = _make_contact(d.contact, dim, efc_address)
  fields.update(ne=ne, nf=nf, nl=nl, nefc=nefc, ncon=ncon, efc_type=efc_type)

  # copy because device_put is async:
  data = types.Data(**{k: copy.copy(v) for k, v in fields.items()})

  return jax.device_put(data, device=device)
