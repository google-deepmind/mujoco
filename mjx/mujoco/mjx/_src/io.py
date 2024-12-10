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
from typing import List, Tuple, Union

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


def _strip_weak_type(tree):
  def f(leaf):
    if isinstance(leaf, jax.Array):
      return leaf.astype(jax.dtypes.canonicalize_dtype(leaf.dtype))
    return leaf

  return jax.tree_util.tree_map(f, tree)


def _make_option(
    o: mujoco.MjOption, _full_compat: bool = False
) -> types.Option:
  """Returns mjx.Option given mujoco.MjOption."""
  if not _full_compat:
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

  has_fluid_params = o.density > 0 or o.viscosity > 0 or o.wind.any()
  implicitfast = o.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
  if not _full_compat:
    if implicitfast and has_fluid_params:
      raise NotImplementedError('implicitfast not implemented for fluid drag.')

  fields = {f.name: getattr(o, f.name, None) for f in types.Option.fields()}
  fields['integrator'] = types.IntegratorType(o.integrator)
  fields['cone'] = types.ConeType(o.cone)
  fields['jacobian'] = types.JacobianType(o.jacobian)
  fields['solver'] = types.SolverType(o.solver)
  fields['disableflags'] = types.DisableBit(o.disableflags)
  fields['has_fluid_params'] = has_fluid_params

  return types.Option(**fields)


def _make_statistic(s: mujoco.MjStatistic) -> types.Statistic:
  """Puts mujoco.MjStatistic onto a device, resulting in mjx.Statistic."""
  return types.Statistic(
      meaninertia=s.meaninertia,
      meanmass=s.meanmass,
      meansize=s.meansize,
      extent=s.extent,
      center=s.center,
  )


def put_model(
    m: mujoco.MjModel, device=None, _full_compat: bool = False  # pylint: disable=invalid-name
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model.

  Args:
    m: the model to put onto device
    device: which device to use - if unspecified picks the default device
    _full_compat: put all MjModel fields onto device irrespective of MJX support
      This is an experimental feature.  Avoid using it for now.

  Returns:
    an mjx.Model placed on device
  """

  mesh_geomid = set()
  for g1, g2, ip in collision_driver.geom_pairs(m):
    t1, t2 = m.geom_type[[g1, g2]]
    # check collision function exists for type pair
    if not collision_driver.has_collision_fn(t1, t2) and not _full_compat:
      t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
      raise NotImplementedError(f'({t1}, {t2}) collisions not implemented.')
    # margin/gap not supported for meshes and height fields
    no_margin = {mujoco.mjtGeom.mjGEOM_MESH, mujoco.mjtGeom.mjGEOM_HFIELD}
    if no_margin.intersection({t1, t2}):
      if ip != -1:
        margin = m.pair_margin[ip]
      else:
        margin = m.geom_margin[g1] + m.geom_margin[g2]
      if margin.any() and not _full_compat:
        t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
        raise NotImplementedError(f'({t1}, {t2}) margin/gap not implemented.')
    for t, g in [(t1, g1), (t2, g2)]:
      if t == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_geomid.add(g)

  # check for spatial tendon internal geom wrapping
  if m.ntendon:
    # find sphere or cylinder geoms (if any exist)
    (wrap_id_geom,) = np.nonzero(
        (m.wrap_type == mujoco.mjtWrap.mjWRAP_SPHERE)
        | (m.wrap_type == mujoco.mjtWrap.mjWRAP_CYLINDER)
    )
    wrap_objid_geom = m.wrap_objid[wrap_id_geom]
    geom_pos = m.geom_pos[wrap_objid_geom]
    geom_size = m.geom_size[wrap_objid_geom, 0]

    # find sidesites (if any exist)
    side_id = np.round(m.wrap_prm[wrap_id_geom]).astype(int)
    side = m.site_pos[side_id]

    # check for sidesite inside geom
    if np.any(
        (np.linalg.norm(side - geom_pos, axis=1) < geom_size) & (side_id >= 0)
    ):
      raise NotImplementedError(
          'Internal wrapping with sphere and cylinder geoms is not'
          ' implemented for spatial tendons.'
      )

  # check for unsupported sensor and equality constraint combinations
  sensor_rne_postconstraint = (
      np.any(m.sensor_type == types.SensorType.ACCELEROMETER)
      | np.any(m.sensor_type == types.SensorType.FORCE)
      | np.any(m.sensor_type == types.SensorType.TORQUE)
  )
  eq_connect_weld = np.any(m.eq_type == types.EqType.CONNECT) | np.any(
      m.eq_type == types.EqType.WELD
  )
  if sensor_rne_postconstraint and eq_connect_weld:
    raise NotImplementedError(
        'rne_postconstraint not implemented with equality constraints:'
        ' connect, weld.'
    )

  for enum_field, enum_type, mj_type in (
      (m.actuator_biastype, types.BiasType, mujoco.mjtBias),
      (m.actuator_dyntype, types.DynType, mujoco.mjtDyn),
      (m.actuator_gaintype, types.GainType, mujoco.mjtGain),
      (m.actuator_trntype, types.TrnType, mujoco.mjtTrn),
      (m.eq_type, types.EqType, mujoco.mjtEq),
      (m.sensor_type, types.SensorType, mujoco.mjtSensor),
      (m.wrap_type, types.WrapType, mujoco.mjtWrap),
  ):
    missing = set(enum_field) - set(enum_type)
    if missing and not _full_compat:
      raise NotImplementedError(
          f'{[mj_type(m) for m in missing]} not supported'
      )

  mj_field_names = {
      f.name
      for f in types.Model.fields()
      if f.metadata.get('restricted_to') != 'mjx'
  }
  fields = {f: getattr(m, f) for f in mj_field_names}
  fields['dof_hasfrictionloss'] = fields['dof_frictionloss'] > 0
  fields['tendon_hasfrictionloss'] = fields['tendon_frictionloss'] > 0
  fields['geom_rbound_hfield'] = fields['geom_rbound']
  fields['cam_mat0'] = fields['cam_mat0'].reshape((-1, 3, 3))
  fields['opt'] = _make_option(m.opt, _full_compat=_full_compat)
  fields['stat'] = _make_statistic(m.stat)

  # Pre-compile meshes for MJX collisions.
  fields['mesh_convex'] = [None] * m.nmesh
  if not _full_compat:
    for i in mesh_geomid:
      dataid = m.geom_dataid[i]
      if fields['mesh_convex'][dataid] is None:
        fields['mesh_convex'][dataid] = mesh.convex(m, dataid)  # pytype: disable=unsupported-operands
    fields['mesh_convex'] = tuple(fields['mesh_convex'])

  model = types.Model(**{k: copy.copy(v) for k, v in fields.items()})

  model = jax.device_put(model, device=device)
  return _strip_weak_type(model)


def make_data(
    m: Union[types.Model, mujoco.MjModel],
    device=None,
    _full_compat: bool = False,  # pylint: disable=invalid-name
) -> types.Data:
  """Allocate and initialize Data.

  Args:
    m: the model to use
    device: which device to use - if unspecified picks the default device
    _full_compat: create all MjData fields on device irrespective of MJX support
      This is an experimental feature.  Avoid using it for now. If using this
      flag, also use _full_compat for put_model.

  Returns:
    an initialized mjx.Data placed on device
  """
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(m, dim, efc_type)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc

  with jax.default_device(device):
    contact = types.Contact(
        dist=jp.zeros((ncon,), dtype=float),
        pos=jp.zeros((ncon, 3), dtype=float),
        frame=jp.zeros((ncon, 3, 3), dtype=float),
        includemargin=jp.zeros((ncon,), dtype=float),
        friction=jp.zeros((ncon, 5), dtype=float),
        solref=jp.zeros((ncon, mujoco.mjNREF), dtype=float),
        solreffriction=jp.zeros((ncon, mujoco.mjNREF), dtype=float),
        solimp=jp.zeros((ncon, mujoco.mjNIMP), dtype=float),
        dim=dim,
        # let jax pick contact.geom int precision, for interop with
        # jax_enable_x64
        geom1=jp.full((ncon,), -1, dtype=int),
        geom2=jp.full((ncon,), -1, dtype=int),
        geom=jp.full((ncon, 2), -1, dtype=int),
        efc_address=efc_address,
    )

    if m.opt.cone == types.ConeType.ELLIPTIC and np.any(contact.dim == 1):
      raise NotImplementedError(
          'condim=1 with ConeType.ELLIPTIC not implemented.'
      )

    zero_fields = {
        'solver_niter': (int,),
        'time': (float,),
        'qvel': (m.nv, float),
        'act': (m.na, float),
        'qacc_warmstart': (m.nv, float),
        'ctrl': (m.nu, float),
        'qfrc_applied': (m.nv, float),
        'xfrc_applied': (m.nbody, 6, float),
        'mocap_pos': (m.nmocap, 3, float),
        'mocap_quat': (m.nmocap, 4, float),
        'qacc': (m.nv, float),
        'act_dot': (m.na, float),
        'userdata': (m.nuserdata, float),
        'sensordata': (m.nsensordata, float),
        'xpos': (m.nbody, 3, float),
        'xquat': (m.nbody, 4, float),
        'xmat': (m.nbody, 3, 3, float),
        'xipos': (m.nbody, 3, float),
        'ximat': (m.nbody, 3, 3, float),
        'xanchor': (m.njnt, 3, float),
        'xaxis': (m.njnt, 3, float),
        'geom_xpos': (m.ngeom, 3, float),
        'geom_xmat': (m.ngeom, 3, 3, float),
        'site_xpos': (m.nsite, 3, float),
        'site_xmat': (m.nsite, 3, 3, float),
        'cam_xpos': (m.ncam, 3, float),
        'cam_xmat': (m.ncam, 3, 3, float),
        'light_xpos': (m.nlight, 3, float),
        'light_xdir': (m.nlight, 3, float),
        'subtree_com': (m.nbody, 3, float),
        'cdof': (m.nv, 6, float),
        'cinert': (m.nbody, 10, float),
        'flexvert_xpos': (m.nflexvert, 3, float),
        'flexelem_aabb': (m.nflexelem, 6, float),
        'flexedge_J_rownnz': (m.nflexedge, jp.int32),
        'flexedge_J_rowadr': (m.nflexedge, jp.int32),
        'flexedge_J_colind': (m.nflexedge, m.nv, jp.int32),
        'flexedge_J': (m.nflexedge, m.nv, float),
        'flexedge_length': (m.nflexedge, float),
        'ten_wrapadr': (m.ntendon, jp.int32),
        'ten_wrapnum': (m.ntendon, jp.int32),
        'ten_J_rownnz': (m.ntendon, jp.int32),
        'ten_J_rowadr': (m.ntendon, jp.int32),
        'ten_J_colind': (m.ntendon, m.nv, jp.int32),
        'ten_J': (m.ntendon, m.nv, float),
        'ten_length': (m.ntendon, float),
        'wrap_obj': (m.nwrap, 2, jp.int32),
        'wrap_xpos': (m.nwrap, 6, float),
        'actuator_length': (m.nu, float),
        'moment_rownnz': (m.nu, jp.int32),
        'moment_rowadr': (m.nu, jp.int32),
        'moment_colind': (m.nJmom, jp.int32),
        'actuator_moment': (m.nu, m.nv, float),
        'crb': (m.nbody, 10, float),
        'qM': (m.nM, float) if support.is_sparse(m) else (m.nv, m.nv, float),
        'qLD': (m.nM, float) if support.is_sparse(m) else (m.nv, m.nv, float),
        'qLDiagInv': (m.nv, float) if support.is_sparse(m) else (0, float),
        'qLDiagSqrtInv': (m.nv, float),
        'bvh_aabb_dyn': (m.nbvhdynamic, 6, float),
        'bvh_active': (m.nbvh, jp.uint8),
        'flexedge_velocity': (m.nflexedge, float),
        'ten_velocity': (m.ntendon, float),
        'actuator_velocity': (m.nu, float),
        'cvel': (m.nbody, 6, float),
        'cdof_dot': (m.nv, 6, float),
        'qfrc_bias': (m.nv, float),
        'qfrc_spring': (m.nv, float),
        'qfrc_damper': (m.nv, float),
        'qfrc_gravcomp': (m.nv, float),
        'qfrc_fluid': (m.nv, float),
        'qfrc_passive': (m.nv, float),
        'subtree_linvel': (m.nbody, 3, float),
        'subtree_angmom': (m.nbody, 3, float),
        'qH': (m.nM, float) if support.is_sparse(m) else (m.nv, m.nv, float),
        'qHDiagInv': (m.nv, float),
        'B_rownnz': (m.nbody, jp.int32),
        'B_rowadr': (m.nbody, jp.int32),
        'B_colind': (m.nB, jp.int32),
        'C_rownnz': (m.nv, jp.int32),
        'C_rowadr': (m.nv, jp.int32),
        'C_diag': (m.nv, jp.int32),
        'C_colind': (m.nC, jp.int32),
        'mapM2C': (m.nC, jp.int32),
        'D_rownnz': (m.nv, jp.int32),
        'D_rowadr': (m.nv, jp.int32),
        'D_diag': (m.nv, jp.int32),
        'D_colind': (m.nD, jp.int32),
        'mapM2D': (m.nD, jp.int32),
        'mapD2M': (m.nM, jp.int32),
        'qDeriv': (m.nD, float),
        'qLU': (m.nD, float),
        'actuator_force': (m.nu, float),
        'qfrc_actuator': (m.nv, float),
        'qfrc_smooth': (m.nv, float),
        'qacc_smooth': (m.nv, float),
        'qfrc_constraint': (m.nv, float),
        'qfrc_inverse': (m.nv, float),
        'cacc': (m.nbody, 6, float),
        'cfrc_int': (m.nbody, 6, float),
        'cfrc_ext': (m.nbody, 6, float),
        'efc_J': (nefc, m.nv, float),
        'efc_pos': (nefc, float),
        'efc_margin': (nefc, float),
        'efc_frictionloss': (nefc, float),
        'efc_D': (nefc, float),
        'efc_aref': (nefc, float),
        'efc_force': (nefc, float),
        '_qM_sparse': (m.nM, float),
        '_qLD_sparse': (m.nM, float),
        '_qLDiagInv_sparse': (m.nv, float),
    }

    if not _full_compat:
      for f in types.Data.fields():
        if f.metadata.get('restricted_to') in ('mujoco', 'mjx'):
          zero_fields[f.name] = (0, zero_fields[f.name][-1])

    zero_fields = {
        k: jp.zeros(v[:-1], dtype=v[-1]) for k, v in zero_fields.items()
    }

  d = types.Data(
      ne=ne,
      nf=nf,
      nl=nl,
      nefc=nefc,
      ncon=ncon,
      qpos=jp.array(m.qpos0),
      contact=contact,
      efc_type=efc_type,
      eq_active=m.eq_active0,
      **zero_fields,
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
    result_i.nJ = nefc * m.nv
    if ncon != result_i.ncon or nefc != result_i.nefc:
      mujoco._functions._realloc_con_efc(result_i, ncon=ncon, nefc=nefc)  # pylint: disable=protected-access
    result_i.efc_J_rownnz[:] = np.repeat(m.nv, nefc)
    result_i.efc_J_rowadr[:] = np.arange(0, nefc * m.nv, m.nv)
    result_i.efc_J_colind[:] = np.tile(np.arange(m.nv), nefc)

    for field in types.Data.fields():
      restricted_to = field.metadata.get('restricted_to')
      if restricted_to == 'mjx':
        continue

      if field.name == 'contact':
        _get_contact(result_i.contact, d_i.contact)
        # efc_address must be updated because rows were deleted above:
        efc_map = np.cumsum(efc_active) - 1
        result_i.contact.efc_address[:] = efc_map[result_i.contact.efc_address]
        continue

      # MuJoCo actuator_moment is sparse, MJX uses a dense representation.
      if field.name == 'actuator_moment':
        moment_rownnz = np.zeros(m.nu, dtype=np.int32)
        moment_rowadr = np.zeros(m.nu, dtype=np.int32)
        moment_colind = np.zeros(m.nJmom, dtype=np.int32)
        actuator_moment = np.zeros(m.nJmom)
        if m.nu:
          mujoco.mju_dense2sparse(
              actuator_moment,
              d_i.actuator_moment,
              moment_rownnz,
              moment_rowadr,
              moment_colind,
          )
        result_i.moment_rownnz[:] = moment_rownnz
        result_i.moment_rowadr[:] = moment_rowadr
        result_i.moment_colind[:] = moment_colind
        result_i.actuator_moment[:] = actuator_moment
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
        if restricted_to in ('mujoco', 'mjx'):
          continue  # don't copy fields that are mujoco-only or MJX-only
        else:
          getattr(result_i, field.name)[:] = value
      else:
        setattr(result_i, field.name, value)


def _make_contact(
    c: mujoco._structs._MjContactList,
    dim: np.ndarray,
    efc_address: np.ndarray,
) -> Tuple[types.Contact, np.ndarray]:
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
    zero['dist'][:] = 1e10
    fields = jax.tree_util.tree_map(lambda *x: np.concatenate(x), fields, zero)
    fields = jax.tree_util.tree_map(lambda x: x[contact_map], fields)

  fields['dim'] = dim
  fields['efc_address'] = efc_address

  return types.Contact(**fields), contact_map


def put_data(
    m: mujoco.MjModel, d: mujoco.MjData, device=None, _full_compat: bool = False  # pylint: disable=invalid-name
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data.

  Args:
    m: the model to use
    d: the data to put on device
    device: which device to use - if unspecified picks the default device
    _full_compat: put all MjModel fields onto device irrespective of MJX support
      This is an experimental feature.  Avoid using it for now. If using this
      flag, also use _full_compat for put_model.

  Returns:
    an mjx.Data placed on device
  """
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(m, dim, efc_type)
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

  fields = {
      f.name: getattr(d, f.name)
      for f in types.Data.fields()
      if f.metadata.get('restricted_to') != 'mjx'
  }

  # MJX prefers square matrices for these fields:
  for fname in ('xmat', 'ximat', 'geom_xmat', 'site_xmat', 'cam_xmat'):
    fields[fname] = fields[fname].reshape((-1, 3, 3))

  # MJX does not support islanding, so only transfer the first solver_niter
  fields['solver_niter'] = fields['solver_niter'][0]

  # convert sparse representation of actuator_moment to dense matrix
  moment = np.zeros((m.nu, m.nv))
  mujoco.mju_sparse2dense(
      moment,
      d.actuator_moment,
      d.moment_rownnz,
      d.moment_rowadr,
      d.moment_colind,
  )
  fields['actuator_moment'] = moment

  contact, contact_map = _make_contact(d.contact, dim, efc_address)

  # pad efc fields: MuJoCo efc arrays are sparse for inactive constraints.
  # efc_J is also optionally column-sparse (typically for large nv).  MJX is
  # neither: it contains zeros for inactive constraints, and efc_J is always
  # (nefc, nv).  this may change in the future.
  if mujoco.mj_isSparse(m):
    efc_j = np.zeros((d.efc_J_rownnz.shape[0], m.nv))
    mujoco.mju_sparse2dense(
        efc_j,
        fields['efc_J'],
        d.efc_J_rownnz,
        d.efc_J_rowadr,
        d.efc_J_colind,
    )
    fields['efc_J'] = efc_j
  else:
    fields['efc_J'] = fields['efc_J'].reshape((-1 if m.nv else 0, m.nv))

  # move efc rows to their correct offsets
  for fname in (
      'efc_J',
      'efc_pos',
      'efc_margin',
      'efc_frictionloss',
      'efc_D',
      'efc_aref',
      'efc_force',
  ):
    value = np.zeros((nefc, m.nv)) if fname == 'efc_J' else np.zeros(nefc)
    for i in range(3):
      value_beg = sum([ne, nf][:i])
      d_beg = sum([d.ne, d.nf][:i])
      size = [d.ne, d.nf, d.nl][i]
      value[value_beg : value_beg + size] = fields[fname][d_beg : d_beg + size]

    # for nc, we may reorder contacts so they match MJX order: group by dim
    for id_to, id_from in enumerate(contact_map):
      if id_from == -1:
        continue
      num_rows = dim[id_to]
      if num_rows > 1 and m.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:
        num_rows = (num_rows - 1) * 2
      efc_i, efc_o = d.contact.efc_address[id_from], efc_address[id_to]
      value[efc_o : efc_o + num_rows] = fields[fname][efc_i : efc_i + num_rows]

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

  if _full_compat:
    # full compatibility mode, we store sparse qM regardless of jacobian setting
    fields['_qM_sparse'] = fields['qM']
    fields['_qLD_sparse'] = fields['qLD']
    fields['_qLDiagInv_sparse'] = fields['qLDiagInv']
  else:
    fields['_qM_sparse'] = jp.zeros(0, dtype=float)
    fields['_qLD_sparse'] = jp.zeros(0, dtype=float)
    fields['_qLDiagInv_sparse'] = jp.zeros(0, dtype=float)
    # otherwise clear out unused arrays
    for f in types.Data.fields():
      if f.metadata.get('restricted_to') == 'mujoco':
        fields[f.name] = np.zeros(0, dtype=fields[f.name].dtype)

  fields['contact'] = contact
  fields.update(ne=ne, nf=nf, nl=nl, nefc=nefc, ncon=ncon, efc_type=efc_type)

  # copy because device_put is async:
  data = types.Data(**{k: copy.copy(v) for k, v in fields.items()})

  return jax.device_put(data, device=device)
