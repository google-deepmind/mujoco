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
import logging
import os
from typing import Any, Dict, List, Optional, Tuple, Union
import warnings

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


def _is_cuda_gpu_device(device: jax.Device) -> bool:
  try:
    cuda_devices = jax.devices('cuda')
  except RuntimeError:
    logging.info('No CUDA GPU devices found in jax.devices("cuda").')
    return False
  return device in cuda_devices


def _resolve_impl(
    device: jax.Device,
) -> types.Impl:
  """Pick a default implementation based on the device specified."""
  if _is_cuda_gpu_device(device):
    # TODO(btaba): Remove flag once Warp is ready to launch.
    mjx_warp_enabled = os.environ.get('MJX_WARP_ENABLED', 'f').lower() == 'true'
    if mjx_warp_enabled:
      logging.debug('Picking default implementation: Warp.')
      return types.Impl.WARP
    logging.info('MJX Warp is disabled via MJX_WARP_ENABLED=false.')

  if device.platform in ('gpu', 'tpu'):
    logging.debug('Picking default implementation: JAX.')
    return types.Impl.JAX

  if device.platform == 'cpu':
    mjx_c_default = (
        os.environ.get('MJX_C_DEFAULT_ENABLED', 'f').lower() == 'true'
    )
    if mjx_c_default:
      logging.debug('Picking default implementation: C.')
      return types.Impl.C
    return types.Impl.JAX

  raise ValueError(f'Unsupported device: {device}')


def _resolve_device(
    impl: types.Impl,
) -> jax.Device:
  """Resolves a device based on the implementation."""
  impl = types.Impl(impl)
  if impl == types.Impl.JAX:
    device_0 = jax.devices()[0]
    logging.debug('Picking default device: %s.', device_0)
    return device_0

  if impl == types.Impl.C:
    cpu_0 = jax.devices('cpu')[0]
    logging.debug('Picking default device: %s', cpu_0)
    return cpu_0

  if impl == types.Impl.WARP:
    # WARP implementation requires a CUDA GPU.
    cuda_gpus = [d for d in jax.devices('cuda')]
    if not cuda_gpus:
      raise AssertionError(
          'No CUDA GPU devices found in'
          f' jax.devices("cuda")={jax.devices("cuda")}.'
      )

    logging.debug('Picking default device: %s', cuda_gpus[0])
    return cuda_gpus[0]

  raise ValueError(f'Unsupported implementation: {impl}')


def _check_impl_device_compatibility(
    impl: Union[str, types.Impl],
    device: jax.Device,
) -> None:
  """Checks that the implementation is compatible with the device."""
  if impl is None:
    raise ValueError('No implementation specified.')

  impl = types.Impl(impl)

  if impl == types.Impl.WARP:
    if not _is_cuda_gpu_device(device):
      raise AssertionError(
          'Warp implementation requires a CUDA GPU device, got '
          f'{device}.'
      )

    mjx_warp_enabled = os.environ.get('MJX_WARP_ENABLED', 'f').lower() == 'true'
    if not mjx_warp_enabled:
      raise AssertionError(
          'Warp implementation is disabled via MJX_WARP_ENABLED=false.'
      )

  is_cpu_device = device.platform == 'cpu'
  if impl == types.Impl.C:
    if not is_cpu_device:
      raise AssertionError(
          f'C implementation requires a CPU device, got {device}.'
      )

  # NB: JAX implementation works with any device.


def _resolve_impl_and_device(
    impl: Optional[Union[str, types.Impl]],
    device: Optional[jax.Device] = None,
) -> Tuple[types.Impl, jax.Device]:
  """Resolves a implementation and device."""
  if impl:
    impl = types.Impl(impl)

  has_impl, has_device = impl is not None, device is not None
  if (has_impl, has_device) == (True, True):
    pass
  elif (has_impl, has_device) == (True, False):
    device = _resolve_device(impl)
  elif (has_impl, has_device) == (False, True):
    impl = _resolve_impl(device)
  else:
    device = jax.devices(jax.default_backend())[0]
    logging.info('Using JAX default device: %s.', device)
    impl = _resolve_impl(device)

  _check_impl_device_compatibility(impl, device)
  return impl, device  # pytype: disable=bad-return-type


def _strip_weak_type(tree):
  def f(leaf):
    if isinstance(leaf, jax.Array):
      return leaf.astype(jax.dtypes.canonicalize_dtype(leaf.dtype))
    return leaf

  return jax.tree_util.tree_map(f, tree)


def _put_option(
    o: mujoco.MjOption,
    impl: types.Impl,
    impl_fields: Optional[dict[str, Any]] = None,
) -> types.Option:
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
    if o.enableflags & 2**i and 2 ** i not in set(types.EnableBit):
      raise NotImplementedError(f'{mujoco.mjtEnableBit(2**i)}')

  fields = {f.name: getattr(o, f.name, None) for f in types.Option.fields()}
  fields['integrator'] = types.IntegratorType(o.integrator)
  fields['cone'] = types.ConeType(o.cone)
  fields['jacobian'] = types.JacobianType(o.jacobian)
  fields['solver'] = types.SolverType(o.solver)
  fields['disableflags'] = types.DisableBit(o.disableflags)
  fields['enableflags'] = types.EnableBit(o.enableflags)

  if impl == types.Impl.JAX:
    has_fluid_params = o.density > 0 or o.viscosity > 0 or o.wind.any()
    implicitfast = o.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if implicitfast and has_fluid_params:
      raise NotImplementedError('implicitfast not implemented for fluid drag.')
    fields['has_fluid_params'] = has_fluid_params
    return types.OptionJAX(**fields, **(impl_fields or {}))

  if impl == types.Impl.C:
    c_field_keys = types.OptionC.__annotations__.keys() - fields.keys()
    c_fields = {k: getattr(o, k, None) for k in c_field_keys}
    return types.OptionC(**fields, **c_fields, **(impl_fields or {}))

  raise NotImplementedError(f'Unsupported implementation: {impl}')


def _put_statistic(s: mujoco.MjStatistic) -> types.Statistic:
  """Puts mujoco.MjStatistic onto a device, resulting in mjx.Statistic."""
  return types.Statistic(
      meaninertia=s.meaninertia,
      meanmass=s.meanmass,
      meansize=s.meansize,
      extent=s.extent,
      center=s.center,
  )


def _put_model_jax(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""
  mesh_geomid = set()
  for g1, g2, ip in collision_driver.geom_pairs(m):
    t1, t2 = m.geom_type[[g1, g2]]
    # check collision function exists for type pair
    if not collision_driver.has_collision_fn(t1, t2):
      t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
      raise NotImplementedError(f'({t1}, {t2}) collisions not implemented.')
    # margin/gap not supported for meshes and height fields
    no_margin = {mujoco.mjtGeom.mjGEOM_MESH, mujoco.mjtGeom.mjGEOM_HFIELD}
    if no_margin.intersection({t1, t2}):
      if ip != -1:
        margin = m.pair_margin[ip]
      else:
        margin = m.geom_margin[g1] + m.geom_margin[g2]
      if margin.any():
        t1, t2 = mujoco.mjtGeom(t1), mujoco.mjtGeom(t2)
        raise NotImplementedError(f'({t1}, {t2}) margin/gap not implemented.')
    for t, g in [(t1, g1), (t2, g2)]:
      if t == mujoco.mjtGeom.mjGEOM_MESH:
        mesh_geomid.add(g)

  # check for unsupported sensor and equality constraint combinations
  sensor_rne_postconstraint = (
      np.any(m.sensor_type == types.SensorType.ACCELEROMETER)
      | np.any(m.sensor_type == types.SensorType.FORCE)
      | np.any(m.sensor_type == types.SensorType.TORQUE)
      | np.any(m.sensor_type == types.SensorType.FRAMELINACC)
      | np.any(m.sensor_type == types.SensorType.FRAMEANGACC)
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
    if missing:
      raise NotImplementedError(
          f'{[mj_type(m) for m in missing]} not supported'
      )

  mj_field_names = {f.name for f in types.Model.fields() if f.name != '_impl'}
  fields = {f: getattr(m, f) for f in mj_field_names}
  fields['cam_mat0'] = fields['cam_mat0'].reshape((-1, 3, 3))
  fields['opt'] = _put_option(m.opt, types.Impl.JAX)
  fields['stat'] = _put_statistic(m.stat)

  fields_jax = {}
  fields_jax['dof_hasfrictionloss'] = fields['dof_frictionloss'] > 0
  fields_jax['tendon_hasfrictionloss'] = fields['tendon_frictionloss'] > 0
  fields_jax['geom_rbound_hfield'] = fields['geom_rbound']

  # spatial tendon wrap inside
  fields_jax['wrap_inside_maxiter'] = 5
  fields_jax['wrap_inside_tolerance'] = 1.0e-4
  fields_jax['wrap_inside_z_init'] = 1.0 - 1.0e-5
  fields_jax['is_wrap_inside'] = np.zeros(0, dtype=bool)
  if m.nsite:
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

    # wrap inside flag
    fields_jax['is_wrap_inside'] = np.array(
        (np.linalg.norm(side - geom_pos, axis=1) < geom_size) & (side_id >= 0)
    )

  # Pre-compile meshes for MJX collisions.
  fields_jax['mesh_convex'] = [None] * m.nmesh
  for i in mesh_geomid:
    dataid = m.geom_dataid[i]
    if fields_jax['mesh_convex'][dataid] is None:
      fields_jax['mesh_convex'][dataid] = mesh.convex(m, dataid)  # pytype: disable=unsupported-operands
  fields_jax['mesh_convex'] = tuple(fields_jax['mesh_convex'])

  jax_impl = types.ModelJAX(**fields_jax)
  model = types.Model(
      **{k: copy.copy(v) for k, v in fields.items()}, _impl=jax_impl
  )

  model = jax.device_put(model, device=device)
  return _strip_weak_type(model)


def _put_model_c(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""
  mj_field_names = {f.name for f in types.Model.fields() if f.name != '_impl'}
  fields = {f: getattr(m, f) for f in mj_field_names}
  fields['cam_mat0'] = fields['cam_mat0'].reshape((-1, 3, 3))
  fields['opt'] = _put_option(m.opt, impl=types.Impl.C)
  fields['stat'] = _put_statistic(m.stat)

  c_impl_keys = (
      types.ModelC.__annotations__.keys() - types.Model.__annotations__.keys()
  )
  c_impl_dict = {k: getattr(m, k) for k in c_impl_keys}
  c_impl_obj = types.ModelC(**{k: copy.copy(v) for k, v in c_impl_dict.items()})

  model = types.Model(
      **{k: copy.copy(v) for k, v in fields.items()}, _impl=c_impl_obj
  )
  model = jax.device_put(model, device=device)
  return _strip_weak_type(model)


def put_model(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
    _full_compat: bool = False,  # pylint: disable=invalid-name
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model.

  Args:
    m: the model to put onto device
    device: which device to use - if unspecified picks the default device
    impl: implementation to use
    _full_compat: put all MjModel fields onto device irrespective of MJX support
      This is an experimental feature.  Avoid using it for now.

  Returns:
    an mjx.Model placed on device

  Raises:
    ValueError: if impl is not supported
    DeprecationWarning: if _full_compat is True
  """

  if _full_compat:
    warnings.warn(
        'mjx.put_model(..., _full_compat=True) is deprecated and will be'
        ' removed in MuJoCo >=3.4.  Use mjx.put_model(..., impl=types.Impl.C)'
        ' instead.',
        DeprecationWarning,
        stacklevel=2,
    )
    impl = types.Impl.C

  impl, device = _resolve_impl_and_device(impl, device)
  if impl == types.Impl.JAX:
    return _put_model_jax(m, device)
  elif impl == types.Impl.C:
    return _put_model_c(m, device)
  elif impl == types.Impl.WARP:
    raise NotImplementedError('Warp implementation not implemented yet.')
  else:
    raise ValueError(f'Unsupported implementation: {impl}')


def _make_data_public_fields(m: types.Model) -> Dict[str, Any]:
  """Create public fields for the Data object."""
  float_ = jp.zeros(1, float).dtype
  zero_fields = {
      'time': (float_,),
      'qvel': (m.nv, float_),
      'act': (m.na, float_),
      'qacc_warmstart': (m.nv, float_),
      'ctrl': (m.nu, float_),
      'qfrc_applied': (m.nv, float_),
      'xfrc_applied': (m.nbody, 6, float_),
      'mocap_pos': (m.nmocap, 3, float_),
      'mocap_quat': (m.nmocap, 4, float_),
      'qacc': (m.nv, float_),
      'act_dot': (m.na, float_),
      'userdata': (m.nuserdata, float_),
      'sensordata': (m.nsensordata, float_),
      'xpos': (m.nbody, 3, float_),
      'xquat': (m.nbody, 4, float_),
      'xmat': (m.nbody, 3, 3, float_),
      'xipos': (m.nbody, 3, float_),
      'ximat': (m.nbody, 3, 3, float_),
      'xanchor': (m.njnt, 3, float_),
      'xaxis': (m.njnt, 3, float_),
      'geom_xpos': (m.ngeom, 3, float_),
      'geom_xmat': (m.ngeom, 3, 3, float_),
      'site_xpos': (m.nsite, 3, float_),
      'site_xmat': (m.nsite, 3, 3, float_),
      'cam_xpos': (m.ncam, 3, float_),
      'cam_xmat': (m.ncam, 3, 3, float_),
      'subtree_com': (m.nbody, 3, float_),
      'actuator_force': (m.nu, float_),
      'qfrc_bias': (m.nv, float_),
      'qfrc_gravcomp': (m.nv, float_),
      'qfrc_fluid': (m.nv, float_),
      'qfrc_passive': (m.nv, float_),
      'qfrc_actuator': (m.nv, float_),
      'qfrc_smooth': (m.nv, float_),
      'qacc_smooth': (m.nv, float_),
      'qfrc_constraint': (m.nv, float_),
      'qfrc_inverse': (m.nv, float_),
      'cvel': (m.nbody, 6, float_),
  }
  zero_fields = {
      k: np.zeros(v[:-1], dtype=v[-1]) for k, v in zero_fields.items()
  }
  return zero_fields


def _make_data_contact_jax(
    condim: np.ndarray, efc_address: np.ndarray
) -> types.Contact:
  """Create contact for the Data object."""
  ncon = condim.size
  float_ = jp.zeros(1, float).dtype
  int_ = jp.zeros(1, int).dtype
  contact = types.Contact(
      dist=np.zeros((ncon,), dtype=float_),
      pos=np.zeros((ncon, 3), dtype=float_),
      frame=np.zeros((ncon, 3, 3), dtype=float_),
      includemargin=np.zeros((ncon,), dtype=float_),
      friction=np.zeros((ncon, 5), dtype=float_),
      solref=np.zeros((ncon, mujoco.mjNREF), dtype=float_),
      solreffriction=np.zeros((ncon, mujoco.mjNREF), dtype=float_),
      solimp=np.zeros((ncon, mujoco.mjNIMP), dtype=float_),
      dim=condim,
      # let jax pick contact.geom int precision, for interop with
      # jax_enable_x64
      geom1=np.full((ncon,), -1, dtype=int_),
      geom2=np.full((ncon,), -1, dtype=int_),
      geom=np.full((ncon, 2), -1, dtype=int_),
      efc_address=efc_address,
  )
  return contact


def _make_data_jax(
    m: Union[types.Model, mujoco.MjModel],
    device: Optional[jax.Device] = None,
) -> types.Data:
  """Allocate and initialize Data for the JAX implementation."""
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc
  efc_address = constraint.make_efc_address(m, dim, efc_type)

  float_ = jp.zeros(1, float).dtype
  int_ = jp.zeros(1, int).dtype
  contact = _make_data_contact_jax(dim, efc_address)

  if m.opt.cone == types.ConeType.ELLIPTIC and np.any(contact.dim == 1):
    raise NotImplementedError(
        'condim=1 with ConeType.ELLIPTIC not implemented.'
    )

  zero_impl_fields = {
      'solver_niter': (int_,),
      'cdof': (m.nv, 6, float_),
      'cinert': (m.nbody, 10, float_),
      'ten_wrapadr': (m.ntendon, np.int32),
      'ten_wrapnum': (m.ntendon, np.int32),
      'ten_J': (m.ntendon, m.nv, float_),
      'ten_length': (m.ntendon, float_),
      'wrap_obj': (m.nwrap, 2, np.int32),
      'wrap_xpos': (m.nwrap, 6, float_),
      'actuator_length': (m.nu, float_),
      'actuator_moment': (m.nu, m.nv, float_),
      'crb': (m.nbody, 10, float_),
      'qM': (m.nM, float_) if support.is_sparse(m) else (m.nv, m.nv, float_),
      'M': (m.nC, float_),
      'qLD': (m.nC, float_) if support.is_sparse(m) else (m.nv, m.nv, float_),
      'qLDiagInv': (m.nv, float_) if support.is_sparse(m) else (0, float_),
      'ten_velocity': (m.ntendon, float_),
      'actuator_velocity': (m.nu, float_),
      'cdof_dot': (m.nv, 6, float_),
      'cacc': (m.nbody, 6, float_),
      'cfrc_int': (m.nbody, 6, float_),
      'cfrc_ext': (m.nbody, 6, float_),
      'subtree_linvel': (m.nbody, 3, float_),
      'subtree_angmom': (m.nbody, 3, float_),
      'efc_J': (nefc, m.nv, float_),
      'efc_pos': (nefc, float_),
      'efc_margin': (nefc, float_),
      'efc_frictionloss': (nefc, float_),
      'efc_D': (nefc, float_),
      'efc_aref': (nefc, float_),
      'efc_force': (nefc, float_),
  }
  zero_impl_fields = {
      k: np.zeros(v[:-1], dtype=v[-1]) for k, v in zero_impl_fields.items()
  }
  impl = types.DataJAX(
      ne=ne,
      nf=nf,
      nl=nl,
      nefc=nefc,
      ncon=ncon,
      contact=contact,
      efc_type=efc_type,
      **zero_impl_fields,
  )

  d = types.Data(
      qpos=jp.array(m.qpos0, dtype=float_),
      eq_active=m.eq_active0,
      _impl=impl,
      **_make_data_public_fields(m),
  )

  if m.nmocap:
    # Set mocap_pos/quat = body_pos/quat for mocap bodies as done in C MuJoCo.
    body_mask = m.body_mocapid >= 0
    body_pos = m.body_pos[body_mask]
    body_quat = m.body_quat[body_mask]
    d = d.replace(
        mocap_pos=body_pos[m.body_mocapid[body_mask]],
        mocap_quat=body_quat[m.body_mocapid[body_mask]],
    )

  d = jax.device_put(d, device=device)
  return d


def _make_data_c(
    m: Union[types.Model, mujoco.MjModel],
    device: Optional[jax.Device] = None,
) -> types.Data:
  """Allocate and initialize Data for the C implementation."""
  # TODO(stunya): The C implementation should not use static dimensions, and
  # the backend implementation details should be kept hidden from JAX
  # altogether.
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(m, dim, efc_type)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc

  float_ = jp.zeros(1, float).dtype
  int_ = jp.zeros(1, int).dtype
  # TODO(stunya): remove the JAX contact from C data.
  contact = _make_data_contact_jax(dim, efc_address)

  def get(m, name: str):
    return getattr(m._impl, name) if hasattr(m, '_impl') else getattr(m, name)  # pylint: disable=protected-access

  nflexvert = get(m, 'nflexvert')
  nflexedge = get(m, 'nflexedge')
  nflexelem = get(m, 'nflexelem')
  nbvh = get(m, 'nbvh')
  nbvhdynamic = get(m, 'nbvhdynamic')
  zero_impl_fields = {
      'solver_niter': (int_,),
      'cdof': (m.nv, 6, float_),
      'cinert': (m.nbody, 10, float_),
      'light_xpos': (m.nlight, 3, float_),
      'light_xdir': (m.nlight, 3, float_),
      'flexvert_xpos': (nflexvert, 3, float_),
      'flexelem_aabb': (nflexelem, 6, float_),
      'flexedge_J_rownnz': (nflexedge, np.int32),
      'flexedge_J_rowadr': (nflexedge, np.int32),
      'flexedge_J_colind': (nflexedge, m.nv, np.int32),
      'flexedge_J': (nflexedge, m.nv, float_),
      'flexedge_length': (nflexedge, float_),
      'ten_J_rownnz': (m.ntendon, np.int32),
      'ten_J_rowadr': (m.ntendon, np.int32),
      'ten_J_colind': (m.ntendon, m.nv, np.int32),
      'ten_J': (m.ntendon, m.nv, float_),
      'ten_length': (m.ntendon, float_),
      'ten_wrapadr': (m.ntendon, np.int32),
      'ten_wrapnum': (m.ntendon, np.int32),
      'wrap_obj': (m.nwrap, 2, np.int32),
      'wrap_xpos': (m.nwrap, 6, float_),
      'actuator_length': (m.nu, float_),
      'moment_rownnz': (m.nu, np.int32),
      'moment_rowadr': (m.nu, np.int32),
      'moment_colind': (m.nJmom, np.int32),
      'actuator_moment': (m.nu, m.nv, float_),
      'bvh_aabb_dyn': (nbvhdynamic, 6, float_),
      'bvh_active': (nbvh, np.uint8),
      'flexedge_velocity': (nflexedge, float_),
      'crb': (m.nbody, 10, float_),
      'qM': (m.nM, float_),
      'M': (m.nC, float_),
      'qLD': (m.nC, float_),
      'qH': (m.nC, float_),
      'qHDiagInv': (m.nv, float_),
      'qLDiagInv': (m.nv, float_),
      'ten_velocity': (m.ntendon, float_),
      'actuator_velocity': (m.nu, float_),
      'plugin_data': (get(m, 'nplugin'), np.uint64),
      'B_rownnz': (m.nbody, np.int32),
      'B_rowadr': (m.nbody, np.int32),
      'B_colind': (m.nB, np.int32),
      'M_rownnz': (m.nv, np.int32),
      'M_rowadr': (m.nv, np.int32),
      'M_colind': (m.nC, np.int32),
      'mapM2M': (m.nC, np.int32),
      'D_rownnz': (m.nv, np.int32),
      'D_rowadr': (m.nv, np.int32),
      'D_diag': (m.nv, np.int32),
      'D_colind': (m.nD, np.int32),
      'mapM2D': (m.nD, np.int32),
      'mapD2M': (m.nM, np.int32),
      'qDeriv': (m.nD, float_),
      'qLU': (m.nD, float_),
      'qfrc_spring': (m.nv, float_),
      'qfrc_damper': (m.nv, float_),
      'cdof_dot': (m.nv, 6, float_),
      'cacc': (m.nbody, 6, float_),
      'cfrc_int': (m.nbody, 6, float_),
      'cfrc_ext': (m.nbody, 6, float_),
      'subtree_linvel': (m.nbody, 3, float_),
      'subtree_angmom': (m.nbody, 3, float_),
      'efc_J': (nefc, m.nv, float_),
      'efc_pos': (nefc, float_),
      'efc_margin': (nefc, float_),
      'efc_frictionloss': (nefc, float_),
      'efc_D': (nefc, float_),
      'efc_aref': (nefc, float_),
      'efc_force': (nefc, float_),
  }
  zero_impl_fields = {
      k: np.zeros(v[:-1], dtype=v[-1]) for k, v in zero_impl_fields.items()
  }
  impl = types.DataC(
      ne=ne,
      nf=nf,
      nl=nl,
      nefc=nefc,
      ncon=ncon,
      contact=contact,
      efc_type=efc_type,
      **zero_impl_fields,
  )

  d = types.Data(
      qpos=jp.array(m.qpos0, dtype=float_),
      eq_active=m.eq_active0,
      _impl=impl,
      **_make_data_public_fields(m),
  )

  if m.nmocap:
    # Set mocap_pos/quat = body_pos/quat for mocap bodies as done in C MuJoCo.
    body_mask = m.body_mocapid >= 0
    body_pos = m.body_pos[body_mask]
    body_quat = m.body_quat[body_mask]
    d = d.replace(
        mocap_pos=body_pos[m.body_mocapid[body_mask]],
        mocap_quat=body_quat[m.body_mocapid[body_mask]],
    )

  d = jax.device_put(d, device=device)
  return d


def make_data(
    m: Union[types.Model, mujoco.MjModel],
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
    _full_compat: bool = False,  # pylint: disable=invalid-name
) -> types.Data:
  """Allocate and initialize Data.

  Args:
    m: the model to use
    device: which device to use - if unspecified picks the default device
    impl: implementation to use ('jax', 'warp')
    _full_compat: put all fields onto device irrespective of MJX support This is
      an experimental feature.  Avoid using it for now. If using this flag, also
      use _full_compat for put_model.

  Returns:
    an initialized mjx.Data placed on device

  Raises:
    ValueError: if the model's impl does not match the make_data impl
    NotImplementedError: if the impl is not implemented yet
    DeprecationWarning: if _full_compat is used
  """
  if _full_compat:
    warnings.warn(
        'mjx.make_data(..., _full_compat=True) is deprecated.  Use'
        ' mjx.make_data(..., impl=types.Impl.C) instead.',
        DeprecationWarning,
        stacklevel=2,
    )
    impl = types.Impl.C

  impl, device = _resolve_impl_and_device(impl, device)

  if isinstance(m, types.Model) and m.impl != impl:
    raise ValueError(
        f'Model impl {m.impl} does not match make_data '
        f'implementation {impl}.'
    )

  if impl == types.Impl.JAX:
    return _make_data_jax(m, device)
  elif impl == types.Impl.C:
    return _make_data_c(m, device)

  raise NotImplementedError(
      f'make_data for implementation "{impl}" not implemented yet.'
  )


def _put_contact(
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
  contact_map = -np.ones_like(dim)
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


def _put_data_public_fields(d: mujoco.MjData) -> Dict[str, Any]:
  """Returns public fields from mujoco.MjData in a dictionary."""
  fields = {
      f.name: getattr(d, f.name)
      for f in types.Data.fields()
      if f.name != '_impl'
  }
  # MJX uses square matrices for these fields:
  for fname in ('xmat', 'ximat', 'geom_xmat', 'site_xmat', 'cam_xmat'):
    fields[fname] = fields[fname].reshape((-1, 3, 3))

  return fields


def _put_data_jax(
    m: mujoco.MjModel, d: mujoco.MjData, device: Optional[jax.Device] = None
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data."""
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

  fields = _put_data_public_fields(d)

  # Implementation specific fields.
  impl_fields = {
      f.name: getattr(d, f.name)
      for f in types.DataJAX.fields()
      if hasattr(d, f.name)
  }
  # MJX does not support islanding, so only transfer the first solver_niter
  impl_fields['solver_niter'] = impl_fields['solver_niter'][0]

  # convert sparse representation of actuator_moment to dense matrix
  moment = np.zeros((m.nu, m.nv))
  mujoco.mju_sparse2dense(
      moment,
      d.actuator_moment,
      d.moment_rownnz,
      d.moment_rowadr,
      d.moment_colind,
  )
  impl_fields['actuator_moment'] = moment

  contact, contact_map = _put_contact(d.contact, dim, efc_address)

  # pad efc fields: MuJoCo efc arrays are sparse for inactive constraints.
  # efc_J is also optionally column-sparse (typically for large nv).  MJX is
  # neither: it contains zeros for inactive constraints, and efc_J is always
  # (nefc, nv).  this may change in the future.
  if mujoco.mj_isSparse(m):
    efc_j = np.zeros((d.efc_J_rownnz.shape[0], m.nv))
    mujoco.mju_sparse2dense(
        efc_j,
        impl_fields['efc_J'],
        d.efc_J_rownnz,
        d.efc_J_rowadr,
        d.efc_J_colind,
    )
    impl_fields['efc_J'] = efc_j
  else:
    impl_fields['efc_J'] = impl_fields['efc_J'].reshape(
        (-1 if m.nv else 0, m.nv)
    )

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
      value[value_beg : value_beg + size] = impl_fields[fname][
          d_beg : d_beg + size
      ]

    # for nc, we may reorder contacts so they match MJX order: group by dim
    for id_to, id_from in enumerate(contact_map):
      if id_from == -1:
        continue
      num_rows = dim[id_to]
      if num_rows > 1 and m.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:
        num_rows = (num_rows - 1) * 2
      efc_i, efc_o = d.contact.efc_address[id_from], efc_address[id_to]
      if efc_i == -1:
        continue
      value[efc_o : efc_o + num_rows] = impl_fields[fname][
          efc_i : efc_i + num_rows
      ]

    impl_fields[fname] = value

  # convert qM and qLD if jacobian is dense
  if not support.is_sparse(m):
    impl_fields['qM'] = np.zeros((m.nv, m.nv))
    mujoco.mj_fullM(m, impl_fields['qM'], d.qM)
    # TODO(erikfrey): derive L*L' from L'*D*L instead of recomputing
    try:
      impl_fields['qLD'], _ = scipy.linalg.cho_factor(impl_fields['qM'])
    except scipy.linalg.LinAlgError:
      # this happens when qM is empty or unstable simulation
      impl_fields['qLD'] = np.zeros((m.nv, m.nv))
    impl_fields['qLDiagInv'] = np.zeros(0)

  impl_fields['contact'] = contact
  impl_fields.update(
      ne=ne, nf=nf, nl=nl, nefc=nefc, ncon=ncon, efc_type=efc_type
  )

  # copy because device_put is async:
  data_jax = types.DataJAX(**{k: copy.copy(v) for k, v in impl_fields.items()})
  data = types.Data(
      **{k: copy.copy(v) for k, v in fields.items()}, _impl=data_jax
  )

  data = jax.device_put(data, device=device)
  return _strip_weak_type(data)


def _put_data_c(
    m: mujoco.MjModel, d: mujoco.MjData, device: Optional[jax.Device] = None
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data."""
  # TODO(stunya): ncon, nefc should potentially be jax.Array, and contact/efc
  # should not be materialized in JAX.
  dim = collision_driver.make_condim(m)
  efc_type = constraint.make_efc_type(m, dim)
  efc_address = constraint.make_efc_address(m, dim, efc_type)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc

  # TODO(stunya): remove this check.
  for d_val, val, name in (
      (d.ncon, ncon, 'ncon'),
      (d.ne, ne, 'ne'),
      (d.nf, nf, 'nf'),
      (d.nl, nl, 'nl'),
      (d.nefc, nefc, 'nefc'),
  ):
    if d_val > val:
      raise ValueError(f'd.{name} too high, d.{name} = {d_val}, model = {val}')

  fields = _put_data_public_fields(d)

  # Implementation specific fields.
  impl_fields = {
      f.name: getattr(d, f.name)
      for f in types.DataC.fields()
      if hasattr(d, f.name)
  }

  # TODO(stunya): support islanding via C impl.
  impl_fields['solver_niter'] = impl_fields['solver_niter'][0]

  # TODO(btaba): remove dense actuator moment.
  # convert sparse representation of actuator_moment to dense matrix
  moment = np.zeros((m.nu, m.nv))
  mujoco.mju_sparse2dense(
      moment,
      d.actuator_moment,
      d.moment_rownnz,
      d.moment_rowadr,
      d.moment_colind,
  )
  impl_fields['actuator_moment'] = moment

  # TODO(btaba): remove reliance on JAX _put_contact.
  contact, contact_map = _put_contact(d.contact, dim, efc_address)

  # TODO(btaba): remove reliance on dense efc_J.
  if mujoco.mj_isSparse(m):
    efc_j = np.zeros((d.efc_J_rownnz.shape[0], m.nv))
    mujoco.mju_sparse2dense(
        efc_j,
        impl_fields['efc_J'],
        d.efc_J_rownnz,
        d.efc_J_rowadr,
        d.efc_J_colind,
    )
    impl_fields['efc_J'] = efc_j
  else:
    impl_fields['efc_J'] = impl_fields['efc_J'].reshape(
        (-1 if m.nv else 0, m.nv)
    )

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
      value[value_beg : value_beg + size] = impl_fields[fname][
          d_beg : d_beg + size
      ]

    # for nc, we may reorder contacts so they match MJX order: group by dim
    for id_to, id_from in enumerate(contact_map):
      if id_from == -1:
        continue
      num_rows = dim[id_to]
      if num_rows > 1 and m.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:
        num_rows = (num_rows - 1) * 2
      efc_i, efc_o = d.contact.efc_address[id_from], efc_address[id_to]
      if efc_i == -1:
        continue
      value[efc_o : efc_o + num_rows] = impl_fields[fname][
          efc_i : efc_i + num_rows
      ]

    impl_fields[fname] = value

  impl_fields['contact'] = contact
  impl_fields.update(
      ne=ne, nf=nf, nl=nl, nefc=nefc, ncon=ncon, efc_type=efc_type
  )

  # copy because device_put is async:
  data_jax = types.DataC(**{k: copy.copy(v) for k, v in impl_fields.items()})
  data = types.Data(
      **{k: copy.copy(v) for k, v in fields.items()}, _impl=data_jax
  )

  data = jax.device_put(data, device=device)
  return _strip_weak_type(data)


def put_data(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
    _full_compat: bool = False,  # pylint: disable=invalid-name
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data.

  Args:
    m: the model to use
    d: the data to put on device
    device: which device to use - if unspecified picks the default device
    impl: implementation to use ('jax', 'warp')
    _full_compat: put all MjModel fields onto device irrespective of MJX support
      This is an experimental feature.  Avoid using it for now. If using this
      flag, also use _full_compat for put_model.

  Returns:
    an mjx.Data placed on device
  """
  if _full_compat:
    warnings.warn(
        'mjx.put_data(..., _full_compat=True) is deprecated.  Use'
        ' mjx.put_data(..., impl=types.Impl.C) instead.',
        DeprecationWarning,
        stacklevel=2,
    )
    impl = types.Impl.C

  impl, device = _resolve_impl_and_device(impl, device)
  if impl == types.Impl.JAX:
    return _put_data_jax(m, d, device)
  elif impl == types.Impl.C:
    return _put_data_c(m, d, device)

  raise NotImplementedError(
      f'put_data for implementation "{impl}" not implemented yet.'
  )


def _get_contact(c: mujoco._structs._MjContactList, cx: types.Contact):
  """Converts mjx.Contact to mujoco._structs._MjContactList."""
  con_id = np.nonzero(cx.dist <= 0)[0]
  for field in types.Contact.fields():
    value = getattr(cx, field.name)[con_id]
    if field.name == 'frame':
      value = value.reshape((-1, 9))
    getattr(c, field.name)[:] = value


def _get_data_into(
    result: Union[mujoco.MjData, List[mujoco.MjData]],
    m: mujoco.MjModel,
    d: types.Data,
):
  """Gets mjx.Data from a device into an existing mujoco.MjData or list."""
  batched = isinstance(result, list)
  d = jax.device_get(d)
  batch_size = d.qpos.shape[0] if batched else 1

  dof_i, dof_j = [], []
  if d.impl == types.Impl.JAX:
    for i in range(m.nv):
      j = i
      while j > -1:
        dof_i.append(i)
        dof_j.append(j)
        j = m.dof_parentid[j]

  for i in range(batch_size):
    d_i = jax.tree_util.tree_map(lambda x, i=i: x[i], d) if batched else d
    result_i = result[i] if batched else result
    ncon = (d_i._impl.contact.dist <= 0).sum()
    efc_active = (d_i._impl.efc_J != 0).any(axis=1)
    nefc = int(efc_active.sum())
    nj = (d_i._impl.efc_J != 0).sum() if support.is_sparse(m) else nefc * m.nv

    if ncon != result_i.ncon or nefc != result_i.nefc or nj != result_i.nJ:
      mujoco._functions._realloc_con_efc(result_i, ncon=ncon, nefc=nefc, nJ=nj)  # pylint: disable=protected-access

    if d.impl == types.Impl.JAX:
      all_fields = types.Data.fields() + types.DataJAX.fields()
    elif d.impl == types.Impl.C:
      all_fields = types.Data.fields() + types.DataC.fields()
    else:
      raise NotImplementedError(
          f'get_data_into for implementation "{d.impl}" not implemented'
          ' yet.'
      )

    for field in all_fields:
      if field.name not in mujoco.MjData.__dict__.keys():
        continue

      if field.name == 'contact':
        _get_contact(result_i.contact, d_i._impl.contact)
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
              d_i._impl.actuator_moment,
              moment_rownnz,
              moment_rowadr,
              moment_colind,
          )
        result_i.moment_rownnz[:] = moment_rownnz
        result_i.moment_rowadr[:] = moment_rowadr
        result_i.moment_colind[:] = moment_colind
        result_i.actuator_moment[:] = actuator_moment
        continue

      if hasattr(d_i._impl, field.name):
        value = getattr(d_i._impl, field.name)
      else:
        value = getattr(d_i, field.name)

      if field.name in ('nefc', 'ncon'):
        value = {'nefc': nefc, 'ncon': ncon}[field.name]
      elif field.name.endswith('xmat') or field.name == 'ximat':
        value = value.reshape((-1, 9))
      elif field.name == 'efc_J':
        value = value[efc_active]
        if support.is_sparse(m):
          efc_J_rownnz = np.zeros(nefc, dtype=np.int32)
          efc_J_rowadr = np.zeros(nefc, dtype=np.int32)
          efc_J_colind = np.zeros(nj, dtype=np.int32)
          efc_J = np.zeros(nj)
          mujoco.mju_dense2sparse(
              efc_J,
              value,
              efc_J_rownnz,
              efc_J_rowadr,
              efc_J_colind,
          )
          result_i.efc_J_rownnz[:] = efc_J_rownnz
          result_i.efc_J_rowadr[:] = efc_J_rowadr
          result_i.efc_J_colind[:] = efc_J_colind
          value = efc_J
        else:
          value = value.reshape(-1)
      elif field.name.startswith('efc_'):
        value = value[efc_active]
      if d.impl == types.Impl.JAX:
        if field.name == 'qM' and not support.is_sparse(m):
          value = value[dof_i, dof_j]
        elif field.name == 'qLD' and not support.is_sparse(m):
          value = np.zeros(m.nC)
        elif field.name == 'qLDiagInv' and not support.is_sparse(m):
          value = np.ones(m.nv)

      if isinstance(value, np.ndarray) and value.shape:
        result_field = getattr(result_i, field.name)
        if result_field.shape != value.shape:
          raise ValueError(
              f'Input field {field.name} has shape {value.shape}, but output'
              f' has shape {result_field.shape}'
          )
        result_field[:] = value
      else:
        setattr(result_i, field.name, value)

    # TODO(taylorhowell): remove mapping once qM is deprecated
    # map inertia (sparse) to reduced inertia (compressed sparse) representation
    result_i.M[:] = result_i.qM[result_i.mapM2M]

    # recalculate qLD and qLDiagInv as MJX and MuJoCo have different
    # representations of the Cholesky decomposition.
    mujoco.mj_factorM(m, result_i)


def get_data_into(
    result: Union[mujoco.MjData, List[mujoco.MjData]],
    m: mujoco.MjModel,
    d: types.Data,
):
  """Gets mjx.Data from a device into an existing mujoco.MjData or list."""
  is_batched = isinstance(result, list)
  if is_batched and len(d.qpos.shape) < 2:
    raise ValueError('destination is a list, but d is not batched.')
  if not is_batched and len(d.qpos.shape) >= 2:
    raise ValueError('destination is a an MjData, but d is batched.')

  d = jax.device_get(d)

  if d.impl in (types.Impl.JAX, types.Impl.C):
    # TODO(stunya): Split out _get_data_into once codepaths diverge enough.
    return _get_data_into(result, m, d)

  raise NotImplementedError(
      f'get_data_into for implementation "{d.impl}" not implemented yet.'
  )


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
