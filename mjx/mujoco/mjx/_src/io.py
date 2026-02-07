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
import jax.experimental
from jax import numpy as jp
from jax.extend import backend
import mujoco
from mujoco.mjx._src import collision_driver
from mujoco.mjx._src import constraint
from mujoco.mjx._src import mesh
from mujoco.mjx._src import support
from mujoco.mjx._src import types
import mujoco.mjx.warp as mjxw
# pylint: disable=g-importing-member
from mujoco.mjx.warp import mjwp_types
from mujoco.mjx.warp import mujoco_warp as mjwp
from mujoco.mjx.warp import warp as wp
# pylint: enable=g-importing-member
import numpy as np
import scipy


def has_cuda_gpu_device() -> bool:
  return 'cuda' in backend.backends()


def _is_cuda_gpu_device(device: jax.Device) -> bool:
  if not has_cuda_gpu_device():
    return False
  return device in jax.devices('cuda')


def _resolve_impl(
    device: jax.Device,
) -> types.Impl:
  """Pick a default implementation based on the device specified."""
  if _is_cuda_gpu_device(device):
    # TODO(btaba): Remove flag once Warp is ready for GPU default.
    mjx_gpu_default_warp = (
        os.environ.get('MJX_GPU_DEFAULT_WARP', 'f').lower() == 'true'
    )
    if mjx_gpu_default_warp and mjxw.WARP_INSTALLED:
      logging.debug('Picking default implementation: Warp.')
      return types.Impl.WARP

  if device.platform in ('gpu', 'tpu'):
    logging.debug('Picking default implementation: JAX.')
    return types.Impl.JAX

  if device.platform == 'cpu':
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

  if impl == types.Impl.C or impl == types.Impl.CPP:
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
    if not mjxw.WARP_INSTALLED:
      raise RuntimeError(
          'Warp is not installed. Cannot use Warp implementation of MJX.'
      )

  is_cpu_device = device.platform == 'cpu'
  if impl == types.Impl.C or impl == types.Impl.CPP:
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
    device = jax.devices()[0]
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


def _wp_to_np_type(wp_field: Any, name: str = '') -> Any:
  """Converts a warp type to an MJX compatible numpy type."""
  if hasattr(wp_field, '_is_batched'):
    wp_field.strides = wp_field.strides[1:]
    wp_field.shape = wp_field.shape[1:]
  # warp scalars
  wp_dtype = type(wp_field)
  if wp_dtype in wp.types.warp_type_to_np_dtype:
    return wp.types.warp_type_to_np_dtype[wp_dtype](wp_field)

  # warp arrays
  if isinstance(wp_field, wp.types.array):
    return wp_field.numpy()

  # static
  static_types = (bool, int, float, np.bool, np.int32, np.int64,
                  np.float32, np.float64)  # fmt: skip
  is_static = lambda x: isinstance(x, static_types)
  if is_static(wp_field):
    return wp_field

  # tuples
  if isinstance(wp_field, tuple) and len(wp_field) == 0:
    return ()
  if isinstance(wp_field, tuple) and isinstance(wp_field[0], wp.types.array):
    return tuple(f.numpy() for f in wp_field)
  if isinstance(wp_field, tuple) and isinstance(
      wp_field[0], mjwp_types.TileSet
  ):
    return tuple(
        mjxw.types.TileSet(wp_field[i].adr.numpy(), wp_field[i].size)
        for i in range(len(wp_field))
    )
  if isinstance(wp_field, mjwp_types.BlockDim):
    return mjxw.types.BlockDim(**wp_field.__dict__)
  if isinstance(wp_field, tuple) and is_static(wp_field[0]):
    return wp_field

  raise NotImplementedError(
      f'Field {name} has unsupported type {type(wp_field)}.'
  )


def _put_option(
    o: mujoco.MjOption,
    impl: types.Impl,
    impl_fields: Optional[Dict[str, Any]] = None,
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

  fields = {
      f.name: getattr(o, f.name, None)
      for f in types.Option.fields()
      if f.name != '_impl'
  }
  fields['integrator'] = types.IntegratorType(o.integrator)
  fields['cone'] = types.ConeType(o.cone)
  fields['solver'] = types.SolverType(o.solver)
  fields['disableflags'] = types.DisableBit(o.disableflags)
  fields['enableflags'] = types.EnableBit(o.enableflags)
  fields['jacobian'] = types.JacobianType(o.jacobian)

  option_obj = {
      types.Impl.C: types.OptionC,
      types.Impl.JAX: types.OptionJAX,
      types.Impl.WARP: mjxw.types.OptionWarp,
  }[impl]
  private_fields = {
      f.name: getattr(o, f.name, None) for f in option_obj.fields()
  }
  impl_fields = impl_fields or {}
  impl_fields = {**private_fields, **impl_fields}

  if impl == types.Impl.JAX:
    has_fluid_params = o.density > 0 or o.viscosity > 0 or o.wind.any()
    implicitfast = o.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    if implicitfast and has_fluid_params:
      raise NotImplementedError('implicitfast not implemented for fluid drag.')
    impl_fields['has_fluid_params'] = has_fluid_params
    return types.Option(**fields, _impl=types.OptionJAX(**impl_fields))

  if impl == types.Impl.C:
    return types.Option(**fields, _impl=types.OptionC(**impl_fields))

  if impl == types.Impl.WARP:
    impl_fields = {k: _wp_to_np_type(v) for k, v in impl_fields.items()}
    return types.Option(**fields, _impl=mjxw.types.OptionWarp(**impl_fields))

  raise NotImplementedError(f'Unsupported implementation: {impl}')


def _put_statistic(
    s: mujoco.MjStatistic, impl: types.Impl
) -> Union[types.Statistic, types.StatisticWarp]:
  """Puts mujoco.MjStatistic onto a device, resulting in mjx.Statistic."""
  if impl == types.Impl.WARP:
    fields = {
        f.name: getattr(s, f.name, None) for f in types.StatisticWarp.fields()
    }
    return types.StatisticWarp(**fields)
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
  if m.nflex:
    raise NotImplementedError('Flex not implemented for JAX backend.')

  # contact sensor
  is_contact_sensor = m.sensor_type == types.SensorType.CONTACT
  if is_contact_sensor.any():
    objtype = m.sensor_objtype[is_contact_sensor]
    reftype = m.sensor_reftype[is_contact_sensor]
    contact_sensor_type = set(np.concatenate([objtype, reftype]))

    # site filter
    if types.ObjType.SITE in set(objtype):
      raise NotImplementedError(
          'Contact sensor with site matching semantics not implemented for JAX'
          ' backend.'
      )

    # body semantics
    if types.ObjType.BODY in contact_sensor_type:
      raise NotImplementedError(
          'Contact sensor with body matching semantics not implemented for JAX'
          ' backend.'
      )

    # subtree semantics
    if types.ObjType.XBODY in contact_sensor_type:
      raise NotImplementedError(
          'Contact sensor with subtree matching semantics not implemented for'
          ' JAX backend.'
      )

    # net force
    if (m.sensor_intprm[is_contact_sensor, 1] == 3).any():
      raise NotImplementedError(
          'Contact sensor with netforce reduction not implemented for JAX'
          ' backend.'
      )

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
  fields['stat'] = _put_statistic(m.stat, types.Impl.JAX)

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
  fields['stat'] = _put_statistic(m.stat, impl=types.Impl.C)

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


def _put_model_warp(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""
  if not mjxw.WARP_INSTALLED:
    raise RuntimeError('Warp not installed.')

  with wp.ScopedDevice('cpu'):  # pylint: disable=undefined-variable
    mw = mjwp.put_model(m)  # pylint: disable=undefined-variable

  fields = {f.name for f in types.Model.fields() if f.name != '_impl'}
  fields = {f: getattr(m, f) for f in fields}
  # Grab MJW private Option fields, and assume that public MjOption fields are
  # directly compatible with MJXW.
  option_keys = {f.name for f in mjxw.types.OptionWarp.fields()} - {
      f.name for f in types.Option.fields()
  }
  private_options = {k: getattr(mw.opt, k) for k in option_keys}
  fields['opt'] = _put_option(m.opt, types.Impl.WARP, private_options)
  fields['stat'] = _put_statistic(m.stat, types.Impl.WARP)

  # Use MJW fields directly instead of MjModel, so that shape and dtype are
  # always compatible with MJXW (e.g. cam_mat0/geom_aabb).
  for k in fields:
    if not hasattr(mw, k) or k in ('stat', 'opt'):
      continue
    field = _wp_to_np_type(getattr(mw, k), k)
    fields[k] = field

  impl_fields = {}
  for k in mjxw.types.ModelWarp.__annotations__.keys():
    field = _wp_to_np_type(getattr(mw, k), k)
    impl_fields[k] = field

  model = types.Model(
      **fields,
      _impl=mjxw.types.ModelWarp(**impl_fields),
  )

  model = jax.device_put(model, device=device)
  return _strip_weak_type(model)


def _put_model_cpp(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model."""

  mj_field_names = {f.name for f in types.Model.fields() if f.name != '_impl'}
  fields = {f: getattr(m, f) for f in mj_field_names}
  fields['cam_mat0'] = fields['cam_mat0'].reshape((-1, 3, 3))
  fields['opt'] = _put_option(m.opt, impl=types.Impl.C)
  fields['stat'] = _put_statistic(m.stat, impl=types.Impl.C)

  # get the pointer address
  # we use a 0-d array
  addr = m._address  # pytype: disable=attribute-error
  # To ensure that we retain the full pointer even if jax.config.enable_x64 is
  # set to True, we store the pointer as two 32-bit values. In the FFI call,
  # we combine the two values into a single pointer value.
  pointer_lo = jp.array(addr & 0xFFFFFFFF, dtype=jp.uint32)
  pointer_hi = jp.array(addr >> 32, dtype=jp.uint32)
  c_pointers_impl = types.ModelCPP(
      pointer_lo=pointer_lo,
      pointer_hi=pointer_hi,
      _model=m,
  )

  model = types.Model(
      **{k: copy.copy(v) for k, v in fields.items()}, _impl=c_pointers_impl
  )
  model = jax.device_put(model, device=device)
  return _strip_weak_type(model)


def put_model(
    m: mujoco.MjModel,
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
) -> types.Model:
  """Puts mujoco.MjModel onto a device, resulting in mjx.Model.

  Args:
    m: the model to put onto device
    device: which device to use - if unspecified picks the default device
    impl: implementation to use

  Returns:
    an mjx.Model placed on device

  Raises:
    ValueError: if impl is not supported
  """

  impl, device = _resolve_impl_and_device(impl, device)
  if impl == types.Impl.JAX:
    return _put_model_jax(m, device)
  elif impl == types.Impl.C:
    return _put_model_c(m, device)
  elif impl == types.Impl.WARP:
    return _put_model_warp(m, device)
  elif impl == types.Impl.CPP:
    return _put_model_cpp(m, device)
  else:
    raise ValueError(f'Unsupported implementation: {impl}')


def _make_data_public_fields(m: types.Model) -> Dict[str, Any]:
  """Create public fields for the Data object."""
  float_ = jp.zeros(1, float).dtype
  zero_fields = {
      'time': (float_,),
      'qvel': (m.nv, float_),
      'act': (m.na, float_),
      'plugin_state': (m.npluginstate, float_),
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
      'actuator_length': (m.nu, float_),
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
      'cdof': (m.nv, 6, float_),
      'cdof_dot': (m.nv, 6, float_),
      'ten_length': (m.ntendon, float_),
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
  dim = collision_driver.make_condim(m, impl=types.Impl.JAX)
  efc_type = constraint.make_efc_type(m, dim)
  ne, nf, nl, nc = constraint.counts(efc_type)
  ncon, nefc = dim.size, ne + nf + nl + nc
  efc_address = constraint.make_efc_address(m, dim, efc_type)

  float_ = jp.zeros(1, float).dtype
  int_ = np.int32
  contact = _make_data_contact_jax(dim, efc_address)

  if m.opt.cone == types.ConeType.ELLIPTIC and np.any(contact.dim == 1):
    raise NotImplementedError(
        'condim=1 with ConeType.ELLIPTIC not implemented.'
    )

  zero_impl_fields = {
      'solver_niter': (int_,),
      'cinert': (m.nbody, 10, float_),
      'ten_wrapadr': (m.ntendon, np.int32),
      'ten_wrapnum': (m.ntendon, np.int32),
      'ten_J': (m.ntendon, m.nv, float_),
      'wrap_obj': (m.nwrap, 2, np.int32),
      'wrap_xpos': (m.nwrap, 6, float_),
      'actuator_moment': (m.nu, m.nv, float_),
      'crb': (m.nbody, 10, float_),
      'qM': (m.nM, float_) if support.is_sparse(m) else (m.nv, m.nv, float_),
      'M': (m.nC, float_),
      'qLD': (m.nC, float_) if support.is_sparse(m) else (m.nv, m.nv, float_),
      'qLDiagInv': (m.nv, float_) if support.is_sparse(m) else (0, float_),
      'ten_velocity': (m.ntendon, float_),
      'actuator_velocity': (m.nu, float_),
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
  dim = collision_driver.make_condim(m, impl=types.Impl.C)
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
      'ten_wrapadr': (m.ntendon, np.int32),
      'ten_wrapnum': (m.ntendon, np.int32),
      'wrap_obj': (m.nwrap, 2, np.int32),
      'wrap_xpos': (m.nwrap, 6, float_),
      'moment_rownnz': (m.nu, np.int32),
      'moment_rowadr': (m.nu, np.int32),
      'moment_colind': (m.nJmom, np.int32),
      'actuator_moment': (m.nJmom, float_),
      'bvh_aabb_dyn': (nbvhdynamic, 6, float_),
      'bvh_active': (nbvh, np.uint8),
      'tree_asleep': (m.ntree, int_),
      'tree_awake': (m.ntree, int_),
      'body_awake': (m.nbody, int_),
      'body_awake_ind': (m.nbody, int_),
      'parent_awake_ind': (m.nbody, int_),
      'dof_awake_ind': (m.nv, int_),
      'tree_island': (m.ntree, int_),
      'map_itree2tree': (m.ntree, int_),
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
      'qDeriv': (m.nD, float_),
      'qLU': (m.nD, float_),
      'qfrc_spring': (m.nv, float_),
      'qfrc_damper': (m.nv, float_),
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


def _get_nested_attr(obj: Any, attr_name: str, split: str) -> Any:
  """Returns the nested attribute from an object."""
  for part in attr_name.split(split):
    obj = getattr(obj, part)
  return obj


def _make_data_warp(
    m: Union[types.Model, mujoco.MjModel],
    device: Optional[jax.Device] = None,
    naconmax: Optional[int] = None,
    njmax: Optional[int] = None,
) -> types.Data:
  """Allocate and initialize Data for the Warp implementation."""
  if not isinstance(m, mujoco.MjModel):
    raise ValueError(
        'make_data for warp, only supports a mujoco.MjModel input, got'
        f' {type(m)}.'
    )

  if not mjxw.WARP_INSTALLED:
    raise RuntimeError('Warp is not installed.')

  with wp.ScopedDevice('cpu'):  # pylint: disable=undefined-variable
    dw = mjwp.make_data(m, nworld=1, naconmax=naconmax, njmax=njmax)  # pylint: disable=undefined-variable

  fields = _make_data_public_fields(m)
  for k in fields:
    if k in {'userdata', 'plugin_state'}:
      continue
    if not hasattr(dw, k):
      raise ValueError(f'Public data field {k} not found in Warp data.')
    field = _wp_to_np_type(getattr(dw, k))
    if mjxw.types._BATCH_DIM['Data'][k]:  # pylint: disable=protected-access
      field = field.reshape(field.shape[1:])
    fields[k] = field

  impl_fields = {}
  for k in mjxw.types.DataWarp.__annotations__.keys():
    field = _get_nested_attr(dw, k, split='__')
    field = _wp_to_np_type(field)
    if mjxw.types._BATCH_DIM['Data'][k]:  # pylint: disable=protected-access
      field = field.reshape(field.shape[1:])
    impl_fields[k] = field

  data = types.Data(
      qpos=m.qpos0.astype(np.float32),
      eq_active=m.eq_active0.astype(bool),
      **fields,
      _impl=mjxw.types.DataWarp(**impl_fields),
  )

  data = jax.device_put(data, device=device)

  with wp.ScopedDevice('cuda:0'):  # pylint: disable=undefined-variable
    # Warm-up the warp kernel cache.
    # TODO(robotics-simulation): remove this warmup compilation once warp
    # stops unloading modules during XLA graph capture for tile kernels.
    # pylint: disable=undefined-variable
    dw = mjwp.make_data(m, nworld=1, naconmax=naconmax, njmax=njmax)
    mw = mjwp.put_model(m)
    _ = mjwp.step(mw, dw)
    # pylint: enable=undefined-variable
  del dw, mw

  return data


def make_data(
    m: Union[types.Model, mujoco.MjModel],
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
    _full_compat: bool = False,  # pylint: disable=invalid-name
    nconmax: Optional[int] = None,
    naconmax: Optional[int] = None,
    njmax: Optional[int] = None,
) -> types.Data:
  """Allocate and initialize Data.

  Args:
    m: the model to use
    device: which device to use - if unspecified picks the default device
    impl: implementation to use ('jax', 'warp')
    nconmax: maximum number of contacts to allocate for warp across all worlds
      Since the number of worlds is **not** pre-defined in JAX, we use the
      `nconmax` argument to set the upper bound for the number of contacts
      across all worlds. In MuJoCo Warp, the analgous field is called
      `naconmax`.
    naconmax: maximum number of contacts to allocate for warp across all worlds
      Since the number of worlds is **not** pre-defined in JAX, we use the
      `naconmax` argument to set the upper bound for the number of contacts
      across all worlds, rather than the `nconmax` argument from MuJoCo Warp.
    njmax: maximum number of constraints to allocate for warp across all worlds

  Returns:
    an initialized mjx.Data placed on device

  Raises:
    ValueError: if the model's impl does not match the make_data impl
    NotImplementedError: if the impl is not implemented yet
    DeprecationWarning: if nconmax is used
  """
  if nconmax is not None:
    warnings.warn(
        'nconmax will be deprecated in mujoco-mjx>=3.5. Use naconmax instead.',
        DeprecationWarning,
        stacklevel=2,
    )

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
  elif impl == types.Impl.WARP:
    naconmax = nconmax if naconmax is None else naconmax
    return _make_data_warp(m, device, naconmax, njmax)

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
  dim = collision_driver.make_condim(m, impl=types.Impl.JAX)
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
  dim = collision_driver.make_condim(m, impl=types.Impl.C)
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

  # TODO(stunya): remove reliance on JAX _put_contact.
  contact, contact_map = _put_contact(d.contact, dim, efc_address)

  # TODO(stunya): remove reliance on dense efc_J.
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


def _put_data_cpp(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    device: Optional[jax.Device] = None,
    dummy_arg_for_batching: Optional[jax.Array] = None,
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data."""

  data_list = []

  def _copy_and_get_addr(unused_jax_array):
    # We use the input to the callback as a dummy dependency to ensure
    # io_callback runs for each element in the batch.
    try:
      new_d = mujoco.MjData(m)
    except mujoco.FatalError as e:
      raise ValueError('Failed to create new MjData') from e
    mujoco.mj_copyState(m, d, new_d, int(mujoco.mjtState.mjSTATE_FULLPHYSICS))
    mujoco.mj_forward(m, new_d)
    data_list.append(new_d)
    addr = new_d._address
    # To ensure that we retain the full pointer even if jax.config.enable_x64 is
    # set to True, we store the pointer as two 32-bit values. In the FFI call,
    # we combine the two values into a single pointer value.
    return (
        np.array(addr & 0xFFFFFFFF, dtype=np.uint32),
        np.array(addr >> 32, dtype=np.uint32),
    )

  # Pass a dummy dependency to ensure io_callback runs across the batch.
  pointer_lo, pointer_hi = jax.experimental.io_callback(
      _copy_and_get_addr,
      (
          jax.ShapeDtypeStruct((), jp.uint32),
          jax.ShapeDtypeStruct((), jp.uint32),
      ),
      dummy_arg_for_batching,
  )

  new_d = data_list[0]
  fields = _put_data_public_fields(new_d)

  c_pointers_impl = types.DataCPP(
      pointer_lo=pointer_lo,
      pointer_hi=pointer_hi,
      _data=data_list,
  )

  data = types.Data(
      _impl=c_pointers_impl,
      **fields,
  )
  data = jax.device_put(data, device=device)
  return _strip_weak_type(data)


def put_data(
    m: mujoco.MjModel,
    d: mujoco.MjData,
    device: Optional[jax.Device] = None,
    impl: Optional[Union[str, types.Impl]] = None,
    nconmax: Optional[int] = None,
    naconmax: Optional[int] = None,
    njmax: Optional[int] = None,
    dummy_arg_for_batching: Optional[jax.Array] = None,
) -> types.Data:
  """Puts mujoco.MjData onto a device, resulting in mjx.Data.

  Args:
    m: the model to use
    d: the data to put on device
    device: which device to use - if unspecified picks the default device
    impl: implementation to use ('jax', 'warp')
    nconmax: maximum number of contacts to allocate for warp
    naconmax: maximum number of contacts to allocate for warp across all worlds
      Since the number of worlds is **not** pre-defined in JAX, we use the
      `naconmax` argument to set the upper bound for the number of contacts
      across all worlds, rather than the `nconmax` argument from MuJoCo Warp.
    njmax: maximum number of constraints to allocate for warp
    dummy_arg_for_batching: dummy argument to use for batching in cpp
      implementation

  Returns:
    an mjx.Data placed on device
    DeprecationWarning: if nconmax is used
  """
  del njmax
  if nconmax is not None:
    warnings.warn(
        'nconmax will be deprecated in mujoco-mjx>=3.5. Use naconmax instead.',
        DeprecationWarning,
        stacklevel=2,
    )

  impl, device = _resolve_impl_and_device(impl, device)
  if impl == types.Impl.JAX:
    return _put_data_jax(m, d, device)
  elif impl == types.Impl.C:
    return _put_data_c(m, d, device)
  elif impl == types.Impl.CPP:
    return _put_data_cpp(
        m, d, device, dummy_arg_for_batching=dummy_arg_for_batching
    )

  # TODO(robotics-team): implement put_data_warp

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


def _get_data_into_warp(
    result: Union[mujoco.MjData, List[mujoco.MjData]],
    m: mujoco.MjModel,
    d: types.Data,
):
  """Gets mjx.Data from a device into an existing mujoco.MjData or list."""
  batched = isinstance(result, list)
  d = jax.device_get(d)
  batch_size = d.qpos.shape[0] if batched else 1

  for i in range(batch_size):
    d_i = (
        jax.tree.map_with_path(
            lambda path, x, i=i: x[i]
            if path[-1].name not in mjxw.types.DATA_NON_VMAP
            else x,
            d,
        )
        if batched
        else d
    )
    result_i = result[i] if batched else result
    ncon = d_i._impl.nacon[0]
    nefc = int(d_i._impl.nefc)
    # nj = int(d_i._impl.nj[0])
    nj = 0  # TODO(btaba): add nj back

    if ncon != result_i.ncon or nefc != result_i.nefc or nj != result_i.nJ:
      mujoco._functions._realloc_con_efc(result_i, ncon=ncon, nefc=nefc, nJ=nj)  # pylint: disable=protected-access

    all_fields = types.Data.fields() + mjxw.types.DataWarp.fields()
    for field in all_fields:
      if field.name not in mujoco.MjData.__dict__.keys():
        continue

      # TODO(btaba): contact
      # TODO(btaba): actuator_moment

      if hasattr(d_i._impl, field.name):
        value = getattr(d_i._impl, field.name)
      else:
        value = getattr(d_i, field.name)

      if field.name in ('ne', 'nl', 'nf'):
        pass
      elif field.name in ('nefc', 'ncon'):
        value = {'nefc': nefc, 'ncon': ncon}[field.name]
      elif field.name.endswith('xmat') or field.name == 'ximat':
        value = value.reshape((-1, 9))
      # elif field.name == 'efc_J':  # TODO(btaba): add this back
      # elif field.name.startswith('efc_'):  # TODO(btaba): add this back
      # TODO(btaba): qM, qLD, qLDiagInv

      if field.name in (
          'actuator_moment',
          'contact',
          'efc_J',
          'qM',
          'qLD',
          'qLDiagInv',
      ):
        continue
      if field.name.startswith('efc_'):
        continue

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

    # TODO(btaba): add M back
    # mujoco.mj_factorM(m, result_i)


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
          if d_i.impl == types.Impl.JAX:
            mujoco.mju_dense2sparse(
                actuator_moment,
                d_i._impl.actuator_moment,
                moment_rownnz,
                moment_rowadr,
                moment_colind,
            )
          else:
            actuator_moment = d_i._impl.actuator_moment
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
        elif field.name == 'qLD':
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
    result_i.M[:] = result_i.qM[m.mapM2M]

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

  if d.impl == types.Impl.WARP:
    return _get_data_into_warp(result, m, d)

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


_STATE_MAP = {
    mujoco.mjtState.mjSTATE_TIME: 'time',
    mujoco.mjtState.mjSTATE_QPOS: 'qpos',
    mujoco.mjtState.mjSTATE_QVEL: 'qvel',
    mujoco.mjtState.mjSTATE_ACT: 'act',
    mujoco.mjtState.mjSTATE_WARMSTART: 'qacc_warmstart',
    mujoco.mjtState.mjSTATE_CTRL: 'ctrl',
    mujoco.mjtState.mjSTATE_QFRC_APPLIED: 'qfrc_applied',
    mujoco.mjtState.mjSTATE_XFRC_APPLIED: 'xfrc_applied',
    mujoco.mjtState.mjSTATE_EQ_ACTIVE: 'eq_active',
    mujoco.mjtState.mjSTATE_MOCAP_POS: 'mocap_pos',
    mujoco.mjtState.mjSTATE_MOCAP_QUAT: 'mocap_quat',
    mujoco.mjtState.mjSTATE_USERDATA: 'userdata',
    mujoco.mjtState.mjSTATE_PLUGIN: 'plugin_state',
}


def _state_elem_size(m: types.Model, state_enum: mujoco.mjtState) -> int:
  """Returns the size of a state component."""
  if state_enum not in _STATE_MAP:
    raise ValueError(f'Invalid state element {state_enum}')
  name = _STATE_MAP[state_enum]
  if name == 'time':
    return 1
  if name in (
      'qpos',
      'qvel',
      'act',
      'qacc_warmstart',
      'ctrl',
      'qfrc_applied',
      'eq_active',
      'mocap_pos',
      'mocap_quat',
      'userdata',
      'plugin_state',
  ):
    val = getattr(
        m,
        {
            'qpos': 'nq',
            'qvel': 'nv',
            'act': 'na',
            'qacc_warmstart': 'nv',
            'ctrl': 'nu',
            'qfrc_applied': 'nv',
            'eq_active': 'neq',
            'mocap_pos': 'nmocap',
            'mocap_quat': 'nmocap',
            'userdata': 'nuserdata',
            'plugin_state': 'npluginstate',
        }[name],
    )
    if name == 'mocap_pos':
      val *= 3
    if name == 'mocap_quat':
      val *= 4
    return val
  if name == 'xfrc_applied':
    return 6 * m.nbody

  raise NotImplementedError(f'state component {name} not implemented')


def state_size(m: types.Model, spec: Union[int, mujoco.mjtState]) -> int:
  """Returns the size of a state vector for a given spec.

  Args:
    m: model describing the simulation
    spec: int bitmask or mjtState enum specifying which state components to
      include

  Returns:
    size of the state vector
  """
  size = 0
  spec_int = int(spec)
  for i in range(mujoco.mjtState.mjNSTATE.value):
    element = mujoco.mjtState(1 << i)
    if element & spec_int:
      size += _state_elem_size(m, element)
  return size


def get_state(
    m: types.Model, d: types.Data, spec: Union[int, mujoco.mjtState]
) -> jax.Array:
  """Gets state from mjx.Data. This is equivalent to `mujoco.mj_getState`.

  Args:
    m: model describing the simulation
    d: data for the simulation
    spec: int bitmask or mjtState enum specifying which state components to
      include

  Returns:
    a flat array of state values
  """
  spec_int = int(spec)
  if spec_int >= (1 << mujoco.mjtState.mjNSTATE.value):
    raise ValueError(f'Invalid state spec {spec}')

  state = []
  for i in range(mujoco.mjtState.mjNSTATE.value):
    element = mujoco.mjtState(1 << i)
    if element & spec_int:
      if element not in _STATE_MAP:
        raise ValueError(f'Invalid state element {element}')
      name = _STATE_MAP[element]
      value = getattr(d, name)
      if element == mujoco.mjtState.mjSTATE_EQ_ACTIVE:
        value = value.astype(jp.float32)
      state.append(value.flatten())

  return jp.concatenate(state) if state else jp.array([])


def set_state(
    m: types.Model,
    d: types.Data,
    state: jax.Array,
    spec: Union[int, mujoco.mjtState],
) -> types.Data:
  """Sets state in mjx.Data. This is equivalent to `mujoco.mj_setState`.

  Args:
    m: model describing the simulation
    d: data for the simulation
    state: a flat array of state values
    spec: int bitmask or mjtState enum specifying which state components to
      include

  Returns:
    data with state set to provided values
  """
  spec_int = int(spec)
  if spec_int >= (1 << mujoco.mjtState.mjNSTATE.value):
    raise ValueError(f'Invalid state spec {spec}')

  expected_size = state_size(m, spec)
  if state.size != expected_size:
    raise ValueError(
        f'state has size {state.size} but expected {expected_size}'
    )

  updates = {}
  offset = 0
  for i in range(mujoco.mjtState.mjNSTATE.value):
    element = mujoco.mjtState(1 << i)
    if element & spec_int:
      if element not in _STATE_MAP:
        raise ValueError(f'Invalid state element {element}')
      name = _STATE_MAP[element]
      size = _state_elem_size(m, element)
      value = state[offset : offset + size]
      if name == 'time':
        value = value[0]
      else:
        orig_shape = getattr(d, name).shape
        value = value.reshape(orig_shape)
      if element == mujoco.mjtState.mjSTATE_EQ_ACTIVE:
        value = value.astype(bool)
      updates[name] = value
      offset += size

  return d.replace(**updates)
