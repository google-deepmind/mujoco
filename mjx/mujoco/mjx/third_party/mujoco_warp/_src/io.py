# Copyright 2025 The Newton Developers
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

import dataclasses
from typing import Any, Optional, Sequence, Union

import mujoco
import numpy as np
import warp as wp

from mujoco.mjx.third_party.mujoco_warp._src import types
from mujoco.mjx.third_party.mujoco_warp._src import warp_util
from mujoco.mjx.third_party.mujoco_warp._src.warp_util import nested_kernel


def _create_array(data: Any, spec: wp.array, sizes: dict[str, int]) -> Union[wp.array, None]:
  """Creates a warp array and populates it with data.

  The array shape is determined by a field spec referencing MjModel / MjData array sizes.
  """
  shape = None
  if spec.shape != (0,):
    shape = tuple(sizes[dim] if isinstance(dim, str) else dim for dim in spec.shape)

  if data is None and shape is None:
    return None  # nothing to do
  elif data is None:
    array = wp.zeros(shape, dtype=spec.dtype)
  else:
    array = wp.array(np.array(data), dtype=spec.dtype, shape=shape)

  if spec.shape[0] == "*":
    # add private attribute for JAX to determine which fields are batched
    array._is_batched = True
    # also set stride 0 to 0 which is expected legacy behavior (but is deprecated)
    array.strides = (0,) + array.strides[1:]
  return array


def is_sparse(mjm: mujoco.MjModel) -> bool:
  if mjm.opt.jacobian == mujoco.mjtJacobian.mjJAC_AUTO:
    if mjm.nv > 32:
      return True
    else:
      return False
  else:
    return bool(mujoco.mj_isSparse(mjm))


def put_model(mjm: mujoco.MjModel) -> types.Model:
  """Creates a model on device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).

  Returns:
    The model containing kinematic and dynamic information (device).
  """
  # check for compatible cuda toolkit and driver versions
  warp_util.check_toolkit_driver()

  # model: check supported features in array types
  for field, field_type in (
    (mjm.actuator_trntype, types.TrnType),
    (mjm.actuator_dyntype, types.DynType),
    (mjm.actuator_gaintype, types.GainType),
    (mjm.actuator_biastype, types.BiasType),
    (mjm.eq_type, types.EqType),
    (mjm.geom_type, types.GeomType),
    (mjm.sensor_type, types.SensorType),
    (mjm.wrap_type, types.WrapType),
  ):
    missing = ~np.isin(field, field_type)
    if missing.any():
      raise NotImplementedError(f"{field_type.__name__}: {field[missing]} not supported.")

  # opt: check supported features in scalar types
  for field, field_type in (
    (mjm.opt.integrator, types.IntegratorType),
    (mjm.opt.cone, types.ConeType),
    (mjm.opt.solver, types.SolverType),
  ):
    if field not in set(field_type):
      raise NotImplementedError(f"{field_type.__name__} {field} is unsupported.")

  # opt: check supported features in scalar flag types
  for field, field_type in (
    (mjm.opt.disableflags, types.DisableBit),
    (mjm.opt.enableflags, types.EnableBit),
  ):
    if field & ~np.bitwise_or.reduce(field_type):
      raise NotImplementedError(f"{field_type.__name__} {field} is unsupported.")

  if ((mjm.flex_contype != 0) | (mjm.flex_conaffinity != 0)).any():
    raise NotImplementedError("Flex collisions are not implemented.")

  if mjm.opt.noslip_iterations > 0:
    raise NotImplementedError(f"noslip solver not implemented.")

  if (mjm.opt.viscosity > 0 or mjm.opt.density > 0) and mjm.opt.integrator in (
    mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    mujoco.mjtIntegrator.mjINT_IMPLICIT,
  ):
    raise NotImplementedError(f"Implicit integrators and fluid model not implemented.")

  if (mjm.body_plugin != -1).any():
    raise NotImplementedError("Body plugins not supported.")

  if (mjm.actuator_plugin != -1).any():
    raise NotImplementedError("Actuator plugins not supported.")

  if (mjm.sensor_plugin != -1).any():
    raise NotImplementedError("Sensor plugins not supported.")

  # TODO(team): remove after _update_gradient for Newton uses tile operations for islands
  nv_max = 60
  if mjm.nv > nv_max and mjm.opt.jacobian == mujoco.mjtJacobian.mjJAC_DENSE:
    raise ValueError(f"Dense is unsupported for nv > {nv_max} (nv = {mjm.nv}).")

  collision_sensors = (mujoco.mjtSensor.mjSENS_GEOMDIST, mujoco.mjtSensor.mjSENS_GEOMNORMAL, mujoco.mjtSensor.mjSENS_GEOMFROMTO)
  is_collision_sensor = np.isin(mjm.sensor_type, collision_sensors)

  def not_implemented(objtype, objid, geomtype):
    if objtype == mujoco.mjtObj.mjOBJ_BODY:
      geomnum = mjm.body_geomnum[objid]
      geomadr = mjm.body_geomadr[objid]
      for geomid in range(geomadr, geomadr + geomnum):
        if mjm.geom_type[geomid] == geomtype:
          return True
    elif objtype == mujoco.mjtObj.mjOBJ_GEOM:
      if mjm.geom_type[objid] == geomtype:
        return True
    return False

  for objtype, objid, reftype, refid in zip(
    mjm.sensor_objtype[is_collision_sensor],
    mjm.sensor_objid[is_collision_sensor],
    mjm.sensor_reftype[is_collision_sensor],
    mjm.sensor_refid[is_collision_sensor],
  ):
    if not_implemented(objtype, objid, types.GeomType.BOX) and not_implemented(reftype, refid, types.GeomType.BOX):
      raise NotImplementedError(f"Collision sensors with box-box collisions are not implemented.")

  # create opt
  opt_kwargs = {f.name: getattr(mjm.opt, f.name, None) for f in dataclasses.fields(types.Option)}
  if hasattr(mjm.opt, "impratio"):
    opt_kwargs["impratio_invsqrt"] = 1.0 / np.sqrt(np.maximum(mjm.opt.impratio, mujoco.mjMINVAL))
  opt = types.Option(**opt_kwargs)

  # C MuJoCo tolerance was chosen for float64 architecture, but we default to float32 on GPU
  # adjust the tolerance for lower precision, to avoid the solver spending iterations needlessly
  # bouncing around the optimal solution
  opt.tolerance = max(opt.tolerance, 1e-6)

  # warp only fields
  opt.is_sparse = is_sparse(mjm)
  ls_parallel_id = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_NUMERIC, "ls_parallel")
  opt.ls_parallel = (ls_parallel_id > -1) and (mjm.numeric_data[mjm.numeric_adr[ls_parallel_id]] == 1)
  opt.ls_parallel_min_step = 1.0e-6  # TODO(team): determine good default setting
  opt.has_fluid = mjm.opt.wind.any() or mjm.opt.density > 0 or mjm.opt.viscosity > 0
  opt.broadphase = types.BroadphaseType.NXN
  opt.broadphase_filter = types.BroadphaseFilter.PLANE | types.BroadphaseFilter.SPHERE | types.BroadphaseFilter.OBB
  opt.graph_conditional = True
  opt.run_collision_detection = True
  contact_sensor_maxmatch_id = mujoco.mj_name2id(mjm, mujoco.mjtObj.mjOBJ_NUMERIC, "contact_sensor_maxmatch")
  if contact_sensor_maxmatch_id > -1:
    opt.contact_sensor_maxmatch = mjm.numeric_data[mjm.numeric_adr[contact_sensor_maxmatch_id]]
  else:
    opt.contact_sensor_maxmatch = 64

  # place opt on device
  for f in dataclasses.fields(types.Option):
    if isinstance(f.type, wp.array):
      setattr(opt, f.name, _create_array(getattr(opt, f.name), f.type, {"*": 1}))
    else:
      setattr(opt, f.name, f.type(getattr(opt, f.name)))

  # create stat
  stat = types.Statistic(meaninertia=mjm.stat.meaninertia)

  # create model
  m = types.Model(**{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model)})

  m.opt = opt
  m.stat = stat

  m.nv_pad = _get_padded_sizes(
    mjm.nv, 0, is_sparse(mjm), types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  )[1]
  m.nacttrnbody = (mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY).sum()
  m.nsensortaxel = mjm.mesh_vertnum[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_TACTILE]].sum()
  m.nsensorcontact = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT).sum()
  m.nrangefinder = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER).sum()
  m.nmaxcondim = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  m.nmaxpyramid = np.maximum(1, 2 * (m.nmaxcondim - 1))
  m.has_sdf_geom = (mjm.geom_type == mujoco.mjtGeom.mjGEOM_SDF).any()
  m.block_dim = types.BlockDim()

  # body ids grouped by tree level
  bodies, body_depth = {}, np.zeros(mjm.nbody, dtype=int) - 1
  for i in range(mjm.nbody):
    body_depth[i] = body_depth[mjm.body_parentid[i]] + 1
    bodies.setdefault(body_depth[i], []).append(i)
  m.body_tree = tuple(wp.array(bodies[i], dtype=int) for i in sorted(bodies))

  m.mocap_bodyid = np.arange(mjm.nbody)[mjm.body_mocapid >= 0]
  m.mocap_bodyid = m.mocap_bodyid[mjm.body_mocapid[mjm.body_mocapid >= 0].argsort()]
  m.body_fluid_ellipsoid = np.zeros(mjm.nbody, dtype=bool)
  m.body_fluid_ellipsoid[mjm.geom_bodyid[mjm.geom_fluid.reshape(mjm.ngeom, mujoco.mjNFLUID)[:, 0] > 0]] = True
  jnt_limited_slide_hinge = mjm.jnt_limited & np.isin(mjm.jnt_type, (mujoco.mjtJoint.mjJNT_SLIDE, mujoco.mjtJoint.mjJNT_HINGE))
  m.jnt_limited_slide_hinge_adr = np.nonzero(jnt_limited_slide_hinge)[0]
  m.jnt_limited_ball_adr = np.nonzero(mjm.jnt_limited & (mjm.jnt_type == mujoco.mjtJoint.mjJNT_BALL))[0]
  m.dof_tri_row, m.dof_tri_col = np.tril_indices(mjm.nv)

  # precalculated geom pairs
  filterparent = not (mjm.opt.disableflags & types.DisableBit.FILTERPARENT)

  geom1, geom2 = np.triu_indices(mjm.ngeom, k=1)
  m.nxn_geom_pair = np.stack((geom1, geom2), axis=1)

  bodyid1 = mjm.geom_bodyid[geom1]
  bodyid2 = mjm.geom_bodyid[geom2]
  contype1 = mjm.geom_contype[geom1]
  contype2 = mjm.geom_contype[geom2]
  conaffinity1 = mjm.geom_conaffinity[geom1]
  conaffinity2 = mjm.geom_conaffinity[geom2]
  weldid1 = mjm.body_weldid[bodyid1]
  weldid2 = mjm.body_weldid[bodyid2]
  weld_parentid1 = mjm.body_weldid[mjm.body_parentid[weldid1]]
  weld_parentid2 = mjm.body_weldid[mjm.body_parentid[weldid2]]

  self_collision = weldid1 == weldid2
  parent_child_collision = (
    filterparent & (weldid1 != 0) & (weldid2 != 0) & ((weldid1 == weld_parentid2) | (weldid2 == weld_parentid1))
  )
  mask = np.array((contype1 & conaffinity2) | (contype2 & conaffinity1), dtype=bool)
  exclude = np.isin((bodyid1 << 16) + bodyid2, mjm.exclude_signature)

  nxn_pairid_contact = -1 * np.ones(len(geom1), dtype=int)
  nxn_pairid_contact[~(mask & ~self_collision & ~parent_child_collision & ~exclude)] = -2

  # contact pairs
  def upper_tri_index(n, i, j):
    i, j = (j, i) if j < i else (i, j)
    return (i * (2 * n - i - 3)) // 2 + j - 1

  for i in range(mjm.npair):
    nxn_pairid_contact[upper_tri_index(mjm.ngeom, mjm.pair_geom1[i], mjm.pair_geom2[i])] = i

  sensor_collision_adr = np.nonzero(is_collision_sensor)[0]
  collision_sensor_adr = np.full(mjm.nsensor, -1)
  collision_sensor_adr[sensor_collision_adr] = np.arange(len(sensor_collision_adr))

  nxn_pairid_collision = -1 * np.ones(len(geom1), dtype=int)
  pairids = []
  sensor_collision_start_adr = []
  for i in range(sensor_collision_adr.size):
    sensorid = sensor_collision_adr[i]
    objtype = mjm.sensor_objtype[sensorid]
    objid = mjm.sensor_objid[sensorid]
    reftype = mjm.sensor_reftype[sensorid]
    refid = mjm.sensor_refid[sensorid]

    # get lists of geoms to collide
    if objtype == types.ObjType.BODY:
      n1 = mjm.body_geomnum[objid]
      id1 = mjm.body_geomadr[objid]
    else:
      n1 = 1
      id1 = objid
    if reftype == types.ObjType.BODY:
      n2 = mjm.body_geomnum[refid]
      id2 = mjm.body_geomadr[refid]
    else:
      n2 = 1
      id2 = refid

    # collide all pairs
    for geom1id in range(id1, id1 + n1):
      for geom2id in range(id2, id2 + n2):
        pairid = upper_tri_index(mjm.ngeom, geom1id, geom2id)

        if pairid in pairids:
          sensor_collision_start_adr.append(nxn_pairid_collision[pairid])
        else:
          npairids = len(pairids)
          nxn_pairid_collision[pairid] = npairids
          sensor_collision_start_adr.append(npairids)
          pairids.append(pairid)

  m.nsensorcollision = (nxn_pairid_collision >= 0).sum()
  m.sensor_collision_start_adr = np.array(sensor_collision_start_adr)
  nxn_include = (nxn_pairid_contact > -2) | (nxn_pairid_collision >= 0)

  if nxn_include.sum() < 250_000:
    opt.broadphase = types.BroadphaseType.NXN
  elif mjm.ngeom < 1000:
    opt.broadphase = types.BroadphaseType.SAP_TILE
  else:
    opt.broadphase = types.BroadphaseType.SAP_SEGMENTED

  m.nxn_geom_pair_filtered = m.nxn_geom_pair[nxn_include]
  m.nxn_pairid = np.hstack([nxn_pairid_contact.reshape((-1, 1)), nxn_pairid_collision.reshape((-1, 1))])
  m.nxn_pairid_filtered = m.nxn_pairid[nxn_include]

  # count contact pair types
  def geom_trid_index(i, j):
    i, j = (j, i) if j < i else (i, j)
    return (i * (2 * len(types.GeomType) - i - 1)) // 2 + j

  m.geom_pair_type_count = tuple(
    np.bincount(
      [geom_trid_index(mjm.geom_type[geom1[i]], mjm.geom_type[geom2[i]]) for i in np.arange(len(geom1)) if nxn_include[i]],
      minlength=len(types.GeomType) * (len(types.GeomType) + 1) // 2,
    )
  )

  # compute nmaxpolygon and nmaxmeshdeg given the geom pairs for the model
  nboxbox = m.geom_pair_type_count[geom_trid_index(types.GeomType.BOX, types.GeomType.BOX)]
  nboxmesh = m.geom_pair_type_count[geom_trid_index(types.GeomType.BOX, types.GeomType.MESH)]
  nmeshmesh = m.geom_pair_type_count[geom_trid_index(types.GeomType.MESH, types.GeomType.MESH)]
  # need at least 4 (square sides) if there's a box collision needing multiccd
  m.nmaxpolygon = 4 * (nboxbox + nboxmesh > 0)
  m.nmaxmeshdeg = 3 * (nboxbox + nboxmesh > 0)
  # possibly need to allocate more memory if there's meshes
  if nmeshmesh + nboxmesh > 0:
    # TODO(kbayes): remove nboxbox or enable ccd for box-box collisions
    m.nmaxpolygon = np.append(mjm.mesh_polyvertnum, m.nmaxpolygon).max()
    m.nmaxmeshdeg = np.append(mjm.mesh_polymapnum, m.nmaxmeshdeg).max()

  # filter plugins for only geom plugins, drop the rest
  m.plugin, m.plugin_attr = [], []
  m.geom_plugin_index = np.full_like(mjm.geom_type, -1)

  for i in range(len(mjm.geom_plugin)):
    if mjm.geom_plugin[i] == -1:
      continue
    p = mjm.geom_plugin[i]
    m.geom_plugin_index[i] = len(m.plugin)
    m.plugin.append(mjm.plugin[p])
    start = mjm.plugin_attradr[p]
    end = mjm.plugin_attradr[p + 1] if p + 1 < mjm.nplugin else len(mjm.plugin_attr)
    values = mjm.plugin_attr[start:end]
    attr_values = []
    current = []
    for v in values:
      if v == 0:
        if current:
          s = "".join(chr(int(x)) for x in current)
          attr_values.append(float(s))
          current = []
      else:
        current.append(v)
    # Pad with zeros if less than 3
    attr_values += [0.0] * (3 - len(attr_values))
    m.plugin_attr.append(attr_values[:3])

  # equality constraint addresses
  m.eq_connect_adr = np.nonzero(mjm.eq_type == types.EqType.CONNECT)[0]
  m.eq_wld_adr = np.nonzero(mjm.eq_type == types.EqType.WELD)[0]
  m.eq_jnt_adr = np.nonzero(mjm.eq_type == types.EqType.JOINT)[0]
  m.eq_ten_adr = np.nonzero(mjm.eq_type == types.EqType.TENDON)[0]
  m.eq_flex_adr = np.nonzero(mjm.eq_type == types.EqType.FLEX)[0]

  # fixed tendon
  m.tendon_jnt_adr, m.wrap_jnt_adr = [], []
  for i in range(mjm.ntendon):
    adr = mjm.tendon_adr[i]
    if mjm.wrap_type[adr] == mujoco.mjtWrap.mjWRAP_JOINT:
      tendon_num = mjm.tendon_num[i]
      for j in range(tendon_num):
        m.tendon_jnt_adr.append(i)
        m.wrap_jnt_adr.append(adr + j)

  # spatial tendon
  m.tendon_site_pair_adr, m.tendon_geom_adr = [], []
  m.ten_wrapadr_site, m.ten_wrapnum_site = [0], []
  for i, tendon_num in enumerate(mjm.tendon_num):
    adr = mjm.tendon_adr[i]
    # sites
    if (mjm.wrap_type[adr : adr + tendon_num] == mujoco.mjtWrap.mjWRAP_SITE).all():
      if i < mjm.ntendon:
        m.ten_wrapadr_site.append(m.ten_wrapadr_site[-1] + tendon_num)
      m.ten_wrapnum_site.append(tendon_num)
    else:
      if i < mjm.ntendon:
        m.ten_wrapadr_site.append(m.ten_wrapadr_site[-1])
      m.ten_wrapnum_site.append(0)

    # geoms
    for j in range(tendon_num):
      wrap_type = mjm.wrap_type[adr + j]
      if j < tendon_num - 1:
        next_wrap_type = mjm.wrap_type[adr + j + 1]
        if wrap_type == mujoco.mjtWrap.mjWRAP_SITE and next_wrap_type == mujoco.mjtWrap.mjWRAP_SITE:
          m.tendon_site_pair_adr.append(i)
      if wrap_type == mujoco.mjtWrap.mjWRAP_SPHERE or wrap_type == mujoco.mjtWrap.mjWRAP_CYLINDER:
        m.tendon_geom_adr.append(i)

  m.tendon_limited_adr = np.nonzero(mjm.tendon_limited)[0]
  m.wrap_site_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_SITE)[0]
  m.wrap_site_pair_adr = np.setdiff1d(m.wrap_site_adr[np.nonzero(np.diff(m.wrap_site_adr) == 1)[0]], mjm.tendon_adr[1:] - 1)
  m.wrap_geom_adr = np.nonzero(np.isin(mjm.wrap_type, [mujoco.mjtWrap.mjWRAP_SPHERE, mujoco.mjtWrap.mjWRAP_CYLINDER]))[0]

  # pulley scaling
  m.wrap_pulley_scale = np.ones(mjm.nwrap, dtype=float)
  pulley_adr = np.nonzero(mjm.wrap_type == mujoco.mjtWrap.mjWRAP_PULLEY)[0]
  for tadr, tnum in zip(mjm.tendon_adr, mjm.tendon_num):
    for padr in pulley_adr:
      if tadr <= padr < tadr + tnum:
        m.wrap_pulley_scale[padr : tadr + tnum] = 1.0 / mjm.wrap_prm[padr]

  m.actuator_trntype_body_adr = np.nonzero(mjm.actuator_trntype == mujoco.mjtTrn.mjTRN_BODY)[0]

  # sensor addresses
  m.sensor_pos_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_POS)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITPOS)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
  )[0]
  m.sensor_limitpos_adr = np.nonzero(
    (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITPOS) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITPOS)
  )[0]
  m.sensor_vel_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_VEL)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITVEL)
    & (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
  )[0]
  m.sensor_limitvel_adr = np.nonzero(
    (mjm.sensor_type == mujoco.mjtSensor.mjSENS_JOINTLIMITVEL) | (mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONLIMITVEL)
  )[0]
  m.sensor_acc_adr = np.nonzero(
    (mjm.sensor_needstage == mujoco.mjtStage.mjSTAGE_ACC)
    & (
      (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TOUCH)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_JOINTLIMITFRC)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
      | (mjm.sensor_type != mujoco.mjtSensor.mjSENS_TENDONACTFRC)
    )
  )[0]
  m.sensor_rangefinder_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER)[0]
  m.rangefinder_sensor_adr = np.full(mjm.nsensor, -1)
  m.rangefinder_sensor_adr[m.sensor_rangefinder_adr] = np.arange(len(m.sensor_rangefinder_adr))
  m.collision_sensor_adr = np.full(mjm.nsensor, -1)
  m.collision_sensor_adr[sensor_collision_adr] = np.arange(len(sensor_collision_adr))
  m.sensor_touch_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TOUCH)[0]
  limitfrc_sensors = (mujoco.mjtSensor.mjSENS_JOINTLIMITFRC, mujoco.mjtSensor.mjSENS_TENDONLIMITFRC)
  m.sensor_limitfrc_adr = np.nonzero(np.isin(mjm.sensor_type, limitfrc_sensors))[0]
  m.sensor_e_potential = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_POTENTIAL).any()
  m.sensor_e_kinetic = (mjm.sensor_type == mujoco.mjtSensor.mjSENS_E_KINETIC).any()
  m.sensor_tendonactfrc_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_TENDONACTFRC)[0]
  subtreevel_sensors = (mujoco.mjtSensor.mjSENS_SUBTREELINVEL, mujoco.mjtSensor.mjSENS_SUBTREEANGMOM)
  m.sensor_subtree_vel = np.isin(mjm.sensor_type, subtreevel_sensors).any()
  m.sensor_contact_adr = np.nonzero(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT)[0]
  m.sensor_adr_to_contact_adr = np.clip(np.cumsum(mjm.sensor_type == mujoco.mjtSensor.mjSENS_CONTACT) - 1, a_min=0, a_max=None)
  m.sensor_rne_postconstraint = np.isin(
    mjm.sensor_type,
    [
      mujoco.mjtSensor.mjSENS_ACCELEROMETER,
      mujoco.mjtSensor.mjSENS_FORCE,
      mujoco.mjtSensor.mjSENS_TORQUE,
      mujoco.mjtSensor.mjSENS_FRAMELINACC,
      mujoco.mjtSensor.mjSENS_FRAMEANGACC,
    ],
  ).any()
  m.sensor_rangefinder_bodyid = mjm.site_bodyid[mjm.sensor_objid[mjm.sensor_type == mujoco.mjtSensor.mjSENS_RANGEFINDER]]
  m.taxel_vertadr = [
    j + mjm.mesh_vertadr[mjm.sensor_objid[i]]
    for i in range(mjm.nsensor)
    if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
    for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
  ]
  m.taxel_sensorid = [
    i
    for i in range(mjm.nsensor)
    if mjm.sensor_type[i] == mujoco.mjtSensor.mjSENS_TACTILE
    for j in range(mjm.mesh_vertnum[mjm.sensor_objid[i]])
  ]

  # qM_tiles records the block diagonal structure of qM
  tile_corners = [i for i in range(mjm.nv) if mjm.dof_parentid[i] == -1]
  tiles = {}
  for i in range(len(tile_corners)):
    tile_beg = tile_corners[i]
    tile_end = mjm.nv if i == len(tile_corners) - 1 else tile_corners[i + 1]
    tiles.setdefault(tile_end - tile_beg, []).append(tile_beg)
  m.qM_tiles = tuple(types.TileSet(adr=wp.array(tiles[sz], dtype=int), size=sz) for sz in sorted(tiles.keys()))

  # qLD_updates has dof tree ordering of qLD updates for sparse factor m
  qLD_updates, dof_depth = {}, np.zeros(mjm.nv, dtype=int) - 1

  for k in range(mjm.nv):
    # skip diagonal rows
    if mjm.M_rownnz[k] == 1:
      continue
    dof_depth[k] = dof_depth[mjm.dof_parentid[k]] + 1
    i = mjm.dof_parentid[k]
    diag_k = mjm.M_rowadr[k] + mjm.M_rownnz[k] - 1
    Madr_ki = diag_k - 1
    while i > -1:
      qLD_updates.setdefault(dof_depth[i], []).append((i, k, Madr_ki))
      i = mjm.dof_parentid[i]
      Madr_ki -= 1
  m.qLD_updates = tuple(wp.array(qLD_updates[i], dtype=wp.vec3i) for i in sorted(qLD_updates))

  # indices for sparse qM_fullm (used in solver)
  m.qM_fullm_i, m.qM_fullm_j = [], []
  for i in range(mjm.nv):
    j = i
    while j > -1:
      m.qM_fullm_i.append(i)
      m.qM_fullm_j.append(j)
      j = mjm.dof_parentid[j]

  # indices for sparse qM mul_m (used in support)
  m.qM_mulm_i, m.qM_mulm_j, m.qM_madr_ij = [], [], []
  for i in range(mjm.nv):
    madr_ij, j = mjm.dof_Madr[i], i

    while True:
      madr_ij, j = madr_ij + 1, mjm.dof_parentid[j]
      if j == -1:
        break
      m.qM_mulm_i.append(i)
      m.qM_mulm_j.append(j)
      m.qM_madr_ij.append(madr_ij)

  # place m on device
  sizes = dict({"*": 1}, **{f.name: getattr(m, f.name) for f in dataclasses.fields(types.Model) if f.type is int})
  for f in dataclasses.fields(types.Model):
    if isinstance(f.type, wp.array):
      setattr(m, f.name, _create_array(getattr(m, f.name), f.type, sizes))

  return m


def _get_padded_sizes(nv: int, njmax: int, is_sparse: bool, tile_size: int):
  # if dense - we just pad to the next multiple of 4 for nv, to get the fast load path.
  #            we pad to the next multiple of tile_size for njmax to avoid out of bounds accesses.
  # if sparse - we pad to the next multiple of tile_size for njmax, and nv.

  def round_up(x, multiple):
    return ((x + multiple - 1) // multiple) * multiple

  njmax_padded = round_up(njmax, tile_size)
  nv_padded = round_up(nv, tile_size) if (is_sparse or nv > 32) else round_up(nv, 4)

  return njmax_padded, nv_padded


def make_data(
  mjm: mujoco.MjModel,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
) -> types.Data:
  """Creates a data object on device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).
    nworld: Number of worlds.
    nconmax: Number of contacts to allocate per world. Contacts exist in large
             heterogeneous arrays: one world may have more than nconmax contacts.
    njmax: Number of constraints to allocate per world. Constraint arrays are
           batched by world: no world may have more than njmax constraints.
    naconmax: Number of contacts to allocate for all worlds. Overrides nconmax.

  Returns:
    The data object containing the current state and output arrays (device).
  """
  # TODO(team): move nconmax, njmax to Model?
  # TODO(team): improve heuristic for nconmax and njmax
  nconmax = nconmax or 20
  njmax = njmax or nconmax * 6

  if nworld < 1:
    raise ValueError(f"nworld must be >= 1")

  if naconmax is None:
    if nconmax < 0:
      raise ValueError("nconmax must be >= 0")
    naconmax = max(512, nworld * nconmax)
  elif naconmax < 0:
    raise ValueError("naconmax must be >= 0")

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  sizes = dict({"*": 1}, **{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model) if f.type is int})
  sizes["nmaxcondim"] = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  sizes["nmaxpyramid"] = np.maximum(1, 2 * (sizes["nmaxcondim"] - 1))
  tile_size = types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  sizes["njmax_pad"], sizes["nv_pad"] = _get_padded_sizes(mjm.nv, njmax, is_sparse(mjm), tile_size)
  sizes["nworld"] = nworld
  sizes["naconmax"] = naconmax
  sizes["njmax"] = njmax

  contact = types.Contact(**{f.name: _create_array(None, f.type, sizes) for f in dataclasses.fields(types.Contact)})
  efc = types.Constraint(**{f.name: _create_array(None, f.type, sizes) for f in dataclasses.fields(types.Constraint)})

  # world body and static geom (attached to the world) poses are precomputed
  # this speeds up scenes with many static geoms (e.g. terrains)
  # TODO(team): remove this when we introduce dof islands + sleeping
  mjd = mujoco.MjData(mjm)
  mujoco.mj_kinematics(mjm, mjd)

  # mocap
  mocap_body = np.nonzero(mjm.body_mocapid >= 0)[0]
  mocap_id = mjm.body_mocapid[mocap_body]

  d_kwargs = {
    "qpos": wp.array(np.tile(mjm.qpos0, nworld), shape=(nworld, mjm.nq), dtype=float),
    "contact": contact,
    "efc": efc,
    "nworld": nworld,
    "naconmax": naconmax,
    "njmax": njmax,
    "qM": None,
    "qLD": None,
    # world body
    "xquat": wp.array(np.tile(mjd.xquat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.quat),
    "xmat": wp.array(np.tile(mjd.xmat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.mat33),
    "ximat": wp.array(np.tile(mjd.ximat, (nworld, 1)), shape=(nworld, mjm.nbody), dtype=wp.mat33),
    # static geoms
    "geom_xpos": wp.array(np.tile(mjd.geom_xpos, (nworld, 1)), shape=(nworld, mjm.ngeom), dtype=wp.vec3),
    "geom_xmat": wp.array(np.tile(mjd.geom_xmat, (nworld, 1)), shape=(nworld, mjm.ngeom), dtype=wp.mat33),
    # mocap
    "mocap_pos": wp.array(np.tile(mjm.body_pos[mocap_body[mocap_id]], (nworld, 1)), shape=(nworld, mjm.nmocap), dtype=wp.vec3),
    "mocap_quat": wp.array(
      np.tile(mjm.body_quat[mocap_body[mocap_id]], (nworld, 1)), shape=(nworld, mjm.nmocap), dtype=wp.quat
    ),
    # equality constraints
    "eq_active": wp.array(np.tile(mjm.eq_active0.astype(bool), (nworld, 1)), shape=(nworld, mjm.neq), dtype=bool),
  }
  for f in dataclasses.fields(types.Data):
    if f.name in d_kwargs:
      continue
    d_kwargs[f.name] = _create_array(None, f.type, sizes)

  d = types.Data(**d_kwargs)

  if is_sparse(mjm):
    d.qM = wp.zeros((nworld, 1, mjm.nM), dtype=float)
    d.qLD = wp.zeros((nworld, 1, mjm.nC), dtype=float)
  else:
    d.qM = wp.zeros((nworld, sizes["nv_pad"], sizes["nv_pad"]), dtype=float)
    d.qLD = wp.zeros((nworld, mjm.nv, mjm.nv), dtype=float)

  return d


def put_data(
  mjm: mujoco.MjModel,
  mjd: mujoco.MjData,
  nworld: int = 1,
  nconmax: Optional[int] = None,
  njmax: Optional[int] = None,
  naconmax: Optional[int] = None,
) -> types.Data:
  """Moves data from host to a device.

  Args:
    mjm: The model containing kinematic and dynamic information (host).
    mjd: The data object containing current state and output arrays (host).
    nworld: The number of worlds.
    nconmax: Number of contacts to allocate per world.  Contacts exist in large
             heterogenous arrays: one world may have more than nconmax contacts.
    njmax: Number of constraints to allocate per world.  Constraint arrays are
           batched by world: no world may have more than njmax constraints.
    naconmax: Number of contacts to allocate for all worlds. Overrides nconmax.

  Returns:
    The data object containing the current state and output arrays (device).
  """
  # TODO(team): move nconmax and njmax to Model?
  # TODO(team): decide what to do about uninitialized warp-only fields created by put_data
  #             we need to ensure these are only workspace fields and don't carry state

  # TODO(team): better heuristic for nconmax and njmax
  nconmax = nconmax or max(5, 4 * mjd.ncon)
  njmax = njmax or max(5, 4 * mjd.nefc)

  if nworld < 1:
    raise ValueError(f"nworld must be >= 1")

  if naconmax is None:
    if nconmax < 0:
      raise ValueError("nconmax must be >= 0")

    if mjd.ncon > nconmax:
      raise ValueError(f"nconmax overflow (nconmax must be >= {mjd.ncon})")

    naconmax = max(512, nworld * nconmax)
  elif naconmax < mjd.ncon * nworld:
    raise ValueError(f"naconmax overflow (naconmax must be >= {mjd.ncon * nworld})")

  if njmax < 0:
    raise ValueError("njmax must be >= 0")

  if mjd.nefc > njmax:
    raise ValueError(f"njmax overflow (njmax must be >= {mjd.nefc})")

  sizes = dict({"*": 1}, **{f.name: getattr(mjm, f.name, None) for f in dataclasses.fields(types.Model) if f.type is int})
  sizes["nmaxcondim"] = np.concatenate(([0], mjm.geom_condim, mjm.pair_dim)).max()
  sizes["nmaxpyramid"] = np.maximum(1, 2 * (sizes["nmaxcondim"] - 1))
  tile_size = types.TILE_SIZE_JTDAJ_SPARSE if is_sparse(mjm) else types.TILE_SIZE_JTDAJ_DENSE
  sizes["njmax_pad"], sizes["nv_pad"] = _get_padded_sizes(mjm.nv, njmax, is_sparse(mjm), tile_size)
  sizes["nworld"] = nworld
  sizes["naconmax"] = naconmax
  sizes["njmax"] = njmax

  # ensure static geom positions are computed
  # TODO: remove once MjData creation semantics are fixed
  mujoco.mj_kinematics(mjm, mjd)

  # create contact
  contact_kwargs = {"efc_address": None, "worldid": None, "type": None, "geomcollisionid": None}
  for f in dataclasses.fields(types.Contact):
    if f.name in contact_kwargs:
      continue
    val = getattr(mjd.contact, f.name)
    val = np.repeat(val, nworld, axis=0)
    width = ((0, naconmax - val.shape[0]),) + ((0, 0),) * (val.ndim - 1)
    val = np.pad(val, width)
    contact_kwargs[f.name] = _create_array(val, f.type, sizes)

  contact = types.Contact(**contact_kwargs)

  contact.efc_address = np.zeros((naconmax, sizes["nmaxpyramid"]), dtype=int)
  for i in range(mjd.ncon):
    efc_address = mjd.contact.efc_address[i]
    if efc_address == -1:
      continue
    condim = mjd.contact.dim[i]
    ndim = max(1, 2 * (condim - 1)) if mjm.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL else condim
    for j in range(nworld):
      contact.efc_address[j * mjd.ncon + i, :ndim] = efc_address + np.arange(ndim)

  contact.efc_address = wp.array(contact.efc_address, dtype=int)
  contact.worldid = np.pad(np.repeat(np.arange(nworld), mjd.ncon), (0, naconmax - nworld * mjd.ncon))
  contact.worldid = wp.array(contact.worldid, dtype=int)
  contact.type = wp.ones((naconmax,), dtype=int)  # TODO(team): set values
  contact.geomcollisionid = wp.empty((naconmax,), dtype=int)  # TODO(team): set values

  # create efc
  efc_kwargs = {"J": None}

  for f in dataclasses.fields(types.Constraint):
    if f.name in efc_kwargs:
      continue
    shape = tuple(sizes[dim] if isinstance(dim, str) else dim for dim in f.type.shape)
    val = np.zeros(shape, dtype=f.type.dtype)
    if f.name in ("type", "id", "pos", "margin", "D", "vel", "aref", "frictionloss", "force"):
      val[:, : mjd.nefc] = np.tile(getattr(mjd, "efc_" + f.name), (nworld, 1))
    efc_kwargs[f.name] = wp.array(val, dtype=f.type.dtype)

  efc = types.Constraint(**efc_kwargs)

  if mujoco.mj_isSparse(mjm):
    efc_j = np.zeros((mjd.nefc, mjm.nv))
    mujoco.mju_sparse2dense(efc_j, mjd.efc_J, mjd.efc_J_rownnz, mjd.efc_J_rowadr, mjd.efc_J_colind)
  else:
    efc_j = mjd.efc_J.reshape((mjd.nefc, mjm.nv))
  efc.J = np.zeros((nworld, sizes["njmax_pad"], sizes["nv_pad"]), dtype=f.type.dtype)
  efc.J[:, : mjd.nefc, : mjm.nv] = np.tile(efc_j, (nworld, 1, 1))
  efc.J = wp.array(efc.J, dtype=float)

  # create data
  d_kwargs = {
    "contact": contact,
    "efc": efc,
    "nworld": nworld,
    "naconmax": naconmax,
    "njmax": njmax,
    # fields set after initialization:
    "solver_niter": None,
    "qM": None,
    "qLD": None,
    "ten_J": None,
    "actuator_moment": None,
    "flexedge_J": None,
    "nacon": None,
    "ne_connect": None,
    "ne_weld": None,
    "ne_jnt": None,
    "ne_ten": None,
    "ne_flex": None,
    "nsolving": None,
  }
  for f in dataclasses.fields(types.Data):
    if f.name in d_kwargs:
      continue
    val = getattr(mjd, f.name, None)
    if val is not None:
      shape = val.shape if hasattr(val, "shape") else ()
      val = np.full((nworld,) + shape, val)
    d_kwargs[f.name] = _create_array(val, f.type, sizes)

  d = types.Data(**d_kwargs)
  d.solver_niter = wp.full((nworld,), mjd.solver_niter[0], dtype=int)

  if is_sparse(mjm):
    d.qM = wp.array(np.full((nworld, 1, mjm.nM), mjd.qM), dtype=float)
    d.qLD = wp.array(np.full((nworld, 1, mjm.nC), mjd.qLD), dtype=float)
  else:
    qM = np.zeros((mjm.nv, mjm.nv))
    mujoco.mj_fullM(mjm, qM, mjd.qM)
    qLD = np.linalg.cholesky(qM) if (mjd.qM != 0.0).any() and (mjd.qLD != 0.0).any() else np.zeros((mjm.nv, mjm.nv))
    padding = sizes["nv_pad"] - mjm.nv
    qM_padded = np.pad(qM, ((0, padding), (0, padding)), mode="constant", constant_values=0.0)
    d.qM = wp.array(np.full((nworld, sizes["nv_pad"], sizes["nv_pad"]), qM_padded), dtype=float)
    d.qLD = wp.array(np.full((nworld, mjm.nv, mjm.nv), qLD), dtype=float)

  if mujoco.mj_isSparse(mjm):
    ten_J = np.zeros((mjm.ntendon, mjm.nv))
    mujoco.mju_sparse2dense(ten_J, mjd.ten_J.reshape(-1), mjd.ten_J_rownnz, mjd.ten_J_rowadr, mjd.ten_J_colind.reshape(-1))
    d.ten_J = wp.array(np.full((nworld, mjm.ntendon, mjm.nv), ten_J), dtype=float)
    flexedge_J = np.zeros((mjm.nflexedge, mjm.nv))
    mujoco.mju_sparse2dense(
      flexedge_J, mjd.flexedge_J.reshape(-1), mjd.flexedge_J_rownnz, mjd.flexedge_J_rowadr, mjd.flexedge_J_colind.reshape(-1)
    )
    d.flexedge_J = wp.array(np.full((nworld, mjm.nflexedge, mjm.nv), flexedge_J), dtype=float)
  else:
    ten_J = mjd.ten_J.reshape((mjm.ntendon, mjm.nv))
    d.ten_J = wp.array(np.full((nworld, mjm.ntendon, mjm.nv), ten_J), dtype=float)
    flexedge_J = mjd.flexedge_J.reshape((mjm.nflexedge, mjm.nv))
    d.flexedge_J = wp.array(np.full((nworld, mjm.nflexedge, mjm.nv), flexedge_J), dtype=float)

  # TODO(taylorhowell): sparse actuator_moment
  actuator_moment = np.zeros((mjm.nu, mjm.nv))
  mujoco.mju_sparse2dense(actuator_moment, mjd.actuator_moment, mjd.moment_rownnz, mjd.moment_rowadr, mjd.moment_colind)
  d.actuator_moment = wp.array(np.full((nworld, mjm.nu, mjm.nv), actuator_moment), dtype=float)

  d.nacon = wp.array([mjd.ncon * nworld], dtype=int)
  d.ne_connect = wp.full(nworld, 3 * np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_CONNECT) & mjd.eq_active), dtype=int)
  d.ne_weld = wp.full(nworld, 6 * np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_WELD) & mjd.eq_active), dtype=int)
  d.ne_jnt = wp.full(nworld, np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_JOINT) & mjd.eq_active), dtype=int)
  d.ne_ten = wp.full(nworld, np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_TENDON) & mjd.eq_active), dtype=int)
  d.ne_flex = wp.full(nworld, np.sum((mjm.eq_type == mujoco.mjtEq.mjEQ_FLEX) & mjd.eq_active), dtype=int)
  d.nsolving = wp.array([nworld], dtype=int)

  return d


def get_data_into(
  result: mujoco.MjData,
  mjm: mujoco.MjModel,
  d: types.Data,
  world_id: int = 0,
):
  """Gets data from a device into an existing mujoco.MjData.

  Args:
    result: The data object containing the current state and output arrays (host).
    mjm: The model containing kinematic and dynamic information (host).
    d: The data object containing the current state and output arrays (device).
    world_id: The id of the world to get the data from.
  """
  # nacon and nefc can overflow.  in that case, only pull up to the max contacts and constraints
  nacon = min(d.nacon.numpy()[0], d.naconmax)
  nefc = min(d.nefc.numpy()[world_id], d.njmax)

  ncon_filter = np.zeros_like(d.contact.worldid.numpy(), dtype=bool)
  ncon_filter[:nacon] = d.contact.worldid.numpy()[:nacon] == world_id
  ncon = ncon_filter.sum()

  if ncon != result.ncon or nefc != result.nefc:
    # TODO(team): if sparse, set nJ based on sparse efc_J
    mujoco._functions._realloc_con_efc(result, ncon=ncon, nefc=nefc, nJ=nefc * mjm.nv)

  ne = d.ne.numpy()[world_id]
  nf = d.nf.numpy()[world_id]
  nl = d.nl.numpy()[world_id]

  # efc indexing
  # mujoco expects contiguous efc ordering for contacts
  # this ordering is not guaranteed with mujoco warp, we enforce order here
  if ncon > 0:
    efc_idx_efl = np.arange(ne + nf + nl)

    contact_dim = d.contact.dim.numpy()[ncon_filter]
    contact_efc_address = d.contact.efc_address.numpy()[ncon_filter]

    efc_idx_c = []
    contact_efc_address_ordered = [ne + nf + nl]
    for i in range(ncon):
      dim = contact_dim[i]
      if mjm.opt.cone == mujoco.mjtCone.mjCONE_PYRAMIDAL:
        ndim = np.maximum(1, 2 * (dim - 1))
      else:
        ndim = dim
      efc_idx_c.append(contact_efc_address[i, :ndim])
      if i < ncon - 1:
        contact_efc_address_ordered.append(contact_efc_address_ordered[-1] + ndim)
    efc_idx = np.concatenate((efc_idx_efl, *efc_idx_c))
    contact_efc_address_ordered = np.array(contact_efc_address_ordered)
  else:
    efc_idx = np.array(np.arange(nefc))
    contact_efc_address_ordered = np.empty(0)

  efc_idx = efc_idx[:nefc]  # dont emit indices for overflow constraints

  result.solver_niter[0] = d.solver_niter.numpy()[world_id]
  result.ncon = ncon
  result.ne = ne
  result.nf = nf
  result.nl = nl
  result.time = d.time.numpy()[world_id]
  result.energy[:] = d.energy.numpy()[world_id]
  result.qpos[:] = d.qpos.numpy()[world_id]
  result.qvel[:] = d.qvel.numpy()[world_id]
  result.act[:] = d.act.numpy()[world_id]
  result.qacc_warmstart[:] = d.qacc_warmstart.numpy()[world_id]
  result.ctrl[:] = d.ctrl.numpy()[world_id]
  result.qfrc_applied[:] = d.qfrc_applied.numpy()[world_id]
  result.xfrc_applied[:] = d.xfrc_applied.numpy()[world_id]
  result.eq_active[:] = d.eq_active.numpy()[world_id]
  result.mocap_pos[:] = d.mocap_pos.numpy()[world_id]
  result.mocap_quat[:] = d.mocap_quat.numpy()[world_id]
  result.qacc[:] = d.qacc.numpy()[world_id]
  result.act_dot[:] = d.act_dot.numpy()[world_id]
  result.xpos[:] = d.xpos.numpy()[world_id]
  result.xquat[:] = d.xquat.numpy()[world_id]
  result.xmat[:] = d.xmat.numpy()[world_id].reshape((-1, 9))
  result.xipos[:] = d.xipos.numpy()[world_id]
  result.ximat[:] = d.ximat.numpy()[world_id].reshape((-1, 9))
  result.xanchor[:] = d.xanchor.numpy()[world_id]
  result.xaxis[:] = d.xaxis.numpy()[world_id]
  result.geom_xpos[:] = d.geom_xpos.numpy()[world_id]
  result.geom_xmat[:] = d.geom_xmat.numpy()[world_id].reshape((-1, 9))
  result.site_xpos[:] = d.site_xpos.numpy()[world_id]
  result.site_xmat[:] = d.site_xmat.numpy()[world_id].reshape((-1, 9))
  result.cam_xpos[:] = d.cam_xpos.numpy()[world_id]
  result.cam_xmat[:] = d.cam_xmat.numpy()[world_id].reshape((-1, 9))
  result.light_xpos[:] = d.light_xpos.numpy()[world_id]
  result.light_xdir[:] = d.light_xdir.numpy()[world_id]
  result.subtree_com[:] = d.subtree_com.numpy()[world_id]
  result.cdof[:] = d.cdof.numpy()[world_id]
  result.cinert[:] = d.cinert.numpy()[world_id]
  result.flexvert_xpos[:] = d.flexvert_xpos.numpy()[world_id]
  result.flexedge_J[:] = d.flexedge_J.numpy()[world_id]
  result.flexedge_length[:] = d.flexedge_length.numpy()[world_id]
  result.flexedge_velocity[:] = d.flexedge_velocity.numpy()[world_id]
  result.actuator_length[:] = d.actuator_length.numpy()[world_id]
  mujoco.mju_dense2sparse(
    result.actuator_moment,
    d.actuator_moment.numpy()[world_id],
    result.moment_rownnz,
    result.moment_rowadr,
    result.moment_colind,
  )
  result.crb[:] = d.crb.numpy()[world_id]
  result.qLDiagInv[:] = d.qLDiagInv.numpy()[world_id]
  result.ten_velocity[:] = d.ten_velocity.numpy()[world_id]
  result.actuator_velocity[:] = d.actuator_velocity.numpy()[world_id]
  result.cvel[:] = d.cvel.numpy()[world_id]
  result.cdof_dot[:] = d.cdof_dot.numpy()[world_id]
  result.qfrc_bias[:] = d.qfrc_bias.numpy()[world_id]
  result.qfrc_spring[:] = d.qfrc_spring.numpy()[world_id]
  result.qfrc_damper[:] = d.qfrc_damper.numpy()[world_id]
  result.qfrc_gravcomp[:] = d.qfrc_gravcomp.numpy()[world_id]
  result.qfrc_fluid[:] = d.qfrc_fluid.numpy()[world_id]
  result.qfrc_passive[:] = d.qfrc_passive.numpy()[world_id]
  result.subtree_linvel[:] = d.subtree_linvel.numpy()[world_id]
  result.subtree_angmom[:] = d.subtree_angmom.numpy()[world_id]
  result.actuator_force[:] = d.actuator_force.numpy()[world_id]
  result.qfrc_actuator[:] = d.qfrc_actuator.numpy()[world_id]
  result.qfrc_smooth[:] = d.qfrc_smooth.numpy()[world_id]
  result.qacc_smooth[:] = d.qacc_smooth.numpy()[world_id]
  result.qfrc_constraint[:] = d.qfrc_constraint.numpy()[world_id]
  result.qfrc_inverse[:] = d.qfrc_inverse.numpy()[world_id]

  # contact
  result.contact.dist[:ncon] = d.contact.dist.numpy()[ncon_filter]
  result.contact.pos[:ncon] = d.contact.pos.numpy()[ncon_filter]
  result.contact.frame[:ncon] = d.contact.frame.numpy()[ncon_filter].reshape((-1, 9))
  result.contact.includemargin[:ncon] = d.contact.includemargin.numpy()[ncon_filter]
  result.contact.friction[:ncon] = d.contact.friction.numpy()[ncon_filter]
  result.contact.solref[:ncon] = d.contact.solref.numpy()[ncon_filter]
  result.contact.solreffriction[:ncon] = d.contact.solreffriction.numpy()[ncon_filter]
  result.contact.solimp[:ncon] = d.contact.solimp.numpy()[ncon_filter]
  result.contact.dim[:ncon] = d.contact.dim.numpy()[ncon_filter]
  result.contact.geom[:ncon] = d.contact.geom.numpy()[ncon_filter]
  result.contact.efc_address[:ncon] = contact_efc_address_ordered[:ncon]

  if is_sparse(mjm):
    result.qM[:] = d.qM.numpy()[world_id, 0]
    result.qLD[:] = d.qLD.numpy()[world_id, 0]
  else:
    qM = d.qM.numpy()[world_id]
    adr = 0
    for i in range(mjm.nv):
      j = i
      while j >= 0:
        result.qM[adr] = qM[i, j]
        j = mjm.dof_parentid[j]
        adr += 1
    mujoco.mj_factorM(mjm, result)

  if nefc > 0:
    if mujoco.mj_isSparse(mjm):
      efc_J = d.efc.J.numpy()[world_id, efc_idx, : mjm.nv]
      mujoco.mju_dense2sparse(result.efc_J, efc_J, result.efc_J_rownnz, result.efc_J_rowadr, result.efc_J_colind)
    else:
      result.efc_J[: nefc * mjm.nv] = d.efc.J.numpy()[world_id, :nefc, : mjm.nv].flatten()

  # efc
  result.efc_type[:] = d.efc.type.numpy()[world_id, efc_idx]
  result.efc_id[:] = d.efc.id.numpy()[world_id, efc_idx]
  result.efc_pos[:] = d.efc.pos.numpy()[world_id, efc_idx]
  result.efc_margin[:] = d.efc.margin.numpy()[world_id, efc_idx]
  result.efc_D[:] = d.efc.D.numpy()[world_id, efc_idx]
  result.efc_vel[:] = d.efc.vel.numpy()[world_id, efc_idx]
  result.efc_aref[:] = d.efc.aref.numpy()[world_id, efc_idx]
  result.efc_frictionloss[:] = d.efc.frictionloss.numpy()[world_id, efc_idx]
  result.efc_state[:] = d.efc.state.numpy()[world_id, efc_idx]
  result.efc_force[:] = d.efc.force.numpy()[world_id, efc_idx]

  # rne_postconstraint
  result.cacc[:] = d.cacc.numpy()[world_id]
  result.cfrc_int[:] = d.cfrc_int.numpy()[world_id]
  result.cfrc_ext[:] = d.cfrc_ext.numpy()[world_id]

  # tendon
  result.ten_length[:] = d.ten_length.numpy()[world_id]
  result.ten_J[:] = d.ten_J.numpy()[world_id]
  result.ten_wrapadr[:] = d.ten_wrapadr.numpy()[world_id]
  result.ten_wrapnum[:] = d.ten_wrapnum.numpy()[world_id]
  result.wrap_obj[:] = d.wrap_obj.numpy()[world_id]
  result.wrap_xpos[:] = d.wrap_xpos.numpy()[world_id]

  # sensors
  result.sensordata[:] = d.sensordata.numpy()[world_id]


def reset_data(m: types.Model, d: types.Data, reset: Optional[wp.array] = None):
  """Clear data, set defaults; optionally by world.

  Args:
    m: The model containing kinematic and dynamic information (device).
    d: The data object containing the current state and output arrays (device).
    reset: Per-world bitmask. Reset if True.
  """

  @nested_kernel(module="unique", enable_backward=False)
  def reset_xfrc_applied(reset_in: wp.array(dtype=bool), xfrc_applied_out: wp.array2d(dtype=wp.spatial_vector)):
    worldid, bodyid, elemid = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    xfrc_applied_out[worldid, bodyid][elemid] = 0.0

  @nested_kernel(module="unique", enable_backward=False)
  def reset_qM(reset_in: wp.array(dtype=bool), qM_out: wp.array3d(dtype=float)):
    worldid, elemid1, elemid2 = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    qM_out[worldid, elemid1, elemid2] = 0.0

  @nested_kernel(module="unique", enable_backward=False)
  def reset_nworld(
    # Model:
    nq: int,
    nv: int,
    nu: int,
    na: int,
    neq: int,
    nsensordata: int,
    qpos0: wp.array2d(dtype=float),
    eq_active0: wp.array(dtype=bool),
    # Data in:
    nworld_in: int,
    # In:
    reset_in: wp.array(dtype=bool),
    # Data out:
    solver_niter_out: wp.array(dtype=int),
    ne_out: wp.array(dtype=int),
    nf_out: wp.array(dtype=int),
    nl_out: wp.array(dtype=int),
    nefc_out: wp.array(dtype=int),
    time_out: wp.array(dtype=float),
    energy_out: wp.array(dtype=wp.vec2),
    qpos_out: wp.array2d(dtype=float),
    qvel_out: wp.array2d(dtype=float),
    act_out: wp.array2d(dtype=float),
    qacc_warmstart_out: wp.array2d(dtype=float),
    ctrl_out: wp.array2d(dtype=float),
    qfrc_applied_out: wp.array2d(dtype=float),
    eq_active_out: wp.array2d(dtype=bool),
    qacc_out: wp.array2d(dtype=float),
    act_dot_out: wp.array2d(dtype=float),
    sensordata_out: wp.array2d(dtype=float),
    nacon_out: wp.array(dtype=int),
    ne_connect_out: wp.array(dtype=int),
    ne_weld_out: wp.array(dtype=int),
    ne_jnt_out: wp.array(dtype=int),
    ne_ten_out: wp.array(dtype=int),
    ne_flex_out: wp.array(dtype=int),
    nsolving_out: wp.array(dtype=int),
  ):
    worldid = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    solver_niter_out[worldid] = 0
    if worldid == 0:
      nacon_out[0] = 0
    ne_out[worldid] = 0
    ne_connect_out[worldid] = 0
    ne_weld_out[worldid] = 0
    ne_jnt_out[worldid] = 0
    ne_ten_out[worldid] = 0
    ne_flex_out[worldid] = 0
    nf_out[worldid] = 0
    nl_out[worldid] = 0
    nefc_out[worldid] = 0
    if worldid == 0:
      nsolving_out[0] = nworld_in
    time_out[worldid] = 0.0
    energy_out[worldid] = wp.vec2(0.0, 0.0)
    qpos0_id = worldid % qpos0.shape[0]
    for i in range(nq):
      qpos_out[worldid, i] = qpos0[qpos0_id, i]
      if i < nv:
        qvel_out[worldid, i] = 0.0
        qacc_warmstart_out[worldid, i] = 0.0
        qfrc_applied_out[worldid, i] = 0.0
        qacc_out[worldid, i] = 0.0
    for i in range(nu):
      ctrl_out[worldid, i] = 0.0
      if i < na:
        act_out[worldid, i] = 0.0
        act_dot_out[worldid, i] = 0.0
    for i in range(neq):
      eq_active_out[worldid, i] = eq_active0[i]
    for i in range(nsensordata):
      sensordata_out[worldid, i] = 0.0

  @nested_kernel(module="unique", enable_backward=False)
  def reset_mocap(
    # Model:
    body_mocapid: wp.array(dtype=int),
    body_pos: wp.array2d(dtype=wp.vec3),
    body_quat: wp.array2d(dtype=wp.quat),
    # In:
    reset_in: wp.array(dtype=bool),
    # Data out:
    mocap_pos_out: wp.array2d(dtype=wp.vec3),
    mocap_quat_out: wp.array2d(dtype=wp.quat),
  ):
    worldid, bodyid = wp.tid()

    if wp.static(reset is not None):
      if not reset_in[worldid]:
        return

    mocapid = body_mocapid[bodyid]

    if mocapid >= 0:
      mocap_pos_out[worldid, mocapid] = body_pos[worldid, bodyid]
      mocap_quat_out[worldid, mocapid] = body_quat[worldid, bodyid]

  @nested_kernel(module="unique", enable_backward=False)
  def reset_contact(
    # Data in:
    nacon_in: wp.array(dtype=int),
    # In:
    reset_in: wp.array(dtype=bool),
    nefcaddress: int,
    # Data out:
    contact_dist_out: wp.array(dtype=float),
    contact_pos_out: wp.array(dtype=wp.vec3),
    contact_frame_out: wp.array(dtype=wp.mat33),
    contact_includemargin_out: wp.array(dtype=float),
    contact_friction_out: wp.array(dtype=types.vec5),
    contact_solref_out: wp.array(dtype=wp.vec2),
    contact_solreffriction_out: wp.array(dtype=wp.vec2),
    contact_solimp_out: wp.array(dtype=types.vec5),
    contact_dim_out: wp.array(dtype=int),
    contact_geom_out: wp.array(dtype=wp.vec2i),
    contact_efc_address_out: wp.array2d(dtype=int),
    contact_worldid_out: wp.array(dtype=int),
    contact_type_out: wp.array(dtype=int),
    contact_geomcollisionid_out: wp.array(dtype=int),
  ):
    conid = wp.tid()

    if conid >= nacon_in[0]:
      return

    worldid = contact_worldid_out[conid]
    if wp.static(reset is not None):
      if worldid >= 0:
        if not reset_in[worldid]:
          return

    contact_dist_out[conid] = 0.0
    contact_pos_out[conid] = wp.vec3(0.0)
    contact_frame_out[conid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    contact_includemargin_out[conid] = 0.0
    contact_friction_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
    contact_solref_out[conid] = wp.vec2(0.0, 0.0)
    contact_solreffriction_out[conid] = wp.vec2(0.0, 0.0)
    contact_solimp_out[conid] = types.vec5(0.0, 0.0, 0.0, 0.0, 0.0)
    contact_dim_out[conid] = 0
    contact_geom_out[conid] = wp.vec2i(0, 0)
    for i in range(nefcaddress):
      contact_efc_address_out[conid, i] = 0
    contact_worldid_out[conid] = 0
    contact_type_out[conid] = 0
    contact_geomcollisionid_out[conid] = 0

  reset_input = reset or wp.ones(d.nworld, dtype=bool)

  wp.launch(reset_xfrc_applied, dim=(d.nworld, m.nbody, 6), inputs=[reset_input], outputs=[d.xfrc_applied])
  wp.launch(
    reset_qM,
    dim=(d.nworld, d.qM.shape[1], d.qM.shape[2]),
    inputs=[reset_input],
    outputs=[d.qM],
  )

  # set mocap_pos/quat = body_pos/quat for mocap bodies
  wp.launch(
    reset_mocap,
    dim=(d.nworld, m.nbody),
    inputs=[m.body_mocapid, m.body_pos, m.body_quat, reset_input],
    outputs=[d.mocap_pos, d.mocap_quat],
  )

  # clear contacts
  wp.launch(
    reset_contact,
    dim=d.naconmax,
    inputs=[d.nacon, reset_input, d.contact.efc_address.shape[1]],
    outputs=[
      d.contact.dist,
      d.contact.pos,
      d.contact.frame,
      d.contact.includemargin,
      d.contact.friction,
      d.contact.solref,
      d.contact.solreffriction,
      d.contact.solimp,
      d.contact.dim,
      d.contact.geom,
      d.contact.efc_address,
      d.contact.worldid,
      d.contact.type,
      d.contact.geomcollisionid,
    ],
  )

  wp.launch(
    reset_nworld,
    dim=d.nworld,
    inputs=[m.nq, m.nv, m.nu, m.na, m.neq, m.nsensordata, m.qpos0, m.eq_active0, d.nworld, reset_input],
    outputs=[
      d.solver_niter,
      d.ne,
      d.nf,
      d.nl,
      d.nefc,
      d.time,
      d.energy,
      d.qpos,
      d.qvel,
      d.act,
      d.qacc_warmstart,
      d.ctrl,
      d.qfrc_applied,
      d.eq_active,
      d.qacc,
      d.act_dot,
      d.sensordata,
      d.nacon,
      d.ne_connect,
      d.ne_weld,
      d.ne_jnt,
      d.ne_ten,
      d.ne_flex,
      d.nsolving,
    ],
  )


def override_model(model: Union[types.Model, mujoco.MjModel], overrides: Union[dict[str, Any], Sequence[str]]):
  """Overrides model parameters.

  Overrides are of the format:
    opt.iterations = 1
    opt.ls_parallel = True
    opt.cone = pyramidal
    opt.disableflags = contact | spring
  """
  enum_fields = {
    "opt.broadphase": types.BroadphaseType,
    "opt.broadphase_filter": types.BroadphaseFilter,
    "opt.cone": types.ConeType,
    "opt.disableflags": types.DisableBit,
    "opt.enableflags": types.EnableBit,
    "opt.integrator": types.IntegratorType,
    "opt.solver": types.SolverType,
  }
  mjw_only_fields = {"opt.broadphase", "opt.broadphase_filter", "opt.ls_parallel", "opt.graph_conditional"}
  mj_only_fields = {"opt.jacobian"}

  if not isinstance(overrides, dict):
    overrides_dict = {}
    for override in overrides:
      if "=" not in override:
        raise ValueError(f"Invalid override format: {override}")
      k, v = override.split("=", 1)
      overrides_dict[k.strip()] = v.strip()
    overrides = overrides_dict

  for key, val in overrides.items():
    # skip overrides on MjModel for properties that are only on mjw.Model
    if key in mjw_only_fields and isinstance(model, mujoco.MjModel):
      continue
    if key in mj_only_fields and isinstance(model, types.Model):
      continue

    obj, attrs = model, key.split(".")
    for i, attr in enumerate(attrs):
      if not hasattr(obj, attr):
        raise ValueError(f"Unrecognized model field: {key}")
      if i < len(attrs) - 1:
        obj = getattr(obj, attr)
        continue

      typ = type(getattr(obj, attr))

      if key in enum_fields and isinstance(val, str):
        # special case: enum value
        enum_members = val.split("|")
        val = 0
        for enum_member in enum_members:
          enum_member = enum_member.strip().upper()
          if enum_member not in enum_fields[key].__members__:
            raise ValueError(f"Unrecognized enum value for {enum_fields[key].__name__}: {enum_member}")
          val |= int(enum_fields[key][enum_member])
      elif typ is bool and isinstance(val, str):
        # special case: "true", "TRUE", "false", "FALSE" etc.
        if val.upper() not in ("TRUE", "FALSE"):
          raise ValueError(f"Unrecognized value for field: {key}")
        val = val.upper() == "TRUE"
      else:
        val = typ(val)

      setattr(obj, attr, val)


def find_keys(model: mujoco.MjModel, keyname_prefix: str) -> list[int]:
  """Finds keyframes that start with keyname_prefix."""
  keys = []

  for keyid in range(model.nkey):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, keyid)
    if name.startswith(keyname_prefix):
      keys.append(keyid)

  return keys


def make_trajectory(model: mujoco.MjModel, keys: list[int]) -> np.ndarray:
  """Make a ctrl trajectory with linear interpolation."""
  ctrls = []
  prev_ctrl_key = np.zeros(model.nu, dtype=np.float64)
  prev_time, time = 0.0, 0.0

  for key in keys:
    ctrl_key, ctrl_time = model.key_ctrl[key], model.key_time[key]
    if not ctrls and ctrl_time != 0.0:
      raise ValueError("first keyframe must have time 0.0")
    elif ctrls and ctrl_time <= prev_time:
      raise ValueError("keyframes must be in time order")

    while time < ctrl_time:
      frac = (time - prev_time) / (ctrl_time - prev_time)
      ctrls.append(prev_ctrl_key * (1 - frac) + ctrl_key * frac)
      time += model.opt.timestep

    ctrls.append(ctrl_key)
    time += model.opt.timestep
    prev_ctrl_key = ctrl_key
    prev_time = time

  return np.array(ctrls)
